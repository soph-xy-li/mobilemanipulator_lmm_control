#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LMM 键盘正/反放脚本
--------------------------------------------------------------------
  按 1  正放:  底盘 前进0.2m + 左移0.2m (不 wait_for_completed)
               同时 机械臂 poly5  0.5s:  m1 0 -> 1.027 rad, m2 0 -> 1.51 rad
               到位后 motor2 增益降到 Kp=0.8, Kd=0.2 (柔顺保持)
  按 2  反放:  底盘 后退0.2m + 右移0.2m
               同时 机械臂 poly5  0.5s 回到 (0, 0), 恢复正常增益
  按 q  退出:  先回零位, 再 disable + 关闭总线

机械臂用一个 500Hz 的后台线程持续下发 MIT 指令, 保证运动结束后
仍然一直在保持位置(CyberGear 需要持续刷指令才不会掉状态)。
"""

import os
import sys
import time
import threading

import numpy as np
import can
from pcan_cybergear import CANMotorController
from robomaster import robot

# ============================ 参数 ============================
# ---- 底盘 ----
CHASSIS_X = 0.2          # 前进 [m]
CHASSIS_Y = 0.2          # 左移 [m]  (RoboMaster: y 正=右, 左移用 -y)
CHASSIS_XY_SPEED = 0.7   # [m/s]

# ---- 机械臂 ----
DT       = 0.002         # 控制周期 [s] -> 500 Hz
T_MOVE   = 0.5           # poly5 运动时长 [s]
Q_HOME   = [0.0,   0.0]  # 零位 (上电位置 set zero)
Q_DEPLOY = [1.027, 1.51] # 正放目标 [rad]   m1, m2

# CAN / 电机 ID
# Jetson 上如果 PEAK 驱动装的是 netdev 模式(或者用板载 mttcan),
# 改成 CAN_INTERFACE="socketcan", CAN_CHANNEL="can0", 并先在 shell 里:
#   sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0
CAN_INTERFACE = "pcan"
CAN_CHANNEL   = "PCAN_USBBUS1"
CAN_BITRATE   = 1000000
M1_ID, M2_ID  = 101, 102
MAIN_CAN_ID   = 254

# 增益
M1_KP, M1_KD = 300.0, 3.0      # motor1 全程用这一组
M2_KP_MOVE, M2_KD_MOVE = 44.4, 2.0    # motor2 运动 / 回零时
M2_KP_HOLD, M2_KD_HOLD = 0.8,  0.2    # motor2 到位后柔顺保持

# 前馈力矩
M1_TAU_FF = 0.0
M2_TAU_FF = 0.3          # motor2 常值前馈 [Nm]


# ========================= poly5 =========================
def poly5(t, T, q0, qf):
    """rest-to-rest 五次多项式, 返回 (pos, vel, acc)"""
    if t <= 0:
        return q0, 0.0, 0.0
    if t >= T:
        return qf, 0.0, 0.0
    s = t / T
    s2, s3, s4, s5 = s * s, s ** 3, s ** 4, s ** 5
    dq = qf - q0
    pos = q0 + dq * (6 * s5 - 15 * s4 + 10 * s3)
    vel = dq * (30 * s4 - 60 * s3 + 30 * s2) / T
    acc = dq * (120 * s3 - 180 * s2 + 60 * s) / T ** 2
    return pos, vel, acc


# ====================== 机械臂控制线程 ======================
class ArmController(threading.Thread):
    def __init__(self, m1, m2):
        super().__init__(daemon=True)
        self.m1, self.m2 = m1, m2
        self._lock = threading.Lock()
        self._alive = True

        self.q_ref = list(Q_HOME)     # 当前参考位置
        self._q_start = list(Q_HOME)
        self._q_goal = list(Q_HOME)
        self._t0 = 0.0
        self._T = T_MOVE
        self._moving = False

        self._compliant = False       # motor2 当前是否低增益
        self._compliant_after = False # 本段运动结束后是否切低增益

        self.q_meas = list(Q_HOME)    # 反馈

    # ---- 外部接口 ----
    def goto(self, q_goal, T=T_MOVE, compliant_after=False):
        with self._lock:
            self._q_start = list(self.q_ref)
            self._q_goal = list(q_goal)
            self._T = T
            self._t0 = time.perf_counter()
            self._moving = True
            self._compliant = False        # 运动过程一律用正常增益
            self._compliant_after = compliant_after

    def is_moving(self):
        with self._lock:
            return self._moving

    def stop(self):
        self._alive = False

    # ---- 主循环 ----
    def run(self):
        # Jetson 上 500Hz 用普通调度抖动比较大, 有 root 就提到 SCHED_FIFO
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
            print("[arm] 控制线程已切到 SCHED_FIFO(80)")
        except (AttributeError, PermissionError, OSError):
            print("[arm] 未获得实时优先级 (非 root), 用普通调度")

        self.overrun = 0
        next_tick = time.perf_counter()
        while self._alive:
            with self._lock:
                if self._moving:
                    t = time.perf_counter() - self._t0
                    ref = [poly5(t, self._T, self._q_start[i], self._q_goal[i])
                           for i in range(2)]
                    if t >= self._T:
                        self._moving = False
                        self._compliant = self._compliant_after
                else:
                    ref = [(self._q_goal[i], 0.0, 0.0) for i in range(2)]
                compliant = self._compliant
                self.q_ref = [ref[0][0], ref[1][0]]

            kp2, kd2 = ((M2_KP_HOLD, M2_KD_HOLD) if compliant
                        else (M2_KP_MOVE, M2_KD_MOVE))

            try:
                r1 = self.m1.send_motor_control_command(
                    torque=M1_TAU_FF,
                    target_angle=ref[0][0], target_velocity=ref[0][1],
                    Kp=M1_KP, Kd=M1_KD)
                r2 = self.m2.send_motor_control_command(
                    torque=M2_TAU_FF,
                    target_angle=ref[1][0], target_velocity=ref[1][1],
                    Kp=kp2, Kd=kd2)
                if r1 is not None and len(r1) >= 2:
                    self.q_meas[0] = r1[1]
                if r2 is not None and len(r2) >= 2:
                    self.q_meas[1] = r2[1]
            except Exception as e:
                print(f"[arm] CAN 发送异常: {e}")

            next_tick += DT
            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                self.overrun += 1
                next_tick = time.perf_counter()   # 掉周期就重新对齐


# ========================= 键盘读取 =========================
if os.name == "nt":
    import msvcrt

    class KeyReader:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            pass

        def get(self):
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    return ch.decode("utf-8", "ignore")
                except Exception:
                    return None
            return None
else:
    import termios
    import tty
    import select

    class KeyReader:
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self

        def __exit__(self, *a):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

        def get(self):
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
            return None


# ============================ 主程序 ============================
def main():
    # ---------- 底盘 ----------
    print("初始化 RoboMaster ...")
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_chassis = ep_robot.chassis

    # ---------- CAN / 电机 ----------
    print("初始化 CAN ...")
    if CAN_INTERFACE == "socketcan":
        bus = can.interface.Bus(interface="socketcan", channel=CAN_CHANNEL)
    else:
        bus = can.interface.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL,
                                bitrate=CAN_BITRATE)
    m1 = CANMotorController(bus, motor_id=M1_ID, main_can_id=MAIN_CAN_ID)
    m2 = CANMotorController(bus, motor_id=M2_ID, main_can_id=MAIN_CAN_ID)

    # 上电位置置零 (置零要求先失能)
    for m in (m1, m2):
        try:
            m.disable()
        except Exception:
            pass
    time.sleep(0.05)
    m1.set_0_pos()
    m2.set_0_pos()
    print("已把当前位置设为 0")
    time.sleep(0.1)

    m1.enable()
    m2.enable()
    time.sleep(0.05)

    arm = ArmController(m1, m2)
    arm.start()

    state = "home"   # home / deploy
    print("""
==================================================
  1 = 正放 (前进0.2 + 左移0.2, 臂展开)
  2 = 反放 (后退0.2 + 右移0.2, 臂回零)
  q = 退出
==================================================""")

    try:
        with KeyReader() as kr:
            while True:
                key = kr.get()
                if key is None:
                    time.sleep(0.02)
                    continue

                if key == "1":
                    if state == "deploy":
                        print("已经在正放位置, 忽略")
                        continue
                    if arm.is_moving():
                        print("机械臂运动中, 忽略")
                        continue
                    print(">>> 正放")
                    ep_chassis.move(x=CHASSIS_X, y=-CHASSIS_Y, z=0,
                                    xy_speed=CHASSIS_XY_SPEED)   # 不等待
                    arm.goto(Q_DEPLOY, T=T_MOVE, compliant_after=True)
                    state = "deploy"

                elif key == "2":
                    if state == "home":
                        print("已经在零位, 忽略")
                        continue
                    if arm.is_moving():
                        print("机械臂运动中, 忽略")
                        continue
                    print(">>> 反放")
                    ep_chassis.move(x=-CHASSIS_X, y=CHASSIS_Y, z=0,
                                    xy_speed=CHASSIS_XY_SPEED)   # 不等待
                    arm.goto(Q_HOME, T=T_MOVE, compliant_after=False)
                    state = "home"

                elif key in ("q", "Q", "\x03"):
                    break

    except KeyboardInterrupt:
        pass

    finally:
        print("\n收尾: 机械臂回零 ...")
        if state != "home":
            arm.goto(Q_HOME, T=1.0, compliant_after=False)
            time.sleep(1.2)
        arm.stop()
        arm.join(timeout=1.0)
        print(f"控制线程掉周期次数: {getattr(arm, 'overrun', 0)}")
        for m in (m1, m2):
            try:
                m.disable()
            except Exception:
                pass
        bus.shutdown()
        ep_robot.close()
        print("Done.")


if __name__ == "__main__":
    main()
