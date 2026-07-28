import asyncio
import csv
import json
from collections import defaultdict

import websockets

import can
from pcan_cybergear import CANMotorController
import robomaster
from robomaster import robot

bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
motor1 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=101, main_can_id=254)
motor1.enable()
motor2.enable()
motor1.set_0_pos()
motor2.set_0_pos()
motor1.set_run_mode(motor1.RunModes.CONTROL_MODE)
motor2.set_run_mode(motor2.RunModes.CONTROL_MODE)

ep_robot = robot.Robot()
ep_robot.initialize(conn_type='rndis')
ep_chassis = ep_robot.chassis


KP = 20
KD = 2

last_cmd_time = 0
WATCHDOG_TIMEOUT = 0.5

last_angletop = None
last_anglebot = None
current_top = 0.0
current_bot = 0.0

chassis_x = 0.0
chassis_y = 0.0
chassis_z = 0.0
zero_x = 0.0
zero_y = 0.0
zero_z = 0.0
fl_speed = fr_speed = rl_speed = rr_speed = 0
fl_angle = fr_angle = rl_angle = rr_angle = 0

sequence_running = False


def apply_command(x_speed: float, y_speed: float, z: float, angletop: float, anglebot: float):
    global last_angletop, last_anglebot, current_top, current_bot

    if last_angletop is None or abs(angletop - last_angletop) > 0.01:
        topresult = motor1.send_motor_control_command(torque=0, target_angle=angletop, target_velocity=0, Kp=KP, Kd=KD)
        if topresult[1] is not None:
            current_top = topresult[1]
    last_angletop = angletop

    if last_anglebot is None or abs(anglebot - last_anglebot) > 0.01:
        botresult = motor2.send_motor_control_command(torque=0, target_angle=anglebot, target_velocity=0, Kp=KP, Kd=KD)
        if botresult[1] is not None:
            current_bot = botresult[1]
    last_anglebot = anglebot

    ep_chassis.drive_speed(x=x_speed, y=y_speed, z=z)

def stop_everything():
    #ep_chassis.drive_speed(x=0, y=0, z=0)
    pass

def position_handler(position):
    global chassis_x, chassis_y
    global zero_x, zero_y
    x, y, __ = position

    chassis_x = x - zero_x
    chassis_y = y - zero_y

def attitude_handler(attitude):
    global chassis_z, zero_z
    yaw, __, __ = attitude
    chassis_z = yaw - zero_z

def esc_handler(esc):
    global fl_speed, fr_speed, rl_speed, rr_speed
    global fl_angle, fr_angle, rl_angle, rr_angle

    speeds, angles, __, __ = esc

    fl_speed, fr_speed, rl_speed, rr_speed = speeds
    fl_angle, fr_angle, rl_angle, rr_angle = angles

def reset_chassis():
    global zero_x, zero_y, zero_z
    global chassis_x, chassis_y, chassis_z

    zero_x += chassis_x
    zero_y += chassis_y
    zero_z += chassis_z
    chassis_x = 0.0
    chassis_y = 0.0
    chassis_z = 0.0

    print("Chassis origin reset.")

#ep_chassis.sub_position(freq=10, callback=position_handler)
#ep_chassis.sub_attitude(freq=10, callback=attitude_handler)
#ep_chassis.sub_esc(freq=10, callback=esc_handler)

def load_sequence(csv_path):
    data = defaultdict(list)
    with open(csv_path, mode="r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for column_name, value in row.items():
                data[column_name].append(value)
    return dict(data)

async def run_sequence(csv_path):
    global sequence_running, current_top, current_bot
    sequence_running = True

    try:
        data = load_sequence(csv_path)
        row_count = len(next(iter(data.values()))) if data else 0
        print(f"[sequence] loaded {row_count} rows from {csv_path}")
        await asyncio.sleep(1)
 
        for i in range(row_count):
            x_m = float(data["x_m"][i])
            q1_rad = float(data["q1_rad"][i])
            q2_raw_rad = float(data["q2_raw_rad"][i])
            
            topresult = motor1.send_motor_control_command(torque=0, target_angle=2.11817603+q2_raw_rad, target_velocity=0, Kp=15, Kd=2)
            if topresult[1] is not None:
                current_top = topresult[1]
 
            botresult = motor2.send_motor_control_command(torque=0, target_angle=1.49824521248188+q1_rad, target_velocity=0, Kp=15, Kd=2)
            if botresult[1] is not None:
                current_bot = botresult[1]
 
            ep_chassis.drive_speed(x=-x_m, y=0, z=0)
            
            await asyncio.sleep(1/60)
 
    except FileNotFoundError:
        print(f"[sequence] error: couldn't find {csv_path}")
    except (KeyError, ValueError) as e:
        print(f"[sequence] error: bad row data in {csv_path} ({e})")
    finally:
        stop_everything()
        sequence_running = False
        print("[sequence] done")
        if websockets is not None:
            try:
                await websockets.send(json.dumps({"type": "sequence_status", "status": "finished"}))
            except websockets.exceptions.ConnectionClosed:
                pass

async def telemetry_sender(websocket):
    while True:
        try:
            await websocket.send(json.dumps({
                "type": "telemetry",
                "current_top": current_top,
                "current_bot": current_bot,
                "chassis_x" : chassis_x,
                "chassis_y" : chassis_y,
                "chassis_z" : chassis_z,
                "esc": {"speed": {"fl": fl_speed, "fr": fr_speed, "rl": rl_speed, "rr": rr_speed,},
                "angle": {"fl": fl_angle, "fr": fr_angle, "rl": rl_angle, "rr": rr_angle,}}
            }))
            await asyncio.sleep(0.05)
        except websockets.exceptions.ConnectionClosed:
            break

async def handle_client(websocket):
    global last_cmd_time

    client = websocket.remote_address
    print(f"[+] Client connected: {client}")
    telemetry_task = asyncio.create_task(telemetry_sender(websocket))

    try:
        await websocket.send(json.dumps({"type": "hello", "msg": "connected"}))

        async for message in websocket:
            try:
                cmd = json.loads(message)
                if cmd.get("type") == "run_sequence":
                    if sequence_running:
                        await websocket.send(json.dumps({"type": "sequence_status", "status": "busy",}))
                        continue
                    csv_path = cmd.get("csv_path", 'x_q1_q2_sequence (1).csv')
                    asyncio.create_task(run_sequence(csv_path))
                    await websocket.send(json.dumps({"type": "sequence_status", "status": "started", "csv_path": csv_path}))
                    continue
 
                if sequence_running:
                    continue

                if cmd.get("type") == "reset_chassis":
                    reset_chassis()
                    last_cmd_time = asyncio.get_event_loop().time()
                    await websocket.send(json.dumps({
                        "type": "reset_done",
                        "chassis_x": chassis_x,
                        "chassis_y": chassis_y,
                        "chassis_z": chassis_z,
                    }))
                    continue
            except json.JSONDecodeError:
                continue

            last_cmd_time = asyncio.get_event_loop().time()

            x_speed = float(cmd.get("x_speed", 0.0))
            y_speed = float(cmd.get("y_speed", 0.0))
            z = float(cmd.get("z", 0.0))
            angletop = float(cmd.get("angletop", 0.0))
            anglebot = float(cmd.get("anglebot", 0.0))

            apply_command(x_speed, y_speed, z, angletop, anglebot)

            '''
            await websocket.send(json.dumps({
                "type": "telemetry",
                "current_top": current_top,
                "current_bot": current_bot,
                "chassis_x" : chassis_x,
                "chassis_y" : chassis_y,
                "chassis_z" : chassis_z,
                "esc": {"speed": {"fl_speed": fl_speed, "fr_speed": fr_speed, "r_speed": rl_speed, "rr_speed": rr_speed,},
                "angle": {"fl_angle": fl_angle, "fr_angle": fr_angle, "rl_angle": rl_angle, "rr_angle": rr_angle,}}
            }))'''
            
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        telemetry_task.cancel()
        print(f"[-] Client disconnected: {client} -> STOP")
        stop_everything()


async def watchdog():
    global last_cmd_time
    while True:
        await asyncio.sleep(0.1)
        now = asyncio.get_event_loop().time()
        if last_cmd_time > 0 and now - last_cmd_time > WATCHDOG_TIMEOUT:
            stop_everything()
            last_cmd_time = 0


async def main():
    print("Server listening on ws://0.0.0.0:8765")
    print("Open index.html on this computer, a tablet, or a phone on the same WiFi.")
    async with websockets.serve(handle_client, "0.0.0.0", 8765):
        await asyncio.gather(watchdog(), asyncio.Future())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stop_everything()
        
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_chassis.unsub_esc()
        motor1.disable()
        motor2.disable()
        bus.shutdown()
        ep_robot.close()
        
        print("Robot closed. Bye!")