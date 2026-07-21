import asyncio
import json

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
    ep_chassis.drive_speed(x=0, y=0, z=0)

def position_handler(position):
    global chassis_x, chassis_y, chassis_z
    global zero_x, zero_y, zero_z
    x, y, z = position

    chassis_x = x - zero_x
    chassis_y = y - zero_y
    chassis_z = z - zero_z

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

ep_chassis.sub_position(freq=10, callback=position_handler)

async def handle_client(websocket):
    global last_cmd_time

    client = websocket.remote_address
    print(f"[+] Client connected: {client}")

    try:
        await websocket.send(json.dumps({"type": "hello", "msg": "connected"}))

        async for message in websocket:
            try:
                cmd = json.loads(message)
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

            await websocket.send(json.dumps({
                "type": "telemetry",
                "current_top": current_top,
                "current_bot": current_bot,
                "chassis_x" : chassis_x,
                "chassis_y" : chassis_y,
                "chassis_z" : chassis_z,
            }))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"[-] Client disconnected: {client} -> STOP")
        stop_everything()


async def watchdog():
    global last_cmd_time
    while True:
        await asyncio.sleep(0.1)
        now = asyncio.get_event_loop().time()
        if last_cmd_time > 0 and now - last_cmd_time > WATCHDOG_TIMEOUT:
            stop_everything()
            last_cmd_time = 0  # reset so we don't spam stop() every 100ms


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
        motor1.disable()
        motor2.disable()
        bus.shutdown()
        ep_robot.close()
        print("Robot closed. Bye!")