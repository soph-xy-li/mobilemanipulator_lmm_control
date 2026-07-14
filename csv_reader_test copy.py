import csv
from collections import defaultdict
import pygame
import math
import time

import can
from pcan_cybergear import CANMotorController

import robomaster
from robomaster import robot

ep_robot = robot.Robot()
ep_robot.initialize(conn_type='sta')
ep_chassis = ep_robot.chassis
'''
bus = can.Bus(
    interface="pcan",
    channel="PCAN_USBBUS1",
    bitrate=1000000,
)

motor1 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=101, main_can_id=254)

motor1.enable()
motor2.enable()
motor1.set_0_pos()
motor2.set_0_pos()
'''
main_dict = defaultdict(list)

with open('x_q1_q2_sequence (1).csv', mode='r',newline='',encoding='utf-8') as file:
    reader = csv.DictReader(file)
    #row_reader = csv.reader(file)
    #row_count = sum(1 for row in row_reader)
    for row in reader:
        for column_name, value in row.items():
            main_dict[column_name].append(value)

main_dict = dict(main_dict)

with open('x_q1_q2_sequence (1).csv', mode='r',newline='',encoding='utf-8') as file:
    #reader = csv.DictReader(file)
    row_reader = csv.reader(file)
    row_count = sum(1 for row in row_reader)
row_count -= 2

#print(main_dict)
#print(row_count)




pygame.init()
font = pygame.font.SysFont("arial", 20, bold=True)

screen_dimensions = (750,350)
screen = pygame.display.set_mode(screen_dimensions)
clock = pygame.time.Clock() 

button_color = (0, 100, 255)
button_pressed_color = (60,135,230)

startbutton_center = (375, 175)
startbutton_radius = 30
startbutton_pressed = False

def draw_fake_3d_button(screen, center, radius, base_color, pressed):
    x, y = center

    shadow_offset = 4
    pygame.draw.circle(screen, (0, 0, 0), (x, y + shadow_offset), radius)

    press_offset = 3 if pressed else 0
    pygame.draw.circle(screen, base_color, (x, y + press_offset), radius)
    pygame.draw.circle(screen, (0, 60, 180), (x, y + press_offset), radius, 2)

    highlight_radius = radius // 2
    pygame.draw.circle(screen, button_pressed_color, (x - 4, y - 4 + press_offset), highlight_radius)

def draw_button_label(screen, text, center, color=(225, 225, 225)):
    label = font.render(text, True, color)
    rect = label.get_rect(center=center)
    screen.blit(label, rect)

running = True
starting_actions = False

while running:
    mousex, mousey = pygame.mouse.get_pos()
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:

            dist = math.hypot(mousex - startbutton_center[0], mousey - startbutton_center[1])

            if dist <= startbutton_radius:
                startbutton_pressed = True
                starting_actions = True

        elif event.type == pygame.MOUSEBUTTONUP:
            startbutton_pressed = False
        
    if starting_actions:
        screen.blit(font.render("running!", True, (225,225,225)), (240, 75))
        pygame.display.flip()
        #motor2.send_motor_control_command(torque=0, target_angle=1.49824521248188-0.17, target_velocity=0, Kp=2, Kd=2)
        #motor1.send_motor_control_command(torque=0, target_angle=0.6136034180209045+1.504572608+0.17, target_velocity=0, Kp=2, Kd=2)
        time.sleep(1.5)
        
        for i in range(row_count):
            ep_chassis.drive_speed(x=-float(main_dict["x_m"][i]), y=0, z=0)
            #print(motor1.send_motor_control_command(torque=0, target_angle=(2.11817603+float(main_dict["q2_raw_rad"][i])), target_velocity=0, Kp=15, Kd=2))
            #print(motor2.send_motor_control_command(torque=0, target_angle=(1.49824521248188+float(main_dict["q1_rad"][i])), target_velocity=0, Kp=15, Kd=2))
            #print(float(main_dict["q2_raw_rad"][i]))
            #print(float(main_dict["q1_rad"][i]))
            time.sleep(1/60)
        
        starting_actions = False
        ep_chassis.drive_speed(x=0, y=0, z=0)

    screen.fill((30, 30, 35))
    startcolor = button_pressed_color if startbutton_pressed else button_color
    draw_fake_3d_button(screen, startbutton_center, startbutton_radius, startcolor, startbutton_pressed)
    draw_button_label(screen, "start", (startbutton_center[0], startbutton_center[1]+3) if startbutton_pressed else startbutton_center)
    
    pygame.display.flip()
    clock.tick(60)

#motor1.disable()
#motor2.disable()
#bus.shutdown()
ep_chassis.drive_speed(x=0, y=0, z=0)
ep_robot.close()

pygame.quit()