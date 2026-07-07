import pygame
import math
import time
import can
from pcan_cybergear import CANMotorController
import robomaster
from robomaster import robot

bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
motor1 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=101, main_can_id=254)

MAX_SPEED = 2
TURN_SPEED = 60
KP = 30
KD = 4
TOP_MAX_MOTOR = math.pi
#TOP_MAX_MOTOR = 2.376783398184177
BOT_MAX_MOTOR = 2.772373540856032

button_color = (0, 100, 255)
button_pressed_color = (60,135,230)

motor1.enable()
motor2.enable()

motor1.set_0_pos()
motor2.set_0_pos()
motor1.set_run_mode(motor1.RunModes.CONTROL_MODE)
motor2.set_run_mode(motor2.RunModes.CONTROL_MODE)

ep_robot = robot.Robot()
ep_robot.initialize(conn_type='sta')
ep_chassis = ep_robot.chassis


# pygame setup
pygame.init()
#font = pygame.font.Font(None, 28)
font = pygame.font.SysFont("arial", 20, bold=True)

screen_dimensions = (750,350)
screen = pygame.display.set_mode(screen_dimensions)
clock = pygame.time.Clock() 

pygame.joystick.init()

controller = None
if pygame.joystick.get_count() > 0:
    controller = pygame.joystick.Joystick(0)
    controller.init()
    print(f"Connected: {controller.get_name()}")
else:
    print("No controller connected")

lx, ly = 0.0, 0.0

# arm sliders
topslider_center = (640, 210)
botslider_center = (640, 290)
slider_length = 150
slider_knob_radius = 23

topslider_x = topslider_center[0] - slider_length //2
botslider_x = botslider_center[0] - slider_length //2
topslider_zero = topslider_x
botslider_zero = botslider_x

# joystick
joystick_center = (102, 245)
joystick_radius = 75
knob_radius = 23

knob_x = joystick_center[0]
knob_y = joystick_center[1]

# buttons
right_button_center = (270,255)
left_button_center = (220,300)
button_radius = 27.5

# touchscreen events
touch_joystick = None
touch_topslider = None
touch_botslider = None
touch_left = None
touch_right = None

def touch_to_screen(event):
    return (
        int(event.x * screen_dimensions[0]),
        int(event.y * screen_dimensions[1]),
    )

# the things
topslider_dragging = False
botslider_dragging = False
last_angletop = None
last_anglebot = None
dragging = False
running = True
turn_right = False
turn_left = False
turn_left_cont = False
turn_right_cont = False
z=0


def draw_fake_3d_button(screen, center, radius, base_color, pressed):
    x, y = center

    shadow_offset = 4
    pygame.draw.circle(screen, (0, 0, 0), (x, y + shadow_offset), radius)

    press_offset = 3 if pressed else 0
    pygame.draw.circle(screen, base_color, (x, y + press_offset), radius)
    pygame.draw.circle(screen, (0, 60, 180), (x, y + press_offset), radius, 2)

    highlight_radius = radius // 2
    pygame.draw.circle(screen, button_pressed_color, (x - 4, y - 4 + press_offset), highlight_radius)

def draw_knob(screen, x, y, radius, dragging):
    x, y = int(x), int(y)

    pygame.draw.circle(screen, (5, 5, 5), (x, y + 4), radius)

    press_offset = 3 if dragging else 0
    base_color = button_color if not dragging else button_pressed_color

    pygame.draw.circle(screen, base_color, (x, y + press_offset), radius)

    pygame.draw.circle(screen, (0, 60, 180), (x, y + press_offset), radius, 2)

    highlight_radius = radius // 2
    pygame.draw.circle(screen, button_pressed_color, (x - 4, y - 4 + press_offset), highlight_radius)

def draw_button_label(screen, text, center, color=(225, 225, 225)):
    label = font.render(text, True, color)
    rect = label.get_rect(center=center)
    screen.blit(label, rect)
try:
    while running:
        mousex, mousey = pygame.mouse.get_pos()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                #joystick_center = (mousex, mousey)
                #for touchscreen, need to figure out how to limit joystick location to not interfere w/ buttons
                dist = math.hypot(mousex - joystick_center[0], mousey - joystick_center[1])
                dist_right_button = math.hypot(mousex - right_button_center[0], mousey - right_button_center[1])
                dist_left_button = math.hypot(mousex - left_button_center[0], mousey - left_button_center[1])
                dist_slidertop = math.hypot(mousex - topslider_x, mousey - topslider_center[1])
                dist_sliderbot = math.hypot(mousex - botslider_x, mousey - botslider_center[1])

                if dist_slidertop <= slider_knob_radius:
                    topslider_dragging = True
                if dist_sliderbot <= slider_knob_radius:
                    botslider_dragging = True
                if dist <= joystick_radius:
                    dragging = True
                if dist_right_button <= button_radius:
                    turn_right = True
                if dist_left_button <= button_radius:
                    turn_left = True

            elif event.type == pygame.MOUSEBUTTONUP:
                dragging = False
                turn_left = False
                turn_right = False
                topslider_dragging = False
                botslider_dragging = False
                z=0

                # joystick to center
                target_x = joystick_center[0]
                target_y = joystick_center[1]
            
            elif event.type == pygame.FINGERDOWN:
                tx, ty = touch_to_screen(event)

                dist = math.hypot(tx - joystick_center[0], ty - joystick_center[1])
                dist_right = math.hypot(tx - right_button_center[0], ty - right_button_center[1])
                dist_left = math.hypot(tx - left_button_center[0], ty - left_button_center[1])
                dist_top = math.hypot(tx - topslider_x, ty - topslider_center[1])
                dist_bot = math.hypot(tx - botslider_x, ty - botslider_center[1])

                if dist <= joystick_radius and touch_joystick is None:
                    touch_joystick = event.finger_id
                    dragging = True
                elif dist_top <= slider_knob_radius and touch_topslider is None:
                    touch_topslider = event.finger_id
                    topslider_dragging = True
                elif dist_bot <= slider_knob_radius and touch_botslider is None:
                    touch_botslider = event.finger_id
                    botslider_dragging = True
                elif dist_left <= button_radius and touch_left is None:
                    touch_left = event.finger_id
                    turn_left = True
                elif dist_right <= button_radius and touch_right is None:
                    touch_right = event.finger_id
                    turn_right = True


            elif event.type == pygame.FINGERMOTION:
                tx, ty = touch_to_screen(event)
                if event.finger_id == touch_joystick:
                    mousex, mousey = tx, ty
                elif event.finger_id == touch_topslider:
                    topslider_x = max(
                        topslider_center[0] - slider_length // 2,
                        min(topslider_center[0] + slider_length // 2, tx))
                elif event.finger_id == touch_botslider:
                    botslider_x = max(
                        botslider_center[0] - slider_length // 2,
                        min(botslider_center[0] + slider_length // 2, tx))


            elif event.type == pygame.FINGERUP:
                if event.finger_id == touch_joystick:
                    touch_joystick = None
                    dragging = False
                    target_x = joystick_center[0]
                    target_y = joystick_center[1]
                elif event.finger_id == touch_topslider:
                    touch_topslider = None
                    topslider_dragging = False
                elif event.finger_id == touch_botslider:
                    touch_botslider = None
                    botslider_dragging = False
                elif event.finger_id == touch_left:
                    touch_left = None
                    turn_left = False
                elif event.finger_id == touch_right:
                    touch_right = None
                    turn_right = False
        
        if controller:
            joycontdeadzone = 0.1
            # left stick
            lx = controller.get_axis(0)
            ly = controller.get_axis(1)
            if abs(lx) < joycontdeadzone:
                lx = 0
            if abs(ly) < joycontdeadzone:
                ly = 0
            knob_x = joystick_center[0] + lx * joystick_radius
            knob_y = joystick_center[1] + ly * joystick_radius
            dragging = True

            # buttons square circle
            square = controller.get_button(2)
            circle = controller.get_button(1)
            turn_left_cont = square
            turn_right_cont = circle

            # top slider
            l1 = controller.get_button(9)
            l2 = (controller.get_axis(4) + 1) / 2
            if l1:
                topslider_x = min(topslider_x + 10, topslider_center[0] + slider_length // 2)
            if l2 > 0.1 and topslider_x > 0:
                topslider_x = max(topslider_x - l2 * 10, topslider_center[0] - slider_length // 2)
            
            # bottom slider
            r1 = controller.get_button(10)
            r2 = (controller.get_axis(5) + 1) / 2
            if r1:
                botslider_x = min(botslider_x + 5, botslider_center[0] + slider_length // 2)
            if r2 > 0.1 and botslider_x > 0:
                botslider_x = max(botslider_x - r2 * 5, botslider_center[0] - slider_length // 2)

        target_x = joystick_center[0]
        target_y = joystick_center[1]
        
        if dragging and not controller:
            dx = mousex - joystick_center[0]
            dy = mousey - joystick_center[1]

            dist = math.hypot(dx, dy)

            if dist > joystick_radius:
                dx *= (joystick_radius / dist)
                dy *= (joystick_radius / dist)

            target_x = joystick_center[0] + dx
            target_y = joystick_center[1] + dy

        if topslider_dragging:
            topslider_x = max(topslider_center[0] - slider_length//2, min(topslider_center[0] + slider_length//2, mousex))
            
        if botslider_dragging:
            botslider_x = max(botslider_center[0] - slider_length//2, min(botslider_center[0] + slider_length//2, mousex))

        z=0

        if turn_right or turn_right_cont:
            z=TURN_SPEED

        elif turn_left or turn_left_cont:
            z=-TURN_SPEED
            

        # smoothing joystick
        smooth = 0.4
        knob_x += (target_x - knob_x) * smooth
        knob_y += (target_y - knob_y) * smooth

        # normalizing chassis
        dx = knob_x - joystick_center[0]
        dy = knob_y - joystick_center[1]

        x_norm = -dy / joystick_radius
        y_norm = dx / joystick_radius

        x_speed = x_norm * MAX_SPEED
        y_speed = y_norm * MAX_SPEED

        # normalizing arm
        top_delta = (topslider_x - topslider_zero) / slider_length
        bot_delta = (botslider_x - botslider_zero) / slider_length

        angletop = top_delta * TOP_MAX_MOTOR
        anglebot = bot_delta * BOT_MAX_MOTOR

        angletop = max(-TOP_MAX_MOTOR, min(TOP_MAX_MOTOR, angletop))
        anglebot = max(-BOT_MAX_MOTOR, min(BOT_MAX_MOTOR, anglebot))

        '''
        if last_angletop is None or abs(angletop - last_angletop) > 0.01:
            motor1.send_motor_control_command(torque=0, target_angle=angletop, target_velocity=0, Kp=KP, Kd=KD)
        last_angletop = angletop

        if last_anglebot is None or abs(anglebot - last_anglebot) > 0.01:
            motor2.send_motor_control_command(torque=0, target_angle=anglebot, target_velocity=0, Kp=KP, Kd=KD)
        last_anglebot = anglebot

        #''
        ep_chassis.drive_speed(x=x_speed, y=y_speed, z=z)
        '''

        # shapes :)
        screen.fill((30, 30, 35))
        for x in range(0, screen_dimensions[0], 25):
            pygame.draw.line(screen, (35,35,40), (x,0), (x,screen_dimensions[1]))
        for y in range(0, screen_dimensions[1], 25):
            pygame.draw.line(screen, (35,35,40), (0,y), (screen_dimensions[0],y))

        # color
        left_color = button_pressed_color if turn_left else button_color
        right_color = button_pressed_color if turn_right else button_color
        left_radius = button_radius - 1 if turn_left else button_radius
        right_radius = button_radius - 1 if turn_right else button_radius
        
        # sliders
        pygame.draw.line(screen, (100,100,100), (topslider_center[0]-slider_length//2, topslider_center[1]), (topslider_center[0]+slider_length//2, topslider_center[1]),16)
        pygame.draw.circle(screen, (100,100,100), (topslider_center[0]-slider_length//2, topslider_center[1]+1), 8)
        pygame.draw.circle(screen, (100,100,100), (topslider_center[0]+slider_length//2, topslider_center[1]+1), 8)
        draw_knob(screen, topslider_x, topslider_center[1], slider_knob_radius, topslider_dragging)
        screen.blit(font.render("Top Arm", True, (225,225,225)), (topslider_center[0]-slider_length/4, topslider_center[1]-43))
        screen.blit(font.render(f"Top: {math.degrees(angletop):.1f}°", True, (225,225,225)), (520, 40))
    
        pygame.draw.line(screen, (100,100,100), (botslider_center[0]-slider_length//2, botslider_center[1]), (botslider_center[0]+slider_length//2, botslider_center[1]),16)
        pygame.draw.circle(screen, (100,100,100), (botslider_center[0]-slider_length//2, botslider_center[1]+1), 8)
        pygame.draw.circle(screen, (100,100,100), (botslider_center[0]+slider_length//2, botslider_center[1]+1), 8)
        draw_knob(screen, botslider_x, botslider_center[1], slider_knob_radius, botslider_dragging)
        screen.blit(font.render("Bottom Arm", True, (225,225,225)), (botslider_center[0]-slider_length/3, botslider_center[1]-43))
        screen.blit(font.render(f"Bottom: {math.degrees(anglebot):.1f}°", True, (225,225,225)), (520, 75))

        # joystick
        pygame.draw.circle(screen, (0, 0, 0), (joystick_center[0],joystick_center[1]+7), joystick_radius)
        pygame.draw.circle(screen, (100, 100, 100), joystick_center, joystick_radius+3)
        pygame.draw.circle(screen, (80, 80, 80), joystick_center, joystick_radius)
        screen.blit(font.render("Drive", True, (225,225,225)), (joystick_center[0]-26, joystick_center[1]-115))
        draw_knob(screen, knob_x, knob_y, knob_radius, dragging)
        
        screenjoydead = 0.25
        gap_distance = math.hypot(x_speed, y_speed)
        diag_margin = 15
        arrow = None
        arrow_x = 0
        arrow_y = 0

        if dragging and gap_distance > screenjoydead:
            arrow_angle = math.degrees(math.atan2(-y_speed, x_speed))
            arrow_angle %= 360
            if arrow_angle < 45 - diag_margin or arrow_angle > 315 + diag_margin:
                arrow = "↑"
                arrow_x = 5
                arrow_y = 12
            elif 45 + diag_margin < arrow_angle < 135 - diag_margin:
                arrow = "←"
                arrow_x = 10
                arrow_y = 10
            elif 135 + diag_margin < arrow_angle < 225 - diag_margin:
                arrow = "↓"
                arrow_x = 5
                arrow_y = 12
            elif 225 + diag_margin < arrow_angle < 315 - diag_margin:
                arrow = "→"
                arrow_x = 10
                arrow_y = 10
            else:
                arrow = None
        
        if arrow:
            screen.blit(font.render(arrow, True, (225,225,225)), (knob_x-arrow_x, knob_y-arrow_y))

        # buttons
        draw_fake_3d_button(screen, left_button_center, left_radius, left_color, turn_left)
        draw_fake_3d_button(screen, right_button_center, right_radius, right_color, turn_right)
        draw_button_label(screen, "L", (left_button_center[0], left_button_center[1]+3) if turn_left else left_button_center)
        draw_button_label(screen, "R", (right_button_center[0], right_button_center[1]+3) if turn_right else right_button_center)

        pygame.display.flip()
        clock.tick(50)
        
except Exception as e:
    import traceback
    traceback.print_exc()
finally:
    
    motor1.disable()
    motor2.disable()
    ep_chassis.drive_speed(x=0, y=0, z=0)
    bus.shutdown()
    ep_robot.close()
    pygame.quit()
    

motor1.disable()
motor2.disable()
ep_chassis.drive_speed(x=0, y=0, z=0)

bus.shutdown()
ep_robot.close()

#pygame.quit()