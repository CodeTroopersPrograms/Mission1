#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import threading
import time

# Create your objects here.
ev3 = EV3Brick()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B)
ext_dr = Motor(Port.D)
ext_st = Motor(Port.A)
robot = DriveBase(left_wheel, right_wheel, wheel_diameter=50, axle_track=110)
gyro = GyroSensor(Port.S3)
#color1 = ColorSensor(Port.S1)
#color2 = ColorSensor(Port.S2)

buttonDown = Button.DOWN
buttonUp = Button.UP
buttonCenter = Button.CENTER
buttonRIGHT = Button.RIGHT

# Functions

def resetAngles():
    gyro.reset_angle(0)
    left_wheel.reset_angle(0)
    right_wheel.reset_angle(0)

def screenct(contor):
    if(contor == 1):
        ev3.screen.print("  **")
        ev3.screen.print(" * *")
        ev3.screen.print("*  *")
        ev3.screen.print("   *")
        ev3.screen.print("   *")
        ev3.screen.print("   *")
    if(contor==2):
        ev3.screen.print("2")

def get_filtered_angle(gyro, samples=5):
    angles = []
    for _ in range(samples):
        angles.append(gyro.angle())
        time.sleep(0.02)
    return sum(angles) / len(angles)


def Turn(degrees):

    while True:
        # Get filtered angle
        filter_angle = get_filtered_angle(gyro)
        
        # Calculate error
        error = filter_angle - degrees
        
        # Stop when close enough to target angle
        if abs(error) < 2:
            robot.stop()
            break
        
        # Adjust speed based on error
        turn_speed = error * 3
        robot.drive(0, turn_speed)  # Adjust turning speed based on error
    print(gyro.angle())
    resetAngles()
    print(gyro.angle())
        


def TurnNew(degrees):
    resetAngles()
    wait(500)
    Kp = 4.0
    Ki = 0.01
    Kd = 1.25

    # Initialize PID variables
    integral = 0.0
    last_error = 0.0
    integral_limit = 1000.0  # Prevent integral from getting too large

    while True:
        # Get filtered angle
        filter_angle = get_filtered_angle(gyro)
        
        # Calculate error (current position - target)
        # Note: If you want the robot to turn positive when error is positive,
        # or the opposite direction, you may invert this calculation as needed.
        error = filter_angle - degrees


        # Check if we are close enough to target
        if abs(error) < 1.5:
            robot.stop()
            resetAngles()
            break
        
        # Calculate integral (accumulated error)
        integral += error
        # Prevent integral windup
        if integral > integral_limit:
            integral = integral_limit
        elif integral < -integral_limit:
            integral = -integral_limit

        # Calculate derivative (change in error)
        derivative = error - last_error

        # Compute PID output
        turn_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Apply the turn speed to the robot
        # drive() takes speed and steering; here speed=0 and steering=turn_speed
        robot.drive(0, turn_speed)

        # Update last_error for the next iteration
        last_error = error

        # Optional small delay to stabilize loop timing
        wait(10)

    # Reset angle after completing the turn, if desired
    resetAngles()
    wait(500)

def Forward(distance, speed):
    initial_distance = robot.distance()

    while abs(robot.distance() - initial_distance) < distance:
        robot.drive(speed, 0)


    robot.stop()
    left_wheel.brake()
    right_wheel.brake()

def Forward_final(distance, speed, stopping_margin=2):
    resetAngles()
    time.sleep(0.1)

    initial_distance = robot.distance()
    target_distance = distance - stopping_margin  # Stop slightly earlier

    while abs(robot.distance() - initial_distance) < target_distance:
        robot.drive(speed, 0)

    # Gradual slowing before stopping
    while abs(robot.distance() - initial_distance) < distance:
        robot.drive(speed * 0.2, 0)
        time.sleep(0.01)  # Allow time for motor update

    robot.stop()
    left_wheel.brake()
    right_wheel.brake()



def TurnNew_final(degrees, Kp=4.0, Ki=0.015, Kd=1.5):
    resetAngles()
    time.sleep(0.1)

    # Initialize PID variables
    integral = 0.0
    last_error = 0.0
    integral_limit = 200.0  # Lower limit to avoid integral windup

    while True:
        # Get the current gyro angle
        current_angle = get_filtered_angle(gyro)
        print(current_angle)

        # Calculate error
        error = current_angle - degrees

        # Check if we are close enough to the target
        if abs(error) < 0.3:  # Tighter stopping threshold for better accuracy
            robot.stop()
            break

        # Calculate PID terms
        integral += error
        integral = max(min(integral, integral_limit), -integral_limit)

        derivative = error - last_error
        turn_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Clamp turn speed to ensure smooth and accurate turns
        turn_speed = max(min(turn_speed, 70), -70)  # Lowered max speed for fine adjustments

        # Apply the turn speed to the robot
        robot.drive(0, turn_speed)

        last_error = error
        time.sleep(0.02)  # Faster updates for finer control

    robot.stop()


def run1():
    ext_st.hold()
    Forward(100, 300)
    TurnNew(-55)
    Forward(350, 300)
    wait(500)
    Forward(145, -300)
    TurnNew(54)
    ext_dr.run_angle(500, -320)
    Forward(425, 250)                                                                                                                                                                                                                                   
    TurnNew(50)
    Forward(45, 300)
    ext_dr.run_angle(500, 320)
    Forward(30, -300)
    TurnNew(140)
    Forward(200, -200)
    Forward(150, 300)
    Turn(70)
    Forward(650, 300)
    Turn(20)
    Forward(200, 300)
    ext_dr.run_angle(500, -320)
    Turn(-25)
    Forward(260, 300)
    Turn(-65)
    Forward(450, 300)

def run1_testFunc():
    ext_st.hold()
    Forward(95, 300)
    TurnNew(-55)
    Forward(355, 300)
    wait(500)
    Forward(150, -300)
    TurnNew(43)
    ext_dr.run_angle(500, -320)
    Forward(400, 250)                                                                                                                                                                                                                                   
    TurnNew(45)
    Forward(45, 300)
    ext_dr.run_angle(500, 320)
    Forward(30, -300) 
    TurnNew(140)
    Forward(230, -200)
    Forward(155, 300)
    TurnNew(86)
    Forward(650, 500)
    TurnNew(30)
    Forward(185, 200)
    TurnNew(-45) 
    ext_dr.run_angle(500, -320)
    Forward(250, 200)
    ext_dr.run_angle(500, 360)
    Forward(50, 150)
    TurnNew(-50)
    Forward(550, 350)

def run2():
    t1 = threading.Thread(target = (ext_st.run_angle), args = (700, -700))
    t2 = threading.Thread(target = (ext_dr.run_angle), args = (700, -400))

    #ext_st.run_angle(500, -100)
    Forward_final(325, 150)
    ext_dr.run_angle(30, 110)
    Forward_final(15, 50)
    ext_dr.run_angle(60, 30)
    Forward_final(25, 50)
    ext_dr.run_angle(150, -50)
    Forward_final(25, 100)
    wait(500)
    Forward_final(400, -250)
    wait(250)
    Forward_final(25, 1000)
    TurnNew_final(40)
    Forward_final(300, 500)
    TurnNew_final(-40)
    Forward_final(240, 300)
    TurnNew_final(-90)
    ext_dr.run_angle(350, 130)
    ext_st.run_angle(100, -15)
    wait(500)
    Forward_final(125, 100)
    ext_st.run_angle(100, -100)
    Forward_final(50, -300)

def run3():
    Forward_final(300, 300)
    TurnNew_final(30)
    Forward_final(100, 300)
    TurnNew_final(-30)
    Forward_final(200, 300)
    ext_dr.run_angle(200, 300)
    TurnNew_final(-25)
    Forward_final(25, 100)
    ext_dr.run_angle(200, -360)
    #Forward(50, 300)
    Forward_final(100, -300)
    TurnNew_final(25)
    Forward_final(50, 200)
    TurnNew_final(43)
    ext_dr.run_angle(200, 50)

# Main program loop
ev3.speaker.beep()
resetAngles()
print(gyro.angle())

contor = 0
canSwitch = True

left_wheel.hold()
right_wheel.hold()

while True:
    if canSwitch:
        for i in ev3.buttons.pressed():
            if i == buttonUp:
                contor += 1
                ev3.screen.print(contor)
                wait(500)  # wait after button press
                break
            elif i == buttonDown and contor > 0:
                contor -= 1
                ev3.screen.print(contor)
                wait(500)  # wait after button press
                break    
            elif i == buttonCenter:
                canSwitch = False
                break    

    if contor == 1 and not canSwitch:
        wait(500)
        run1_testFunc()
        canSwitch = True
    elif contor == 2 and not canSwitch:
        wait(500)
        run2()
        canSwitch = True
    elif contor == 3 and not canSwitch:
        wait(500)
        run3()
        canSwitch = True
