#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import threading


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

def get_filtered_angle(gyro, samples=3):
    angles = []
    for _ in range(samples):
        angles.append(gyro.angle())
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
    
    Kp = 3.0
    Ki = 0.0
    Kd = 1.0

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
        if abs(error) < 2:
            robot.stop()
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

def Forward(distance, speed):
    initial_distance = robot.distance()

    while abs(robot.distance() - initial_distance) < distance:
        robot.drive(speed, 0)

    robot.stop()
    left_wheel.brake()
    right_wheel.brake()


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
    Forward(350, 300)
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
    TurnNew(83)
    Forward(650, 1000)
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

    Forward(250, 300)
    TurnNew(45)    
    Forward(155, 250)
    TurnNew(-45)
    Forward(120, 250)
    TurnNew(-93)
    #ext_st.run_angle(200, 200)
    Forward(165, 200)
    #TurnNew(-5)

    ext_dr.run_angle(700, -400)
    #t1.start()
    #t2.start()


def run3():
    Forward(300, 300)
    TurnNew(30)
    Forward(100, 300)
    TurnNew(-30)
    Forward(200, 300)
    ext_dr.run_angle(200, 150)
    Turn(-35)
    ext_dr.run_angle(700, -300)
    #Forward(50, 300)
           

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
