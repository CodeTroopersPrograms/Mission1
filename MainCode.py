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
left_wheel = Motor(Port.B)
right_wheel = Motor(Port.C)
ext_st = Motor(Port.D)
ext_dr = Motor(Port.A)
robot = DriveBase(left_wheel, right_wheel, wheel_diameter=50, axle_track=110)
gyro = GyroSensor(Port.S2)
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
    ext_st.reset_angle(0)
    ext_dr.reset_angle(0)

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

def TurnNew(target_angle, Kp=4.0, Ki=0.0, Kd=1.5, integral_limit=200.0):
    """
    Turns the robot to the target angle using PID control.
    
    Parameters:
    - target_angle: The target angle to turn to (in degrees).
    - Kp: Proportional constant (default 1.0).
    - Ki: Integral constant (default 0.1).
    - Kd: Derivative constant (default 0.05).
    - integral_limit: The maximum allowed value for the integral term (default 200.0).
    """
    # Initialize PID variables
    integral = 0.0  # Integral term starts at 0
    last_error = 0.0  # Last error term starts at 0

    # Normalize target angle to ensure it's between 0 and 360 degrees
    target_angle = target_angle % 360
    if target_angle < 0:
        target_angle += 360  # Ensure that negative angles are converted to the positive range
    
    while True:
        # Get the current gyro angle (you must define `get_filtered_angle(gyro)` elsewhere)
        current_angle = get_filtered_angle(gyro)
        print("Current Angle: {current_angle}")
        
        # Normalize current angle to ensure it's between 0 and 360 degrees
        current_angle = current_angle % 360
        if current_angle < 0:
            current_angle += 360  # Normalize to positive range
        
        # Calculate the shortest path to the target angle
        error = target_angle - current_angle
        
        # If the error is greater than 180 degrees, take the shorter path (wrap around)
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # If we are close enough to the target angle, stop turning
        if abs(error) < 0.5:  # Small threshold for precision
            robot.stop()
            print("Turn complete! Reached target: {target_angle}°")
            break
        
        # Calculate PID terms for smooth turning
        integral += error
        integral = max(min(integral, integral_limit), -integral_limit)  # Clamp integral to avoid windup
        derivative = error - last_error
        
        # Calculate the turn speed using the PID formula
        turn_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)
        turn_speed = max(min(turn_speed, 100), -100)  # Clamp to [-100, 100] for motor limits
        
        # Apply the turn speed to the robot
        robot.drive(0, turn_speed)  # Assuming 0 is forward/backward speed, turn_speed is for rotation
        
        # Update the last error for the next iteration
        last_error = error
        
        # Sleep to avoid overloading the loop
        time.sleep(0.02)  # Update every 20ms for smoother control

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
        error = degrees - current_angle

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
        turn_speed = max(min(turn_speed, 100), -100)  # Lowered max speed for fine adjustments

        # Apply the turn speed to the robot
        robot.drive(0, turn_speed)

        last_error = error
        time.sleep(0.002)  # Faster updates for finer control

    robot.stop()

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
        error = 0 - gyro.angle()
        robot.drive(speed, error)

    # Gradual slowing before stopping
    while abs(robot.distance() - initial_distance) < distance:
        error = 0 - gyro.angle()
        robot.drive(speed * 0.2, error)
        time.sleep(0.01)  # Allow time for motor update

    robot.stop()
    left_wheel.brake()
    right_wheel.brake()

def run1(progression=0):
    if(progression == 0):
        Forward_final(20, 300)
        TurnNew_final(-55)
        Forward_final(375, 300)
        Forward_final(400, -300)
    elif (progression == 1):
        Forward_final(500, -300)
        TurnNew_final(40)
        Forward_final(150, -300)
        wait(100)
        Forward_final(200 , 300)
        TurnNew_final(-55)
        Forward_final(460, 250)
    elif(progression == 2):
        Forward_final(520, 300)
        TurnNew_final(88)
        Forward_final(650, -300)
        TurnNew_final(30)
        Forward_final(140, -300)
        TurnNew_final(-47)
        Forward_final(350, -300)
        TurnNew_final(-45)
        Forward_final(500, -300)
    

def run1_test():
    resetAngles()
    
    Forward_final(30, 300)
    TurnNew(-55)
    Forward_final(350, 300)
    Forward_final(400, -300)
    TurnNew(-160)
    Forward_final(230, -300)
    TurnNew(28)
    Forward_final(200, -300)
    TurnNew(27)
    Forward_final(115, -300)
    Forward_final(30, 300)
    TurnNew(-23)
    Forward_final(200, -200)
    Forward_final(120, 200)
    TurnNew(-92)
    Forward_final(550, -300)
    TurnNew(25)
    Forward_final(275, -300)
    TurnNew(-30)
    Forward_final(150, -300)
    TurnNew(-45)
    Forward_final(700, -500)

def run2():
    a = 50
    Forward_final(350, 250)
    wait(100)
    ext_st.run_angle(10, 10)
    for i in range(0, 5):
        ext_st.run_angle(20, 10)
        Forward_final(3, 20)
    #ext_dr.run_angle(50, 80)
    #ext_st.run_angle(50, 57) 
    #Forward_final(20, 15)
    ext_st.run_angle(30, -20)
    wait(100)
    Forward_final(50, -300)
    TurnNew_final(90)
    ext_st.run_angle(70, 40)
    Forward_final(245, 250)
    TurnNew_final(-90)
    Forward_final(225 , 250)
    TurnNew_final(-82)
    #ext_dr.run_angle(75, -60)
    wait(100)
    Forward_final(200, 100)
    ext_dr.run_angle(10, 35)
    Forward_final(100, -150)
    TurnNew_final(96)
    wait(500)
    ext_dr.run_angle(50, -15)
    Forward_final(150, 250)
    ext_dr.run_angle(50, -15)
    wait(1000)
    Forward_final(1000, -1000)

def run2_test():
    a = 50
    Forward_final(330, 250)
    wait(100)
    ext_st.run_angle(10, 10)
    for i in range(0, 5):
        ext_st.run_angle(20, 10)
        Forward_final(3, 20)
    #ext_dr.run_angle(50, 80)
    #ext_st.run_angle(50, 57) 
    #Forward_final(20, 15)
    ext_st.run_angle(30, -20)
    wait(100)
    Forward_final(50, -300)
    TurnNew(90)
    ext_st.run_angle(70, 40)
    Forward_final(275, 250)
    TurnNew(-90)
    Forward_final(250 , 250)
    TurnNew(-90)
    #ext_dr.run_angle(75, -60)
    wait(100)
    Forward_final(200, 100)
    ext_dr.run_angle(10, 35)
    Forward_final(100, -150)
    TurnNew(100)
    wait(500)
    ext_dr.run_angle(50, -15)
    Forward_final(150, 250)
    ext_dr.run_angle(50, -15)
    wait(1000)
    Forward_final(350, -1000)

def run3():
    ext_dr.run_angle(100, -50)
    Forward_final(225, 1000)
    TurnNew_final(60)
    Forward_final(235, 500)
    TurnNew_final(-57)
    Forward_final(300, 1000)
    ext_dr.run_angle(200, 80)
    ext_dr.run_angle(200, -45)
    wait(100)
    TurnNew_final(-40)
    ext_dr.run_angle(200, -75)
    Forward_final(100, 500)
    TurnNew_final(30)
    Forward_final(100, -250)
    TurnNew_final(20)
    Forward_final(200, -500)
    ext_dr.run_angle(200, 80)
    TurnNew_final(60)
    Forward_final(125, 500)
    ext_dr.run_angle(50 , -47)


def run4():
    TurnNew(90)
    wait(100)
    TurnNew(30)
    TurnNew(-180)
    TurnNew(360)
    
    

def run5():
   Forward_final (250 , 300)

def washing_tires():
    while(True):
        Forward_final(1000, -300)



# Main program loop
ev3.speaker.beep()
resetAngles()
print(gyro.angle())

contor = 0
progression = 0
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
        run1(progression)
        progression += 1
        canSwitch = True
    elif contor == 2 and not canSwitch:
        wait(500)
        run2_test()
        canSwitch = True
    elif contor == 3 and not canSwitch:
        wait(500)
        run3()
        canSwitch = True
    elif contor == 4 and not canSwitch:
        wait(500)
        run4()
        canSwitch = True

    elif contor == 5 and not canSwitch:
        wait(500)
        run5()
        canSwitch = True

    elif contor == 0 and not canSwitch:
        wait(500)
        washing_tires()
        canSwitch = True
