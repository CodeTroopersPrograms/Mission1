#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import threading


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B)
ext_dr = Motor(Port.D)
ext_st = Motor(Port.A)
robot = DriveBase(left_wheel, right_wheel, wheel_diameter = 50, axle_track = 110)
gyro = GyroSensor(Port.S3)
color1 = ColorSensor(Port.S1)
color2 = ColorSensor(Port.S2)

buttonDown = Button.DOWN
buttonUp = Button.UP
buttonCenter = Button.CENTER



#Settings

robot.settings(550, 550, 180, 180)



#Functions

def get_filtered_angle(gyro, samples = 3):
    angles = []
    for _ in range(samples):
        angles.append(gyro.angle())
    return sum(angles) / len(angles)



def Turn(degrees):
    gyro.reset_angle(0)

    while True:
        # Obtine unghiul filtrat
        filter_angle = get_filtered_angle(gyro)
        
        # Calculeaza eroarea
        error = filter_angle - degrees
        
        # Opreste cand ajunge la unghiul dorit
        if abs(error) < 2:
            robot.stop()
            break
        
        # Ajusteaza viteza
        turn_speed = error * 3
        robot.drive(0, turn_speed)  # Ajusteaza rotirea bazat pe eroare

    gyro.reset_angle(0)
    
def Forward(distance, speed):
    while abs(robot.distance()) <= distance:
        robot.drive(speed, 0)

    robot.stop()
    left_wheel.brake()
    right_wheel.brake()
    robot.reset()
    gyro.reset_angle(0)


def run1():
    ext_st.hold()
    Forward(100,300)
    Turn(-55)
    Forward(350, 300)
    wait(500)
    Forward(150,-300)
    Turn(45)
    ext_dr.run_angle(500, -320)
    Forward(350,250)
    Turn(50)
    Forward(80, 300)
    ext_dr.run_angle(500, 320)
    Forward(45, -300)
    Turn(140) 
    Forward(300, -200)
    Forward(125, 300)
    Turn(85)
    Forward(650, 300)
    Turn(20)
    Forward(200, 300)

    ext_dr.run_angle(500, -320)
    Turn(-20)
    Forward(180, 300)
    Turn(-50)
    Forward(50, 300)
    Turn(30)



def run2():
    Forward(200, 300)

# Write your program here.
ev3.speaker.beep()
gyro.reset_angle(0)
robot.reset()

#t1 = threading.Thread(target = Forward, args = (100, 150))
#t2 = threading.Thread(target = ext_st.run_angle, args = (500, 360))

contor = 0
canSwitch = True


while True:
    if canSwitch:
        for i in ev3.buttons.pressed():
            if i == buttonUp:
                contor += 1
                ev3.screen.print(contor)
                wait(500)
                break
            elif i == buttonDown and contor > 0:
                contor -= 1
                ev3.screen.print(contor)
                wait(500)
                break    
            elif i == buttonCenter:
                canSwitch = False
                break    

    if contor == 1 and canSwitch == False:
        wait(500)
        run1()    
        canSwitch = True
    elif contor == 2 and canSwitch == False:
        wait(500)
        run2()
        canSwitch = True                
