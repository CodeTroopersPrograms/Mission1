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
ext_dr = Motor(Port.A)
ext_st = Motor(Port.D)
robot = DriveBase(left_wheel, right_wheel, wheel_diameter = 50, axle_track = 110)
gyro = GyroSensor(Port.S3)
color1 = ColorSensor(Port.S1)
color2 = ColorSensor(Port.S2)


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
        if abs(error) < 1.75:
            robot.stop()
            break
        
        # Ajusteaza viteza
        turn_speed = error * 3 
        robot.drive(0, turn_speed)  # Ajusteaza rotirea bazat pe eroare

    gyro.reset_angle(0)
    
def Forward(distance, speed):
    while abs(robot.distance()) <= abs(distance):
        #corectie = 0 - gyro.angle() # de testat daca functioneaza
        robot.drive(speed, 0)
    robot.stop()
    left_wheel.brake()
    right_wheel.brake()
    robot.reset()           #reset sa nu fie probleme (oare chiar e nevoie?)
    gyro.reset_angle(0)


# Write your program here.
ev3.speaker.beep()
gyro.reset_angle(0)
robot.reset()

t1 = threading.Thread(target = Forward, args = (100, 150))            #threading dar nu prea merge si oricum nu ne trebuie :)
t2 = threading.Thread(target = ext_st.run_angle, args = (500, 360))


Forward(100,300)
Turn(-55)
Forward(350, 300)
wait(500)
Forward(150,-300)
Turn(43)
ext_dr.run_angle(500,320)
Forward(350,250)
Turn(50)
Forward(60, 300)
ext_dr.run_angle(500,-320)
Forward(60, -320)
Turn(71)
robot.straight(45)
ext_st.run_angle(1000, 320)
