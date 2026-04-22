"""
game_moveArm.py
- set distance for the motor arm to move 
- move meArm down to allow the uptake of water from the syringe
- testing
-- run multiple y direction configs to see the best placement the arm can reach syringe
-- run multiple x direction configs to see the min distance before it hits 2nd meArm
- *figure out how to connect the syringe with code*
"""
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time

import os
import json
import logging
from typing import Tuple, Optional

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

import math
import kinematics

#constant variables
arm_configs=[0,0,0] #cm
x_minArmDistance=0 #Distance before the arm hits the other arm
position_arm=[0,0,0] #Change based on current point [base, elbow, wrist] --> gripper don't changex

#Initalize the I2C address 
i2c = board.I2C()
pca = PCA9685(i2c, address = 0x6F, reference_clock_speed = 25000000)
pca.frequency = 50

joint_limits: Dict[str, Dict[str, float]] = {
    'base':     {'min_deg':  -90.0,  'max_deg':  90.0},
    'shoulder': {'min_deg':    0.0,  'max_deg': 150.0},
    'elbow':    {'min_deg': -180.0,  'max_deg':  30.0},
    'gripper':  {'min_deg':    0.0,  'max_deg': 125.0},
}
def run_test():
    for joint in joint_limits:

if()



if __name__ == "__main__":
    main()
