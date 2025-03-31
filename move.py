#!/usr/bin/env python3
# coding=utf-8
# File name   : move.py
# Description : Control Motor
# Website     : www.adeept.com
# Author      : Devin
# Date        : 2024/03/10
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

MOTOR_M1_IN1 = 15  # Define the positive pole of M1
MOTOR_M1_IN2 = 14  # Define the negative pole of M1
MOTOR_M2_IN1 = 12  # Define the positive pole of M2
MOTOR_M2_IN2 = 13  # Define the negative pole of M2

M1_Direction = 1
M2_Direction = 1

FREQ = 50

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

def setup():  # Motor initialization
    global motor1, motor2, pwm_motor
    i2c = busio.I2C(SCL, SDA)
    pwm_motor = PCA9685(i2c, address=0x5f)  # default 0x40
    pwm_motor.frequency = FREQ

    motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1], pwm_motor.channels[MOTOR_M1_IN2])
    motor1.decay_mode = motor.SLOW_DECAY
    motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1], pwm_motor.channels[MOTOR_M2_IN2])
    motor2.decay_mode = motor.SLOW_DECAY

def motorStop():  # Motor stops
    global motor1, motor2
    motor1.throttle = 0
    motor2.throttle = 0

def Motor(channel, direction, motor_speed):
    if motor_speed > 100:
        motor_speed = 100
    elif motor_speed < 0:
        motor_speed = 0

    speed = map(motor_speed, 0, 100, 0, 1.0)

    if direction == 1:
        speed = -speed

    if channel == 1:
        motor1.throttle = speed
    elif channel == 2:
        motor2.throttle = speed

def move(speed, direction, turn='mid'):  # 0 < radius <= 1
    # direction = -direction
    if speed == 0:
        motorStop()  # all motor stop.
    else:
        if direction == 1:  # forward
            if turn == 'left':  # left forward
                Motor(1, M1_Direction, speed)
                Motor(2, -M2_Direction, speed)
            elif turn == 'right':  # right forward
                Motor(1, -M1_Direction, speed)
                Motor(2, M2_Direction, speed)
            else:  # forward  (mid)
                Motor(1, -M1_Direction, speed)
                Motor(2, -M2_Direction, speed)
        elif direction == -1:  # backward
            Motor(1, M1_Direction, speed)
            Motor(2, M2_Direction, speed)

def move_forward_distance(speed=1.0, distance=10):
    speed_factor = 30 
    time_to_move = distance / (speed_factor * (speed / 100.0))
    move(speed, 1, 'mid')
    time.sleep(time_to_move)
    motorStop()

def move_backward_distance(speed=1.0, distance=10):
    speed_factor = 30 
    time_to_move = distance / (speed_factor * (speed / 100.0))
    move(speed, -1, 'mid')
    time.sleep(time_to_move)
    motorStop()


    
def turn_to_side(angle_size, direction):
    # se vrti 45 stepeni/sekunda koga e so brzina 100
    time_to_move = angle_size/45
    
    if direction == 'left':
        move(100,1,'left')
        time.sleep(time_to_move)
    else:
        move(100,1,'right')
        time.sleep(time_to_move)

    return 0

def destroy():
    motorStop()
    pwm_motor.deinit()

if __name__ == '__main__':
    try:
        speed_set = 100
        setup()
        # turn_to_side(17.5,'left')
        # time.sleep(1)
        # motorStop()
        # time.sleep(1)
        # move(speed_set, 1,'right')
        # time.sleep(2)
        # motorStop()
        move_forward_distance(speed_set, 10)
    except KeyboardInterrupt:
        destroy()

