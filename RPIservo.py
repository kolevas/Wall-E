#!/usr/bin/env python3
# File name   : RPiservo.py
# Description : Multi-threaded Control Servos
# Author      : devin
from __future__ import division
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import threading

# When the motor occupies pins 9-15 on the PCA9685, 
# the servo can only use pins 0-8.

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pwm_servo = PCA9685(i2c, address=0x5f) #default 0x40

pwm_servo.frequency = 50

init_pwm0 = 90
init_pwm1 = 90
init_pwm2 = 90
init_pwm3 = 90

init_pwm4 = 90
init_pwm5 = 90
init_pwm6 = 90
init_pwm7 = 90

servo_num = 8

class ServoCtrl(threading.Thread):

    def __init__(self, *args, **kwargs):
        self.i2c = busio.I2C(SCL, SDA)
        self.pwm_servo = PCA9685(self.i2c, address=0x5f) #default 0x40
        self.pwm_servo.frequency = 50

        self.initPos = [init_pwm0, init_pwm1, init_pwm2, init_pwm3,
                        init_pwm4, init_pwm5, init_pwm6, init_pwm7]
        self.goalPos = [90, 90, 90, 90, 90, 90, 90, 90]
        self.nowPos = [90, 90, 90, 90, 90, 90, 90, 90]
        self.bufferPos = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.lastPos = [90, 90, 90, 90, 90, 90, 90, 90]
        self.maxPos = [180, 180, 180, 180, 180, 180, 180, 180]
        self.minPos = [0, 0, 0, 0, 0, 0, 0, 0]

        super(ServoCtrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def set_angle(self, ID, angle):
        servo_angle = servo.Servo(self.pwm_servo.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
        servo_angle.angle = angle

    def moveHand(self, angle):
        # Assuming servo 1 is used for the hand
        self.set_angle(1, angle)
        print(f"Moving hand to angle {angle}")

    def moveArm(self, angle):
        # Assuming servo 0 is used for the arm (swapped with the twisting motor)
        self.set_angle(0, angle)
        print(f"Moving arm to angle {angle}")

    def moveTwist(self, angle):
        # Assuming servo 2 is used for the twisting motor (swapped with the arm motor)
        self.set_angle(2, angle)
        print(f"Twisting to angle {angle}")

    def grabObject(self):
        self.moveArm(25)
        time.sleep(1)

        self.moveHand(180)
        time.sleep(1)  

        # self.moveArm(0)
        # time.sleep(1) 

        self.set_angle(3, 0)
        time.sleep(1) 



    def releaseObject(self):
        # CRVENI
        # self.moveArm(150)
        # time.sleep(1)  # Wait for the arm to move

        # # Move the hand down by 100 degrees
        # self.moveHand(20)
        # time.sleep(1)  # Wait for the hand to move

        # # Open the gripper to release the object
        # self.set_angle(3, 90)
        # time.sleep(1)  # Wait for the gripper to open

        # ZELENI

        self.moveArm(140)
        time.sleep(1)  # Wait for the arm to move

        # Move the hand down by 100 degrees
        self.moveHand(35)
        time.sleep(1)  # Wait for the hand to move

        # Open the gripper to release the object
        self.set_angle(3, 90)
        time.sleep(1)  # Wait for the gripper to open

    def pause(self):
        print('......................pause..........................')
        self.__flag.clear()

    def resume(self):
        print('resume')
        self.__flag.set()

    def moveInit(self):
        self.scMode = 'init'
        for i in range(0, servo_num):
            self.set_angle(i, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def initConfig(self, ID, initInput, moveTo):
        if initInput > self.minPos[ID] and initInput < self.maxPos[ID]:
            self.initPos[ID] = initInput
            if moveTo:
                self.set_angle(ID, self.initPos[ID])
        else:
            print('initPos Value Error.')

    def moveServoInit(self, ID):
        self.scMode = 'init'
        for i in range(0, len(ID)):
            self.set_angle(ID[i], self.initPos[ID[i]])
            self.lastPos[ID[i]] = self.initPos[ID[i]]
            self.nowPos[ID[i]] = self.initPos[ID[i]]
            self.bufferPos[ID[i]] = float(self.initPos[ID[i]])
            self.goalPos[ID[i]] = self.initPos[ID[i]]
        self.pause()

    def posUpdate(self):
        self.goalUpdate = 1
        for i in range(0, servo_num):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def returnServoAngle(self, ID):
        return self.nowPos[ID]

    def speedUpdate(self, IDinput, speedInput):
        for i in range(0, len(IDinput)):
            self.scSpeed[IDinput[i]] = speedInput[i]

    def moveAuto(self):
        for i in range(0, servo_num):
            self.ingGoal[i] = self.goalPos[i]

        for i in range(0, self.scSteps):
            for dc in range(0, servo_num):
                if not self.goalUpdate:
                    self.nowPos[dc] = int(round((self.lastPos[dc] + (((self.goalPos[dc] - self.lastPos[dc]) / self.scSteps) * (i + 1))), 0))
                    self.set_angle(dc, self.nowPos[dc])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    time.sleep(self.scTime / self.scSteps)
                    return 1
            time.sleep((self.scTime / self.scSteps - self.scMoveTime))

        self.posUpdate()
        self.pause()
        return 0

    def moveCert(self):
        for i in range(0, servo_num):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(0, servo_num):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow > self.goalPos[i]: newNow = self.goalPos[i]
                    self.nowPos[i] = newNow
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow < self.goalPos[i]: newNow = self.goalPos[i]
                    self.nowPos[i] = newNow

                if not self.goalUpdate:
                    self.set_angle(i, self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    return 1
            self.posUpdate()
            time.sleep(self.scDelay - self.scMoveTime)

        else:
            self.pause()
            return 0

    def pwmGenOut(self, angleInput):
        return int(round(((self.ctrlRangeMax - self.ctrlRangeMin) / self.angleRange * angleInput), 0))

    def setAutoTime(self, autoSpeedSet):
        self.scTime = autoSpeedSet

    def setDelay(self, delaySet):
        self.scDelay = delaySet

    def autoSpeed(self, ID, angleInput):
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(0, len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i]) * self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]: newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]: newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.goalUpdate = 0
        self.resume()

    def certSpeed(self, ID, angleInput, speedSet):
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(0, len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i]) * self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]: newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]: newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.speedUpdate(ID, speedSet)
        self.goalUpdate = 0
        self.resume()

    def moveWiggle(self):
        self.bufferPos[self.wiggleID] += self.wiggleDirection * self.sc_direction[self.wiggleID] * self.pwmGenOut(self.scSpeed[self.wiggleID]) / (1 / self.scDelay)
        newNow = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]: self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]: self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = newNow
        self.lastPos[self.wiggleID] = newNow
        if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
            self.set_angle(self.wiggleID, self.nowPos[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scDelay - self.scMoveTime)

    def stopWiggle(self):
        self.pause()
        self.posUpdate()

    def singleServo(self, ID, direcInput, speedSet):
        self.wiggleID = ID
        self.wiggleDirection = direcInput
        self.scSpeed[ID] = speedSet
        self.scMode = 'wiggle'
        self.posUpdate()
        self.resume()

    def moveAngle(self, ID, angleInput):
        self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID] * self.pwmGenOut(angleInput))
        if self.nowPos[ID] > self.maxPos[ID]: self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]: self.nowPos[ID] = self.minPos[ID]
        self.lastPos[ID] = self.nowPos[ID]
        self.set_angle(ID, self.nowPos[ID])

    def scMove(self):
        if self.scMode == 'init':
            self.moveInit()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()

    def setPWM(self, ID, PWM_input):
        self.lastPos[ID] = PWM_input
        self.nowPos[ID] = PWM_input
        self.bufferPos[ID] = float(PWM_input)
        self.goalPos[ID] = PWM_input
        self.set_angle(ID, PWM_input)
        self.pause()

    def run(self):
        while 1:
            self.__flag.wait()
            self.scMove()
            pass

def get_distance():
    # Placeholder function to simulate getting distance from ultrasonic sensor
    # Replace this with actual code to get distance from your sensor
    return 7  # Simulating a distance of 7 cm

if __name__ == '__main__':
    scGear = ServoCtrl()
    scGear.moveInit()
    sc = ServoCtrl()
    sc.start()

    # while True:
    #     distance = get_distance()
    #     if distance <= 7:
    #         scGear.grabObject()
    #     else:
    #         scGear.releaseObject()
    #     time.sleep(1)
    
    # scGear.grabObject()
    # scGear.releaseObject()

    scGear.moveHand(90)
