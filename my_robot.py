import os
import cv2
import numpy as np
import threading
import time
import math
import datetime
import imutils
import picamera2
import libcamera
from picamera2 import Picamera2, Preview
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from gpiozero import DistanceSensor
from time import sleep
from move import move, motorStop, setup, move_forward_distance, turn_to_side, move_backward_distance
from RPIservo import ServoCtrl

try:
    from greenlet import getcurrent as get_ident
except ImportError:
    try:
        from thread import get_ident
    except ImportError:
        from _thread import get_ident

# Initialize Distance Sensor
TRIG_PIN = 23
ECHO_PIN = 24
sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, max_distance=2)  # Maximum detection distance 2m.

# Initialize PCA9685 for motor control
i2c = busio.I2C(SCL, SDA)
try:
    pwm = PCA9685(i2c, address=0x5f)
    pwm.frequency = 50
except ValueError as e:
    print(f"Error initializing PCA9685: {e}")
    exit(1)

# Motor pins
MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13

# Initialize motors
motor1 = motor.DCMotor(pwm.channels[MOTOR_M1_IN1], pwm.channels[MOTOR_M1_IN2])
motor2 = motor.DCMotor(pwm.channels[MOTOR_M2_IN1], pwm.channels[MOTOR_M2_IN2])

# Initialize PCA9685 for servo control
try:
    pwm_servo = PCA9685(i2c, address=0x5f)
    pwm_servo.frequency = 50
except ValueError as e:
    print(f"Error initializing PCA9685 for servos: {e}")
    exit(1)

# Servo initial positions
init_pwm0 = 90
init_pwm1 = 90
init_pwm2 = 90
init_pwm3 = 90
init_pwm4 = 90
init_pwm5 = 90
init_pwm6 = 90
init_pwm7 = 90

servo_num = 8

# Adjusted color ranges for red and green cuboids
LOWER_GREEN = np.array([35, 100, 100])  # Adjusted HSV range for green
UPPER_GREEN = np.array([85, 255, 255])
LOWER_RED_1 = np.array([0, 120, 70])  # Red has two ranges in HSV
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

# Camera class
class Camera:
    def __init__(self):
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640,480)}))
            self.picam2.start()
        except Exception as e:
            print(f"Error initializing camera: {e}")
            self.picam2 = None

    def get_frame(self):
        try:
            if self.picam2:
                frame = self.picam2.capture_array()
                frame = cv2.flip(frame, -1)
                return frame
            else:
                print("Camera not initialized.")
                return None
        except Exception as e:
            print(f"Error capturing frame: {e}")
            return None

# Robot class
class Robot:
    def __init__(self):
        self.camera = Camera()
        self.distance_sensor = sensor
        self.motor1 = motor1
        self.motor2 = motor2
        self.servo_ctrl = ServoCtrl()
        self.servo_ctrl.moveInit()
        self.object_detected = False
        self.object_centered = False
        self.scanList = [0, 0]

    def get_distance(self):
        try:
            return self.distance_sensor.distance * 100  # Convert to cm
        except Exception as e:
            print(f"Error getting distance: {e}")
            return None

    def move_forward(self, speed=1.0):
        try:
            move(speed*100, 1, 'mid')
            time.sleep(0.3)
            # print(f"Moving forward with speed {speed}")
            # self.motor1.throttle = speed
            # self.motor2.throttle = speed
        except Exception as e:
            print(f"Error moving forward: {e}")

    def move_backward(self, speed=1.0):
        try:
            move(speed*100, -1, 'mid')
            time.sleep(0.1)
            # print(f"Moving backward with speed {speed}")
            # self.motor1.throttle = -speed
            # self.motor2.throttle = -speed
        except Exception as e:
            print(f"Error moving backward: {e}")

    def move_left(self, speed=1.0):
        try:
            move(speed*100, 1, 'left')
            time.sleep(0.5)
            # print(f"Moving left with speed {speed}")
            # self.motor1.throttle = -speed
            # self.motor2.throttle = speed
        except Exception as e:
            print(f"Error moving left: {e}")

    def move_right(self, speed=1.0):
        try:
            move(speed*100, 1, 'right')
            time.sleep(0.4)
            # print(f"Moving right with speed {speed}")
            # self.motor1.throttle = speed
            # self.motor2.throttle = -speed
        except Exception as e:
            print(f"Error moving right: {e}")

    def stop(self):
        try:
            print("Stopping motors")
            self.motor1.throttle = 0
            self.motor2.throttle = 0
        except Exception as e:
            print(f"Error stopping: {e}")

    def get_camera_frame(self):
        return self.camera.get_frame()

    def move_arm(self, angle):
        """ Move the arm servo to the specified angle (0 to 180 degrees). """
        try:
            print(f"Moving arm to angle {angle}")
            self.servo_ctrl.moveArm(angle)
        except Exception as e:
            print(f"Error moving arm: {e}")

    def move_hand(self, angle):
        """ Move the hand servo to the specified angle (0 to 180 degrees). """
        try:
            print(f"Moving hand to angle {angle}")
            self.servo_ctrl.moveHand(angle)
        except Exception as e:
            print(f"Error moving hand: {e}")

    def move_gripper(self, angle):
        """ Move the gripper servo to the specified angle (0 to 180 degrees). """
        try:
            print(f"Moving gripper to angle {angle}")
            self.servo_ctrl.set_angle(3, angle)
        except Exception as e:
            print(f"Error moving gripper: {e}")

    def grab_object(self):
        try:
            print("Grabbing object")
            self.servo_ctrl.grabObject()
        except Exception as e:
            print(f"Error grabbing object: {e}")

    def release_object(self):
        """ Release an object using the arm, hand, and gripper servos. """
        try:
            print("Releasing object")
            self.servo_ctrl.releaseObject()
        except Exception as e:
            print(f"Error releasing object: {e}")

    def distRedress(self): 
        """Improved distance reading function with proper handling of invalid readings."""
        mark = 0
        distValue = self.distance_sensor.distance * 100
        
        while mark < 3:
            distValue = self.distance_sensor.distance * 100
            if distValue > 900:
                mark += 1
            else:
                break
            print(distValue)
        
        return round(distValue, 2)

    def calculate_object_size(self, object_px, distance_cm, fov_degrees=53.5, image_width_px=640):
        fov_radians = math.radians(fov_degrees / 2)
        fov_width_m = 2 * distance_cm * math.tan(fov_radians) #distance is in cm
        real_object_width_m = (object_px / image_width_px) * fov_width_m
        return real_object_width_m
    
    def detect_objects(self, frame):
        """ Detect red and green cuboids in the frame. """
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
            cv2.imwrite('mask_green.png', mask_green)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            objects = []

            for contour in contours_green:
                area = cv2.contourArea(contour)
                if area > 500:  
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h 
                    if w > 20 and h > 20:  
                        objects.append(("green", (x + w // 2, y + h // 2, w//1, h//1)))
                        print(f"Detected objects: {objects}")
                        return objects if objects else []
        except Exception as e:
                        print(f"Error detecting objects: {e}")
                        return []

    def get_frame_color(self, frame):
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Presmetka na prosečna boja (BGR)
        B, G, R = frame_bgr.mean(axis=0).mean(axis=0).astype(int)

        # Proverka koja boja dominira
        if G > B + 40 and G > R + 40:
            return "Green"
        return "Unknown"

    
    def automatic_processing(self, object_was_detected=False):
        self.object_detected = False
        self.object_centered=False
        print('Starting automatic processing...')
        large_objects = []
        objects = []
        try:
            while True:
                frame = self.get_camera_frame()
                if frame is not None:
                    objects = self.detect_objects(frame)  # Detect objects
                    if objects:
                        color, (x, y, w, h) = objects[0]
                        self.object_detected = True
                        print("DETECTED OBJECT")
                    else:
                        print("NO DETECTED OBJECT")
                        self.object_detected = False

                self.stop()
                dist = self.distRedress()
                if dist == 200.0:
                    dist = 0
                if dist < 5:
                    print("Object too close to detect properly, adjusting...")
                    move_backward_distance(100, 5)
                    time.sleep(0.5)
                    continue  # Retry detection after adjusting

                print(f"Distance: {dist} cm")
                time.sleep(0.1)  # Reduced delay for more frequent checks

                if self.object_detected:
                    frame = self.get_camera_frame()
                    objects = self.detect_objects(frame)
                    if objects:
                        color, (x, y, w, h) = objects[0]
                        object_size = self.calculate_object_size(object_px=w, distance_cm=dist / 100)
                        if object_size > 9:
                            self.object_detected = False
                            large_objects.append(objects[0])
                            print("Cannot grab object")
                        else:
                            if not self.object_centered:
                                for _ in range(3):  # Re-check positioning multiple times
                                    frame = self.get_camera_frame()
                                    if frame is not None:
                                        objects = self.detect_objects(frame)
                                        if objects:
                                            color, center = objects[0]
                                            x, y, w, h = center
                                            print(f"Detected {color} object at {center}")
                                            if 260 <= x <= 310:
                                                print(f"Object centered at {center}")
                                                self.object_centered = True
                                                break
                                            elif x < 260:
                                                print("Object to the left, adjusting left")
                                                turn_to_side(angle_size=2, direction='left')
                                            else:
                                                print("Object to the right, adjusting right")
                                                turn_to_side(angle_size=2, direction='right')
                                            self.stop()
                                            time.sleep(0.2)

                            if self.object_centered:
                                turn_to_side(angle_size=0.35, direction='left')
                                print("CENTERED")
                                frame = self.get_camera_frame()
                                dist = self.distRedress()
                                # if dist == 200.0:
                                #     dist = 0
                                temp_dist = dist
                                if frame is not None:
                                    objects = self.detect_objects(frame)
                                    if objects:
                                        dist = self.distRedress()
                                        if dist == 200.0:
                                            dist = 5
                                        if dist < 6:
                                            print("Too close to detect, adjusting...")
                                            move_backward_distance(100, 6)
                                            time.sleep(0.5)
                                            continue

                                        print(f"Distance to object: {dist} cm")
                                        
                                        move_forward_distance(100, max(dist - 15,0)) 
                                        print("--------------------------------------------------------CHECKPOINT 1")
                                        frame = self.get_camera_frame()
                                        dist = self.distRedress()
                                        print(dist)
                                        # if dist == 200.0:
                                        #     dist = 5
                                        # temp_dist = dist

                                        if frame is not None:
                                            objects = self.detect_objects(frame)
                                            
                                            # Wait for a short time to ensure it's not a temporary detection issue
                                            if not objects:
                                                time.sleep(0.3)  # Small delay before second check
                                                frame = self.get_camera_frame()
                                                objects = self.detect_objects(frame)
                                            
                                            if not objects:  
                                                print("Object lost! Moving backward to reposition.")
                                                move_backward_distance(100, 15)  # Move back a smaller distance first
                                                time.sleep(0.5)

                                                # Try detecting again after moving back
                                                frame = self.get_camera_frame()
                                                objects = self.detect_objects(frame)
                                                
                                                if not objects:  # If still not detected, move back more
                                                    print("Still lost! Moving back further.")
                                                    move_backward_distance(100, 15) 
                                        dist = self.distRedress()          
                                        print(dist)
                                        while dist>20:
                                            move_forward_distance(100,5)
                                            dist=self.distRedress()

                                        print("Object within grabbing distance.")
                                        self.stop()
                                        # frame = self.get_camera_frame()
                                        # detected_color = self.get_frame_color(frame)
                                        # if detected_color == "Green":
                                        self.grab_object()
                                        time.sleep(2)
                                        self.release_object()
                                        move_backward_distance(100,30)
                                        time.sleep(2)
                                        self.object_detected = False
                                        self.object_centered = False
                                        frame = self.get_camera_frame()
                                        objects = self.detect_objects(frame)
                                        if not objects == []:
                                            self.object_detected = True
                                            continue
                                        self.stop()
                                        dist = self.distRedress()
                                        if dist == 200.0:
                                            dist = 0
                                        time.sleep(0.1)
                                    else:
                                        move_backward_distance(100, 20)
                                        self.object_centered = False

                if not self.object_detected:
                    self.stop()
                    dist = self.distRedress()
                    if dist == 200.0:
                        dist = 0
                    if dist < 5:
                        print("Too close, moving back...")
                        move_backward_distance(100, 5)
                        time.sleep(0.5)
                        continue

                    if dist >= 40:
                        move_forward_distance(speed=100, distance=40)
                        print("Moving forward")
                    elif 20 <= dist < 40:
                        print("Scanning for better path...")
                        turn_to_side(angle_size=10, direction='left')
                        time.sleep(0.2)
                        self.stop()
                        distLeft = self.distRedress()

                        turn_to_side(angle_size=20, direction='right')
                        time.sleep(0.4)
                        self.stop()
                        distRight = self.distRedress()

                        if distLeft <40 and distRight < 40:
                            move_backward_distance(100, 10)
                            continue

                        if distLeft >= distRight:
                            turn_to_side(angle_size=20, direction='left')
                            print("Turning left")
                        else:
                            print("Turning right")
                        time.sleep(0.5)
                    else:
                        self.move_backward(1)
                        print("Moving backward")
        except Exception as e:
            print(f"Error in automatic processing: {e}")




            




# Main function to test the robot
if __name__ == "__main__":
    setup()  # Initialize motors
    robot = Robot()
    try:
        # move_forward_distance(100, 10)
        robot.automatic_processing()
    except KeyboardInterrupt:
        print("Shutting down robot...")
    finally:
        robot.stop()
        cv2.destroyAllWindows()

