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
    from ultralytics import YOLO
except ImportError:
    print("YOLO library not found. Please install it using 'pip install ultralytics'.")
    YOLO = None

try:
    from greenlet import getcurrent as get_ident
except ImportError:
    try:
        from thread import get_ident
    except ImportError:
        from _thread import get_ident

TRIG_PIN = 23
ECHO_PIN = 24
sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, max_distance=2)

i2c = busio.I2C(SCL, SDA)
try:
    pwm = PCA9685(i2c, address=0x5f)
    pwm.frequency = 50
except ValueError as e:
    print(f"Error initializing PCA9685: {e}")
    exit(1)

MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 12
MOTOR_M2_IN2 = 13

motor1 = motor.DCMotor(pwm.channels[MOTOR_M1_IN1], pwm.channels[MOTOR_M1_IN2])
motor2 = motor.DCMotor(pwm.channels[MOTOR_M2_IN1], pwm.channels[MOTOR_M2_IN2])

try:
    pwm_servo = PCA9685(i2c, address=0x5f)
    pwm_servo.frequency = 50
except ValueError as e:
    print(f"Error initializing PCA9685 for servos: {e}")
    exit(1)

LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

try:
    model = YOLO("/home/finkirobot/skripti/ADR013-V4.1_RaspTank_SmartCarKit_for_RPi-20250219/Code/adeept_rasptank2/best.pt")
    print("YOLO model loaded successfully")
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    model = None

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
                # Convert 4-channel (RGBA) to 3-channel (BGR) if necessary for OpenCV/YOLO
                if frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                return frame
            else:
                print("Camera not initialized.")
                return None
        except Exception as e:
            print(f"Error capturing frame: {e}")
            return None

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
        self.target_class = None
        self.min_confidence = 0.5
        self.min_object_width = 30
        self.min_object_height = 30
        self.max_object_width = 200
        self.max_object_height = 200
        
        self.min_object_distance = 5
        self.max_object_distance = 150
        self.verification_distance = 30
        self.grab_distance = 15
        self.safe_backup_distance = 20


    def get_distance(self):
        try:
            return self.distance_sensor.distance * 100
        except Exception as e:
            print(f"Error getting distance: {e}")
            return None

    def move_forward(self, speed=1.0, duration=0.05):
        try:
            move(speed*100, 1, 'mid')
            time.sleep(duration)
            self.stop()
        except Exception as e:
            print(f"Error moving forward: {e}")

    def move_backward(self, speed=1.0, duration=0.05):
        try:
            move(speed*100, -1, 'mid')
            time.sleep(duration)
            self.stop()
        except Exception as e:
            print(f"Error moving backward: {e}")

    def move_left(self, speed=1.0, duration=0.05):
        try:
            move(speed*100, 1, 'left')
            time.sleep(duration)
            self.stop()
        except Exception as e:
            print(f"Error moving left: {e}")

    def move_right(self, speed=1.0, duration=0.05):
        try:
            move(speed*100, 1, 'right')
            time.sleep(duration)
            self.stop()
        except Exception as e:
            print(f"Error moving right: {e}")

    def stop(self):
        try:
            motorStop()
        except Exception as e:
            print(f"Error stopping: {e}")

    def get_camera_frame(self):
        return self.camera.get_frame()

    def move_arm(self, angle):
        try:
            print(f"Moving arm to angle {angle}")
            self.servo_ctrl.moveArm(angle)
            time.sleep(0.5)
        except Exception as e:
            print(f"Error moving arm: {e}")

    def move_hand(self, angle):
        try:
            print(f"Moving hand to angle {angle}")
            self.servo_ctrl.moveHand(angle)
            time.sleep(0.5)
        except Exception as e:
            print(f"Error moving hand: {e}")

    def move_gripper(self, angle):
        try:
            print(f"Moving gripper to angle {angle}")
            self.servo_ctrl.set_angle(3, angle)
            time.sleep(0.5)
        except Exception as e:
            print(f"Error moving gripper: {e}")

    def grab_object(self):
        try:
            print("Grabbing object")
            self.servo_ctrl.grabObject()
            time.sleep(2)
            print("Object grabbed successfully")
        except Exception as e:
            print(f"Error grabbing object: {e}")

    def release_object(self):
        try:
            print("Releasing object")
            self.servo_ctrl.releaseObject()
            time.sleep(2)
            print("Object released successfully")
        except Exception as e:
            print(f"Error releasing object: {e}")

    def distRedress(self, samples=3):
        valid_readings = []
        for i in range(samples * 2):
            distance = self.get_distance()
            if distance < 300:
                valid_readings.append(distance)
            time.sleep(0.1)
            if len(valid_readings) >= samples:
                break

        if valid_readings:
            valid_readings.sort()
            median_distance = valid_readings [len(valid_readings) // 2]
            print (f"Distance: {median_distance:.2f}cm (from {len(valid_readings)} samples)")
            return median_distance
        else:
            print("No valid distance readings, returning max distance")
            return 300.0

    def calculate_object_size(self, object_px, distance_cm, fov_degrees=53.5, image_width_px=640):
        fov_radians = math.radians(fov_degrees / 2)
        fov_width_cm = 2 * distance_cm * math.tan(fov_radians)
        real_object_width_cm = (object_px / image_width_px) * fov_width_cm
        return real_object_width_cm

    def is_object_suitable_size(self, width_px, height_px, distance):
        real_width = self.calculate_object_size(width_px, distance)
        real_height = self.calculate_object_size(height_px, distance)

        min_size, max_size = 3.0, 15.0
        suitable = (min_size <= real_width <= max_size and
                    min_size <= real_height <= max_size)

        print(f"Object real size: {real_width:.1f}cm x {real_height:.1f}cm, Suitable: {suitable}")
        return suitable

    def detect_objects_color(self, frame):
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

            mask_red1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
            mask_red2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            objects = []

            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_green:
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w >= self.min_object_width and h >= self.min_object_height:
                        objects.append(("green_cube", (x + w // 2, y + h // 2, w, h), 1.0))

            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_red:
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w >= self.min_object_width and h >= self.min_object_height:
                        objects.append(("red_cube", (x + w // 2, y + h // 2, w, h), 1.0))
            return objects
        except Exception as e:
            print(f"Error in color detection: {e}")
            return []

    def detect_objects_yolo(self, frame):
        try:
            if model is None:
                print("YOLO model not loaded, falling back to color detection")
                return self.detect_objects_color(frame)
                
            results = model.predict(source=frame, show=False, conf=self.min_confidence,
                                  iou=0.8, agnostic_nms=True, verbose=False)
            objects = []
            
            for r in results:
                boxes = r.boxes
                if boxes is not None:
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = box.conf[0].item()
                        cls_id = int(box.cls[0].item())
                        label = model.names[cls_id]
                        
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        width = x2 - x1
                        height = y2 - y1
                        
                        if (width >= self.min_object_width and
                            height >= self.min_object_height):
                            objects.append((label, (center_x, center_y, width, height), conf))
                            print(f"Detected {label} at ({center_x}, {center_y}) "
                                  f"size: {width}x{height}px, confidence: {conf:.2f}")
            return objects
            
        except Exception as e:
            print(f"Error in YOLO detection: {e}")
            return self.detect_objects_color(frame)

    def scan_for_target_object(self):
        print("Scanning for initial target object with YOLO...")
        for attempt in range(5):
            frame = self.get_camera_frame()
            if frame is not None:
                objects = self.detect_objects_yolo(frame)
                if objects:
                    best_obj = max(objects, key=lambda item: item[2])
                    self.target_class = best_obj[0]
                    print(f"Initial target object set: {self.target_class} (from YOLO detection)")
                    return True
            time.sleep(0.5)

        print("No object found in current view, performing 360-degree YOLO scan...")
        for angle in range(0, 360, 45):
            turn_to_side(angle_size=45, direction='right')
            time.sleep(1)

            for attempt in range(3):
                frame = self.get_camera_frame()
                if frame is not None:
                    objects = self.detect_objects_yolo(frame)
                    if objects:
                        best_obj = max(objects, key=lambda item: item[2])
                        self.target_class = best_obj[0]
                        print(f"Target object '{self.target_class}' found during YOLO scan.")
                        return True
                time.sleep(0.3)
        print("No target object found during complete YOLO scan.")
        return False

    def verify_target_object(self, frame, distance=None):
        if self.target_class is None:
            return False

        objects = self.detect_objects_yolo(frame)
        for obj_class, (center_x, center_y, width, height), conf in objects:
            if obj_class == self.target_class:
                print(f"Target object {obj_class} detected with confidence {conf:.2f}")
                if distance and self.is_object_suitable_size(width, height, distance):
                    print("Object size is suitable for grabbing!")
                    return True
                elif distance:
                    print("Object detected but size not suitable for grabbing")
                    return False
                else:
                    print("Target object confirmed (size check skipped no distance)")
                    return True
        print(f"No matching target object found")
        return False

    def center_on_object(self, frame):
        objects = self.detect_objects_yolo(frame)
        for obj_class, (center_x, center_y, width, height), conf in objects:
            if obj_class == self.target_class:
                frame_center_x = frame.shape[1] // 2
                error_x = center_x - frame_center_x
                print(f"Object center: {center_x}, Frame center: {frame_center_x}, Error: {error_x}")

                if abs(error_x) > 50:
                    if error_x > 0:
                        print("Adjusting right to center object")
                        turn_to_side(angle_size=5, direction='right')
                    else:
                        print("Adjusting left to center object")
                        turn_to_side(angle_size=5, direction='left')
                    return False
                else:
                    print("Object is centered!")
                    return True
        return False

    def approach_object_carefully(self, target_distance=15):
        print(f"Approaching object to {target_distance}cm...")
        
        for attempt in range(10):
            current_distance = self.distRedress()
            
            if abs(current_distance - target_distance) <= 2:
                print(f"Successfully reached target distance: {current_distance}cm")
                return True
            
            if current_distance > target_distance:
                move_distance = min(current_distance - target_distance, 10)
                print(f"Moving forward {move_distance}cm (current: {current_distance}cm)")
                move_forward_distance(100, move_distance)
            else:
                move_distance = min(target_distance - current_distance, 5)
                print(f"Moving backward {move_distance}cm (current: {current_distance}cm)")
                move_backward_distance(100, move_distance)
            
            time.sleep(0.5)
        
        print(f"Could not reach exact target distance, current: {self.distRedress()}cm")
        return False

    def automatic_processing(self):
        print('Starting enhanced automatic processing with YOLO...')

        if not self.scan_for_target_object():
            print("Failed to find target object. Exiting.")
            return

        print(f"\nTarget object '{self.target_class}' detected!")
        print("Type 'ok' and press Enter to start the search:")
        while True:
            user_input = input().strip().lower()
            if user_input == 'ok':
                print("Starting search for target object...")
                break
            else:
                print("Please type 'ok' to continue:")
        
        search_attempts = 0
        max_search_attempts = 100

        try:
            while search_attempts < max_search_attempts:
                search_attempts += 1
                print(f"\n--- Search attempt {search_attempts} ---")

                self.stop()
                dist = self.distRedress()
                
                print(f"Distance: {dist}cm")

                if dist < self.min_object_distance:
                    print("Too close to obstacle, backing up...")
                    move_backward_distance(100, self.safe_backup_distance)
                    continue

                if self.min_object_distance < dist <= self.max_object_distance:
                    print(f"Potential object detected at {dist}cm")
                    self.object_detected = True

                    if dist > self.verification_distance:
                        approach_distance = dist - self.verification_distance
                        print(f"Moving {approach_distance}cm closer for YOLO verification")
                        move_forward_distance(100, approach_distance)
                        time.sleep(1)
                    elif dist < self.verification_distance:
                        backup_distance = self.verification_distance - dist
                        print(f"Backing up {backup_distance}cm for optimal YOLO distance")
                        move_backward_distance(100, backup_distance)
                        time.sleep(1)

                    actual_dist = self.distRedress()
                    print(f"Distance after positioning: {actual_dist}cm")

                    print("Using YOLO for object verification...")
                    frame = self.get_camera_frame()
                    if frame is not None and self.verify_target_object(frame, actual_dist): # Changed if frame to if frame is not None
                        print("✓ Target object confirmed by YOLO!")

                        centering_attempts = 0
                        while centering_attempts < 5:
                            frame = self.get_camera_frame()
                            if frame is not None and self.center_on_object(frame): # Changed if frame to if frame is not None
                                break
                            centering_attempts += 1
                            time.sleep(0.5)

                        if self.approach_object_carefully(self.grab_distance):
                            print(" 🎯 Object within grabbing distance!")
                            self.stop()

                            print("Executing grab sequence...")
                            self.grab_object()
                            time.sleep(2)

                            move_backward_distance(100, 10)
                            self.release_object()
                            print(" 🎉 Mission completed! Target object collected and released.")
                            return True
                        else:
                            print("Failed to approach object to grabbing distance")
                    else:
                        print(" ❌ YOLO verification failed - not the target object")
                        move_backward_distance(100, 30)
                        self.object_detected = False

                if not self.object_detected or dist > self.max_object_distance:
                    self.stop()
                    if dist >= 50:
                        print("Clear path - moving forward")
                        move_forward_distance(100, 30)
                    elif 20 <= dist < 50:
                        print("Obstacle ahead - scanning for best path...")
                        
                        turn_to_side(angle_size=15, direction='left')
                        time.sleep(0.5)
                        dist_left = self.distRedress()

                        turn_to_side(angle_size=30, direction='right')
                        time.sleep(0.5)
                        dist_right = self.distRedress()

                        turn_to_side(angle_size=15, direction='left')
                        if dist_left < 30 and dist_right < 30:
                            print("No clear path - backing up")
                            move_backward_distance(100, 20)
                        elif dist_left >= dist_right:
                            print("Turning left for better path")
                            turn_to_side(angle_size=20, direction='left')
                        else:
                            print("Turning right for better path")
                            turn_to_side(angle_size=20, direction='right')
                        time.sleep(0.5)

        except KeyboardInterrupt:
            print("Automatic processing interrupted by user.")
        except Exception as e:
            print(f"An unexpected error occurred during automatic processing: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()
            self.servo_ctrl.moveInit()

if __name__ == "__main__":
    setup()
    robot = Robot()
    try:
        robot.automatic_processing()
    except KeyboardInterrupt:
        print("Shutting down robot...")
    finally:
        robot.stop()
        cv2.destroyAllWindows()
