import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import numpy as np
import sys # Import sys for sys.exit

# Load your trained YOLO model
model = YOLO("/home/finkirobot/skripti/ADR013-V4.1_RaspTank_SmartCarKit_for_RPi-20250219/Code/adeept_rasptank2/best.pt")

# --- Picamera2 Initialization ---
picam2 = None
try:
    picam2 = Picamera2()

    # Configure the camera for the desired resolution and format.
    # RGB888 is suitable for direct processing by YOLO.
    camera_config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        # hflip=True, # Uncomment and set to True if you need to flip horizontally
        # vflip=True  # Uncomment and set to True if you need to flip vertically
    )
    picam2.configure(camera_config)

    picam2.start()

    print("Picamera2 initialized and started. Processing frames (Ctrl+C to stop).")

    frame_count = 0
    while True:
        frame = picam2.capture_array() # Capture frame as a NumPy array (RGB)

        if frame is None or frame.size == 0:
            print("Failed to grab frame from Picamera2. Exiting.")
            break
        
        # Original flip (only if not handled by hflip/vflip in config)
        # frame = cv2.flip(frame, -1) 

        # Perform inference. IMPORTANT: show=False to prevent display window.
        results = model.predict(source=frame, show=False, conf=0.86, iou=0.8, agnostic_nms=True, verbose=False)

        frame_count += 1
        print(f"\n--- Frame {frame_count} Detections ---")

        # --- Process and print what the model detects ---
        detections_found_in_frame = False
        for r in results: # 'results' is a list of Results objects (one per image/frame)
            boxes = r.boxes # Boxes object for bounding box outputs

            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0]) # Coordinates
                    conf = box.conf[0].item() # Confidence score
                    cls_id = int(box.cls[0].item()) # Class ID
                    label = model.names[cls_id] # Class name

                    print(f"  Detected: {label} (Conf: {conf:.2f}) Box: [{x1}, {y1}, {x2}, {y2}]")
                    detections_found_in_frame = True
            
        if not detections_found_in_frame:
            print("  No objects detected in this frame.")


except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting gracefully.")
except Exception as e:
    print(f"An unexpected error occurred: {e}", file=sys.stderr)

finally:
    # Clean up Picamera2 resources
    if picam2:
        print("Stopping Picamera2.")
        picam2.stop()
    # No cv2.destroyAllWindows() needed as no windows are created