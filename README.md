#  YOLOv8n Object-Collecting Robot

This project runs a **YOLOv8n** (nano) object detection model on a **Raspberry Pi-powered robot** to detect and collect four specific types of objects in real time.

---

## 🔍 Object Detection & Collection

The robot is equipped to:

- Detect and classify:
  - `blue_ball`
  - `green_cube`
  - `red_ball`
  - `red_cube`
- Move toward detected objects
- Check if the object is grabbable based on size
- Collect the object using a motorized grabbing mechanism

---

##  Model Training

The YOLOv8n model is trained on a separate computer and then transferred to the Raspberry Pi for inference.

**Training repository:**  
 [kolevas/Object-detection-model](https://github.com/kolevas/Object-detection-model) (this repository contains more info about the model itself and its performance)

###  Training Command

```bash
yolo detect train \
  model=yolov8n.pt \
  data=/Users/snezhanakoleva/PycharmProject/VNP/dataset/data.yaml \
  epochs=250 \
  imgsz=640 \
  batch=16 \
  name=model_nano \
  patience=20 \
  optimizer=AdamW \
  lr0=0.001 \
  lrf=0.01 \
  degrees=20 \
  scale=0.85 \
  translate=0.2 \
  shear=10 \
  perspective=0.0005 \
  hsv_h=0.015 \
  hsv_s=0.3 \
  hsv_v=0.2 \
  mosaic=1.0 \
  mixup=0.1 \
  copy_paste=0.1 \
  flipud=0.0 \
  fliplr=0.5
```


