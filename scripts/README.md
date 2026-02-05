# Scripts Directory

Utility scripts for data collection, model training, and system testing.

## Structure
```
scripts/
├── data_collection/
│   ├── collect_rosbag.py          Record demonstration data
│   ├── extract_images.py          Extract images from rosbag
│   └── label_traffic_signs.py     Annotate traffic signs
├── training/
│   ├── train_detector.py          Train traffic sign detector
│   ├── evaluate_detector.py       Evaluate model performance
│   └── export_model.py            Export to deployment format
├── testing/
│   ├── test_detection.py          Test traffic sign detection
│   ├── test_fsm.py                Test FSM state transitions
│   └── benchmark_performance.py   System performance metrics
└── utils/
    ├── visualize_detections.py    Visualize detection results
    ├── calibrate_camera.py        Camera calibration utility
    └── analyze_logs.py            Parse and analyze log files
```

## Data Collection Scripts

### collect_rosbag.py
Record synchronized sensor data for training and evaluation.

**Usage:**
```bash
python scripts/data_collection/collect_rosbag.py \
  --output data/demo1.bag \
  --duration 300 \
  --topics /camera/image_raw /scan /cmd_vel
```

**Features:**
- Records camera, LiDAR, IMU, and control data
- Configurable duration and topics
- Automatic timestamp synchronization

### extract_images.py
Extract images from rosbag for dataset creation.

**Usage:**
```bash
python scripts/data_collection/extract_images.py \
  --input data/demo1.bag \
  --output data/images/ \
  --topic /camera/image_raw \
  --rate 5
```

**Features:**
- Extracts at specified frame rate
- Saves with timestamps as filenames
- Supports PNG and JPG formats

### label_traffic_signs.py
Interactive tool for annotating traffic signs in images.

**Usage:**
```bash
python scripts/data_collection/label_traffic_signs.py \
  --input data/images/ \
  --output data/annotations.json
```

**Features:**
- GUI for bounding box annotation
- 12 traffic sign classes
- COCO format output

---

## Training Scripts

### train_detector.py
Train traffic sign detection model (YOLO/CNN).

**Usage:**
```bash
python scripts/training/train_detector.py \
  --data data/annotations.json \
  --model yolo \
  --epochs 100 \
  --batch-size 16 \
  --output models/traffic_detector.pth
```

**Arguments:**
```
--data          Path to annotation file
--model         Model architecture (yolo, ssd, custom)
--epochs        Training epochs
--batch-size    Batch size
--lr            Learning rate (default: 0.001)
--output        Path to save trained model
```

**Features:**
- Supports multiple architectures
- Data augmentation (flip, rotate, brightness)
- Validation split and early stopping
- TensorBoard logging

### evaluate_detector.py
Comprehensive model evaluation on test set.

**Usage:**
```bash
python scripts/training/evaluate_detector.py \
  --model models/traffic_detector.pth \
  --data data/test_annotations.json \
  --output results/evaluation.json
```

**Metrics:**
- Per-class accuracy
- Mean Average Precision (mAP)
- Inference time
- False positive/negative rates

### export_model.py
Export model for deployment (ONNX, TorchScript).

**Usage:**
```bash
python scripts/training/export_model.py \
  --model models/traffic_detector.pth \
  --format onnx \
  --output models/traffic_detector.onnx
```

---

## Testing Scripts

### test_detection.py
Test traffic sign detection on images or video.

**Usage:**
```bash
# Test on image
python scripts/testing/test_detection.py \
  --model models/traffic_detector.pth \
  --input test_image.jpg

# Test on video
python scripts/testing/test_detection.py \
  --model models/traffic_detector.pth \
  --input test_video.mp4 \
  --output results/detections.mp4
```

### test_fsm.py
Unit tests for FSM state transitions.

**Usage:**
```bash
python scripts/testing/test_fsm.py
```

**Tests:**
- State transition logic
- Timeout handling
- Emergency stop conditions
- Traffic sign response

### benchmark_performance.py
System performance benchmarking.

**Usage:**
```bash
python scripts/testing/benchmark_performance.py \
  --duration 60 \
  --output results/benchmark.json
```

**Metrics:**
- Detection rate (Hz)
- Control loop frequency
- CPU/memory usage
- End-to-end latency

---

## Utility Scripts

### visualize_detections.py
Visualize detection results on images.

**Usage:**
```bash
python scripts/utils/visualize_detections.py \
  --predictions results/detections.json \
  --images data/test_images/ \
  --output results/visualizations/
```

### calibrate_camera.py
Camera calibration using checkerboard pattern.

**Usage:**
```bash
python scripts/utils/calibrate_camera.py \
  --images calibration_images/ \
  --pattern 9x6 \
  --square-size 0.025 \
  --output config/camera_params.yaml
```

### analyze_logs.py
Parse and analyze ROS2 logs for debugging.

**Usage:**
```bash
python scripts/utils/analyze_logs.py \
  --log ~/.ros/log/latest/ \
  --output results/log_analysis.txt
```

---

## Example Workflows

### Complete Training Pipeline
```bash
# 1. Collect data
python scripts/data_collection/collect_rosbag.py \
  --output data/training_data.bag \
  --duration 600

# 2. Extract images
python scripts/data_collection/extract_images.py \
  --input data/training_data.bag \
  --output data/images/ \
  --rate 5

# 3. Label data
python scripts/data_collection/label_traffic_signs.py \
  --input data/images/ \
  --output data/annotations.json

# 4. Train model
python scripts/training/train_detector.py \
  --data data/annotations.json \
  --epochs 100 \
  --output models/traffic_detector.pth

# 5. Evaluate
python scripts/training/evaluate_detector.py \
  --model models/traffic_detector.pth \
  --data data/test_annotations.json

# 6. Export for deployment
python scripts/training/export_model.py \
  --model models/traffic_detector.pth \
  --format onnx
```

### Testing and Validation
```bash
# Test detection
python scripts/testing/test_detection.py \
  --model models/traffic_detector.pth \
  --input test_data/

# Test FSM
python scripts/testing/test_fsm.py

# Benchmark system
python scripts/testing/benchmark_performance.py \
  --duration 120
```

---

## Requirements
```
opencv-python>=4.5.0
torch>=1.9.0
rosbag2-py
numpy>=1.19.0
matplotlib>=3.3.0
tqdm
```

Install: `pip install -r requirements.txt`

---

## Notes

- All scripts assume ROS2 workspace is sourced
- Camera calibration needed before deployment
- Training scripts support GPU acceleration
- Use `--help` flag for detailed usage of any script
