# YOLO Inference and Training for ROS 2

This reference covers using YOLO (You Only Look Once) for object detection in ROS 2,
including training custom models for defect detection, creating inference nodes,
and managing the full ML lifecycle.

---

## 1. YOLO Overview

YOLO is a neural network that takes an image as input and outputs a list of detected objects,
each with a bounding box, class label, and confidence score.

```
Input:  640x640 pixel image (RGB)
Output: [
    { "class": "defect_a", "confidence": 0.92, "bbox": [x1, y1, x2, y2] },
    { "class": "defect_b", "confidence": 0.87, "bbox": [x1, y1, x2, y2] },
    { "class": "defect_c", "confidence": 0.78, "bbox": [x1, y1, x2, y2] }
]
```

### YOLO Versions

| Version | Year | Library | Notes |
|---------|------|---------|-------|
| YOLOv5 | 2020 | ultralytics (PyTorch) | Mature, well-documented, large community |
| YOLOv8 | 2023 | ultralytics | Current recommended starting point |
| YOLOv11 | 2024 | ultralytics | Latest, improved accuracy and speed |

**Recommendation:** Start with YOLOv8. It has the best documentation, the largest community,
and the Ultralytics Python API makes it accessible to developers without ML experience.
Upgrade to YOLOv11 when you need the extra performance.

### Model Sizes

Each YOLO version comes in multiple sizes. Larger models are more accurate but slower.

| Size | Parameters | Speed (GPU) | Speed (CPU) | mAP | Use Case |
|------|-----------|-------------|-------------|-----|----------|
| **Nano (n)** | 3.2M | ~1ms | ~40ms | Lower | Edge devices (Jetson Nano) |
| **Small (s)** | 11.2M | ~2ms | ~100ms | Good | Jetson Orin NX, real-time |
| **Medium (m)** | 25.9M | ~4ms | ~300ms | Better | Desktop GPU, balanced |
| **Large (l)** | 43.7M | ~6ms | ~600ms | High | Training/validation |
| **XLarge (x)** | 68.2M | ~10ms | ~1200ms | Highest | Maximum accuracy, not real-time |

**For inspection robotics on Jetson Orin NX:** Start with YOLOv8s (small). It gives good accuracy
at real-time speeds (30+ FPS with TensorRT). If accuracy is insufficient, try medium.

---

## 2. Ultralytics Python API

The Ultralytics library provides a simple, high-level Python API for YOLO.

### Installation

```bash
pip install ultralytics
```

### Quick Inference

```python
from ultralytics import YOLO

# Load a pre-trained model
model = YOLO('yolov8s.pt')  # Downloads automatically on first use

# Run inference on an image
results = model('image.jpg')

# Access detections
for result in results:
    boxes = result.boxes
    for box in boxes:
        # Bounding box coordinates (x1, y1, x2, y2)
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # Confidence score (0.0 to 1.0)
        confidence = box.conf[0].item()

        # Class ID and name
        class_id = int(box.cls[0].item())
        class_name = result.names[class_id]

        print(f'{class_name}: {confidence:.2f} at [{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]')
```

### Understanding Results

The `results` object contains everything about the detections:

| Attribute | Type | What It Contains |
|-----------|------|-----------------|
| `results[0].boxes.xyxy` | Tensor | Bounding boxes as [x1, y1, x2, y2] |
| `results[0].boxes.xywh` | Tensor | Bounding boxes as [center_x, center_y, width, height] |
| `results[0].boxes.conf` | Tensor | Confidence scores |
| `results[0].boxes.cls` | Tensor | Class IDs |
| `results[0].names` | dict | Class ID to name mapping |
| `results[0].orig_shape` | tuple | Original image dimensions |
| `results[0].plot()` | ndarray | Image with detections drawn on it |

---

## 3. Creating a ROS 2 Inference Node

Here is the architecture for a YOLO inference node in ROS 2:

```
/camera/image_raw                    /detections
   [Image]          +----------+     [Detection2DArray]
   --------->       | YOLO     |     --------->
                    | Inference |
   <parameters>     | Node     |     /detections/image
   - model_path     +----------+     [Image] (annotated)
   - confidence                      --------->
   - device
```

### Detection Message Format

Use `vision_msgs/msg/Detection2DArray` for publishing detections. This is the standard
ROS 2 message for 2D object detections.

```python
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
```

If vision_msgs is not available or too complex for prototyping, a custom message works too:

```python
# Custom alternative: publish as JSON string (quick prototyping only)
from std_msgs.msg import String
import json

detection_data = {
    'detections': [
        {'class': 'defect_a', 'confidence': 0.92, 'bbox': [100, 200, 300, 400]},
    ]
}
msg = String()
msg.data = json.dumps(detection_data)
```

### Complete Inference Node

```python
#!/usr/bin/env python3
"""
YOLOv8 inference node for ROS 2.

Subscribes to a camera image, runs YOLO inference, and publishes:
- Detection2DArray with bounding boxes and class labels
- Annotated image with detections drawn on it

Usage:
    ros2 run my_vision_pkg yolo_detector --ros-args \
        -p model_path:=/path/to/best.pt \
        -p confidence_threshold:=0.5 \
        -p input_topic:=/camera/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from ultralytics import YOLO
import time


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('model_path', 'yolov8s.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('device', 'cuda:0')  # 'cuda:0', 'cpu', or '0' for GPU
        self.declare_parameter('image_size', 640)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        input_topic = self.get_parameter('input_topic').value
        device = self.get_parameter('device').value
        self.img_size = self.get_parameter('image_size').value

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(
            f'Model loaded. Classes: {self.model.names}'
        )

        # Warm up the model (first inference is slow due to initialization)
        self.get_logger().info('Warming up model...')
        import numpy as np
        dummy = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        self.get_logger().info('Model warm-up complete')

        # Subscribers and publishers
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Image, input_topic, self.image_callback, image_qos
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )
        self.img_pub = self.create_publisher(
            Image, '/detections/image', 10
        )

        # FPS tracking
        self.frame_count = 0
        self.last_fps_time = time.time()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run inference
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            imgsz=self.img_size,
            verbose=False
        )

        # Build Detection2DArray message
        det_array = Detection2DArray()
        det_array.header = msg.header

        result = results[0]
        for box in result.boxes:
            det = Detection2D()

            # Bounding box (center x, center y, width, height)
            cx, cy, w, h = box.xywh[0].tolist()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            # Classification result
            hyp = ObjectHypothesisWithPose()
            class_id = int(box.cls[0].item())
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = box.conf[0].item()
            det.results.append(hyp)

            det_array.detections.append(det)

        # Publish detections
        self.det_pub.publish(det_array)

        # Publish annotated image
        annotated = result.plot()  # Returns BGR numpy array with boxes drawn
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.img_pub.publish(annotated_msg)

        # Log FPS
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now = time.time()
            fps = 30.0 / (now - self.last_fps_time)
            n_det = len(det_array.detections)
            self.get_logger().info(f'{fps:.1f} FPS, {n_det} detections')
            self.last_fps_time = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4. Custom Training Workflow

Training a custom YOLO model follows five stages. Each stage has clear inputs and outputs,
like a CI/CD pipeline.

### Stage 1: Data Collection

You need images of the things you want to detect. For defect detection:

**Sources of training images:**

| Source | Images | Quality | Cost |
|--------|--------|---------|------|
| Domain-specific datasets (public) | Varies | Real footage | Free |
| Roboflow Universe | Varies | Pre-annotated datasets | Free tier available |
| Your own camera footage | As needed | Matches your conditions | Time only |
| Synthetic data (Blender) | Unlimited | Simulated, less realistic | Setup time |

**Collecting from ROS 2 bags:**

```bash
# Record camera images to a bag file
ros2 bag record /camera/image_raw -o inspection_footage

# Extract images from bag to files
ros2 bag play inspection_footage
# In another terminal, run a node that saves images:
```

```python
#!/usr/bin/env python3
"""Save images from a ROS 2 topic to disk for annotation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.count = 0
        self.output_dir = 'dataset/images'
        os.makedirs(self.output_dir, exist_ok=True)

        self.declare_parameter('save_every_n', 10)  # Save every Nth frame
        self.n = self.get_parameter('save_every_n').value

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10
        )

    def callback(self, msg):
        self.count += 1
        if self.count % self.n != 0:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = os.path.join(self.output_dir, f'frame_{self.count:06d}.jpg')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved {filename}')
```

**How many images do you need?**

| Scenario | Images per Class | Total (5 classes) | Expected mAP |
|----------|-----------------|-------------------|--------------|
| Quick prototype | 50-100 | 250-500 | 0.4-0.6 |
| Working model | 200-500 | 1000-2500 | 0.6-0.8 |
| Production model | 500-2000 | 2500-10000 | 0.8-0.95 |

More data is always better, but diminishing returns start around 500 images per class.
Data quality (accurate annotations, diverse conditions) matters more than quantity.

### Stage 2: Annotation

Annotation means drawing bounding boxes around objects in your images and labeling them.

**Annotation tools:**

| Tool | Type | Cost | Best For |
|------|------|------|----------|
| **Roboflow** | Cloud | Free tier (1000 images) | Easiest, auto-augmentation, team collaboration |
| **CVAT** | Self-hosted / Cloud | Free (open source) | Large datasets, advanced features |
| **Label Studio** | Self-hosted / Cloud | Free (open source) | Flexible, multiple annotation types |
| **LabelImg** | Desktop app | Free | Simple, offline |

**Recommendation:** Start with Roboflow. It handles annotation, augmentation, dataset splitting,
and export to YOLO format. The free tier is enough for prototyping.

**Annotation best practices:**

1. **Draw tight boxes** -- the box should closely fit the object, not include lots of background
2. **Be consistent** -- if you include partial defects, always include them (or never)
3. **Label everything** -- if a crack is visible, label it, even if it is small
4. **Include hard examples** -- images with no defects, ambiguous cases, multiple defects
5. **Split your dataset** -- 70% train, 20% validation, 10% test (Roboflow does this automatically)

### Stage 3: Training

Training teaches YOLO to recognize your specific target objects by showing it thousands of
annotated examples.

**YOLO dataset format:**

```
dataset/
  train/
    images/
      frame_000001.jpg
      frame_000002.jpg
    labels/
      frame_000001.txt    # One .txt per image
      frame_000002.txt
  val/
    images/
    labels/
  data.yaml               # Dataset configuration
```

Each label file contains one line per object:

```
# class_id center_x center_y width height (all normalized 0-1)
0 0.45 0.32 0.12 0.08
2 0.72 0.61 0.25 0.15
```

**data.yaml:**

```yaml
train: ./train/images
val: ./val/images

nc: 5  # number of classes
names:
  0: defect_type_a
  1: defect_type_b
  2: defect_type_c
  3: defect_type_d
  4: defect_type_e
```

**Training command:**

```python
from ultralytics import YOLO

# Start from a pre-trained model (transfer learning)
model = YOLO('yolov8s.pt')

# Train on your dataset
results = model.train(
    data='dataset/data.yaml',
    epochs=100,           # Number of training passes
    imgsz=640,            # Input image size
    batch=16,             # Images per batch (reduce if OOM)
    patience=20,          # Stop early if no improvement for 20 epochs
    device='0',           # GPU device ID
    project='runs/train', # Output directory
    name='defect_detection', # Experiment name
    pretrained=True,      # Use pre-trained weights (transfer learning)
    augment=True,         # Data augmentation
)
```

**Training on Google Colab (free GPU):**

```python
# In a Colab notebook:
!pip install ultralytics
from google.colab import drive
drive.mount('/content/drive')

from ultralytics import YOLO
model = YOLO('yolov8s.pt')
model.train(data='/content/drive/MyDrive/dataset/data.yaml', epochs=100, imgsz=640)
```

**Training time estimates:**

| Hardware | 1000 images, 100 epochs | 5000 images, 100 epochs |
|----------|------------------------|------------------------|
| Google Colab (T4) | ~1-2 hours | ~5-8 hours |
| RTX 3060 | ~30-60 min | ~3-5 hours |
| RTX 4090 | ~15-30 min | ~1-2 hours |
| Jetson Orin NX | ~3-6 hours | ~12-24 hours |
| CPU only | ~12-24 hours | Not practical |

### Stage 4: Validation and Metrics

After training, evaluate how well your model performs.

**Key metrics explained:**

| Metric | What It Means | Good Value |
|--------|--------------|------------|
| **Precision** | Of all detections, how many are correct | > 0.8 |
| **Recall** | Of all real objects, how many were detected | > 0.8 |
| **mAP@50** | Mean Average Precision at 50% IoU | > 0.7 |
| **mAP@50:95** | mAP averaged across IoU thresholds | > 0.5 |
| **F1 Score** | Harmonic mean of precision and recall | > 0.8 |

**What is IoU (Intersection over Union)?**

IoU measures how much your predicted bounding box overlaps with the true bounding box.

```
IoU = Area of Overlap / Area of Union

IoU = 1.0  -> Perfect match
IoU = 0.5  -> 50% overlap (minimum threshold for "correct" detection)
IoU = 0.0  -> No overlap (wrong detection)
```

**Running validation:**

```python
from ultralytics import YOLO

model = YOLO('runs/train/defect_detection/weights/best.pt')
metrics = model.val(data='dataset/data.yaml')

print(f"mAP@50: {metrics.box.map50:.3f}")
print(f"mAP@50:95: {metrics.box.map:.3f}")
print(f"Precision: {metrics.box.mp:.3f}")
print(f"Recall: {metrics.box.mr:.3f}")
```

**Confusion matrix:** Shows which classes are confused with each other. If one defect type is
often misclassified as another, you need more diverse training examples of both.

### Stage 5: Export to ONNX/TensorRT

Export converts your PyTorch model to an optimized format for deployment.

```python
from ultralytics import YOLO

model = YOLO('runs/train/defect_detection/weights/best.pt')

# Export to ONNX (universal format, works everywhere)
model.export(format='onnx', imgsz=640, simplify=True)

# Export to TensorRT (NVIDIA GPUs only, fastest inference)
# Must run on the target hardware (Jetson) or matching GPU
model.export(format='engine', imgsz=640, half=True)  # FP16 for speed
```

| Format | Speed | Portability | When to Use |
|--------|-------|-------------|-------------|
| PyTorch (.pt) | Baseline | Python only | Training, prototyping |
| ONNX (.onnx) | 1.5-2x faster | Universal | Cross-platform deployment |
| TensorRT (.engine) | 2-10x faster | NVIDIA GPU only | Production on Jetson/GPU |

---

## 5. Inference Node Architecture

### Processing Pipeline

```
Camera Frame (BGR, 1920x1080)
    |
    v
[Preprocessing]
    - Resize to 640x640
    - Convert BGR to RGB
    - Normalize (0-255 -> 0-1)
    - Convert to tensor
    |
    v
[YOLO Inference]
    - Forward pass through neural network
    - Raw output: thousands of candidate boxes
    |
    v
[Postprocessing]
    - Non-Maximum Suppression (NMS)
    - Filter by confidence threshold
    - Scale boxes back to original image size
    |
    v
[Publish Results]
    - Detection2DArray message
    - Annotated image
```

**What is NMS (Non-Maximum Suppression)?**

The neural network outputs many overlapping boxes for each object. NMS removes duplicates
by keeping only the highest-confidence box for each detected object.

```
Before NMS: 50 boxes for one defect (overlapping, different confidences)
After NMS:  1 box for one defect (the best one)
```

Ultralytics handles NMS automatically. You can tune it:

```python
results = model(
    image,
    conf=0.5,     # Minimum confidence (higher = fewer detections, less noise)
    iou=0.45,     # NMS IoU threshold (higher = more overlapping boxes kept)
    max_det=100,  # Maximum detections per image
)
```

---

## 6. Handling Multiple Classes

For defect detection, you typically need multiple defect classes. Define your target objects
based on your domain-specific classification standards.

### Example Defect Classes

| Class | Description | Visual Characteristics |
|-------|------------|----------------------|
| **Crack** | Linear fracture in surface | Dark line, may branch, varies in width |
| **Corrosion** | Material degradation | Discoloration, rough texture |
| **Deformation** | Shape change | Unexpected geometry |
| **Surface damage** | General wear | Rough/pitted surface, missing material |
| **Obstruction** | Blockage or foreign object | Accumulation of debris |

### Class Hierarchy Strategy

Start with fewer classes and split later:

**Phase 1 (prototype):** Binary detection -- "defect" vs "no defect"
**Phase 2 (classification):** 3 classes -- crack, corrosion, other
**Phase 3 (full):** 5-7 specific defect classes

This approach lets you validate the pipeline quickly before investing in detailed annotation.

---

## 7. Confidence Thresholds and Tuning

The confidence threshold controls the trade-off between precision and recall:

| Threshold | Effect | Use Case |
|-----------|--------|----------|
| **0.25** (low) | Many detections, some false positives | Don't want to miss anything |
| **0.50** (medium) | Balanced | General use, operator review |
| **0.75** (high) | Few detections, high confidence only | Automated decision-making |

**For inspection robotics:** Start at 0.5. If operators report too many false alarms, raise it.
If they report missed defects, lower it. This is an operational tuning decision, not a
technical one.

### Making Thresholds Configurable

Always make the threshold a ROS 2 parameter so it can be adjusted at runtime:

```python
self.declare_parameter('confidence_threshold', 0.5)

# In the callback:
conf = self.get_parameter('confidence_threshold').value
results = self.model(image, conf=conf)
```

```bash
# Adjust at runtime without restarting the node
ros2 param set /yolo_detector confidence_threshold 0.6
```

---

## 8. Batch Inference vs Single-Frame

| Mode | Latency | Throughput | When to Use |
|------|---------|------------|-------------|
| **Single-frame** | Lowest (per frame) | Lower | Real-time, live camera |
| **Batch** | Higher (per frame) | Higher (total) | Offline processing, ROS bag analysis |

For live camera feeds, always use single-frame inference. The camera sends one frame at a time,
and you want the lowest latency for each frame.

For offline analysis (processing recorded footage):

```python
# Batch inference on a directory of images
results = model('dataset/images/', batch=8)  # Process 8 images at once
```

---

## 9. Model Versioning and A/B Testing

Track your models like you track code:

### Versioning Convention

```
models/
  defect_detection_v1_yolov8s_640_20260315.pt    # date-stamped
  defect_detection_v2_yolov8s_640_20260322.pt
  defect_detection_v2_yolov8m_640_20260322.pt    # larger model variant
```

### A/B Testing in ROS 2

Run two inference nodes simultaneously and compare:

```bash
# Model A (current production)
ros2 run my_vision_pkg yolo_detector --ros-args \
    -p model_path:=models/v1.pt \
    -r /detections:=/detections_a

# Model B (candidate)
ros2 run my_vision_pkg yolo_detector --ros-args \
    -p model_path:=models/v2.pt \
    -r /detections:=/detections_b
```

Then write a comparison node that subscribes to both and logs differences.

### Metrics to Track

| Metric | How to Measure |
|--------|---------------|
| mAP on test set | `model.val(data='test_data.yaml')` |
| Inference time (ms) | Time the model call in your node |
| False positive rate | Manual review of a sample |
| Missed defect rate | Compare against ground truth annotations |

---

## 10. Defect Detection: Practical Guidance

### Training Data Strategies

**Problem:** You probably do not have thousands of annotated domain images on day one.

**Solutions, ranked by effort:**

1. **Public datasets first** -- Search for domain-specific datasets on academic repositories or
   Roboflow Universe. Even 200-500 well-annotated images give a usable model.

2. **Transfer learning** -- Start from YOLO pre-trained on COCO (80 common objects). The model
   already knows about edges, textures, and shapes. Fine-tuning on 200 domain images works
   surprisingly well.

3. **Active learning loop** -- Deploy a weak model, have it flag low-confidence detections for
   human review. The human corrects and adds those images to the training set. Retrain. Repeat.

4. **Synthetic data** -- Generate images in Blender with procedural textures and defects.
   Useful for augmenting real data, not as a sole source.

5. **Data augmentation** -- Ultralytics applies augmentation automatically during training
   (flips, rotations, brightness changes, mosaic). This effectively multiplies your dataset size.

### Common Inspection Challenges

| Challenge | Impact | Mitigation |
|-----------|--------|------------|
| Low/uneven lighting | Missed detections in dark areas | LED lighting, histogram equalization, train on dark images |
| Reflections | False positives on reflective surfaces | Include reflection images in training, raise confidence threshold |
| Camera motion blur | Blurry images reduce accuracy | Shorter exposure time, include blurry images in training |
| Scale variation | Same defect looks different at different distances | Multi-scale training (already in YOLO augmentation) |
| Material variation | Different materials look different | Include all material types in training data |
| Environmental variation | Same defect looks different under different conditions | Include diverse conditions in training data |

### Mapping to Domain-Specific Classification Standards

If you need to map detections to domain-specific classification standards (e.g., industry
codes or grading systems), create a mapping between your YOLO class names and the standard
codes. This allows your detection system to generate reports in formats that your domain
already uses.

---

## 11. Complete Example: YOLOv8 ROS 2 Inference Node

See the complete node in Section 3 above. Here is how to set it up as a proper ROS 2 package:

### Package Structure

```
my_vision_pkg/
  my_vision_pkg/
    __init__.py
    yolo_detector.py     # The inference node from Section 3
  resource/
    my_vision_pkg
  config/
    yolo_params.yaml     # Default parameters
  launch/
    yolo_detector.launch.py
  models/                # Store your .pt files here
    yolov8s.pt
  package.xml
  setup.py
  setup.cfg
```

### Launch File

```python
# launch/yolo_detector.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_vision_pkg',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[{
                'model_path': '/path/to/models/best.pt',
                'confidence_threshold': 0.5,
                'input_topic': '/camera/image_raw',
                'device': 'cuda:0',
                'image_size': 640,
            }],
            remappings=[
                ('/detections', '/inspection/detections'),
                ('/detections/image', '/inspection/detections/image'),
            ],
        ),
    ])
```

### Parameter File

```yaml
# config/yolo_params.yaml
yolo_detector:
  ros__parameters:
    model_path: "models/defect_detection_v1.pt"
    confidence_threshold: 0.5
    input_topic: "/camera/image_raw"
    device: "cuda:0"
    image_size: 640
```

### setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'yolo_detector = my_vision_pkg.yolo_detector:main',
        ],
    },
)
```

### package.xml Dependencies

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>vision_msgs</depend>
<depend>cv_bridge</depend>
<exec_depend>python3-opencv</exec_depend>
```
