---
name: cv-detection
description: >
  Expert guidance for computer vision and object detection in ROS 2 — OpenCV, YOLO, depth cameras, and training pipelines. Use when working with OpenCV, cv2, cv_bridge, image_transport, YOLO, YOLOv8, YOLOv11, ultralytics, object detection, defect detection, anomaly detection, TensorRT, ONNX, inference, camera calibration, OAK-D, DepthAI, depthai_ros, point cloud, PCL, vision_msgs, Detection2D, sensor_msgs/Image, Jetson Orin, Isaac ROS, CUDA, or when a camera image is distorted, inference is slow, detections are missing, or a model needs training or annotation.
---

# Computer Vision & Object Detection for ROS 2

> **Quick navigation**
> - OpenCV integration, cv_bridge, image_transport, calibration -> `references/opencv-ros2.md`
> - YOLO inference, training, annotation, defect detection -> `references/yolo-inference.md`
> - OAK-D, depth images, point clouds, stereo vision -> `references/depth-sensing.md`
> - Jetson optimization, TensorRT, Isaac ROS, profiling -> `references/jetson-optimization.md`

---

## Who This Skill Is For

This skill is written for **developers new to computer vision**. If you have general
programming experience but have never worked with image tensors, camera intrinsics, or
bounding boxes, this is your starting point.

An inference pipeline is just a data pipeline. A trained model is just a function that takes
pixels and returns structured data. Annotations are just labeled training examples.

---

## Vision Pipeline Architecture

Every vision system in ROS 2 follows this pipeline:

```
Camera Hardware          ROS 2 Transport         Processing              Action
+-------------+    +--------------------+    +----------------+    +-------------+
| USB Camera  | -> | /camera/image_raw  | -> | OpenCV resize, | -> | Publish     |
| OAK-D Lite  |    | sensor_msgs/Image  |    | color convert  |    | detections  |
| RealSense   |    +--------------------+    +-------+--------+    | to Nav2     |
+-------------+                                      |             | or operator |
                                                     v             +-------------+
                                              +----------------+
                                              | YOLO inference |
                                              | or classical   |
                                              | CV detection   |
                                              +----------------+
```

ROS 2 topics work as pub/sub message channels. The camera publishes images to a topic. Your
detection node subscribes, processes each frame, and publishes results to another topic.
Downstream nodes (navigation, alerts, UI) subscribe to your detection results.

### Pipeline Stages

| Stage | What Happens | ROS 2 Component |
|-------|-------------|-----------------|
| **Capture** | Camera hardware produces raw pixels | Camera driver node (usb_cam, depthai_ros) |
| **Transport** | Image data moves between nodes | image_transport (raw, compressed, theora) |
| **Preprocessing** | Resize, color convert, normalize | Your node using OpenCV (cv2) |
| **Detection** | Find objects/defects in the image | YOLO inference or classical CV algorithms |
| **Postprocessing** | Filter results, track across frames | Confidence thresholds, NMS, tracking |
| **Action** | React to detections | Publish markers, send alerts, update costmap |

---

## Which Detection Approach?

Not every problem needs deep learning. This table helps you choose:

| Approach | Best For | Training Data Needed | Compute Required | Accuracy | Setup Time |
|----------|----------|---------------------|------------------|----------|------------|
| **Classical CV** (OpenCV) | Color detection, edge detection, simple shapes, water level | None | CPU only | High for simple tasks | Hours |
| **YOLOv8/v11** (Ultralytics) | Multi-class object detection, defect classification | 200-2000+ labeled images | GPU recommended | High for complex tasks | Days to weeks |
| **Custom CNN** | Specialized tasks (anomaly detection, segmentation) | 1000+ labeled images | GPU required | Highest (when tuned) | Weeks to months |
| **On-device NN** (OAK-D) | Real-time detection at the edge, low-power systems | Same as YOLO | OAK-D VPU | Good | Days |

### Decision Flowchart

```
Is the target visually simple? (solid color, straight edge, known shape)
+-- YES --> Classical CV with OpenCV
|           Fast, no training data, runs on any CPU
|
+-- NO  --> Do you need to detect multiple defect types?
            +-- YES --> YOLO (YOLOv8 or YOLOv11)
            |           Best balance of accuracy and speed
            |           See references/yolo-inference.md
            |
            +-- NO  --> Do you need pixel-level segmentation?
                        +-- YES --> YOLO-Seg or custom segmentation model
                        +-- NO  --> Start with YOLO, it handles most cases
```

**For inspection and defect detection:** Start with YOLOv8 or YOLOv11. Real-world defects
(cracks, corrosion, deformations, foreign objects) are visually complex and vary widely.
Classical CV cannot reliably detect them. YOLO gives you multi-class detection with bounding
boxes, which is exactly what an operator or autonomous system needs.

---

## ROS 2 Image Format Guide

Images in ROS 2 are structured messages. Understanding the formats prevents hours of debugging.

### sensor_msgs/msg/Image

The primary image message. Fields that matter:

| Field | Type | What It Means |
|-------|------|--------------|
| `header.stamp` | builtin_interfaces/Time | When the image was captured |
| `header.frame_id` | string | Camera TF frame (e.g., "camera_color_optical_frame") |
| `height` | uint32 | Image height in pixels |
| `width` | uint32 | Image width in pixels |
| `encoding` | string | Pixel format (see table below) |
| `step` | uint32 | Row length in bytes (width x bytes_per_pixel) |
| `data` | uint8[] | Raw pixel data |

### Common Encodings

| Encoding | Bytes/Pixel | Use Case | Notes |
|----------|-------------|----------|-------|
| `bgr8` | 3 | OpenCV default color | OpenCV uses BGR, not RGB |
| `rgb8` | 3 | Display, some cameras | Convert to BGR before OpenCV processing |
| `mono8` | 1 | Grayscale | Edge detection, feature matching |
| `16UC1` | 2 | Depth images | Values in millimeters (typically) |
| `32FC1` | 4 | Depth images (float) | Values in meters |
| `bgra8` | 4 | Color + alpha | Rare in robotics |

**The #1 image encoding bug:** OpenCV uses BGR channel order. If your image looks blue-orange
instead of normal colors, you are displaying RGB data as BGR (or vice versa). Use
`cv2.cvtColor(img, cv2.COLOR_RGB2BGR)` to convert.

### sensor_msgs/msg/CompressedImage

Smaller than raw images. Use for bandwidth-constrained links (WiFi, remote monitoring).

| Field | What It Means |
|-------|--------------|
| `format` | Compression type: "jpeg", "png", "h264", "h265" |
| `data` | Compressed image bytes |

**When to use compressed:** Always for remote viewing or recording. JPEG at quality 80 is
roughly 10x smaller than raw. For local processing between nodes on the same machine,
raw is fine and avoids decode overhead.

### sensor_msgs/msg/CameraInfo

Camera intrinsic parameters. Published alongside images. You need this for:
- Undistorting images (removing lens distortion)
- Converting pixel coordinates to 3D rays
- Projecting 3D points onto the image

---

## Common Vision Failure Diagnosis

When something goes wrong (and it will), start here:

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Image topic exists but no data | QoS mismatch between publisher and subscriber | Match QoS profiles: camera usually uses `best_effort`, set subscriber to `best_effort` too |
| Image is all black | Camera not initialized, wrong exposure | Check camera driver logs, verify device permissions (`/dev/video*`) |
| Image is green/purple tinted | Wrong encoding or Bayer pattern not decoded | Check if camera outputs Bayer (e.g., `bayer_grbg8`) and debayer it |
| Image is distorted (fisheye) | Camera not calibrated or calibration not applied | Run `ros2 run camera_calibration cameracalibrator`, see `references/opencv-ros2.md` |
| Detections are jittery (flickering) | Low confidence threshold or no temporal filtering | Raise confidence threshold (0.5+), add frame-to-frame tracking |
| Model detects nothing | Wrong input size, bad preprocessing, untrained class | Verify model input dimensions match your preprocessing, check class names |
| Inference is too slow (<5 FPS) | Running on CPU, model too large, no TensorRT | Export to TensorRT on Jetson, use smaller model (YOLOv8n), see `references/jetson-optimization.md` |
| Out of memory during inference | Model + image buffer exceeds GPU/RAM | Reduce batch size to 1, reduce image resolution, use FP16 inference |
| Point cloud is empty | Depth image not aligned with color, wrong frame | Check TF tree, verify depth and color topics are in the same frame |
| Depth is noisy/invalid at close range | Object too close to stereo baseline | OAK-D Lite min range is ~20cm; filter out depth < min_range |
| cv_bridge conversion error | Encoding mismatch | Use `desired_encoding` parameter in `cv_bridge.imgmsg_to_cv2()` |
| Detection bounding boxes are offset | Image was resized without adjusting coordinates | Scale bounding box coordinates by the resize ratio |

---

## Camera Selection Guide

Choosing the right camera depends on your use case:

| Camera | Type | Price | Resolution | Depth | ROS 2 Driver | Best For |
|--------|------|-------|------------|-------|--------------|----------|
| **Logitech C920/C922** | USB webcam | $60-80 | 1080p @ 30fps | No | usb_cam | Prototyping, basic CV |
| **OAK-D Lite** | Stereo + AI | $150 | 4K color + depth | Yes (stereo) | depthai_ros | Depth + on-device inference |
| **OAK-D Pro** | Stereo + AI + IR | $300 | 4K color + depth | Yes + active IR | depthai_ros | Low-light depth, active IR |
| **Intel RealSense D435i** | Stereo + IMU | $350 | 1080p + depth | Yes (active IR stereo) | realsense2_camera | Depth + IMU for SLAM |
| **Intel RealSense D456** | Stereo + IMU | $400 | 1080p + depth | Yes (active IR) | realsense2_camera | Longer range depth |
| **Arducam USB3** | Industrial USB | $50-200 | Various | No | usb_cam / v4l2 | Custom lens, global shutter |

### Recommendation for Inspection Robots

**Primary: OAK-D Pro** ($300). The active IR illumination is valuable for low-light
environments. You get:
- Color camera for visual inspection and defect detection
- Stereo depth for obstacle avoidance and dimensional measurement
- On-device neural inference (run YOLO directly on the camera's VPU)
- Built-in IR illumination for low-light conditions

**Budget alternative: OAK-D Lite** ($150) with external lighting. You lose the IR depth
assist but can add your own lighting. Good for prototyping before committing to the Pro.

**For initial prototyping:** A USB webcam ($60) is fine for developing your detection pipeline.
You can train and test your YOLO model on webcam footage before deploying to the OAK-D.

---

## Image Processing Performance Rules

These rules prevent the most common performance problems:

### 1. Never Queue Images

```python
# BAD: default QoS queues images, subscriber falls behind
self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

# GOOD: only process the latest frame, drop old ones
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, qos)
```

### 2. Resize Before Processing

Inference on a 1920x1080 image is 4x slower than 640x480. YOLO resizes internally, but
you save memory and transfer time by resizing early.

### 3. Use Compressed Transport for Remote Viewing

```bash
# Remap to use compressed transport
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed
```

### 4. Profile Before Optimizing

Measure actual FPS before changing anything. The bottleneck might not be where you think.

```python
import time

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.last_time = time.time()
        self.frame_count = 0

    def callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now = time.time()
            fps = 30.0 / (now - self.last_time)
            self.get_logger().info(f'Processing at {fps:.1f} FPS')
            self.last_time = now
```

---

## Defect Detection: Quick Start Path

Here is the recommended progression for building an inspection/detection system:

### Phase 1: Camera Integration (Week 1)
1. Get a USB webcam publishing images in ROS 2 (`usb_cam` package)
2. Write a node that subscribes to images and displays them with OpenCV
3. Learn cv_bridge and image encodings
4. See `references/opencv-ros2.md`

### Phase 2: Classical CV Experiments (Week 2)
1. Implement edge detection on camera feed (Canny, find contours)
2. Implement color-based detection (HSV thresholding for water/sediment)
3. Learn image preprocessing (resize, blur, threshold)
4. See `references/opencv-ros2.md`

### Phase 3: YOLO Training and Inference (Weeks 3-4)
1. Collect or download domain-specific images (Roboflow, public datasets)
2. Annotate images with defect bounding boxes
3. Train YOLOv8 on your dataset
4. Create a ROS 2 inference node
5. See `references/yolo-inference.md`

### Phase 4: Depth Integration (Week 5)
1. Set up OAK-D Lite with depthai_ros
2. Combine color detection with depth measurement
3. Publish defect detections with 3D positions
4. See `references/depth-sensing.md`

### Phase 5: Jetson Deployment (Week 6)
1. Export YOLO model to TensorRT
2. Deploy on Jetson Orin NX/Nano
3. Optimize for real-time inference (15+ FPS target)
4. See `references/jetson-optimization.md`

---

## Key Vocabulary

| CV Term | Meaning |
|---------|---------|
| **Inference** | Running a trained model on new data to get predictions |
| **Training** | Teaching a model to recognize patterns from labeled examples |
| **Annotation** | Drawing bounding boxes around objects in images and labeling them |
| **Epoch** | One complete iteration through all training images |
| **Batch size** | Number of images processed at once during training |
| **Confidence score** | How sure the model is about a detection (0.0 to 1.0) |
| **IoU** (Intersection over Union) | How much a predicted box overlaps with the true box |
| **NMS** (Non-Maximum Suppression) | Removing duplicate detections of the same object |
| **mAP** (Mean Average Precision) | Overall detection quality across all classes |
| **Tensor** | The data format neural networks use (a multi-dimensional array of values) |
| **Backbone** | The base network architecture that extracts features from images |
| **Fine-tuning** | Adapting a pre-trained model to your specific task |
| **FP16** | Using 16-bit floats instead of 32-bit for faster inference |
| **TensorRT** | NVIDIA tool that optimizes models for GPU inference |
| **ONNX** | Universal model format for neural networks |

---

## Common ROS 2 Vision Packages

| Package | Purpose | Install |
|---------|---------|---------|
| `cv_bridge` | Convert ROS Image <-> OpenCV | `sudo apt install ros-${ROS_DISTRO}-cv-bridge` |
| `image_transport` | Efficient image pub/sub | `sudo apt install ros-${ROS_DISTRO}-image-transport` |
| `image_transport_plugins` | JPEG/PNG/Theora compression | `sudo apt install ros-${ROS_DISTRO}-image-transport-plugins` |
| `camera_calibration` | Camera intrinsic calibration | `sudo apt install ros-${ROS_DISTRO}-camera-calibration` |
| `image_proc` | Rectification, color conversion | `sudo apt install ros-${ROS_DISTRO}-image-proc` |
| `depth_image_proc` | Depth to point cloud | `sudo apt install ros-${ROS_DISTRO}-depth-image-proc` |
| `vision_msgs` | Standard detection messages | `sudo apt install ros-${ROS_DISTRO}-vision-msgs` |
| `usb_cam` | USB camera driver | `sudo apt install ros-${ROS_DISTRO}-usb-cam` |
| `depthai_ros` | OAK-D camera driver | See Luxonis docs for install |
| `realsense2_camera` | Intel RealSense driver | `sudo apt install ros-${ROS_DISTRO}-realsense2-camera` |

---

## Reference File Index

| File | Read When |
|------|-----------|
| `references/opencv-ros2.md` | Working with cv_bridge, image_transport, camera calibration, basic OpenCV operations in ROS 2, image pipeline architecture |
| `references/yolo-inference.md` | Training YOLO models, creating inference nodes, annotation workflows, defect detection, model export and optimization |
| `references/depth-sensing.md` | Setting up OAK-D cameras, working with depth images and point clouds, on-device neural inference, depth filtering and alignment |
| `references/jetson-optimization.md` | Deploying on Jetson hardware, TensorRT conversion, Isaac ROS, power management, profiling, multi-camera inference |
