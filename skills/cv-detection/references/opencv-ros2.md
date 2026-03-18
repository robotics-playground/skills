# OpenCV and ROS 2 Integration

This reference covers integrating OpenCV with ROS 2: converting between image formats,
using image_transport for efficient transfer, calibrating cameras, and building image
processing pipelines.

---

## 1. cv_bridge: The Translation Layer

cv_bridge converts between ROS 2 `sensor_msgs/msg/Image` messages and OpenCV `numpy` arrays.
This is the single most important package for vision work in ROS 2.

### Installation

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge python3-opencv
```

### Python API

```python
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

# ROS Image -> OpenCV (numpy array)
cv_image = bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')

# OpenCV -> ROS Image
ros_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

# ROS CompressedImage -> OpenCV
cv_image = bridge.compressed_imgmsg_to_cv2(compressed_msg, desired_encoding='bgr8')

# OpenCV -> ROS CompressedImage
compressed_msg = bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
```

### The `desired_encoding` Parameter

This parameter tells cv_bridge to convert the image to your requested format. If the source
image is `rgb8` and you request `bgr8`, cv_bridge handles the conversion automatically.

Common values:
- `'bgr8'` -- Use this for OpenCV processing (OpenCV default)
- `'rgb8'` -- Use this for display or libraries expecting RGB
- `'mono8'` -- Grayscale, 8-bit
- `'passthrough'` -- No conversion, get the raw data as-is
- `'16UC1'` -- Depth image in millimeters (16-bit unsigned)
- `'32FC1'` -- Depth image in meters (32-bit float)

### Common Mistakes

```python
# WRONG: not specifying encoding -- you get whatever the camera publishes
cv_image = bridge.imgmsg_to_cv2(msg)  # might be RGB, BGR, Bayer, anything

# RIGHT: always specify what you want
cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

# WRONG: publishing without matching encoding
result_msg = bridge.cv2_to_imgmsg(bgr_image, encoding='rgb8')  # colors will be wrong!

# RIGHT: encoding matches the actual data format
result_msg = bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
```

---

## 2. Image Encodings Deep Dive

### What Is an Encoding?

An encoding describes how pixel data is stored in memory. A 640x480 BGR8 image is a contiguous
array of 640 * 480 * 3 = 921,600 bytes. Each pixel is 3 bytes: Blue, Green, Red (in that order).

### Encoding Reference Table

| Encoding | Channels | Bytes/Pixel | Data Type | Use Case |
|----------|----------|-------------|-----------|----------|
| `bgr8` | 3 | 3 | uint8 | OpenCV color processing |
| `rgb8` | 3 | 3 | uint8 | Display, some camera drivers |
| `mono8` | 1 | 1 | uint8 | Grayscale, edge detection |
| `mono16` | 1 | 2 | uint16 | High-dynamic-range grayscale |
| `16UC1` | 1 | 2 | uint16 | Depth images (mm) |
| `32FC1` | 1 | 4 | float32 | Depth images (meters) |
| `bgra8` | 4 | 4 | uint8 | Color with alpha channel |
| `bayer_rggb8` | 1 | 1 | uint8 | Raw Bayer pattern (needs debayering) |
| `bayer_grbg8` | 1 | 1 | uint8 | Raw Bayer pattern |

### Bayer Patterns

Some cameras output raw Bayer data instead of color. A Bayer image looks like a grayscale
image with a colored grid pattern. You must debayer (demosaic) it to get color.

```python
# If camera publishes bayer_grbg8, convert to color:
cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# cv_bridge handles debayering automatically when you request bgr8/rgb8
```

If cv_bridge does not handle it (older versions), do it manually:

```python
raw = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
color = cv2.cvtColor(raw, cv2.COLOR_BayerGR2BGR)
```

---

## 3. image_transport: Efficient Image Transfer

image_transport is a ROS 2 package that provides transparent compression for image topics.
Instead of sending raw pixels (which can be 30+ MB/s for a 1080p camera), it can send
JPEG-compressed images at a fraction of the bandwidth.

### How It Works

When you publish to `/camera/image_raw`, image_transport automatically creates:
- `/camera/image_raw` -- Raw (uncompressed)
- `/camera/image_raw/compressed` -- JPEG or PNG
- `/camera/image_raw/theora` -- Theora video stream
- `/camera/image_raw/compressedDepth` -- For depth images

Subscribers can choose which transport to use.

### Python Usage

For most Python nodes, you use standard subscribers with the compressed topic directly:

```python
from sensor_msgs.msg import CompressedImage

self.sub = self.create_subscription(
    CompressedImage,
    '/camera/image_raw/compressed',
    self.callback,
    qos_profile
)

def callback(self, msg):
    cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
```

### Transport Selection Guide

| Transport | Bandwidth | Latency | CPU Cost | When to Use |
|-----------|-----------|---------|----------|-------------|
| `raw` | Highest | Lowest | None | Same-machine processing |
| `compressed` (JPEG) | ~10x smaller | Low | Low | WiFi links, recording, remote viewing |
| `compressed` (PNG) | ~3x smaller | Medium | Medium | When you need lossless (depth images) |
| `theora` | Smallest | Higher | Medium | Video streaming to operator |
| `compressedDepth` | ~5x smaller | Low | Low | Depth image transmission |

### Configuring JPEG Quality

```bash
# Set JPEG quality (0-100, default 80)
ros2 param set /camera_node image_transport.compressed.jpeg_quality 50
```

Lower quality = smaller size = less bandwidth. For inference, quality 50-70 is often fine.

---

## 4. Camera Calibration

Camera calibration determines the camera's intrinsic parameters (focal length, distortion
coefficients). Without calibration, your images are distorted, measurements are wrong, and
depth calculations are inaccurate.

### Why Calibrate?

Every camera lens introduces distortion. Straight lines in the real world appear curved in
the image. Calibration computes a mathematical model of this distortion so you can correct it.

Calibration computes a mathematical model of the lens distortion so you can correct it.

### Using the ROS 2 Calibration Tool

```bash
# Install
sudo apt install ros-${ROS_DISTRO}-camera-calibration

# Print a checkerboard pattern (8x6 inner corners, 25mm squares)
# Download from: https://calib.io/pages/camera-calibration-pattern-generator

# Run calibration (with camera already publishing)
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

Move the checkerboard around in front of the camera. The tool needs to see it at various
angles, distances, and positions. When all bars are green, click "Calibrate", then "Save".

### Where Calibration Data Goes

The calibration tool saves a YAML file with the camera matrix and distortion coefficients.
Camera drivers load this file and publish it as `sensor_msgs/msg/CameraInfo` alongside
each image.

```yaml
# camera_info.yaml example
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.12, 0.08, 0.001, -0.002, 0.0]
```

### Undistorting Images

The `image_proc` package applies calibration automatically:

```bash
# Publishes rectified (undistorted) images
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
# Output: /image_rect
```

Or do it manually in Python:

```python
import cv2
import numpy as np

# From CameraInfo message
K = np.array(camera_info_msg.k).reshape(3, 3)
D = np.array(camera_info_msg.d)

# Undistort
undistorted = cv2.undistort(cv_image, K, D)
```

---

## 5. CameraInfo Message and How to Use It

The `sensor_msgs/msg/CameraInfo` message contains everything about the camera's geometry.

### Key Fields

| Field | Shape | What It Is | When You Need It |
|-------|-------|-----------|-----------------|
| `k` | 3x3 | Camera intrinsic matrix | Projecting 3D points to pixels, undistortion |
| `d` | 1xN | Distortion coefficients | Undistorting raw images |
| `r` | 3x3 | Rectification matrix | Stereo camera alignment |
| `p` | 3x4 | Projection matrix | 3D-to-2D projection for rectified images |
| `width`, `height` | scalars | Image dimensions | Validating image size matches calibration |
| `distortion_model` | string | "plumb_bob" or "rational_polynomial" | Choosing the right undistortion function |

### Using Camera Matrix for 3D Projection

If you have a depth value at pixel (u, v), you can compute the 3D point:

```python
import numpy as np

def pixel_to_3d(u, v, depth, K):
    """Convert pixel coordinate + depth to 3D point in camera frame."""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return np.array([x, y, z])
```

---

## 6. Basic OpenCV Operations in ROS 2 Nodes

### Resize

Resizing is the most common operation. Always resize before expensive processing.

```python
# Resize to fixed dimensions
resized = cv2.resize(cv_image, (640, 480))

# Resize by factor
half = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
```

### Color Space Conversions

```python
# BGR to grayscale
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

# BGR to HSV (for color-based detection)
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

# BGR to RGB (for display or ML models expecting RGB)
rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
```

### HSV Color Detection

HSV (Hue, Saturation, Value) is better than BGR for detecting colors because it separates
color (hue) from brightness (value). This makes detection robust to lighting changes.

```python
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

# Define range for "orange" (e.g., rust or discoloration)
lower_orange = np.array([10, 100, 100])
upper_orange = np.array([25, 255, 255])

# Create binary mask
mask = cv2.inRange(hsv, lower_orange, upper_orange)

# Find contours of detected regions
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter by area (ignore tiny noise)
significant = [c for c in contours if cv2.contourArea(c) > 500]
```

### Edge Detection

```python
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

# Gaussian blur to reduce noise (kernel size must be odd)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Canny edge detection
# Low threshold: edges below this are rejected
# High threshold: edges above this are strong edges
edges = cv2.Canny(blurred, 50, 150)

# Find contours from edges
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

### Thresholding

```python
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

# Simple threshold
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Adaptive threshold (handles uneven lighting -- common in enclosed environments)
adaptive = cv2.adaptiveThreshold(
    gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
)

# Otsu's threshold (automatic threshold selection)
_, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
```

---

## 7. Image Pipeline Node Architecture

A well-structured vision node follows this pattern:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2


class ImagePipelineNode(Node):
    """Base pattern for image processing nodes in ROS 2."""

    def __init__(self):
        super().__init__('image_pipeline_node')

        self.bridge = CvBridge()

        # Declare parameters for runtime configuration
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/processed/image')
        self.declare_parameter('target_width', 640)
        self.declare_parameter('target_height', 480)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # QoS: best_effort + keep_last(1) for camera images
        # This drops old frames instead of queuing them
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, image_qos
        )

        # Output uses reliable QoS (processed results should not be dropped)
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'Pipeline: {input_topic} -> processing -> {output_topic}'
        )

    def image_callback(self, msg):
        """Process incoming image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Resize to target dimensions
        width = self.get_parameter('target_width').value
        height = self.get_parameter('target_height').value
        cv_image = cv2.resize(cv_image, (width, height))

        # Process the image (override this in subclasses)
        result = self.process(cv_image)

        # Publish result
        if result is not None:
            result_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            result_msg.header = msg.header  # Preserve timestamp and frame_id
            self.publisher.publish(result_msg)

    def process(self, image):
        """Override this method with your processing logic."""
        return image


def main(args=None):
    rclpy.init(args=args)
    node = ImagePipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 8. Subscriber QoS for Images

QoS (Quality of Service) mismatches are the #1 reason image topics "don't work." The camera
publishes with one QoS profile, and if your subscriber uses an incompatible profile, no
messages are received and there is no error message.

### QoS Compatibility Rules

| Publisher | Subscriber | Works? |
|-----------|-----------|--------|
| RELIABLE | RELIABLE | Yes |
| RELIABLE | BEST_EFFORT | Yes |
| BEST_EFFORT | BEST_EFFORT | Yes |
| BEST_EFFORT | RELIABLE | **No** -- subscriber expects reliability the publisher cannot guarantee |

Most camera drivers publish with `BEST_EFFORT` reliability. Your subscriber MUST also use
`BEST_EFFORT` (or a compatible profile).

### Checking a Topic's QoS

```bash
# See QoS profile of a topic
ros2 topic info /camera/image_raw --verbose
```

Look for the `Reliability` field under the publisher's QoS.

### Sensor Data QoS Profile

ROS 2 provides a built-in profile for sensor data:

```python
from rclpy.qos import qos_profile_sensor_data

self.sub = self.create_subscription(
    Image, '/camera/image_raw', self.callback, qos_profile_sensor_data
)
```

This uses BEST_EFFORT reliability with KEEP_LAST(5), which works with most camera drivers.

---

## 9. Performance: Processing Rate vs Camera Rate

A 30 FPS camera publishes 30 images per second. If your processing takes 100ms per frame
(10 FPS), you MUST NOT queue the backlog. Drop frames instead.

### The Problem

```
Camera: 30 FPS
Processing: 10 FPS
Queue depth: 10

After 0.5 seconds: queue is full, processing is 0.5 seconds behind
After 10 seconds: queue is always full, latency keeps growing
```

### The Solution

Use KEEP_LAST with depth=1 and BEST_EFFORT. Process only the latest frame.

If you need to control processing rate explicitly:

```python
import time

class ThrottledProcessor(Node):
    def __init__(self):
        super().__init__('throttled_processor')
        self.declare_parameter('max_fps', 10.0)
        self.last_process_time = 0.0

        # Subscribe with BEST_EFFORT, depth=1
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, image_qos
        )

    def callback(self, msg):
        now = time.time()
        min_interval = 1.0 / self.get_parameter('max_fps').value
        if now - self.last_process_time < min_interval:
            return  # Skip this frame
        self.last_process_time = now

        # Process the frame
        self.process(msg)
```

---

## 10. Complete Example: Edge Detection ROS 2 Node

A full, runnable ROS 2 node that subscribes to a camera topic, detects edges using Canny,
draws contours, and publishes the annotated result.

```python
#!/usr/bin/env python3
"""
Edge detection ROS 2 node.

Subscribes to a camera image topic, applies Canny edge detection,
finds contours, and publishes an annotated image showing detected edges.

Usage:
    ros2 run my_vision_pkg edge_detector --ros-args \
        -p input_topic:=/camera/image_raw \
        -p output_topic:=/edges/image \
        -p canny_low:=50 \
        -p canny_high:=150 \
        -p min_contour_area:=100
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import time


class EdgeDetectorNode(Node):

    def __init__(self):
        super().__init__('edge_detector')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/edges/image')
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('blur_kernel_size', 5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Image QoS: match camera driver (best_effort, keep_last 1)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, image_qos
        )
        self.publisher = self.create_publisher(Image, output_topic, 10)

        # FPS tracking
        self.frame_count = 0
        self.last_fps_time = time.time()

        self.get_logger().info(
            f'Edge detector: {input_topic} -> {output_topic}'
        )

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Get parameters
        canny_low = self.get_parameter('canny_low').value
        canny_high = self.get_parameter('canny_high').value
        min_area = self.get_parameter('min_contour_area').value
        blur_k = self.get_parameter('blur_kernel_size').value

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)

        # Canny edge detection
        edges = cv2.Canny(blurred, canny_low, canny_high)

        # Find contours
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Filter by area and draw
        result = cv_image.copy()
        significant_count = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                significant_count += 1
                # Draw contour in green
                cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                # Draw bounding box in blue
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 1)

        # Add text overlay with detection count
        cv2.putText(
            result, f'Edges: {significant_count}',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
        )

        # Publish result
        result_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        result_msg.header = msg.header
        self.publisher.publish(result_msg)

        # Log FPS every 30 frames
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now = time.time()
            fps = 30.0 / (now - self.last_fps_time)
            self.get_logger().info(
                f'Processing at {fps:.1f} FPS, {significant_count} edges detected'
            )
            self.last_fps_time = now


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectorNode()
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

### Running the Example

```bash
# Terminal 1: Start a USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# Terminal 2: Run the edge detector
ros2 run my_vision_pkg edge_detector --ros-args \
    -p input_topic:=/camera/image_raw \
    -p canny_low:=30 \
    -p canny_high:=100

# Terminal 3: View the result
ros2 run rqt_image_view rqt_image_view /edges/image
```

### Package Setup (setup.py entry point)

```python
# In setup.py
entry_points={
    'console_scripts': [
        'edge_detector = my_vision_pkg.edge_detector:main',
    ],
},
```

### Package Dependencies (package.xml)

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<exec_depend>python3-opencv</exec_depend>
```
