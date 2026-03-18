# Depth Sensing with OAK-D and ROS 2

This reference covers stereo depth cameras (primarily OAK-D Lite and OAK-D Pro),
depth image processing, point clouds, and on-device neural inference for robotics
applications including robotics inspection.

---

## 1. OAK-D Lite Overview

The OAK-D Lite is a stereo depth camera with an on-board AI accelerator (VPU). It is made
by Luxonis and is one of the most popular depth cameras in the ROS 2 ecosystem.

### What You Get

| Feature | OAK-D Lite | OAK-D Pro |
|---------|-----------|-----------|
| Color camera | 4K (4056x3040) IMX214 | 4K (4056x3040) IMX378 |
| Stereo pair | 640x480 mono, 72 deg FOV | 1280x800 mono, 127 deg FOV |
| Depth range | 20cm - 15m | 20cm - 35m (with active IR) |
| AI accelerator | Intel Myriad X VPU | Intel Myriad X VPU |
| AI performance | 4 TOPS | 4 TOPS |
| IR illumination | No | Yes (active IR dot projector + flood) |
| IMU | No | BMI270 6-axis |
| USB | USB 3.0 Type-C | USB 3.0 Type-C |
| Price | ~$150 | ~$300 |

**For inspection robotics:** The OAK-D Pro is recommended because the active IR illumination
helps produce clean depth data in dark, low-light environments. The OAK-D Lite relies on
visible light for stereo matching, which fails in the dark.

### How Stereo Depth Works

Stereo depth works like human binocular vision. Two cameras (the "stereo pair") see the same
scene from slightly different positions. The difference in position of an object between the
two images (called "disparity") is inversely proportional to its distance.

```
Left camera     Object     Right camera
    O ----------- X ----------- O
    |<--- baseline (75mm) --->|

Close objects: large disparity (big shift between left and right image)
Far objects: small disparity (small shift)

depth = focal_length * baseline / disparity
```

Close objects have large disparity (big shift between left and right image), while far objects
have small disparity.

---

## 2. ROS 2 Integration with depthai_ros

The `depthai_ros` package is the official ROS 2 driver for OAK-D cameras.

### Installation

```bash
# Method 1: apt (recommended for Humble/Jazzy)
sudo apt install ros-${ROS_DISTRO}-depthai-ros

# Method 2: From source (latest features)
cd ~/ros2_ws/src
git clone https://github.com/luxonis/depthai-ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select depthai_ros
```

### Launch the Camera

```bash
# Basic launch with all streams
ros2 launch depthai_ros_driver camera.launch.py

# Custom configuration
ros2 launch depthai_ros_driver camera.launch.py \
    camera_model:=OAK-D-LITE \
    tf_prefix:=oak \
    enable_depth:=true \
    enable_spatial_nn:=false
```

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/oak/rgb/image_raw` | sensor_msgs/Image | Color camera image |
| `/oak/rgb/camera_info` | sensor_msgs/CameraInfo | Color camera calibration |
| `/oak/stereo/depth` | sensor_msgs/Image (16UC1) | Depth image (mm) |
| `/oak/stereo/camera_info` | sensor_msgs/CameraInfo | Stereo calibration |
| `/oak/left/image_rect` | sensor_msgs/Image | Left mono camera (rectified) |
| `/oak/right/image_rect` | sensor_msgs/Image | Right mono camera (rectified) |
| `/oak/points` | sensor_msgs/PointCloud2 | 3D point cloud (if enabled) |
| `/oak/imu/data` | sensor_msgs/Imu | IMU data (OAK-D Pro only) |

### Depth Image Format

The depth image is published as `sensor_msgs/Image` with encoding `16UC1`:
- Each pixel is a 16-bit unsigned integer
- Value represents distance in millimeters
- Value 0 means "invalid" (no depth data at this pixel)
- Typical range: 200mm (20cm) to 15000mm (15m)

```python
import numpy as np
from cv_bridge import CvBridge

bridge = CvBridge()

def depth_callback(msg):
    # Convert to numpy array (dtype: uint16, values in mm)
    depth_mm = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    # Convert to meters (float32)
    depth_m = depth_mm.astype(np.float32) / 1000.0

    # Get depth at a specific pixel
    center_y, center_x = depth_mm.shape[0] // 2, depth_mm.shape[1] // 2
    center_depth = depth_mm[center_y, center_x]

    if center_depth > 0:
        print(f'Center depth: {center_depth} mm ({center_depth / 1000.0:.2f} m)')
    else:
        print('No depth data at center pixel')
```

---

## 3. Depth Image vs Point Cloud: When to Use Which

| Format | Memory | Processing Speed | Use Case |
|--------|--------|-----------------|----------|
| **Depth image** (16UC1) | Small (640x480 = 600KB) | Fast | 2D operations: obstacle in front? How far? |
| **Point cloud** (PointCloud2) | Large (300K points = 3.6MB) | Slower | 3D operations: mapping, object shape, SLAM |

### Decision Guide

```
Do you need 3D geometry (shape, surface, mapping)?
+-- YES --> Point cloud
|           - Structural measurement
|           - 3D reconstruction
|           - Surface defect shape analysis
|
+-- NO  --> Depth image
            - "Is there an obstacle ahead?"
            - "How far is the nearest surface?"
            - "Overlay depth on color image"
            - Depth-to-costmap for Nav2
```

**For inspection robotics:** Start with depth images. They are simpler to work with and sufficient
for obstacle avoidance and basic distance measurement. Add point clouds later if you need
3D reconstruction or precise geometry measurement.

---

## 4. DepthAI Pipeline Concepts

The OAK-D has an on-board processor (Myriad X VPU) that runs a configurable pipeline. This
pipeline runs entirely on the camera -- your host computer (or Jetson) just receives the
results.

### Pipeline Components

| Component | What It Does | Output |
|-----------|-------------|--------|
| **ColorCamera** | Captures color images | Color frames (4K, 1080p, etc.) |
| **MonoCamera** (x2) | Left and right stereo cameras | Grayscale frames |
| **StereoDepth** | Computes depth from stereo pair | Depth image |
| **NeuralNetwork** | Runs AI model on VPU | Detection/classification results |
| **ImageManip** | Resize, crop, warp | Transformed images |
| **ObjectTracker** | Tracks detections across frames | Tracked objects with IDs |

### Pipeline Architecture

```
MonoCamera (Left)  ----+
                       +--> StereoDepth --> Depth Image
MonoCamera (Right) ----+

ColorCamera --> ImageManip (resize) --> NeuralNetwork --> Detections
     |
     +--> Color Image (for display/recording)
```

### Configuring the Pipeline via ROS 2

```bash
# Set stereo depth parameters
ros2 param set /oak stereo.confidence_threshold 200  # 0-255
ros2 param set /oak stereo.lr_check true             # Left-right consistency check
ros2 param set /oak stereo.subpixel true             # Sub-pixel accuracy
ros2 param set /oak stereo.extended_disparity true   # For close objects (<70cm)
```

---

## 5. On-Device Neural Inference with OAK-D

One of the OAK-D's most powerful features is running neural networks directly on the camera's
VPU. This means you can get detection results without using your Jetson's GPU at all.

### Why On-Device Inference?

| Approach | Latency | CPU/GPU Load | Power | Best For |
|----------|---------|-------------|-------|----------|
| Host inference (Jetson GPU) | ~5-15ms + transfer | High | High | Highest accuracy, large models |
| On-device inference (OAK VPU) | ~10-30ms | Near zero | Low | Always-on detection, power constrained |

### Supported Model Formats

The OAK-D VPU runs models in OpenVINO IR format (.blob). You can convert from:
- PyTorch -> ONNX -> OpenVINO IR -> .blob
- TensorFlow -> OpenVINO IR -> .blob
- YOLO (Ultralytics) -> ONNX -> OpenVINO IR -> .blob

### Converting YOLO for OAK-D

```bash
# Step 1: Export YOLO to ONNX
python -c "
from ultralytics import YOLO
model = YOLO('best.pt')
model.export(format='onnx', imgsz=416, simplify=True)
"

# Step 2: Convert ONNX to OpenVINO IR
pip install openvino-dev
mo --input_model best.onnx --data_type FP16

# Step 3: Compile to .blob using Luxonis blobconverter
pip install blobconverter
python -c "
import blobconverter
blob_path = blobconverter.from_openvino(
    xml='best.xml',
    bin='best.bin',
    shaves=6  # Number of VPU compute units (1-16, more = faster)
)
print(f'Blob saved to: {blob_path}')
"
```

### Using On-Device Detection in ROS 2

```bash
# Launch with spatial detection (YOLO on VPU + depth)
ros2 launch depthai_ros_driver camera.launch.py \
    enable_spatial_nn:=true \
    nn_model_path:=/path/to/best.blob
```

The camera publishes detection results directly:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/oak/nn/detections` | vision_msgs/Detection2DArray | 2D detections from VPU |
| `/oak/nn/spatial_detections` | depthai_msgs/SpatialDetectionArray | 3D detections (with depth) |

### On-Device Performance

| Model | Input Size | OAK-D FPS | Notes |
|-------|-----------|-----------|-------|
| YOLOv8n (nano) | 416x416 | ~15-20 FPS | Good for real-time |
| YOLOv8s (small) | 416x416 | ~8-12 FPS | Better accuracy |
| YOLOv8n | 320x320 | ~20-25 FPS | Fastest, lower accuracy |
| MobileNet-SSD | 300x300 | ~25-30 FPS | Simple detection |

---

## 6. depth_image_proc: Converting Depth to Point Cloud

The `depth_image_proc` package converts depth images to point clouds and back.

### Installation

```bash
sudo apt install ros-${ROS_DISTRO}-depth-image-proc
```

### Depth to Point Cloud

```bash
# Convert depth image to PointCloud2
ros2 run depth_image_proc point_cloud_xyzrgb_node --ros-args \
    -r depth_registered/image_rect:=/oak/stereo/depth \
    -r rgb/image_rect_color:=/oak/rgb/image_raw \
    -r rgb/camera_info:=/oak/rgb/camera_info \
    -r depth_registered/points:=/oak/points
```

This produces a colored 3D point cloud by combining depth values with color pixels.

---

## 7. Point Cloud Basics

### PointCloud2 Message

The `sensor_msgs/msg/PointCloud2` message is a binary blob containing 3D points. Each point
has at minimum (x, y, z) coordinates, and optionally color (rgb), intensity, or other fields.

```python
import numpy as np
from sensor_msgs.msg import PointCloud2
import struct

def read_points(cloud_msg):
    """Read x, y, z from a PointCloud2 message."""
    points = []
    # Each point is 16 bytes: x(4) + y(4) + z(4) + rgb(4)
    point_step = cloud_msg.point_step
    data = np.frombuffer(cloud_msg.data, dtype=np.uint8)

    for i in range(0, len(data), point_step):
        x = struct.unpack('f', data[i:i+4])[0]
        y = struct.unpack('f', data[i+4:i+8])[0]
        z = struct.unpack('f', data[i+8:i+12])[0]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            points.append((x, y, z))

    return np.array(points)
```

**Better approach: use the `sensor_msgs_py` helper:**

```python
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

def pointcloud_callback(msg):
    # Read all points as a structured numpy array
    points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'])

    # Filter out NaN/invalid points
    valid = points[~np.isnan(points).any(axis=1)]

    print(f'Valid points: {len(valid)} / {len(points)}')
    print(f'Min depth: {valid[:, 2].min():.2f} m')
    print(f'Max depth: {valid[:, 2].max():.2f} m')
```

### PCL (Point Cloud Library) in Python

For advanced point cloud processing (filtering, segmentation, registration), use `python-pcl`
or the `open3d` library:

```python
import open3d as o3d
import numpy as np

def process_pointcloud(points_np):
    """Process a point cloud using Open3D."""
    # Convert numpy array to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    # Statistical outlier removal (remove noise)
    pcd_clean, indices = pcd.remove_statistical_outlier(
        nb_neighbors=20,    # Number of neighbors to consider
        std_ratio=2.0       # Standard deviation threshold
    )

    # Voxel downsampling (reduce point count for speed)
    pcd_down = pcd_clean.voxel_down_sample(voxel_size=0.01)  # 1cm voxels

    # Estimate normals (needed for surface reconstruction)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
    )

    return pcd_down
```

---

## 8. Depth to Costmap Layer

For obstacle avoidance, you can convert depth data into a costmap layer that Nav2 uses for
path planning.

### How It Works

```
Depth Image (2D, distances in mm)
    |
    v
Project to 3D points (using camera intrinsics)
    |
    v
Filter by height (keep only points at robot height)
    |
    v
Project to 2D ground plane (bird's eye view)
    |
    v
Costmap layer (occupancy grid: 0=free, 100=obstacle)
```

### Using depth_to_costmap (Nav2)

Nav2 supports depth cameras natively through the `obstacle_layer` plugin:

```yaml
# nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: depth_camera
        depth_camera:
          topic: /oak/stereo/depth
          sensor_frame: oak_depth_optical_frame
          data_type: "PointCloud2"  # or "LaserScan"
          clearing: true
          marking: true
          max_obstacle_height: 0.5   # meters
          min_obstacle_height: 0.05  # meters
          obstacle_max_range: 3.0    # meters
          obstacle_min_range: 0.2    # meters
```

### Converting Depth to LaserScan

Sometimes it is easier to convert depth to a 2D laser scan (simpler for Nav2):

```bash
sudo apt install ros-${ROS_DISTRO}-depthimage-to-laserscan

ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \
    -r depth:=/oak/stereo/depth \
    -r depth_camera_info:=/oak/stereo/camera_info \
    -p scan_height:=10 \
    -p range_min:=0.2 \
    -p range_max:=5.0 \
    -p output_frame_id:=oak_depth_optical_frame
```

---

## 9. Filtering Point Clouds

Raw point clouds are noisy. Filtering improves quality and reduces data volume.

### Passthrough Filter

Keep only points within a specific range:

```python
def passthrough_filter(points, axis, min_val, max_val):
    """Keep points where axis value is within [min_val, max_val]."""
    # axis: 0=x, 1=y, 2=z
    mask = (points[:, axis] >= min_val) & (points[:, axis] <= max_val)
    return points[mask]

# Example: keep points between 0.2m and 3.0m depth
filtered = passthrough_filter(points, axis=2, min_val=0.2, max_val=3.0)
```

### Voxel Grid Filter

Downsample by averaging points within each voxel (3D grid cell):

```python
def voxel_grid_filter(points, voxel_size=0.02):
    """Downsample point cloud to one point per voxel."""
    # Quantize coordinates to voxel grid
    quantized = np.floor(points / voxel_size).astype(int)

    # Find unique voxels and average points within each
    _, indices, inverse = np.unique(
        quantized, axis=0, return_index=True, return_inverse=True
    )

    # One point per voxel (using the first point in each)
    return points[indices]
```

### Statistical Outlier Removal

Remove isolated noisy points:

```python
from scipy.spatial import cKDTree

def statistical_outlier_removal(points, k=20, std_ratio=2.0):
    """Remove points that are far from their neighbors."""
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k + 1)  # +1 because point is its own neighbor
    mean_distances = distances[:, 1:].mean(axis=1)  # Exclude self

    threshold = mean_distances.mean() + std_ratio * mean_distances.std()
    mask = mean_distances < threshold
    return points[mask]
```

---

## 10. Registration and Alignment

### Depth to Color Frame Alignment

The depth camera and color camera have different positions and fields of view. To overlay
depth on color (or get colored point clouds), the images must be aligned.

**OAK-D handles this on-device.** The `depthai_ros` driver publishes aligned depth:

```bash
# Enable depth alignment to color camera
ros2 param set /oak stereo.align_depth true
```

After alignment, each pixel in the depth image corresponds to the same pixel in the color image.

### TF Frames

The camera publishes several TF frames:

```
oak_base_frame
  |-- oak_rgb_camera_optical_frame    (color camera)
  |-- oak_left_camera_optical_frame   (left mono)
  |-- oak_right_camera_optical_frame  (right mono)
  |-- oak_depth_optical_frame         (depth, may be aligned to color)
  |-- oak_imu_frame                   (IMU, OAK-D Pro only)
```

All point cloud and depth data is in the optical frame convention:
- Z points forward (away from camera)
- X points right
- Y points down

This is different from the ROS convention (X forward, Y left, Z up). The TF tree handles
the transformation automatically.

---

## 11. Depth Quality: Noise, Range, and Accuracy

### Stereo Depth Limitations

| Factor | Impact | Mitigation |
|--------|--------|------------|
| **Minimum range** | No depth below ~20cm | Filter out depth < 200mm |
| **Textureless surfaces** | No stereo matches, depth holes | Active IR (OAK-D Pro), add texture (projected pattern) |
| **Reflective surfaces** | Incorrect depth values | Filter outliers, use confidence map |
| **Repetitive patterns** | Wrong matches (periodic error) | Post-filtering, confidence threshold |
| **Distance accuracy** | Depth error grows with distance squared | Depth accuracy: ~1% at 1m, ~4% at 2m, ~9% at 3m |
| **Low light** | Mono cameras cannot see, no depth | Active IR illumination (OAK-D Pro) |
| **Depth holes** | Missing depth at edges and occluded areas | Spatial filtering, temporal filtering |

### Improving Depth Quality

```bash
# Enable spatial filter (fills holes by interpolating neighbors)
ros2 param set /oak stereo.spatial_filter true

# Enable temporal filter (smooths depth over time)
ros2 param set /oak stereo.temporal_filter true

# Increase confidence threshold (reject uncertain depth)
ros2 param set /oak stereo.confidence_threshold 200  # 0-255, higher = stricter

# Enable left-right consistency check (rejects mismatches)
ros2 param set /oak stereo.lr_check true
```

### Validating Depth Data

```python
def validate_depth(depth_mm, min_range_mm=200, max_range_mm=5000):
    """Filter depth image to valid range and compute statistics."""
    import numpy as np

    # Mask invalid pixels
    valid = depth_mm.copy().astype(np.float32)
    valid[valid == 0] = np.nan              # 0 means no data
    valid[valid < min_range_mm] = np.nan    # Too close
    valid[valid > max_range_mm] = np.nan    # Too far

    # Statistics
    valid_count = np.count_nonzero(~np.isnan(valid))
    total_count = valid.size
    coverage = valid_count / total_count

    return {
        'valid_pixels': valid_count,
        'total_pixels': total_count,
        'coverage': coverage,         # What fraction of pixels have valid depth
        'min_depth_mm': np.nanmin(valid) if valid_count > 0 else 0,
        'max_depth_mm': np.nanmax(valid) if valid_count > 0 else 0,
        'mean_depth_mm': np.nanmean(valid) if valid_count > 0 else 0,
    }
```

---

## 12. Complete Example: OAK-D Lite Node

A ROS 2 node that subscribes to OAK-D color and depth, detects objects by depth threshold,
and publishes annotated results.

```python
#!/usr/bin/env python3
"""
OAK-D depth detection node.

Combines color and depth images to detect objects at specific distances.
Useful for obstacle detection, surface distance measurement, or
identifying objects within a depth range.

Usage:
    ros2 run my_vision_pkg depth_detector --ros-args \
        -p min_depth_mm:=200 \
        -p max_depth_mm:=2000 \
        -p color_topic:=/oak/rgb/image_raw \
        -p depth_topic:=/oak/stereo/depth
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import numpy as np
import time


class DepthDetectorNode(Node):

    def __init__(self):
        super().__init__('depth_detector')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('min_depth_mm', 200)
        self.declare_parameter('max_depth_mm', 2000)
        self.declare_parameter('color_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/stereo/depth')
        self.declare_parameter('output_topic', '/depth_detection/image')

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        output_topic = self.get_parameter('output_topic').value

        # QoS for camera topics
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Synchronized subscribers (color + depth arrive at the same time)
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=image_qos)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=image_qos)

        # ApproximateTimeSynchronizer matches messages by timestamp
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=5,
            slop=0.1  # Maximum time difference in seconds
        )
        self.sync.registerCallback(self.synced_callback)

        # Publisher
        self.publisher = self.create_publisher(Image, output_topic, 10)

        # FPS tracking
        self.frame_count = 0
        self.last_fps_time = time.time()

        self.get_logger().info(
            f'Depth detector: color={color_topic}, depth={depth_topic}'
        )

    def synced_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth images."""
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        min_d = self.get_parameter('min_depth_mm').value
        max_d = self.get_parameter('max_depth_mm').value

        # Resize depth to match color if dimensions differ
        if depth.shape[:2] != color.shape[:2]:
            depth = cv2.resize(
                depth, (color.shape[1], color.shape[0]),
                interpolation=cv2.INTER_NEAREST  # NEAREST for depth to avoid interpolation artifacts
            )

        # Create mask: pixels within depth range
        mask = ((depth > min_d) & (depth < max_d)).astype(np.uint8) * 255

        # Clean up mask with morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill holes

        # Find contours of detected regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw on color image
        result = color.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:  # Ignore tiny regions
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Get average depth in the bounding box
            roi_depth = depth[y:y+h, x:x+w]
            valid_depths = roi_depth[(roi_depth > min_d) & (roi_depth < max_d)]
            if len(valid_depths) == 0:
                continue
            avg_depth_mm = np.mean(valid_depths)

            # Draw bounding box and depth label
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f'{avg_depth_mm / 1000.0:.2f}m'
            cv2.putText(
                result, label, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

        # Add depth range info
        cv2.putText(
            result, f'Range: {min_d/1000:.1f}-{max_d/1000:.1f}m',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        # Publish annotated image
        result_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        result_msg.header = color_msg.header
        self.publisher.publish(result_msg)

        # Log FPS
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now = time.time()
            fps = 30.0 / (now - self.last_fps_time)
            self.get_logger().info(f'{fps:.1f} FPS')
            self.last_fps_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthDetectorNode()
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
# Terminal 1: Launch OAK-D camera
ros2 launch depthai_ros_driver camera.launch.py \
    enable_depth:=true \
    stereo.align_depth:=true

# Terminal 2: Run depth detector
ros2 run my_vision_pkg depth_detector --ros-args \
    -p min_depth_mm:=300 \
    -p max_depth_mm:=1500

# Terminal 3: View result
ros2 run rqt_image_view rqt_image_view /depth_detection/image
```
