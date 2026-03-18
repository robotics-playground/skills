# Jetson Optimization for Vision Inference

This reference covers deploying computer vision models on NVIDIA Jetson hardware,
including TensorRT optimization, Isaac ROS integration, power management, and
profiling tools.

---

## 1. Jetson Platform Overview

NVIDIA Jetson modules are small, power-efficient computers with dedicated GPUs. They are
the standard platform for running neural network inference on robots.

### Jetson Module Comparison

| Module | GPU Cores | AI Performance | RAM | Power | Price | Use Case |
|--------|-----------|---------------|-----|-------|-------|----------|
| **Jetson Orin NX 16GB** | 1024 CUDA | 100 TOPS (INT8) | 16GB | 10-25W | ~$600 | Production robotics |
| **Jetson Orin NX 8GB** | 1024 CUDA | 70 TOPS (INT8) | 8GB | 10-25W | ~$400 | Budget production |
| **Jetson Orin Nano 8GB** | 512 CUDA | 40 TOPS (INT8) | 8GB | 7-15W | ~$250 | Dev kits, prototyping |
| **Jetson Orin Nano 4GB** | 512 CUDA | 20 TOPS (INT8) | 4GB | 7-15W | ~$200 | Lightweight inference |
| **Jetson AGX Orin 64GB** | 2048 CUDA | 275 TOPS (INT8) | 64GB | 15-60W | ~$1600 | Multi-model, high-end |

**For inspection robotics:** The Jetson Orin NX 16GB is the sweet spot. Enough GPU for
real-time YOLO inference, enough RAM for ROS 2 + multiple camera streams, and within the
power budget of a battery-powered robot.

### JetPack and L4T

| Term | What It Is |
|------|-----------|
| **JetPack** | NVIDIA's SDK for Jetson. Includes Linux, CUDA, cuDNN, TensorRT, and development tools. |
| **L4T** (Linux for Tegra) | The Linux distribution that runs on Jetson. Based on Ubuntu. |
| **JetPack 6.x** | Based on Ubuntu 22.04, supports CUDA 12, ROS 2 Humble/Jazzy |
| **JetPack 7.x** | Based on Ubuntu 24.04, latest CUDA/TensorRT, ROS 2 Jazzy/Rolling |

**Always match your JetPack version to your ROS 2 distribution.** JetPack 6.x pairs with
Humble/Jazzy. JetPack 7.x pairs with Jazzy/Rolling. Mismatches cause library conflicts.

### Setting Up a Jetson

```bash
# Flash JetPack using NVIDIA SDK Manager (from a host Ubuntu PC)
# Or use the pre-flashed SD card image for dev kits

# After boot, verify CUDA
nvcc --version

# Verify TensorRT
python3 -c "import tensorrt; print(tensorrt.__version__)"

# Install ROS 2 (same as desktop Ubuntu)
# Follow standard ROS 2 install instructions for your Ubuntu version
```

---

## 2. TensorRT: What It Is and Why It Matters

TensorRT is NVIDIA's inference optimizer. It takes a trained model (PyTorch, ONNX) and
produces an optimized version that runs 2-10x faster on NVIDIA GPUs.

TensorRT compiles a trained model into optimized GPU code. The model does the same thing,
but runs much faster.

### What TensorRT Does

| Optimization | What Happens | Speedup |
|-------------|-------------|---------|
| **Layer fusion** | Combines multiple operations into one GPU kernel | 1.5-2x |
| **Precision reduction** | Converts FP32 to FP16 or INT8 | 2-4x |
| **Kernel auto-tuning** | Selects the fastest GPU algorithm for each operation | 1.2-1.5x |
| **Memory optimization** | Reuses GPU memory between layers | Allows larger models |
| **Dynamic shapes** | Optimizes for your specific input dimensions | 1.1-1.3x |

### Performance Comparison (YOLOv8s, 640x640, Jetson Orin NX)

| Format | Precision | Inference Time | FPS | Relative Speed |
|--------|-----------|---------------|-----|----------------|
| PyTorch (.pt) | FP32 | ~35ms | ~28 | 1.0x (baseline) |
| ONNX (.onnx) | FP32 | ~25ms | ~40 | 1.4x |
| TensorRT (.engine) | FP32 | ~15ms | ~66 | 2.3x |
| TensorRT (.engine) | FP16 | ~8ms | ~125 | 4.4x |
| TensorRT (.engine) | INT8 | ~5ms | ~200 | 7.0x |

**Key takeaway:** Going from PyTorch to TensorRT FP16 gives you a 4x speedup with minimal
accuracy loss. This is the single most impactful optimization you can make.

---

## 3. Converting YOLO to TensorRT

### Method 1: Ultralytics Export (Simplest)

```python
from ultralytics import YOLO

# Load your trained model
model = YOLO('best.pt')

# Export directly to TensorRT engine
# This MUST be run on the target Jetson hardware
model.export(
    format='engine',   # TensorRT engine
    imgsz=640,         # Input size
    half=True,         # FP16 precision (recommended)
    device=0,          # GPU device
    workspace=4,       # GPU memory workspace in GB
)
# Creates: best.engine
```

**Important:** TensorRT engines are hardware-specific. An engine built on Jetson Orin NX
will NOT work on Jetson Orin Nano or a desktop GPU. Always build on the target hardware.

### Method 2: ONNX to TensorRT (More Control)

```bash
# Step 1: Export to ONNX (can be done on any machine)
python3 -c "
from ultralytics import YOLO
model = YOLO('best.pt')
model.export(format='onnx', imgsz=640, simplify=True, opset=17)
"

# Step 2: Convert ONNX to TensorRT (must be on target Jetson)
/usr/src/tensorrt/bin/trtexec \
    --onnx=best.onnx \
    --saveEngine=best.engine \
    --fp16 \
    --workspace=4096 \
    --verbose
```

### Method 3: INT8 Quantization (Maximum Speed)

INT8 gives the fastest inference but requires a calibration dataset (representative images):

```python
from ultralytics import YOLO

model = YOLO('best.pt')
model.export(
    format='engine',
    imgsz=640,
    int8=True,          # INT8 quantization
    data='data.yaml',   # Calibration dataset (uses validation images)
    workspace=4,
)
```

**INT8 accuracy impact:** Typically 0.5-2% mAP drop compared to FP16. Test on your
validation set to verify it is acceptable for your use case.

### Using the TensorRT Engine in Your Node

```python
from ultralytics import YOLO

# Load TensorRT engine (same API as PyTorch model)
model = YOLO('best.engine')

# Inference is identical -- Ultralytics handles the backend
results = model(image, conf=0.5)
```

---

## 4. CUDA Inference Basics

For most use cases, TensorRT through Ultralytics is sufficient. You only need raw CUDA/cuDNN
when:
- Building custom preprocessing (GPU-accelerated resize, color conversion)
- Implementing custom post-processing
- Integrating with non-Ultralytics frameworks

### When TensorRT Suffices (Most Cases)

```
Camera Image -> CPU preprocessing -> TensorRT inference -> CPU postprocessing -> Publish
```

This pipeline is fast enough for most robotics applications (30+ FPS with YOLO on Orin NX).

### When You Need CUDA

```
Camera Image -> GPU preprocessing (CUDA) -> TensorRT inference -> GPU postprocessing (CUDA) -> Publish
```

This eliminates CPU-GPU memory copies and is needed when:
- Processing 4K images at 30+ FPS
- Running multiple models in sequence
- CPU is the bottleneck (unlikely on Orin NX)

### GPU-Accelerated Preprocessing with OpenCV CUDA

```python
import cv2

# Check if OpenCV was built with CUDA support
print(cv2.cuda.getCudaEnabledDeviceCount())

# Upload image to GPU
gpu_image = cv2.cuda_GpuMat()
gpu_image.upload(cv_image)

# Resize on GPU
gpu_resized = cv2.cuda.resize(gpu_image, (640, 640))

# Color conversion on GPU
gpu_rgb = cv2.cuda.cvtColor(gpu_image, cv2.COLOR_BGR2RGB)

# Download back to CPU
result = gpu_resized.download()
```

**Note:** OpenCV CUDA requires building OpenCV from source with CUDA flags. The default
apt-installed OpenCV does not include CUDA support.

---

## 5. Isaac ROS DNN Inference

NVIDIA Isaac ROS provides optimized ROS 2 nodes for DNN inference on Jetson. These nodes
handle the full pipeline (subscribe to image, run inference, publish results) with GPU
acceleration.

### Installation

```bash
# Isaac ROS is distributed as Docker containers or apt packages
# See: https://nvidia-isaac-ros.github.io/

# Install Isaac ROS DNN Inference
sudo apt install ros-${ROS_DISTRO}-isaac-ros-dnn-image-encoder
sudo apt install ros-${ROS_DISTRO}-isaac-ros-tensor-rt
sudo apt install ros-${ROS_DISTRO}-isaac-ros-triton
```

### Isaac ROS vs Custom Node

| Feature | Isaac ROS | Custom Python Node |
|---------|-----------|-------------------|
| Performance | Highest (zero-copy GPU pipeline) | Good (CPU-GPU copies) |
| Flexibility | Limited to supported models | Fully customizable |
| Setup complexity | Higher (Docker, specific formats) | Lower (pip install ultralytics) |
| Debugging | Harder (GPU internals) | Easier (Python debugger) |

**Recommendation:** Start with a custom Python node using Ultralytics + TensorRT. Move to
Isaac ROS only if you need the extra performance (multi-camera, 4K, <5ms latency).

---

## 6. Power Modes

Jetson modules support multiple power modes that trade performance for power consumption.

### Jetson Orin NX Power Modes

| Mode | Power | GPU Freq | CPU Freq | Use Case |
|------|-------|----------|----------|----------|
| MAXN | 25W | Max | Max | Maximum performance, wall power |
| 25W | 25W | Max | Max | Same as MAXN on Orin NX |
| 15W | 15W | Reduced | Reduced | Battery operation, moderate performance |
| 10W | 10W | Further reduced | Further reduced | Extended battery life |

### Setting Power Mode

```bash
# List available modes
sudo nvpmodel -q

# Set power mode (requires reboot or mode switch)
sudo nvpmodel -m 0    # MAXN (maximum performance)
sudo nvpmodel -m 1    # 15W
sudo nvpmodel -m 2    # 10W

# Maximize clocks within current power mode
sudo jetson_clocks
```

### Performance Impact on YOLO Inference

| Power Mode | YOLOv8s FP16 (ms) | FPS | Battery Impact |
|-----------|-------------------|-----|----------------|
| MAXN (25W) | ~8ms | ~125 | Highest draw |
| 15W | ~12ms | ~83 | Moderate |
| 10W | ~18ms | ~55 | Lowest draw |

**For inspection robotics:** 15W mode gives 80+ FPS with YOLOv8s, which is well above the
30 FPS camera rate. Use 15W for normal operation and MAXN only for demanding multi-model
pipelines.

---

## 7. Memory Management on Jetson

Jetson uses unified memory -- the CPU and GPU share the same physical RAM. This is different
from desktop systems where the GPU has its own dedicated memory (VRAM).

### Implications

| Desktop GPU | Jetson |
|------------|--------|
| 16GB system RAM + 8GB VRAM | 16GB shared RAM |
| Model loaded into VRAM | Model loaded into shared RAM |
| CPU-GPU copy needed | Zero-copy possible |
| Out of VRAM = GPU OOM | Out of RAM = system crash |

### Memory Budget (Jetson Orin NX 16GB)

| Component | Typical Usage |
|-----------|--------------|
| Linux + desktop | 1-2 GB |
| ROS 2 + nodes | 1-2 GB |
| YOLOv8s TensorRT FP16 | 0.5-1 GB |
| Camera buffers (2 cameras) | 0.5-1 GB |
| Point cloud processing | 0.5-1 GB |
| **Available for other tasks** | **~9-12 GB** |

### Monitoring Memory

```bash
# Real-time memory usage
free -h

# GPU-specific memory (Jetson unified memory)
cat /sys/kernel/debug/nvmap/iovmm/clients

# Or use jtop (recommended)
sudo pip3 install jetson-stats
jtop
```

### Avoiding OOM

```python
import torch

# Clear GPU cache after model operations
torch.cuda.empty_cache()

# Set memory fraction limit (percentage of total RAM for PyTorch)
torch.cuda.set_per_process_memory_fraction(0.3)  # Use max 30% of RAM

# Monitor in your node
import psutil
mem = psutil.virtual_memory()
if mem.percent > 80:
    self.get_logger().warn(f'Memory usage high: {mem.percent}%')
```

---

## 8. Multi-Stream Inference

Running inference on multiple camera feeds simultaneously.

### Architecture Options

**Option A: Sequential processing (simplest)**

```python
# One inference node, multiple camera subscriptions
# Process cameras round-robin
class MultiCameraDetector(Node):
    def __init__(self):
        super().__init__('multi_camera_detector')
        self.model = YOLO('best.engine')

        self.sub_front = self.create_subscription(
            Image, '/front_camera/image_raw', self.front_callback, qos
        )
        self.sub_rear = self.create_subscription(
            Image, '/rear_camera/image_raw', self.rear_callback, qos
        )
```

**Option B: Separate nodes per camera**

```bash
# Each camera gets its own inference node
ros2 run my_vision_pkg yolo_detector --ros-args \
    -p input_topic:=/front_camera/image_raw \
    -r __node:=front_detector

ros2 run my_vision_pkg yolo_detector --ros-args \
    -p input_topic:=/rear_camera/image_raw \
    -r __node:=rear_detector
```

**Option C: Batch inference (most efficient GPU use)**

```python
# Collect frames from multiple cameras, batch them for inference
import numpy as np

class BatchDetector(Node):
    def __init__(self):
        super().__init__('batch_detector')
        self.model = YOLO('best.engine')
        self.frames = {}

    def front_callback(self, msg):
        self.frames['front'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_batch_inference()

    def rear_callback(self, msg):
        self.frames['rear'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_batch_inference()

    def try_batch_inference(self):
        if len(self.frames) < 2:
            return
        batch = list(self.frames.values())
        results = self.model(batch, conf=0.5)  # Process both at once
        self.frames.clear()
```

### Multi-Camera Performance (Jetson Orin NX, YOLOv8s FP16)

| Cameras | Strategy | Per-Camera FPS | Total GPU Load |
|---------|----------|---------------|----------------|
| 1 | Single node | 125 | ~35% |
| 2 | Separate nodes | 60 each | ~70% |
| 2 | Batch inference | 80 each | ~65% |
| 3 | Batch inference | 50 each | ~85% |
| 4 | Batch inference | 35 each | ~95% |

---

## 9. Profiling Tools

### tegrastats

Built-in Jetson monitoring tool:

```bash
# Real-time stats (RAM, GPU, CPU, temperature, power)
tegrastats

# Output example:
# RAM 4815/15839MB (lfb 1524x4MB) SWAP 0/7919MB CPU [46%@1510, 43%@1510, ...]
# GR3D_FREQ 61% VIC_FREQ 0% ... GPU 45C CPU 42C ... VDD_IN 12500mW
```

Key values to watch:
- `RAM`: Total memory usage
- `GR3D_FREQ`: GPU utilization percentage
- `CPU [%@MHz]`: Per-core CPU usage and frequency
- `GPU xxC`: GPU temperature (throttles at ~90C)
- `VDD_IN`: Total power consumption in milliwatts

### jtop (Recommended)

A visual, interactive monitoring tool:

```bash
sudo pip3 install jetson-stats
jtop
```

jtop provides:
- Real-time GPU/CPU/memory graphs
- Temperature monitoring
- Power mode selection
- Fan control
- Process list sorted by GPU/CPU usage

### NVIDIA Nsight Systems (Advanced)

For detailed profiling of GPU kernels and CPU-GPU interaction:

```bash
# Profile your inference node
nsys profile --trace=cuda,nvtx,osrt \
    python3 yolo_inference.py

# Open the resulting .qdrep file in Nsight Systems GUI
```

### Python-Level Profiling

```python
import time

class ProfiledDetector(Node):
    def callback(self, msg):
        t0 = time.perf_counter()

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        t1 = time.perf_counter()

        results = self.model(cv_image, conf=0.5, verbose=False)
        t2 = time.perf_counter()

        # Post-processing and publishing
        self.publish_results(results)
        t3 = time.perf_counter()

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Timings - cv_bridge: {(t1-t0)*1000:.1f}ms, '
                f'inference: {(t2-t1)*1000:.1f}ms, '
                f'publish: {(t3-t2)*1000:.1f}ms, '
                f'total: {(t3-t0)*1000:.1f}ms'
            )
```

---

## 10. Docker on Jetson

Docker is the recommended way to deploy ROS 2 + inference pipelines on Jetson. NVIDIA provides
base images with CUDA, TensorRT, and other libraries pre-installed.

### NVIDIA Container Runtime

Jetson uses the NVIDIA container runtime to give Docker containers access to the GPU:

```bash
# Verify NVIDIA runtime is available
docker info | grep -i nvidia

# Run with GPU access
docker run --runtime nvidia -it nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3

# Or use --gpus flag
docker run --gpus all -it nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3
```

### Isaac ROS Docker Containers

NVIDIA provides pre-built Docker images for Isaac ROS:

```bash
# Pull Isaac ROS base image (JetPack 7 / Jazzy)
docker pull nvcr.io/isaac/ros:jazzy-ros2_jazzy-latest

# Run with ROS 2 and GPU
docker run --runtime nvidia \
    --network host \
    -v /dev:/dev \
    --privileged \
    -it nvcr.io/isaac/ros:jazzy-ros2_jazzy-latest
```

### Custom Dockerfile for Vision Pipeline

```dockerfile
# Dockerfile for YOLO inference on Jetson (JetPack 7 / Ubuntu 24.04)
FROM nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3

# Install ROS 2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install ultralytics opencv-python-headless

# Copy your ROS 2 workspace
COPY ./ros2_ws /workspace/ros2_ws
WORKDIR /workspace/ros2_ws

# Build
RUN . /opt/ros/jazzy/setup.sh && colcon build

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

### entrypoint.sh

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
exec "$@"
```

### Running the Container

```bash
docker build -t robot-vision .

docker run --runtime nvidia \
    --network host \
    -v /dev/video0:/dev/video0 \
    --privileged \
    robot-vision \
    ros2 launch my_vision_pkg yolo_detector.launch.py
```

---

## 11. Thermal Management

Jetson modules throttle when they get too hot. In an enclosed robot, thermal management matters.

| Temperature | Status | Action |
|------------|--------|--------|
| < 70C | Normal | None needed |
| 70-80C | Warm | Monitor, ensure airflow |
| 80-90C | Hot | GPU begins throttling |
| > 90C | Critical | Severe throttling, potential shutdown |

### Monitoring Temperature

```bash
# Read all thermal zones
cat /sys/devices/virtual/thermal/thermal_zone*/temp
# Values are in millidegrees: 45000 = 45.0C

# Or use tegrastats/jtop (see Section 9)
```

### Cooling Strategies

1. **Active cooling (fan):** The Jetson dev kit includes a fan. For custom carrier boards,
   add a 40mm fan with PWM control.
2. **Heatsink:** Essential. Use the included heatsink or a larger aftermarket one.
3. **Thermal pad:** Ensure good contact between the module and heatsink.
4. **Airflow:** Design the robot enclosure with ventilation paths.
5. **Power mode:** Use 15W mode instead of MAXN to reduce heat generation.

```bash
# Manual fan control (dev kit)
sudo sh -c 'echo 200 > /sys/devices/pwm-fan/target_pwm'  # 0-255
```

---

## 12. Complete Example: TensorRT YOLOv8 on Jetson

End-to-end workflow from trained model to real-time inference on Jetson.

### Step 1: Export Model to TensorRT (On Jetson)

```bash
# Copy your trained model to Jetson
scp best.pt jetson@192.168.1.100:~/models/

# SSH into Jetson
ssh jetson@192.168.1.100

# Export to TensorRT FP16
python3 -c "
from ultralytics import YOLO
model = YOLO('models/best.pt')
model.export(format='engine', imgsz=640, half=True)
print('Export complete: models/best.engine')
"
```

### Step 2: Benchmark the Engine

```python
#!/usr/bin/env python3
"""Benchmark TensorRT engine on Jetson."""

from ultralytics import YOLO
import numpy as np
import time

model = YOLO('models/best.engine')

# Create a dummy image
dummy = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

# Warm up
for _ in range(10):
    model(dummy, verbose=False)

# Benchmark
times = []
for _ in range(100):
    t0 = time.perf_counter()
    model(dummy, verbose=False)
    t1 = time.perf_counter()
    times.append((t1 - t0) * 1000)

times = np.array(times)
print(f'Inference time: {times.mean():.1f} +/- {times.std():.1f} ms')
print(f'FPS: {1000 / times.mean():.0f}')
print(f'Min: {times.min():.1f} ms, Max: {times.max():.1f} ms')
```

### Step 3: Deploy as ROS 2 Node

Use the YOLOv8 inference node from `references/yolo-inference.md`, but point it to the
TensorRT engine:

```bash
ros2 run my_vision_pkg yolo_detector --ros-args \
    -p model_path:=/home/jetson/models/best.engine \
    -p confidence_threshold:=0.5 \
    -p input_topic:=/oak/rgb/image_raw \
    -p device:='cuda:0' \
    -p image_size:=640
```

### Step 4: Verify Performance

```bash
# In another terminal, monitor system resources
jtop

# Check inference FPS from node logs
ros2 topic hz /detections

# Check image output
ros2 run rqt_image_view rqt_image_view /detections/image
```

### Expected Performance (Jetson Orin NX 16GB, 15W Mode)

| Component | Metric |
|-----------|--------|
| YOLOv8s TensorRT FP16 inference | ~12ms per frame |
| Total pipeline (subscribe + preprocess + inference + publish) | ~18ms per frame |
| Effective FPS | ~55 FPS |
| GPU utilization | ~40% |
| RAM usage (ROS 2 + node + model) | ~4GB |
| Power consumption | ~12W |
| GPU temperature (with fan) | ~55C |

This leaves significant headroom for additional processing (depth, SLAM, navigation) on the
same Jetson.
