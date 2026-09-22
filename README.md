# HandLandmark Robotic Hand Control System
![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)
![Embedded Linux](https://img.shields.io/badge/Embedded_Linux-FCC624?style=flat&logo=linux&logoColor=black)
![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi-A22846?style=flat&logo=raspberrypi&logoColor=white)
![TensorFlow Lite](https://img.shields.io/badge/TensorFlow_Lite-FF6F00?style=flat&logo=tensorflow&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat&logo=opencv&logoColor=white)
![Robotics](https://img.shields.io/badge/Hardware-Robotic_Hand-E34F26?style=flat)

![HandLandmark System Demonstration](./assets/demo.gif)

A real-time, AI-driven robotic hand control system that captures human hand gestures via an IP camera, processes landmarks using TensorFlow Lite, and translates these gestures into physical servo actuations via I2C.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Prerequisites and Dependencies](#prerequisites-and-dependencies)
- [Installation and Environment Configuration](#installation-and-environment-configuration)
- [Usage](#usage)
- [Testing and Deployment Protocols](#testing-and-deployment-protocols)

## Architecture Overview

The system is built as a multithreaded C++17 application composed of the following components:

![System Architecture Diagram](./assets/architecture_diagram.png)

*Data flow: IP Camera → OpenCV → TFLite Interpreter → Angle Calculation → Thread Mutex → I2C Driver → PCA9685 → Servos*

### Vision & AI Thread
- Ingests MJPEG frames from a hardcoded IP camera
- Preprocesses frames (RGB conversion, 224x224 resize)
- Runs inference using a TFLite FlatBuffer model
- Extracts 21 3D hand landmarks and calculates finger joint angles via vector dot/cross products
- Hosts an MJPEG debug stream showing the skeletal overlay

### Control Thread
- Continuously polls the global target hand state
- Applies an exponential moving average (EMA) smoothing algorithm (`ALPHA = 0.2f`) to prevent servo jitter

### Hardware Interface
- Uses the `<linux/i2c-dev.h>` API on a Raspberry Pi 4B to communicate with a PCA9685 PWM driver over I2C
- Sends calculated pulse widths to control 5 dedicated finger servos

### Concurrency
- Utilizes `std::mutex` (`state_mutex`) to safely share the `HandState` structure between the asynchronous vision and control loops

## Prerequisites and Dependencies

### Build System & Compiler
- CMake (minimum version 3.10)
- C++ compiler with C++17 standard support

### Libraries & SDKs
- **OpenCV** — required for frame ingestion, preprocessing, and drawing skeletal overlays
- **TensorFlow Lite (C++ API)** — required for model inference; must be built from source (specifically linked against paths `/home/omar/tflite_build` and `/home/omar/tensorflow`)
- **libgpiod (gpiodcxx)** — C++ bindings for GPIO interactions
- **pthread / dl** — core Linux threading and dynamic linking libraries
- **nadjieb/mjpeg_streamer** — header-only library for broadcasting the annotated debug stream over HTTP

### Hardware & OS Requirements
- Raspberry Pi 4B running a Linux environment (e.g., Ubuntu or Raspberry Pi OS) with I2C enabled (`/dev/i2c-1`)
- PCA9685 16-channel PWM servo driver (default address: `0x40`)
- IP camera serving a video stream
- Pre-trained TFLite model: `hand_landmark_full.tflite`

![Hardware Wiring Setup](./assets/hardware_wiring.jpg)

## Installation and Environment Configuration

1. **Clone and prepare directory**

   Ensure all source files (`h_landmark.cpp`, `pca9685.hpp`, `CMakeLists.txt`) and the `hand_landmark_full.tflite` model are in the same directory on the Raspberry Pi 4B.

2. **TFLite installation**

   The TFLite library must be installed locally. Run the following inside the `~/tflite_build` directory:

   ```bash
   sudo cmake --install .
   ```

3. **CMake configuration & TFLite dependency fix**

   The `CMakeLists.txt` file specifically targets static library dependencies found in `/home/omar/tflite_build/_deps/` and `/pthreadpool/`.

   **Critical fix applied:** To avoid build conflicts, the configuration excludes all `fft2d-build` libraries from the automatic dependency list. It then manually appends only the two specific `fft2d` archives required by TFLite: `libfft2d_fftsg.a` and `libfft2d_fftsg2d.a`.

4. **Build the executable**

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

## Usage

1. **Network configuration**

   Ensure the target IP camera stream matches the hardcoded `IP_STREAM_URL`:

   ```
   http://[USER]:[PASSWORD]@[IP_ADDRESS]:[PORT]/video
   ```

2. **Execution**

   Run the compiled binary from the directory containing the model file:

   ```bash
   ./h_landmark
   ```

3. **Debug stream**

   While the system is running, access the live annotated skeletal feed by navigating a web browser to:

   ```
   http://<raspberry-pi-ip>:8080/stream
   ```

   ![Annotated Web Stream Interface](./assets/stream_screenshot.jpg)

4. **Terminal output**

   The CLI will actively print initialization statuses (`AI Init Success`, `Cam Init Success`) followed by a carriage-returned live feed of calculated servo angles for the Thumb (T), Index (I), Middle (M), Ring+Pinky (R+P), and Palm (TOP).

## Testing and Deployment Protocols

### I2C Bus Verification

Before running, verify the PCA9685 is detected on the Raspberry Pi 4B's bus 1:

```bash
i2cdetect -y 1
```

Look for address `0x40` in the output grid.

### Permissions

The executing user must have read/write access to `/dev/i2c-1`. Add the user to the `i2c` group to avoid running the binary as root:

```bash
sudo usermod -aG i2c $USER
```

### Servo Calibration

The `pca9685.hpp` driver maps 0–180 degree angles to a PWM pulse range of 150–570 (at 50Hz). Physical servos should be calibrated to match this theoretical pulse-width mapping.

### Network Firewall

Port `8080` (TCP) must be open on the Raspberry Pi 4B to allow external access to the MJPEG debug stream.
