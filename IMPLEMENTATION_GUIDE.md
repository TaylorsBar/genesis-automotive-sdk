# Genesis Automotive SDK - Implementation Guide

**Repository**: https://github.com/TaylorsBar/genesis-automotive-sdk  
**Created**: November 15, 2025  
**Author**: Taylor Berger (@TaylorsBar) | CartelWorx / KC Speed Lab
IMPLEMENTATION_GUIDE.md

---

## 📚 Table of Contents

1. [Quick Start](#quick-start)
2. [Project Structure](#project-structure)
3. [Core C++ Implementation](#core-cpp-implementation)
4. [iOS Platform Integration](#ios-platform-integration)
5. [Build System Setup](#build-system-setup)
6. [Development Workflow](#development-workflow)
7. [Testing & Validation](#testing--validation)
8. [Deployment Strategy](#deployment-strategy)

---

## Quick Start

### Prerequisites

```bash
# macOS
brew install cmake eigen opencv

# Ubuntu/Debian
sudo apt-get install cmake libeigen3-dev libopencv-dev

# Windows
vcpkg install eigen3 opencv4
```

### Clone and Build

```bash
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git
cd genesis-automotive-sdk

# Create build directory
mkdir build && cd build

# Configure
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
make -j$(nproc)

# Run tests
ctest --output-on-failure
```

---

## Project Structure

Create this directory structure locally

AutomotiveSDK/
├── Core/                    # Cross-platform C++ engine
│   ├── Fusion/
│   │   ├── GenesisEKF.hpp
│   │   ├── GenesisEKF.cpp
│   │   ├── SensorFusion.hpp
│   │   └── SensorFusion.cpp
│   ├── Vision/
│   │   ├── OpticalFlow.hpp
│   │   ├── OpticalFlow.cpp
│   │   ├── FeatureTracker.hpp
│   │   └── QualityAssessor.hpp
│   ├── Diagnostics/
│   │   ├── OBDProtocol.hpp
│   │   ├── CANDecoder.hpp
│   │   └── DTCDatabase.hpp
│   └── AI/
│       ├── ShiftDetection.hpp
│       └── AnomalyDetection.hpp
├── Platform/
│   ├── iOS/
│   │   ├── GenesisSDK.swift
│   │   ├── SensorManager.swift
│   │   ├── ELM327Manager.swift
│   │   └── Bridge/CoreBridge.mm
│   └── Android/
│       ├── GenesisSDK.kt
│       └── jni/CoreBridge.cpp
├── Documentation/
├── Examples/
├── Tests/
└── CMakeLists.txt
```

---

## Core C++ Implementation

### GenesisEKF.hpp - Extended Kalman Filter

**Location**: `Core/Fusion/GenesisEKF.hpp`

**Source**: Google AI Studio session - Complete 12-state EKF implementation

**Key Features**:
- 12-dimensional state vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, bias_ax, bias_ay, bias_az]
- Adaptive noise tuning based on motion intensity
- WGS84 ↔ ECEF ↔ NED coordinate transforms
- Multi-tier fallback logic (Tier 1-4)

**Reference**: See your Google AI Studio file for complete implementation at:
https://aistudio.google.com/prompts/1W2-c8JXOj9QnUrafPQrhs_MxbLRZIV16

### Key Code Sections to Transfer:

1. **State Vector Definition** (lines 14-50 in AI Studio)
2. **EKF Prediction Step** (lines 150-250)
3. **GNSS Update Logic** (lines 300-400)
4. **Vision Integration** (lines 450-500)
5. **Coordinate Transforms** (lines 600-700)

---

## iOS Platform Integration

### SensorManager.swift - High-Frequency Capture

**Location**: `Platform/iOS/SensorManager.swift`

**Implementation from AI Studio**:
- CoreLocation for GNSS (10-25 Hz)
- CoreMotion for IMU (100+ Hz)
- CADisplayLink for prediction timing
- Tiered fallback management

**Key Functions**:
```swift
func startHighPerformanceCapture()
@objc func runPredictionStep()  // Called at 60-120 Hz
func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation])
```

---

## Build System Setup

### CMakeLists.txt

Create in repository root:

```cmake
cmake_minimum_required(VERSION 3.15)
project(GenesisAutomotiveSDK VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4.0 REQUIRED)

# Core library
add_library(genesis_core
    Core/Fusion/GenesisEKF.cpp
    Core/Vision/OpticalFlow.cpp
    Core/Diagnostics/CANDecoder.cpp
)

target_include_directories(genesis_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(genesis_core PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

# Tests
enable_testing()
add_subdirectory(Tests)
```

---

## Development Workflow

### Day 1: Setup
```bash
# 1. Clone and setup
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git
cd genesis-automotive-sdk
mkdir -p Core/Fusion Core/Vision Core/Diagnostics Platform/iOS

# 2. Copy code from AI Studio
# - Open your AI Studio session
# - Copy GenesisEKF.hpp and GenesisEKF.cpp
# - Paste into Core/Fusion/

# 3. Initial commit
git add .
git commit -m "feat: Add core EKF implementation"
git push origin main
```

### Day 2-3: iOS Integration
```bash
# 1. Create iOS example project
cd Examples/iOS
open -a Xcode GenesisSDKExample.xcodeproj

# 2. Add Swift SDK files from AI Studio
# - SensorManager.swift
# - CoreBridge.mm
# - GenesisSDK.swift

# 3. Test with iPhone
# Build and run on physical device for sensor access
```

### Day 4-7: Testing & Validation
```bash
# 1. Unit tests for EKF
cd Tests
mkdir EKFTests
# Add Google Test framework tests

# 2. Field testing
# Take app for drive test
# Compare against Dragy device
# Record accuracy metrics
```

---

## Testing & Validation

### Unit Test Example

**File**: `Tests/EKFTests/test_ekf_prediction.cpp`

```cpp
#include <gtest/gtest.h>
#include "GenesisEKF.hpp"

TEST(EKFTest, PredictionStep) {
    CartelWorx::GenesisEKF ekf;
    
    // Initialize with known position
    CartelWorx::GNSSMeasurement initial;
    initial.latitude = -37.7749;
    initial.longitude = 175.2834;
    initial.altitude = 50.0;
    ekf.initialize(initial);
    
    // Create IMU measurement
    CartelWorx::IMUMeasurement imu;
    imu.timestamp = 0.01;
    imu.acceleration = Eigen::Vector3d(0.1, 0.0, 9.81);
    imu.angular_velocity = Eigen::Vector3d::Zero();
    
    // Predict
    ekf.predict(imu, 0.01);
    
    // Verify state update
    auto state = ekf.getState();
    EXPECT_GT(state(3), 0.0);  // vx should increase
}
```

---

## Deployment Strategy

### Phase 1: Local Development (Week 1)
- ✅ Repository created
- ⏳ Core C++ code uploaded
- ⏳ iOS example app functional
- ⏳ Basic unit tests passing

### Phase 2: Beta Release (Weeks 2-4)
- ⏳ TestFlight distribution
- ⏳ 5-10 workshop beta testers
- ⏳ Field validation vs. Dragy
- ⏳ Performance optimization

### Phase 3: Production (Q1 2025)
- ⏳ CocoaPods/SPM packages
- ⏳ Android SDK release
- ⏳ 40+ workshop deployment
- ⏳ Cloud dashboard integration

---

## Quick Reference Commands

```bash
# Clone repository
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git

# Build C++ core
mkdir build && cd build
cmake .. && make

# Run tests
ctest --verbose

# Create new branch for feature
git checkout -b feature/optical-flow-optimization

# Commit changes
git add .
git commit -m "feat: Optimize optical flow tracking"
git push origin feature/optical-flow-optimization
```

---

## Support Resources

- **Repository**: https://github.com/TaylorsBar/genesis-automotive-sdk
- **AI Studio Session**: https://aistudio.google.com/prompts/1W2-c8JXOj9QnUrafPQrhs_MxbLRZIV16
- **Documentation**: See `Documentation/` folder
- **Issues**: https://github.com/TaylorsBar/genesis-automotive-sdk/issues

---

## Next Immediate Steps

1. **Transfer code from AI Studio** (2-3 hours)
   - Copy all C++ files to Core/
   - Copy all iOS files to Platform/iOS/
   - Commit to repository

2. **Create CMakeLists.txt** (30 minutes)
   - Use template above
   - Test local build

3. **Build iOS example** (1-2 hours)
   - Create Xcode project
   - Integrate SDK
   - Test on device

4. **Field test** (1 day)
   - Drive test with app
   - Compare vs. Dragy
   - Document accuracy

**Total time to working prototype**: 2-3 days focused work

---

**Built by CartelWorx / KC Speed Lab**  
**Powering the next generation of automotive intelligence**:

```bash
Genesis
