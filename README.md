# Genesis Automotive SDK

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-iOS%20%7C%20Android-lightgrey.svg)](https://github.com/TaylorsBar/genesis-automotive-sdk)
[![Language](https://img.shields.io/badge/language-C%2B%2B17%20%7C%20Swift%20%7C%20Kotlin-orange.svg)](https://github.com/TaylorsBar/genesis-automotive-sdk)

**Production-grade automotive telemetry SDK with advanced sensor fusion, OBD-II diagnostics, and real-time performance analysis**

[Features](#-features) • [Architecture](#-architecture) • [Installation](#-installation) • [Usage](#-usage) • [Documentation](#-documentation) • [Contributing](#-contributing)

</div>

---

## 🚗 Overview

Genesis Automotive SDK is a comprehensive, production-ready telemetry platform designed for high-performance automotive applications. Built on a cross-platform C++ core with native iOS and Android SDKs, it provides:

- **Advanced Sensor Fusion**: 12-state Extended Kalman Filter (EKF) fusing GNSS, IMU, Computer Vision, and OBD-II data
- **Multi-Tier Fallback System**: Graceful degradation from full-fidelity (±0.01s) to dead reckoning modes
- **On-Device AI**: CoreML/TensorFlow Lite models for gear shift detection and anomaly recognition  
- **OBD-II Diagnostics**: Universal protocol support with 10,000+ DTC definitions
- **Real-Time Performance**: Sub-10ms latency for competitive motorsports applications

## ✨ Features

### Sensor Fusion Engine
- ✅ **Extended Kalman Filter (EKF)**: 12-dimensional state vector with IMU bias estimation
- ✅ **GNSS Integration**: WGS84 coordinate transforms with adaptive noise tuning
- ✅ **Computer Vision**: Lucas-Kanade optical flow with RANSAC outlier rejection
- ✅ **IMU Processing**: High-frequency (100+ Hz) accelerometer and gyroscope integration
- ✅ **Barometric Altitude**: Pressure-based altitude compensation

### Tiered Fallback System
| Tier | Mode | Accuracy | Sensors Active |
|------|------|----------|----------------|
| **Tier 1** | Full Fidelity | ±0.01s | GNSS + IMU + Vision + OBD |
| **Tier 2** | Vision Degraded | ±0.05s | GNSS + IMU + OBD |
| **Tier 3** | Dead Reckoning | ±0.2s | IMU + OBD only |
| **Tier 4** | Emergency | Last known | Position hold + warning |

### OBD-II Diagnostics
- ✅ ELM327 Bluetooth/WiFi support
- ✅ CAN bus decoding (11-bit & 29-bit identifiers)
- ✅ 10,000+ manufacturer-specific DTCs
- ✅ Real-time PID monitoring (RPM, MAF, throttle position, etc.)
- ✅ Freeze frame data capture

### On-Device Machine Learning
- ✅ Gear shift detection (CoreML for iOS, TensorFlow Lite for Android)
- ✅ Launch detection and traction loss identification
- ✅ Anomaly detection for predictive maintenance
- ✅ Sub-50ms inference latency

## 🏗️ Architecture

```
GenesisAutomotiveSDK/
├── Core/                    # Cross-platform C++ engine
│   ├── Fusion/
│   │   ├── GenesisEKF.hpp/.cpp          # Extended Kalman Filter
│   │   ├── SensorFusion.hpp/.cpp        # Multi-sensor integration
│   │   └── StateEstimator.hpp/.cpp      # State management
│   ├── Vision/
│   │   ├── OpticalFlow.hpp/.cpp         # Lucas-Kanade + RANSAC
│   │   ├── FeatureTracker.hpp/.cpp      # Feature point tracking
│   │   └── QualityAssessor.hpp/.cpp     # Vision quality metrics
│   ├── Diagnostics/
│   │   ├── OBDProtocol.hpp/.cpp         # Universal OBD-II
│   │   ├── CANDecoder.hpp/.cpp          # CAN frame parser
│   │   ├── DTCDatabase.hpp/.cpp         # 10,000+ DTC definitions
│   │   └── ManufacturerPIDs.hpp/.cpp    # OEM-specific PIDs
│   └── AI/
│       ├── ShiftDetection.hpp/.cpp      # Gear shift ML models
│       └── AnomalyDetection.hpp/.cpp    # Predictive maintenance
│
├── Platform/
│   ├── iOS/
│   │   ├── GenesisSDK.swift             # Public Swift API
│   │   ├── SensorManager.swift          # CoreLocation/CoreMotion
│   │   ├── ELM327Manager.swift          # Bluetooth OBD-II
│   │   └── Bridge/
│   │       └── CoreBridge.mm            # Objective-C++ bridge
│   └── Android/
│       ├── GenesisSDK.kt                # Public Kotlin API
│       ├── SensorManager.kt             # FusedLocation/SensorManager
│       ├── ELM327Manager.kt             # Bluetooth OBD-II
│       └── jni/
│           └── CoreBridge.cpp           # JNI bridge
│
├── Cloud/                   # Optional cloud services
│   ├── FastAPI/
│   │   ├── api.py                       # Telemetry ingestion
│   │   ├── models.py                    # Pydantic schemas
│   │   └── ai_engine.py                 # LLM co-pilot
│   └── Firebase/
│       ├── functions/                   # Cloud Functions
│       ├── firestore.rules              # Security rules
│       └── storage.rules
│
└── Documentation/
    ├── API_Reference.md
    ├── Integration_Guide.md
    └── Performance_Tuning.md
```

## 📦 Installation

### iOS (CocoaPods)

```ruby
# Podfile
pod 'GenesisAutomotiveSDK', '~> 1.0'
```

### iOS (Swift Package Manager)

```swift
dependencies: [
    .package(url: "https://github.com/TaylorsBar/genesis-automotive-sdk.git", from: "1.0.0")
]
```

### Android (Gradle)

```gradle
dependencies {
    implementation 'com.cartelworx:genesis-sdk:1.0.0'
}
```

### Build from Source

```bash
# Clone repository
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git
cd genesis-automotive-sdk

# Build C++ core
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run tests
ctest --output-on-failure
```

## 🚀 Usage

### iOS (Swift)

```swift
import GenesisSDK

// Initialize SDK
let genesis = GenesisSensorManager()

// Start high-performance capture
genesis.startHighPerformanceCapture()

// Access fusion metrics
let metrics = genesis.coreBridge.getFusionMetrics()
print("Current Tier: \(metrics.current_tier)")
print("Position Uncertainty: \(metrics.position_uncertainty_m)m")

// Get state vector
let state = genesis.coreBridge.getStateVector()
let velocity = state.segment(3, 3) // [vx, vy, vz]
print("Velocity: \(velocity.norm()) m/s")
```

### Android (Kotlin)

```kotlin
import com.cartelworx.genesis.GenesisSDK

// Initialize SDK
val genesis = GenesisSensorManager(applicationContext)

// Start capture
genesis.startHighPerformanceCapture()

// Subscribe to updates
genesis.fusionMetrics.observe(this) { metrics ->
    Log.d("Genesis", "Tier: ${metrics.currentTier}")
    Log.d("Genesis", "Position Uncertainty: ${metrics.positionUncertaintyM}m")
}
```

### C++ (Direct)

```cpp
#include "GenesisEKF.hpp"

using namespace CartelWorx;

// Initialize EKF
GenesisEKF ekf;

// Provide initial GNSS lock
GNSSMeasurement initial_gnss;
initial_gnss.latitude = -37.7749;
initial_gnss.longitude = 175.2834;
initial_gnss.altitude = 50.0;
ekf.initialize(initial_gnss);

// High-frequency prediction loop (100+ Hz)
while (running) {
    IMUMeasurement imu = readIMU();
    ekf.predict(imu, timestamp);
    
    // Update with GNSS when available (1-10 Hz)
    if (gnss_available) {
        ekf.updateGNSS(gnss_data);
    }
    
    // Update with vision (30+ Hz)
    if (vision_available) {
        ekf.updateVision(vision_data);
    }
    
    // Get fused state
    StateVector state = ekf.getState();
    FusionMetrics metrics = ekf.getMetrics();
}
```

## 📚 Documentation

- [API Reference](./Documentation/API_Reference.md) - Complete API documentation
- [Integration Guide](./Documentation/Integration_Guide.md) - Step-by-step integration instructions  
- [Performance Tuning](./Documentation/Performance_Tuning.md) - Optimization guidelines
- [Examples](./Examples/) - Sample projects for iOS, Android, and C++

## 🎯 Use Cases

- **Motorsports Timing**: Professional-grade lap timing and performance analysis
- **Fleet Management**: Real-time vehicle tracking and diagnostics
- **Autonomous Vehicles**: Sensor fusion and state estimation for L2+ ADAS
- **Insurance Telematics**: Usage-based insurance (UBI) data collection
- **Performance Tuning**: Dyno-less power measurement and diagnostics

## 🔬 Technical Specifications

### Sensor Fusion Performance
- **Latency**: <10ms end-to-end (sensor to state output)
- **Update Rate**: 100+ Hz (IMU prediction), 10 Hz (GNSS update), 30 Hz (vision)
- **Accuracy**: ±0.01s (Tier 1), ±0.05s (Tier 2), ±0.2s (Tier 3)
- **Position Drift**: <1m per 30 seconds (dead reckoning mode)

### System Requirements
#### iOS
- iOS 14.0+
- iPhone 8 or newer (A11 Bionic+ for on-device ML)
- GPS + GLONASS/Galileo receiver
- CoreMotion + CoreLocation access

#### Android  
- Android 8.0 (API 26)+
- ARMv8-A or newer
- GPS + GLONASS/Galileo receiver
- Location + Sensor permissions

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### Development Setup

1. Install dependencies:
   - C++17 compiler (GCC 7+, Clang 5+, MSVC 2017+)
   - CMake 3.15+
   - Eigen 3.3+
   - OpenCV 4.0+ (for vision module)

2. Build and test:
   ```bash
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Debug ..
   make -j$(nproc)
   ctest --verbose
   ```

3. Format code:
   ```bash
   clang-format -i Core/**/*.{hpp,cpp}
   ```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Dragy Performance**: Inspiration for high-accuracy GPS-based timing
- **RTKLIB**: GNSS positioning algorithms
- **Eigen**: Linear algebra library
- **OpenCV**: Computer vision algorithms

## 📞 Support

- **Documentation**: [https://genesis-sdk.cartelworx.com](https://genesis-sdk.cartelworx.com)
- **Issues**: [GitHub Issues](https://github.com/TaylorsBar/genesis-automotive-sdk/issues)
- **Email**: support@cartelworx.com
- **Discord**: [CartelWorx Community](https://discord.gg/cartelworx)

## 🗺️ Roadmap

- [ ] **Q1 2025**: Production 1.0 release
- [ ] **Q2 2025**: Web dashboard for telemetry visualization
- [ ] **Q3 2025**: Real-time AI co-pilot integration (Gemini Live API)
- [ ] **Q4 2025**: Autonomous vehicle sensor suite expansion

---

<div align="center">

**Built with ❤️ by [CartelWorx](https://cartelworx.com) | [KC Speed Lab](https://kcspeedlab.co.nz)**

Powering the next generation of automotive intelligence

</div>
