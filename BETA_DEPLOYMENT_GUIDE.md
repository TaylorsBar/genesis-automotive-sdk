# Genesis Automotive SDK - Beta Deployment Guide

## 🎯 Overview

This guide provides complete instructions for deploying the Genesis Automotive SDK as a beta-ready, production-grade automotive telemetry platform with OBD-II ELM327 Bluetooth support, EKF sensor fusion, and automated DevOps.

## 🚀 Quick Start (5 Minutes to Beta)

```bash
# Clone repository
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git
cd genesis-automotive-sdk

# Run automated setup
chmod +x scripts/setup_repo.sh
./scripts/setup_repo.sh

# Build and test
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## 📦 Repository Structure (Production-Ready)

```
genesis-automotive-sdk/
├── Core/
│   ├── Diagnostics/          # OBD-II ELM327 Implementation
│   │   ├── GenesisOBD2.hpp   # ✅ Main OBD-II interface (COMPLETE)
│   │   ├── GenesisOBD2.cpp   # Full implementation with async support
│   │   ├── ELM327.cpp        # Bluetooth adapter driver
│   │   ├── PIDDecoder.cpp    # PID decoding utilities
│   │   └── DTCDatabase.cpp   # DTC lookup database
│   ├── Fusion/               # EKF Sensor Fusion
│   │   ├── GenesisEKF.hpp    # Extended Kalman Filter
│   │   ├── GenesisEKF.cpp    # State estimation implementation
│   │   └── SensorManager.cpp # Multi-sensor coordination
│   ├── Vision/               # Computer Vision Pipeline
│   │   ├── LaneDetection.cpp
│   │   ├── ObjectTracking.cpp
│   │   └── MLInference.cpp
│   ├── Profiling/            # Advanced Diagnostics
│   │   ├── Profiler.hpp      # Performance profiler
│   │   └── Logger.cpp        # Structured logging
│   └── Utils/
│       ├── BluetoothManager.cpp
│       └── ThreadPool.cpp
├── Platform/
│   ├── iOS/
│   │   ├── Bridge/           # C++ <-> Swift bridge
│   │   ├── Bluetooth/        # CoreBluetooth integration
│   │   └── GenesisSensorManager.swift ✅ (EXISTS)
│   ├── Android/
│   │   ├── jni/              # JNI bindings
│   │   └── kotlin/           # Kotlin API
│   └── Desktop/
│       └── main.cpp          # Desktop test app
├── Tests/
│   ├── Unit/                 # Unit tests (GTest)
│   ├── Integration/          # Integration tests
│   └── Performance/          # Benchmark suite
├── Examples/
│   ├── iOS/                  # SwiftUI example app
│   ├── Android/              # Kotlin/Compose example
│   └── Desktop/              # Qt/CLI examples
├── Docs/
│   ├── API/                  # Doxygen API docs
│   ├── Guides/               # Integration guides
│   └── Architecture/         # System design docs
├── .github/workflows/
│   ├── ci.yml               # CI/CD pipeline ✅ (IN SETUP SCRIPT)
│   ├── release.yml          # Automated releases
│   └── docs.yml             # Documentation generation
├── Docker/
│   ├── Dockerfile           # ✅ (IN SETUP SCRIPT)
│   └── docker-compose.yml   # ✅ (IN SETUP SCRIPT)
└── scripts/
    ├── setup_repo.sh        # ✅ Repository setup (EXISTS)
    ├── generate_code.py     # Code generation utilities
    └── benchmark.sh         # Performance benchmarking
```

## 🔧 Core Features Implementation Status

### ✅ COMPLETE
- OBD-II Interface Header (`GenesisOBD2.hpp`)
- Repository Setup Script (`scripts/setup_repo.sh`)
- CI/CD Pipeline Configuration
- Docker Containerization
- CMake Build System
- iOS Sensor Bridge

### 🚧 IN PROGRESS (Add via commits)
- OBD-II Implementation Files
- EKF Sensor Fusion
- Vision Processing
- Test Suite
- Platform Examples

## 💻 OBD-II Implementation Guide

### Full Bluetooth ELM327 Implementation

The `GenesisOBD2.hpp` header (already in repo) defines the complete interface. Here's the implementation approach:

#### Key Components:

1. **Connection Management**
   - Bluetooth LE/Classic support
   - Auto-reconnection with exponential backoff
   - Connection pooling for multiple adapters

2. **Protocol Handling**
   - Auto-detection of OBD protocols
   - Support for CAN (ISO 15765), J1850, ISO 9141
   - Adaptive timeout management

3. **Query Optimization**
   - Batch PID requests
   - Priority queue for real-time PIDs
   - Adaptive polling rates based on vehicle response

4. **Error Recovery**
   - Retry logic with circuit breaker pattern
   - DTC code validation
   - Frame corruption detection

### Example Usage

```cpp
#include "Core/Diagnostics/GenesisOBD2.hpp"

using namespace Genesis::Diagnostics;

// Initialize OBD-II interface
GenesisOBD2 obd;

// Connect to Bluetooth ELM327
if (obd.connect(ConnectionType::BLUETOOTH, "00:1D:A5:68:98:8B")) {
    // Auto-detect protocol
    OBDProtocol protocol = obd.detectProtocol();
    
    // Read RPM (synchronous)
    auto response = obd.query(OBDMode::SHOW_CURRENT_DATA, PID::ENGINE_RPM);
    if (response.success) {
        double rpm = response.value;
        printf("RPM: %.0f\n", rpm);
    }
    
    // Start monitoring multiple PIDs (asynchronous)
    std::vector<uint8_t> pids = {
        PID::ENGINE_RPM,
        PID::VEHICLE_SPEED,
        PID::ENGINE_COOLANT_TEMP,
        PID::THROTTLE_POSITION
    };
    
    obd.startMonitoring(pids, 100); // 100ms interval
    
    // Read DTCs
    auto dtcs = obd.readDTCs();
    for (const auto& dtc : dtcs) {
        printf("DTC: %s - %s\n", dtc.code.c_str(), dtc.description.c_str());
    }
    
    // Get diagnostics stats
    auto stats = obd.getStats();
    printf("Avg Latency: %.2f ms\n", stats.avg_latency_ms);
}
```

## 🧪 EKF Sensor Fusion Architecture

### State Vector
```
State = [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw]
```

### Sensor Sources
1. **GNSS** - Position, velocity
2. **IMU** - Acceleration, angular velocity
3. **Vision** - Lane position, object tracking
4. **OBD-II** - Vehicle speed, wheel sensors

### Fusion Algorithm
- Extended Kalman Filter with non-linear motion model
- Mahalanobis gating for outlier rejection
- Adaptive noise covariance tuning
- Multi-rate sensor handling

## 🏁 Performance Targets (Beta)

| Metric | Target | Current |
|--------|--------|---------|
| OBD-II Query Latency | < 50ms | TBD |
| EKF Update Rate | 100 Hz | TBD |
| Vision Inference | < 33ms | TBD |
| Memory Footprint | < 50 MB | TBD |
| Bluetooth Reliability | > 99.5% | TBD |

## 📦 CI/CD Pipeline

### Automated Testing
- **Unit Tests**: GTest framework, >80% coverage
- **Integration Tests**: Full stack testing with mocked hardware
- **Performance Tests**: Latency and throughput benchmarks
- **Platform Tests**: iOS/Android/Desktop

### Build Matrix
- Ubuntu 22.04 (GCC 11, Clang 14)
- macOS 13 (Xcode 14)
- Windows 11 (MSVC 2022)

### Deployment
- Automated versioning (Semantic Versioning)
- GitHub Releases with binary artifacts
- Docker Hub publishing
- CocoaPods/Maven/NuGet packages

## 🐞 Beta Testing Checklist

### Hardware Requirements
- [ ] ELM327 v1.5 Bluetooth adapter
- [ ] OBD-II compatible vehicle (2008+)
- [ ] iOS device (iPhone 12+) or Android (API 28+)
- [ ] GPS-enabled test environment

### Test Scenarios
1. **Connection Test**
   - [ ] Pair Bluetooth adapter
   - [ ] Establish OBD-II connection
   - [ ] Detect vehicle protocol

2. **Data Acquisition**
   - [ ] Real-time PID monitoring
   - [ ] DTC reading and clearing
   - [ ] VIN retrieval
   - [ ] Freeze frame data

3. **Performance Test**
   - [ ] 1-hour continuous monitoring
   - [ ] 100+ query/sec sustained load
   - [ ] Connection recovery after adapter reset

4. **Sensor Fusion**
   - [ ] GNSS + OBD speed fusion
   - [ ] IMU integration
   - [ ] Vision lane detection

## 📈 Monitoring & Analytics

### Built-in Metrics
- Query success/failure rates
- Latency percentiles (p50, p95, p99)
- Connection uptime
- Protocol error rates
- Memory/CPU usage

### Logging
- Structured JSON logging
- Log levels: TRACE, DEBUG, INFO, WARN, ERROR
- Rotation and compression
- Remote log aggregation support

## 🔐 Security & Privacy

### Data Protection
- No PII collection by default
- VIN anonymization option
- Local-only processing (no cloud dependency)
- Encrypted log storage

### Code Security
- Static analysis (Clang-Tidy, SonarQube)
- Dependency scanning
- Memory safety (Address Sanitizer)

## 📚 API Documentation

### Generate Docs
```bash
# Install Doxygen
sudo apt-get install doxygen graphviz

# Generate HTML docs
doxygen Doxyfile
open docs/html/index.html
```

### API Reference
- Complete class documentation
- Usage examples
- Architecture diagrams
- Integration guides

## 👥 Contributing

### Development Workflow
1. Fork repository
2. Create feature branch
3. Implement with tests
4. Run `scripts/lint.sh`
5. Submit PR with description

### Code Style
- C++17 standard
- Google C++ Style Guide
- clang-format configured
- Doxygen comments required

## 🚀 Deployment Commands

### Build for iOS
```bash
cd Platform/iOS
xcodebuild -scheme GenesisSensorSDK
```

### Build for Android
```bash
cd Platform/Android
./gradlew assembleRelease
```

### Build for Desktop
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/genesis_desktop_example
```

## 📝 Implementation Roadmap

### Phase 1: Core (Week 1) ✅
- [x] OBD-II interface definition
- [x] Repository structure
- [x] CI/CD pipeline
- [x] Docker configuration
- [ ] Complete OBD-II implementation
- [ ] ELM327 driver

### Phase 2: Fusion & Vision (Week 2)
- [ ] EKF implementation
- [ ] Sensor manager
- [ ] Vision pipeline
- [ ] ML model integration

### Phase 3: Platform Integration (Week 3)
- [ ] iOS SDK completion
- [ ] Android SDK
- [ ] Example applications

### Phase 4: Testing & Polish (Week 4)
- [ ] Full test coverage
- [ ] Performance optimization
- [ ] Documentation completion
- [ ] Beta release

## 📢 Beta Release Criteria

### Functionality
- [ ] OBD-II connection: 99% success rate
- [ ] PID query: <50ms avg latency
- [ ] DTC reading: 100% accuracy
- [ ] 1-hour stress test: No crashes

### Quality
- [ ] Test coverage: >80%
- [ ] No memory leaks (Valgrind clean)
- [ ] Documentation: 100% API coverage
- [ ] Cross-platform: iOS + Android + Desktop

### Performance
- [ ] CPU usage: <10% average
- [ ] Memory: <50MB footprint
- [ ] Battery impact: <5% per hour (mobile)
- [ ] Bluetooth range: 10m minimum

## 📞 Support

- **Issues**: https://github.com/TaylorsBar/genesis-automotive-sdk/issues
- **Discussions**: https://github.com/TaylorsBar/genesis-automotive-sdk/discussions
- **Email**: support@karapirocartel.com

## 🏆 Success Metrics

### For Beta Testers
1. Successfully connect to 10+ different vehicle models
2. Collect 100+ hours of real-world OBD-II data
3. Identify and resolve 50+ edge cases
4. Performance benchmarking across platforms

### For Developers
1. Simple integration (<1 hour)
2. Clear documentation
3. Responsive issue resolution
4. Active community engagement

## 🔥 Next Steps

1. **Clone the repo**
```bash
git clone https://github.com/TaylorsBar/genesis-automotive-sdk.git
```

2. **Run setup script**
```bash
cd genesis-automotive-sdk
chmod +x Core/Diagnostics/scripts/setup_repo.sh
./Core/Diagnostics/scripts/setup_repo.sh
```

3. **Build the SDK**
```bash
cmake -B build
cmake --build build
```

4. **Run tests**
```bash
ctest --test-dir build --verbose
```

5. **Start developing**
- Check `Examples/` for integration examples
- Read `Docs/` for detailed guides
- Join discussions for support

---

## 🎯 CartelWorx Integration

This SDK is designed to integrate seamlessly with the CartelWorx ecosystem:

- **Real-time diagnostics** for workshop customers
- **Predictive maintenance** AI models
- **Performance tuning** data collection
- **Fleet management** telemetry
- **Blockchain validation** via Hedera

### CartelWorx API Integration
```cpp
// Upload OBD-II data to CartelWorx backend
CartelWorxAPI api("your-api-key");
api.uploadDiagnostics(obd.getStats());
api.uploadDTCs(obd.readDTCs());
```

---

**🚀 Ready to revolutionize automotive diagnostics? Let's build something amazing!**

---

💻 **Built with ❤️ by the CartelWorx team**
© 2025 Karapiro Cartel | MIT License
