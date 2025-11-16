//
// GenesisOBD2.hpp
// Genesis Automotive SDK - OBD-II ELM327 Diagnostics
//
// Production-grade OBD-II interface with ELM327 support
// Full Bluetooth/WiFi/USB support with async operations
//

#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

namespace Genesis {
namespace Diagnostics {

// OBD-II Protocol Types
enum class OBDProtocol {
    AUTO = 0,
    SAE_J1850_PWM = 1,
    SAE_J1850_VPW = 2,
    ISO_9141_2 = 3,
    ISO_14230_4_KWP_5BAUD = 4,
    ISO_14230_4_KWP_FAST = 5,
    ISO_15765_4_CAN_11BIT_500K = 6,
    ISO_15765_4_CAN_29BIT_500K = 7,
    ISO_15765_4_CAN_11BIT_250K = 8,
    ISO_15765_4_CAN_29BIT_250K = 9,
    SAE_J1939_CAN = 10
};

// OBD-II Service Modes
enum class OBDMode {
    SHOW_CURRENT_DATA = 0x01,
    SHOW_FREEZE_FRAME = 0x02,
    SHOW_STORED_DTC = 0x03,
    CLEAR_DTC = 0x04,
    TEST_RESULTS_O2 = 0x05,
    TEST_RESULTS_OTHER = 0x06,
    SHOW_PENDING_DTC = 0x07,
    CONTROL_OPERATION = 0x08,
    REQUEST_VEHICLE_INFO = 0x09,
    PERMANENT_DTC = 0x0A
};

// Standard OBD-II PIDs
namespace PID {
    constexpr uint8_t PIDS_SUPPORTED_01_20 = 0x00;
    constexpr uint8_t MONITOR_STATUS = 0x01;
    constexpr uint8_t FREEZE_DTC = 0x02;
    constexpr uint8_t FUEL_SYSTEM_STATUS = 0x03;
    constexpr uint8_t ENGINE_LOAD = 0x04;
    constexpr uint8_t ENGINE_COOLANT_TEMP = 0x05;
    constexpr uint8_t SHORT_TERM_FUEL_TRIM_BANK1 = 0x06;
    constexpr uint8_t LONG_TERM_FUEL_TRIM_BANK1 = 0x07;
    constexpr uint8_t SHORT_TERM_FUEL_TRIM_BANK2 = 0x08;
    constexpr uint8_t LONG_TERM_FUEL_TRIM_BANK2 = 0x09;
    constexpr uint8_t FUEL_PRESSURE = 0x0A;
    constexpr uint8_t INTAKE_MAP = 0x0B;
    constexpr uint8_t ENGINE_RPM = 0x0C;
    constexpr uint8_t VEHICLE_SPEED = 0x0D;
    constexpr uint8_t TIMING_ADVANCE = 0x0E;
    constexpr uint8_t INTAKE_AIR_TEMP = 0x0F;
    constexpr uint8_t MAF_FLOW_RATE = 0x10;
    constexpr uint8_t THROTTLE_POSITION = 0x11;
    constexpr uint8_t OXYGEN_SENSORS_PRESENT = 0x13;
    constexpr uint8_t RUNTIME_SINCE_START = 0x1F;
    constexpr uint8_t DISTANCE_WITH_MIL = 0x21;
    constexpr uint8_t FUEL_RAIL_PRESSURE = 0x22;
    constexpr uint8_t FUEL_RAIL_GAUGE_PRESSURE = 0x23;
    constexpr uint8_t COMMANDED_EGR = 0x2C;
    constexpr uint8_t EGR_ERROR = 0x2D;
    constexpr uint8_t FUEL_LEVEL = 0x2F;
    constexpr uint8_t DISTANCE_SINCE_DTC_CLEAR = 0x31;
    constexpr uint8_t BAROMETRIC_PRESSURE = 0x33;
    constexpr uint8_t CATALYST_TEMP_BANK1_SENSOR1 = 0x3C;
    constexpr uint8_t CONTROL_MODULE_VOLTAGE = 0x42;
    constexpr uint8_t ABSOLUTE_LOAD_VALUE = 0x43;
    constexpr uint8_t COMMANDED_EQUIV_RATIO = 0x44;
    constexpr uint8_t RELATIVE_THROTTLE_POSITION = 0x45;
    constexpr uint8_t AMBIENT_AIR_TEMP = 0x46;
    constexpr uint8_t ACCELERATOR_PEDAL_D = 0x49;
    constexpr uint8_t ACCELERATOR_PEDAL_E = 0x4A;
    constexpr uint8_t ACCELERATOR_PEDAL_F = 0x4B;
    constexpr uint8_t COMMANDED_THROTTLE_ACTUATOR = 0x4C;
    constexpr uint8_t FUEL_TYPE = 0x51;
    constexpr uint8_t ETHANOL_FUEL_PERCENT = 0x52;
    constexpr uint8_t BOOST_PRESSURE_A = 0x70;
}

// DTC (Diagnostic Trouble Code) Structure
struct DTC {
    std::string code;
    std::string description;
    enum class Type { POWERTRAIN, CHASSIS, BODY, NETWORK } type;
    bool isPending;
    bool isConfirmed;
    std::chrono::system_clock::time_point timestamp;
};

// OBD Response Structure
struct OBDResponse {
    bool success;
    std::string raw;
    std::vector<uint8_t> data;
    double value;
    std::string unit;
    uint32_t latency_ms;
    std::string error;
};

// Connection Types
enum class ConnectionType {
    BLUETOOTH,
    WIFI,
    USB,
    SIMULATOR
};

// Connection Status
enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    INITIALIZING,
    READY,
    ERROR
};

// Callback Types
using OBDCallback = std::function<void(const OBDResponse&)>;
using DTCCallback = std::function<void(const std::vector<DTC>&)>;
using ConnectionCallback = std::function<void(ConnectionStatus)>;

// Main OBD-II Interface Class
class GenesisOBD2 {
public:
    GenesisOBD2();
    virtual ~GenesisOBD2();

    // Connection Management
    bool connect(ConnectionType type, const std::string& address);
    void disconnect();
    bool isConnected() const;
    ConnectionStatus getStatus() const;
    void setConnectionCallback(ConnectionCallback callback);

    // Protocol Configuration
    bool setProtocol(OBDProtocol protocol);
    OBDProtocol detectProtocol();
    OBDProtocol getCurrentProtocol() const;

    // Synchronous Operations
    OBDResponse query(OBDMode mode, uint8_t pid);
    std::vector<DTC> readDTCs();
    bool clearDTCs();
    std::string getVIN();
    std::map<std::string, std::string> getVehicleInfo();

    // Asynchronous Operations
    void queryAsync(OBDMode mode, uint8_t pid, OBDCallback callback);
    void readDTCsAsync(DTCCallback callback);
    void startMonitoring(const std::vector<uint8_t>& pids, uint32_t interval_ms);
    void stopMonitoring();

    // Batch Operations
    std::map<uint8_t, OBDResponse> queryMultiple(const std::vector<uint8_t>& pids);
    
    // Advanced Features
    bool getSupportedPIDs(std::vector<uint8_t>& pids);
    double getPIDUpdateRate(uint8_t pid);
    void setPriority(uint8_t pid, int priority);
    
    // ELM327 Specific Commands
    std::string sendRawCommand(const std::string& command);
    bool resetAdapter();
    std::string getAdapterVersion();
    bool setAdapterTimeout(uint32_t ms);
    bool enableHeaders(bool enable);
    bool enableEcho(bool enable);

    // Diagnostics & Profiling
    struct DiagnosticStats {
        uint32_t total_queries;
        uint32_t successful_queries;
        uint32_t failed_queries;
        uint32_t timeouts;
        double avg_latency_ms;
        double max_latency_ms;
        double min_latency_ms;
        std::chrono::system_clock::time_point session_start;
    };
    DiagnosticStats getStats() const;
    void resetStats();

    // Data Decoding Utilities
    static double decodePID(uint8_t pid, const std::vector<uint8_t>& data);
    static std::string decodeDTC(const std::string& raw);
    static std::string formatValue(double value, const std::string& unit);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// ELM327 Bluetooth Adapter Class
class ELM327Adapter {
public:
    explicit ELM327Adapter(const std::string& device_address);
    virtual ~ELM327Adapter();

    bool open();
    void close();
    bool isOpen() const;

    std::string sendCommand(const std::string& command, uint32_t timeout_ms = 5000);
    bool initializeAdapter();
    bool setProtocol(OBDProtocol protocol);
    std::vector<std::string> readResponse(uint32_t timeout_ms = 1000);

    void setEcho(bool enable);
    void setLinefeed(bool enable);
    void setHeaders(bool enable);
    void setSpaces(bool enable);
    void setTimeout(uint8_t n);

    std::string getVersion();
    std::string getDeviceDescription();

private:
    class AdapterImpl;
    std::unique_ptr<AdapterImpl> pImpl;
};

} // namespace Diagnostics
} // namespace Genesis
