//
// SensorManager.swift
// Genesis Automotive SDK - iOS Sensor Integration
//
// Created by KC Speed Lab / CartelWorx
// Copyright © 2025 TaylorsBar. MIT Licensed.
//

import Foundation
import CoreMotion
import CoreLocation

public class GenesisSensorManager: NSObject {
    
    // MARK: - Properties
    
    private let motionManager = CMMotionManager()
    private let locationManager = CLLocationManager()
    private let altimeter = CMAltimeter()
    
    private var bridge: GenesisCoreBridge?
    private var lastIMUTimestamp: TimeInterval = 0
    
    // Update rates
    private let imuUpdateInterval: TimeInterval = 0.01  // 100 Hz
    private let locationAccuracy: CLLocationAccuracy = kCLLocationAccuracyBestForNavigation
    
    // MARK: - Initialization
    
    public override init() {
        super.init()
        bridge = GenesisCoreBridge()
        setupSensors()
    }
    
    // MARK: - Sensor Setup
    
    private func setupSensors() {
        setupIMU()
        setupGNSS()
        setupBarometer()
    }
    
    private func setupIMU() {
        guard motionManager.isDeviceMotionAvailable else {
            print("[Genesis] Device motion not available")
            return
        }
        
        motionManager.deviceMotionUpdateInterval = imuUpdateInterval
        motionManager.showsDeviceMovementDisplay = true
        
        // TODO: Start device motion updates with reference frame
        // motionManager.startDeviceMotionUpdates(using: .xTrueNorthZVertical,
        //                                        to: .main) { [weak self] motion, error in
        //     guard let motion = motion, error == nil else { return }
        //     self?.processIMU(motion)
        // }
    }
    
    private func setupGNSS() {
        locationManager.delegate = self
        locationManager.desiredAccuracy = locationAccuracy
        locationManager.distanceFilter = kCLDistanceFilterNone
        locationManager.allowsBackgroundLocationUpdates = true
        
        // TODO: Request authorization and start updates
        // locationManager.requestAlwaysAuthorization()
        // locationManager.startUpdatingLocation()
    }
    
    private func setupBarometer() {
        guard CMAltimeter.isRelativeAltitudeAvailable() else {
            print("[Genesis] Barometric altimeter not available")
            return
        }
        
        // TODO: Start altimeter updates
        // altimeter.startRelativeAltitudeUpdates(to: .main) { [weak self] data, error in
        //     guard let data = data, error == nil else { return }
        //     self?.processBarometer(data)
        // }
    }
    
    // MARK: - Data Processing
    
    private func processIMU(_ motion: CMDeviceMotion) {
        let timestamp = Date().timeIntervalSince1970
        
        // Extract acceleration (m/s^2)
        let accel = motion.userAcceleration
        let gravity = motion.gravity
        
        // Total acceleration in device frame
        let ax = accel.x + gravity.x
        let ay = accel.y + gravity.y
        let az = accel.z + gravity.z
        
        // Extract rotation rate (rad/s)
        let gyro = motion.rotationRate
        let gx = gyro.x
        let gy = gyro.y
        let gz = gyro.z
        
        // TODO: Send to EKF bridge
        // bridge?.updateIMU(timestamp, ax: ax, ay: ay, az: az,
        //                   gx: gx, gy: gy, gz: gz)
    }
    
    private func processLocation(_ location: CLLocation) {
        let timestamp = location.timestamp.timeIntervalSince1970
        let lat = location.coordinate.latitude
        let lon = location.coordinate.longitude
        let alt = location.altitude
        let accuracy = location.horizontalAccuracy
        
        // TODO: Send to EKF bridge
        // bridge?.updateGNSS(timestamp, lat: lat, lon: lon,
        //                    alt: alt, accuracy: accuracy)
    }
    
    private func processBarometer(_ data: CMAltitudeData) {
        let timestamp = Date().timeIntervalSince1970
        let pressure = data.pressure.doubleValue * 10.0  // kPa to hPa
        
        // TODO: Send to EKF bridge
        // bridge?.updateBarometer(timestamp, pressure: pressure)
    }
    
    // MARK: - Public Interface
    
    public func start() {
        // TODO: Start all sensor updates
        print("[Genesis] Starting sensor fusion...")
    }
    
    public func stop() {
        motionManager.stopDeviceMotionUpdates()
        locationManager.stopUpdatingLocation()
        altimeter.stopRelativeAltitudeUpdates()
        print("[Genesis] Stopped sensor fusion")
    }
    
    public func getCurrentState() -> [String: Any]? {
        return bridge?.getCurrentState() as? [String: Any]
    }
}

// MARK: - CLLocationManagerDelegate

extension GenesisSensorManager: CLLocationManagerDelegate {
    
    public func locationManager(_ manager: CLLocationManager,
                               didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        processLocation(location)
    }
    
    public func locationManager(_ manager: CLLocationManager,
                               didFailWithError error: Error) {
        print("[Genesis] Location error: \(error.localizedDescription)")
    }
    
    public func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        let status = manager.authorizationStatus
        print("[Genesis] Location authorization: \(status.rawValue)")
    }
}

// NOTE: This is a streamlined reference implementation.
// For complete production code with full error handling, coordinate transforms,
// calibration routines, and optimizations, see CODE_TRANSFER_PACKAGE.md.
