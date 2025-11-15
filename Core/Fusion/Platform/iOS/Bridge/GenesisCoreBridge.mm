//
// GenesisCoreBridge.mm
// Genesis Automotive SDK - iOS Bridge
//
// Created by KC Speed Lab / CartelWorx
// Copyright © 2025 TaylorsBar. MIT Licensed.
//

#import <Foundation/Foundation.h>
#import "../../Core/Fusion/GenesisEKF_Ultimate.hpp"

using namespace Genesis;

@interface GenesisCoreBridge : NSObject {
    @private
    GenesisEKFUltimate* _ekf;
}

- (instancetype)init;
- (void)dealloc;

// Sensor input methods
- (void)updateIMU:(double)timestamp
              ax:(double)ax ay:(double)ay az:(double)az
              gx:(double)gx gy:(double)gy gz:(double)gz;

- (void)updateGNSS:(double)timestamp
               lat:(double)lat lon:(double)lon alt:(double)alt
         accuracy:(double)accuracy;

- (void)updateVision:(double)timestamp
                  x:(double)x y:(double)y z:(double)z
            quality:(double)quality;

- (void)updateBarometer:(double)timestamp
              pressure:(double)pressure;

// State retrieval
- (NSDictionary*)getCurrentState;
- (NSArray*)getPosition;  // [x, y, z] in NED frame
- (NSArray*)getVelocity;  // [vx, vy, vz] in NED frame
- (NSArray*)getAttitude;  // [roll, pitch, yaw] in radians

@end

@implementation GenesisCoreBridge

- (instancetype)init {
    self = [super init];
    if (self) {
        _ekf = new GenesisEKFUltimate();
    }
    return self;
}

- (void)dealloc {
    if (_ekf) {
        delete _ekf;
        _ekf = nullptr;
    }
}

- (void)updateIMU:(double)timestamp
              ax:(double)ax ay:(double)ay az:(double)az
              gx:(double)gx gy:(double)gy gz:(double)gz {
    // TODO: Implement IMU update
    // Convert iOS coordinate system to NED
    // Call _ekf->predict(dt, accel_body, gyro_body)
}

- (void)updateGNSS:(double)timestamp
               lat:(double)lat lon:(double)lon alt:(double)alt
         accuracy:(double)accuracy {
    // TODO: Implement GNSS update
    // Call _ekf->updateGNSS(lat, lon, alt, accuracy)
}

- (void)updateVision:(double)timestamp
                  x:(double)x y:(double)y z:(double)z
            quality:(double)quality {
    // TODO: Implement vision update
    // Convert vision pose to EKF measurement
    // Call _ekf->updateVision(position_ned, quality)
}

- (void)updateBarometer:(double)timestamp
              pressure:(double)pressure {
    // TODO: Implement barometer update
    // Convert pressure to altitude
    // Call _ekf->updateBarometer(altitude)
}

- (NSDictionary*)getCurrentState {
    // TODO: Extract full state from EKF
    return @{
        @"position": [self getPosition],
        @"velocity": [self getVelocity],
        @"attitude": [self getAttitude],
        @"timestamp": @([[NSDate date] timeIntervalSince1970])
    };
}

- (NSArray*)getPosition {
    // TODO: Get position from _ekf->getState()
    return @[@0.0, @0.0, @0.0];
}

- (NSArray*)getVelocity {
    // TODO: Get velocity from _ekf->getState()
    return @[@0.0, @0.0, @0.0];
}

- (NSArray*)getAttitude {
    // TODO: Get attitude from _ekf->getState()
    return @[@0.0, @0.0, @0.0];
}

@end

// NOTE: This is a streamlined reference implementation.
// For complete production code with full error handling, coordinate transforms,
// and optimizations, see CODE_TRANSFER_PACKAGE.md in the repository root.
