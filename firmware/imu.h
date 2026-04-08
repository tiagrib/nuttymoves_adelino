#ifndef ADELINO_IMU_H
#define ADELINO_IMU_H

#include "config.h"

// ============================================================
// BNO085 IMU Helper (only compiled when IMU_ENABLED == 1)
// ============================================================

#if IMU_ENABLED

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

static BNO08x imu_sensor;
static bool imu_initialized = false;

// Initialize the BNO085 IMU on I2C.
// Returns true on success.
inline bool imu_init() {
    Wire.begin();

    if (!imu_sensor.begin(0x4A, Wire)) {  // Default I2C address for BNO085
        return false;
    }

    // Request rotation vector reports at ~50Hz (20ms interval)
    if (!imu_sensor.enableRotationVector(20)) {
        return false;
    }

    imu_initialized = true;
    return true;
}

// Non-blocking read of IMU quaternion.
// Returns true if new data was available, and fills quat[4] with [w, x, y, z].
inline bool imu_read(float* quat) {
    if (!imu_initialized) return false;

    if (imu_sensor.getSensorEvent()) {
        if (imu_sensor.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            quat[0] = imu_sensor.getQuatReal();
            quat[1] = imu_sensor.getQuatI();
            quat[2] = imu_sensor.getQuatJ();
            quat[3] = imu_sensor.getQuatK();
            return true;
        }
    }
    return false;
}

#else  // IMU_ENABLED == 0

inline bool imu_init() { return false; }
inline bool imu_read(float* quat) { (void)quat; return false; }

#endif // IMU_ENABLED

#endif // ADELINO_IMU_H
