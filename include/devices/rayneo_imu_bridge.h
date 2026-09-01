#pragma once

#include "device_imu.h"
#include "imu_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define RAYNEO_IMU_SAMPLE_RATE_HZ 500

// RayNeo SDK IMU gyro units are not documented in FXRApi.h / FXRMacro.h.
// Treat gyro as rad/s and convert to deg/s for xrDeviceKit, matching VITURE Beast.
// Accel is passed through as Fusion g. Live pose is GetHeadTrackerPose (EUS→NWU).
#define RAYNEO_GYRO_TO_DPS 57.29577951308232f

// After still-cal, hold the last SDK pose while |gyro - still-mean| stays this small.
#define RAYNEO_STILL_HOLD_ENTER_DPS 0.75f
#define RAYNEO_STILL_HOLD_EXIT_DPS 2.0f
#define RAYNEO_STILL_HOLD_ENTER_SAMPLES 50

// Still calibration matches device calibration_wait_s (5s at 500 Hz).
#define RAYNEO_STILL_CAL_WAIT_S 5
#define RAYNEO_STILL_CAL_ITERATIONS ((uint32_t)(RAYNEO_STILL_CAL_WAIT_S * RAYNEO_IMU_SAMPLE_RATE_HZ))

// Mean |mag| over the still-window prefix. Zeros and near-zeros are dead (AirPin reports
// no magnetometer). Earth-field-scale triads are live. Heading change is proven in AE2
// by rotating mag after cal, not by turning during the 5s still.
#define RAYNEO_MAG_LIVE_MEAN_ABS_THRESHOLD 0.05f
#define RAYNEO_MAG_OBSERVE_SAMPLES 100

extern const imu_protocol rayneo_imu_bridge_protocol;

void rayneo_imu_bridge_reset(void);
void rayneo_imu_bridge_enqueue(const float acc[3], const float gyro[3], const float mag[3],
                               uint64_t timestamp_ns);
int rayneo_imu_bridge_next_sample(device_imu_type* dev, imu_sample* out, int timeout_ms);
bool rayneo_imu_bridge_attach(device_imu_type* device);

// While true, the IMU callback must enqueue+signal only (no device_imu_read).
void rayneo_imu_bridge_set_calibrating(bool calibrating);
bool rayneo_imu_bridge_is_calibrating(void);

// Wake blocking next_sample waiters so calibrate can exit on device stop.
void rayneo_imu_bridge_request_stop(void);

// Test feeder pacing: wait until the one-sample slot is empty (or stop is requested).
void rayneo_imu_bridge_wait_slot_empty(void);

// Mag-observe prefix, then device_imu_calibrate(gyro+accel always; mag iff live).
device_imu_error_type rayneo_imu_bridge_run_still_calibration(device_imu_type* device);

// Recenter is owned by driver.c reference pose ("Centering screen"). Fusion must not
// rebuild gyro bias or call SDK Recenter() on that path. This hook is a documented no-op.
void rayneo_imu_bridge_on_recenter(void);
uint32_t rayneo_imu_bridge_calibrate_invocations(void);
bool rayneo_imu_bridge_mag_is_live(void);
uint64_t rayneo_imu_bridge_last_timestamp_ns(void);

// Still-window mean gyro (deg/s). Used to freeze GetHeadTrackerPose while parked.
void rayneo_imu_bridge_commit_still_gyro_bias(void);
bool rayneo_imu_bridge_still_gyro_ready(void);
float rayneo_imu_bridge_gyro_excess_dps(void);
