#define _POSIX_C_SOURCE 200809L

#include "devices/rayneo_imu_bridge.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

static imu_sample rayneo_raw_sample;
static bool rayneo_raw_pending = false;
static bool rayneo_calibrating = false;
static bool rayneo_mag_live = false;
static bool rayneo_shutdown = false;
static bool rayneo_observing_mag = false;
static double rayneo_mag_abs_sum = 0.0;
static uint32_t rayneo_mag_observe_count = 0;
static uint32_t rayneo_calibrate_invocations = 0;
static uint64_t rayneo_last_timestamp_ns = 0;
static float rayneo_last_gx = 0.0f;
static float rayneo_last_gy = 0.0f;
static float rayneo_last_gz = 0.0f;
static double rayneo_gyro_sum_x = 0.0;
static double rayneo_gyro_sum_y = 0.0;
static double rayneo_gyro_sum_z = 0.0;
static uint32_t rayneo_gyro_n = 0;
static float rayneo_gyro_bias_x = 0.0f;
static float rayneo_gyro_bias_y = 0.0f;
static float rayneo_gyro_bias_z = 0.0f;
static bool rayneo_gyro_bias_ready = false;
static pthread_mutex_t rayneo_sample_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rayneo_sample_cv = PTHREAD_COND_INITIALIZER;

void rayneo_imu_bridge_reset(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    rayneo_raw_pending = false;
    memset(&rayneo_raw_sample, 0, sizeof(rayneo_raw_sample));
    rayneo_calibrating = false;
    rayneo_mag_live = false;
    rayneo_shutdown = false;
    rayneo_observing_mag = false;
    rayneo_mag_abs_sum = 0.0;
    rayneo_mag_observe_count = 0;
    rayneo_calibrate_invocations = 0;
    rayneo_last_timestamp_ns = 0;
    rayneo_last_gx = 0.0f;
    rayneo_last_gy = 0.0f;
    rayneo_last_gz = 0.0f;
    rayneo_gyro_sum_x = 0.0;
    rayneo_gyro_sum_y = 0.0;
    rayneo_gyro_sum_z = 0.0;
    rayneo_gyro_n = 0;
    rayneo_gyro_bias_x = 0.0f;
    rayneo_gyro_bias_y = 0.0f;
    rayneo_gyro_bias_z = 0.0f;
    rayneo_gyro_bias_ready = false;
    pthread_cond_broadcast(&rayneo_sample_cv);
    pthread_mutex_unlock(&rayneo_sample_mu);
}

void rayneo_imu_bridge_enqueue(const float acc[3], const float gyro[3], const float mag[3],
                               uint64_t timestamp_ns) {
    if (!acc || !gyro) return;

    imu_sample s = {0};
    // Same SDK IMU triads the callback always delivered. Gyro rad/s → deg/s for Fusion
    // and for the still-hold gyro mean (live pose is GetHeadTrackerPose, not Fusion).
    s.gx = gyro[0] * RAYNEO_GYRO_TO_DPS;
    s.gy = gyro[1] * RAYNEO_GYRO_TO_DPS;
    s.gz = gyro[2] * RAYNEO_GYRO_TO_DPS;
    s.ax = acc[0];
    s.ay = acc[1];
    s.az = acc[2];
    s.timestamp_ns = timestamp_ns;
    s.flags = 0;

    pthread_mutex_lock(&rayneo_sample_mu);
    if (rayneo_observing_mag && mag) {
        float mx = mag[0];
        float my = mag[1];
        float mz = mag[2];
        rayneo_mag_abs_sum += sqrtf(mx * mx + my * my + mz * mz);
        rayneo_mag_observe_count++;
    }
    // Dead/zero mag is not passed into Fusion. Live mag is passed through.
    if (rayneo_mag_live && mag) {
        s.mx = mag[0];
        s.my = mag[1];
        s.mz = mag[2];
    } else {
        s.mx = 0.0f;
        s.my = 0.0f;
        s.mz = 0.0f;
    }
    rayneo_raw_sample = s;
    rayneo_raw_pending = true;
    rayneo_last_timestamp_ns = timestamp_ns;
    rayneo_last_gx = s.gx;
    rayneo_last_gy = s.gy;
    rayneo_last_gz = s.gz;
    if (rayneo_calibrating) {
        rayneo_gyro_sum_x += (double)s.gx;
        rayneo_gyro_sum_y += (double)s.gy;
        rayneo_gyro_sum_z += (double)s.gz;
        rayneo_gyro_n++;
    }
    pthread_cond_signal(&rayneo_sample_cv);
    pthread_mutex_unlock(&rayneo_sample_mu);
}

int rayneo_imu_bridge_next_sample(device_imu_type* dev, imu_sample* out, int timeout_ms) {
    if (!dev || !out) return -1;

    pthread_mutex_lock(&rayneo_sample_mu);
    if (timeout_ms == 0) {
        if (!rayneo_raw_pending) {
            pthread_mutex_unlock(&rayneo_sample_mu);
            return 0;
        }
        *out = rayneo_raw_sample;
        rayneo_raw_pending = false;
        pthread_cond_signal(&rayneo_sample_cv);
        pthread_mutex_unlock(&rayneo_sample_mu);
        return 1;
    }

    if (timeout_ms < 0) {
        while (!rayneo_raw_pending && !rayneo_shutdown) {
            pthread_cond_wait(&rayneo_sample_cv, &rayneo_sample_mu);
        }
    } else {
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_sec += timeout_ms / 1000;
        deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec += 1;
            deadline.tv_nsec -= 1000000000L;
        }
        while (!rayneo_raw_pending && !rayneo_shutdown) {
            int rc = pthread_cond_timedwait(&rayneo_sample_cv, &rayneo_sample_mu, &deadline);
            if (rc == ETIMEDOUT) break;
        }
    }

    if (!rayneo_raw_pending) {
        int rc = rayneo_shutdown ? -1 : 0;
        pthread_mutex_unlock(&rayneo_sample_mu);
        return rc;
    }
    *out = rayneo_raw_sample;
    rayneo_raw_pending = false;
    pthread_cond_signal(&rayneo_sample_cv);
    pthread_mutex_unlock(&rayneo_sample_mu);
    return 1;
}

static bool rayneo_bridge_open(device_imu_type* dev, const imu_hid_info* info) {
    (void)info;
    if (!dev) return false;
    dev->handle = dev;
    return true;
}

static void rayneo_bridge_close(device_imu_type* dev) {
    if (dev) dev->handle = NULL;
}

static bool rayneo_bridge_start_stream(device_imu_type* dev) {
    (void)dev;
    return true;
}

static bool rayneo_bridge_stop_stream(device_imu_type* dev) {
    (void)dev;
    return true;
}

static bool rayneo_bridge_get_static_id(device_imu_type* dev, uint32_t* out_id) {
    (void)dev;
    if (out_id) *out_id = 0x5241594E; /* RAYN */
    return true;
}

static bool rayneo_bridge_load_calibration_json(device_imu_type* dev, uint32_t* len, char** data) {
    (void)dev;
    if (len) *len = 0;
    if (data) *data = NULL;
    return false;
}

const imu_protocol rayneo_imu_bridge_protocol = {
    .open                  = rayneo_bridge_open,
    .close                 = rayneo_bridge_close,
    .start_stream          = rayneo_bridge_start_stream,
    .stop_stream           = rayneo_bridge_stop_stream,
    .get_static_id         = rayneo_bridge_get_static_id,
    .load_calibration_json = rayneo_bridge_load_calibration_json,
    .next_sample           = rayneo_imu_bridge_next_sample,
};

void rayneo_imu_bridge_set_calibrating(bool calibrating) {
    pthread_mutex_lock(&rayneo_sample_mu);
    rayneo_calibrating = calibrating;
    pthread_mutex_unlock(&rayneo_sample_mu);
}

bool rayneo_imu_bridge_is_calibrating(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    bool v = rayneo_calibrating;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return v;
}

void rayneo_imu_bridge_request_stop(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    rayneo_shutdown = true;
    pthread_cond_broadcast(&rayneo_sample_cv);
    pthread_mutex_unlock(&rayneo_sample_mu);
}

void rayneo_imu_bridge_wait_slot_empty(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    while (rayneo_raw_pending && !rayneo_shutdown) {
        pthread_cond_wait(&rayneo_sample_cv, &rayneo_sample_mu);
    }
    pthread_mutex_unlock(&rayneo_sample_mu);
}

device_imu_error_type rayneo_imu_bridge_run_still_calibration(device_imu_type* device) {
    if (!device || !device->protocol) return DEVICE_IMU_ERROR_NO_DEVICE;

    pthread_mutex_lock(&rayneo_sample_mu);
    rayneo_calibrating = true;
    rayneo_observing_mag = true;
    rayneo_mag_abs_sum = 0.0;
    rayneo_mag_observe_count = 0;
    rayneo_mag_live = false;
    rayneo_gyro_sum_x = 0.0;
    rayneo_gyro_sum_y = 0.0;
    rayneo_gyro_sum_z = 0.0;
    rayneo_gyro_n = 0;
    rayneo_gyro_bias_ready = false;
    pthread_mutex_unlock(&rayneo_sample_mu);

    for (uint32_t i = 0; i < RAYNEO_MAG_OBSERVE_SAMPLES; i++) {
        imu_sample s = {0};
        int n = rayneo_imu_bridge_next_sample(device, &s, -1);
        if (n < 0) {
            rayneo_imu_bridge_set_calibrating(false);
            return DEVICE_IMU_ERROR_UNPLUGGED;
        }
    }

    pthread_mutex_lock(&rayneo_sample_mu);
    rayneo_observing_mag = false;
    float mean_abs = 0.0f;
    if (rayneo_mag_observe_count > 0) {
        mean_abs = (float)(rayneo_mag_abs_sum / (double)rayneo_mag_observe_count);
    }
    rayneo_mag_live = mean_abs > RAYNEO_MAG_LIVE_MEAN_ABS_THRESHOLD;
    bool mag_live = rayneo_mag_live;
    pthread_mutex_unlock(&rayneo_sample_mu);

    // Gyro+accel still-cal only. Live mag is not fused: published pose is
    // GetHeadTrackerPose, and still-window mag iron is singular without a heading sweep.
    (void)mag_live;
    device_imu_error_type err = device_imu_calibrate(
        device, RAYNEO_STILL_CAL_ITERATIONS, true, true, false);
    device->last_timestamp = rayneo_imu_bridge_last_timestamp_ns();

    pthread_mutex_lock(&rayneo_sample_mu);
    if (err == DEVICE_IMU_ERROR_NO_ERROR) {
        rayneo_calibrate_invocations++;
    }
    if (rayneo_gyro_n > 0) {
        rayneo_gyro_bias_x = (float)(rayneo_gyro_sum_x / (double)rayneo_gyro_n);
        rayneo_gyro_bias_y = (float)(rayneo_gyro_sum_y / (double)rayneo_gyro_n);
        rayneo_gyro_bias_z = (float)(rayneo_gyro_sum_z / (double)rayneo_gyro_n);
        rayneo_gyro_bias_ready = true;
    }
    rayneo_calibrating = false;
    pthread_mutex_unlock(&rayneo_sample_mu);

    return err;
}

void rayneo_imu_bridge_on_recenter(void) {
    // No-op: driver.c owns recenter via reference pose. Do not rebuild gyro bias.
}

uint32_t rayneo_imu_bridge_calibrate_invocations(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    uint32_t n = rayneo_calibrate_invocations;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return n;
}

bool rayneo_imu_bridge_mag_is_live(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    bool v = rayneo_mag_live;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return v;
}

uint64_t rayneo_imu_bridge_last_timestamp_ns(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    uint64_t t = rayneo_last_timestamp_ns;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return t;
}

void rayneo_imu_bridge_commit_still_gyro_bias(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    if (rayneo_gyro_n > 0) {
        rayneo_gyro_bias_x = (float)(rayneo_gyro_sum_x / (double)rayneo_gyro_n);
        rayneo_gyro_bias_y = (float)(rayneo_gyro_sum_y / (double)rayneo_gyro_n);
        rayneo_gyro_bias_z = (float)(rayneo_gyro_sum_z / (double)rayneo_gyro_n);
        rayneo_gyro_bias_ready = true;
    }
    pthread_mutex_unlock(&rayneo_sample_mu);
}

bool rayneo_imu_bridge_still_gyro_ready(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    bool v = rayneo_gyro_bias_ready;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return v;
}

float rayneo_imu_bridge_gyro_excess_dps(void) {
    pthread_mutex_lock(&rayneo_sample_mu);
    if (!rayneo_gyro_bias_ready) {
        pthread_mutex_unlock(&rayneo_sample_mu);
        return 1.0e6f;
    }
    float dx = rayneo_last_gx - rayneo_gyro_bias_x;
    float dy = rayneo_last_gy - rayneo_gyro_bias_y;
    float dz = rayneo_last_gz - rayneo_gyro_bias_z;
    pthread_mutex_unlock(&rayneo_sample_mu);
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

bool rayneo_imu_bridge_attach(device_imu_type* device) {
    if (!device) return false;

    memset(device, 0, sizeof(*device));
    rayneo_imu_bridge_reset();

    const imu_hid_info info = {
        .product_id = 0xaf50,
        .interface_number = -1,
        .path = NULL,
    };
    if (!rayneo_imu_bridge_protocol.open(device, &info)) return false;
    device->protocol = &rayneo_imu_bridge_protocol;
    device->sample_rate = RAYNEO_IMU_SAMPLE_RATE_HZ;
    return true;
}
