#include "device_imu.h"
#include "devices/rayneo_imu_bridge.h"
#include "imu.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int g_failures = 0;
static int g_update_count = 0;
static device_imu_quat_type g_last_orientation;

#define EXPECT(cond, msg)                                                          \
    do {                                                                           \
        if (!(cond)) {                                                             \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);          \
            g_failures++;                                                          \
        }                                                                          \
    } while (0)

#define NS_PER_SAMPLE 2000000ULL
#define BIAS_DPS 2.0f
#define WALK_LIMIT_DEG 1.25f
#define SETTLE_SAMPLES (4 * RAYNEO_IMU_SAMPLE_RATE_HZ)
#define MINUTE_SAMPLES (60 * RAYNEO_IMU_SAMPLE_RATE_HZ)

static void test_fusion_event(uint64_t timestamp, device_imu_event_type event,
                              const device_imu_ahrs_type* ahrs) {
    (void)timestamp;
    if (event != DEVICE_IMU_EVENT_UPDATE) return;
    g_update_count++;
    g_last_orientation = device_imu_get_orientation(ahrs);
}

static float wrap_delta_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static float hypot2f(float a, float b) {
    return sqrtf(a * a + b * b);
}

typedef struct {
    volatile int stop;
    float acc[3];
    float gyro[3];
    float mag[3];
    uint64_t t_ns;
    pthread_mutex_t mu;
} feeder_t;

static void feeder_set(feeder_t* f, const float acc[3], const float gyro[3], const float mag[3]) {
    pthread_mutex_lock(&f->mu);
    memcpy(f->acc, acc, sizeof(f->acc));
    memcpy(f->gyro, gyro, sizeof(f->gyro));
    memcpy(f->mag, mag, sizeof(f->mag));
    pthread_mutex_unlock(&f->mu);
}

static void* feeder_thread(void* arg) {
    feeder_t* f = (feeder_t*)arg;
    while (!f->stop) {
        float acc[3], gyro[3], mag[3];
        uint64_t t;
        pthread_mutex_lock(&f->mu);
        memcpy(acc, f->acc, sizeof(acc));
        memcpy(gyro, f->gyro, sizeof(gyro));
        memcpy(mag, f->mag, sizeof(mag));
        t = f->t_ns;
        f->t_ns += NS_PER_SAMPLE;
        pthread_mutex_unlock(&f->mu);
        rayneo_imu_bridge_wait_slot_empty();
        if (f->stop) break;
        rayneo_imu_bridge_enqueue(acc, gyro, mag, t);
    }
    return NULL;
}

static int open_fusion(device_imu_type* device) {
    if (!rayneo_imu_bridge_attach(device)) {
        fprintf(stderr, "FAIL: rayneo_imu_bridge_attach failed\n");
        g_failures++;
        return 0;
    }
    device_imu_error_type err = device_imu_open(device, test_fusion_event);
    if (err != DEVICE_IMU_ERROR_NO_ERROR) {
        fprintf(stderr, "FAIL: device_imu_open returned %d\n", err);
        g_failures++;
        return 0;
    }
    return 1;
}

static void close_fusion(device_imu_type* device) {
    rayneo_imu_bridge_request_stop();
    device_imu_close(device);
    rayneo_imu_bridge_reset();
}

static void pump_reads(device_imu_type* device, int n) {
    for (int i = 0; i < n; i++) {
        device_imu_read(device, -1);
    }
}

static device_imu_euler_type capture_euler(void) {
    imu_quat_type fused = {
        .w = g_last_orientation.w,
        .x = g_last_orientation.x,
        .y = g_last_orientation.y,
        .z = g_last_orientation.z,
    };
    imu_quat_type nwu = quaternion_eus_to_nwu(fused);
    (void)nwu;
    return device_imu_get_euler(g_last_orientation);
}

static void still_vectors(float acc[3], float gyro[3], float mag[3], bool mag_live) {
    acc[0] = 0.0f;
    acc[1] = 0.0f;
    acc[2] = 1.0f;
    gyro[0] = 0.0f;
    gyro[1] = 0.0f;
    gyro[2] = BIAS_DPS / RAYNEO_GYRO_TO_DPS;
    if (mag_live) {
        mag[0] = 1.0f;
        mag[1] = 0.0f;
        mag[2] = 0.0f;
    } else {
        mag[0] = mag[1] = mag[2] = 0.0f;
    }
}

static void start_still_feeder(feeder_t* f, pthread_t* thread, bool mag_live) {
    memset(f, 0, sizeof(*f));
    pthread_mutex_init(&f->mu, NULL);
    f->t_ns = NS_PER_SAMPLE;
    float acc[3], gyro[3], mag[3];
    still_vectors(acc, gyro, mag, mag_live);
    feeder_set(f, acc, gyro, mag);
    pthread_create(thread, NULL, feeder_thread, f);
}

static void stop_feeder(feeder_t* f, pthread_t thread) {
    f->stop = 1;
    rayneo_imu_bridge_request_stop();
    pthread_join(thread, NULL);
    pthread_mutex_destroy(&f->mu);
    rayneo_imu_bridge_request_stop();
}

static void* blocking_waiter(void* arg) {
    imu_sample* out = (imu_sample*)arg;
    device_imu_type dummy = {0};
    int n = rayneo_imu_bridge_next_sample(&dummy, out, -1);
    return (void*)(intptr_t)n;
}

static void test_next_sample_timeout_waits(void) {
    rayneo_imu_bridge_reset();
    imu_sample got = {0};
    pthread_t t;
    pthread_create(&t, NULL, blocking_waiter, &got);
    usleep(80000);
    const float acc[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float mag[3] = {0.0f, 0.0f, 0.0f};
    rayneo_imu_bridge_enqueue(acc, gyro, mag, 4000000ULL);
    void* retval = NULL;
    pthread_join(t, &retval);
    int n = (int)(intptr_t)retval;
    EXPECT(n == 1, "next_sample(-1) must wait for a sample rather than return 0 immediately");
    EXPECT(got.timestamp_ns == 4000000ULL, "blocking next_sample should deliver the enqueued sample");
}

static void test_ae1_still_head_walk_after_cal(void) {
    device_imu_type device;
    if (!open_fusion(&device)) return;

    g_update_count = 0;
    feeder_t feeder;
    pthread_t ft;
    start_still_feeder(&feeder, &ft, false);

    device_imu_error_type err = rayneo_imu_bridge_run_still_calibration(&device);
    EXPECT(err == DEVICE_IMU_ERROR_NO_ERROR, "still calibration should succeed");
    EXPECT(rayneo_imu_bridge_calibrate_invocations() >= 1,
           "still calibration should invoke device_imu_calibrate once");
    EXPECT(!rayneo_imu_bridge_mag_is_live(), "zero mag during still should be treated as dead");

    pump_reads(&device, SETTLE_SAMPLES);
    device_imu_euler_type before = capture_euler();
    int updates_before = g_update_count;

    pump_reads(&device, MINUTE_SAMPLES);
    device_imu_euler_type after = capture_euler();

    EXPECT(g_update_count > updates_before + 1000, "60s stream should produce many AHRS updates");

    float dyaw = fabsf(wrap_delta_deg(after.yaw, before.yaw));
    float dpitch = fabsf(wrap_delta_deg(after.pitch, before.pitch));
    if (hypot2f(dyaw, dpitch) > WALK_LIMIT_DEG) {
        fprintf(stderr, "FAIL AE1 walk yaw=%.3f pitch=%.3f (limit %.2f); uncalibrated bias*60s would be ~%.0f deg\n",
                dyaw, dpitch, WALK_LIMIT_DEG, BIAS_DPS * 60.0f);
        g_failures++;
    }

    imu_quat_type fused = {
        .w = g_last_orientation.w,
        .x = g_last_orientation.x,
        .y = g_last_orientation.y,
        .z = g_last_orientation.z,
    };
    imu_quat_type nwu = quaternion_eus_to_nwu(fused);
    float nwu_n = sqrtf(nwu.w * nwu.w + nwu.x * nwu.x + nwu.y * nwu.y + nwu.z * nwu.z);
    EXPECT(nwu_n > 0.5f, "EUS→NWU of a Fusion quat should remain a unit orientation");

    stop_feeder(&feeder, ft);
    close_fusion(&device);
}

static void test_ae3_dead_mag_look_around_then_park(void) {
    device_imu_type device;
    if (!open_fusion(&device)) return;

    feeder_t feeder;
    pthread_t ft;
    start_still_feeder(&feeder, &ft, false);

    rayneo_imu_bridge_run_still_calibration(&device);
    EXPECT(!rayneo_imu_bridge_mag_is_live(), "all-zero mag is dead");

    pump_reads(&device, SETTLE_SAMPLES);
    device_imu_euler_type pre_turn = capture_euler();

    float acc[3], gyro[3], mag[3];
    still_vectors(acc, gyro, mag, false);
    const float turn_dps = 90.0f;
    gyro[2] = (turn_dps + BIAS_DPS) / RAYNEO_GYRO_TO_DPS;
    feeder_set(&feeder, acc, gyro, mag);
    pump_reads(&device, RAYNEO_IMU_SAMPLE_RATE_HZ);

    still_vectors(acc, gyro, mag, false);
    feeder_set(&feeder, acc, gyro, mag);
    pump_reads(&device, SETTLE_SAMPLES);
    device_imu_euler_type parked = capture_euler();

    float turn_yaw = fabsf(wrap_delta_deg(parked.yaw, pre_turn.yaw));
    EXPECT(turn_yaw > 20.0f, "look-around with dead mag is allowed to change world yaw");

    pump_reads(&device, MINUTE_SAMPLES);
    device_imu_euler_type after = capture_euler();
    float dyaw = fabsf(wrap_delta_deg(after.yaw, parked.yaw));
    float dpitch = fabsf(wrap_delta_deg(after.pitch, parked.pitch));
    if (hypot2f(dyaw, dpitch) > WALK_LIMIT_DEG) {
        fprintf(stderr, "FAIL AE3 parked walk yaw=%.3f pitch=%.3f\n", dyaw, dpitch);
        g_failures++;
    }

    stop_feeder(&feeder, ft);
    close_fusion(&device);
}

static void test_ae4_pitch_tilt_returns_to_level(void) {
    device_imu_type device;
    if (!open_fusion(&device)) return;

    feeder_t feeder;
    pthread_t ft;
    start_still_feeder(&feeder, &ft, false);

    rayneo_imu_bridge_run_still_calibration(&device);
    pump_reads(&device, SETTLE_SAMPLES);
    device_imu_euler_type level = capture_euler();

    float acc[3], gyro[3], mag[3];
    still_vectors(acc, gyro, mag, false);
    const float tilt = 30.0f * (float)M_PI / 180.0f;
    acc[0] = sinf(tilt);
    acc[2] = cosf(tilt);
    gyro[1] = (30.0f + BIAS_DPS) / RAYNEO_GYRO_TO_DPS;
    gyro[2] = BIAS_DPS / RAYNEO_GYRO_TO_DPS;
    feeder_set(&feeder, acc, gyro, mag);
    pump_reads(&device, RAYNEO_IMU_SAMPLE_RATE_HZ);

    still_vectors(acc, gyro, mag, false);
    gyro[1] = (-30.0f + BIAS_DPS) / RAYNEO_GYRO_TO_DPS;
    feeder_set(&feeder, acc, gyro, mag);
    pump_reads(&device, RAYNEO_IMU_SAMPLE_RATE_HZ);

    still_vectors(acc, gyro, mag, false);
    feeder_set(&feeder, acc, gyro, mag);
    pump_reads(&device, SETTLE_SAMPLES);

    device_imu_euler_type back = capture_euler();
    float dpitch = fabsf(wrap_delta_deg(back.pitch, level.pitch));
    if (dpitch > 5.0f) {
        fprintf(stderr, "FAIL AE4 pitch offset after tilt-return=%.3f\n", dpitch);
        g_failures++;
    }

    stop_feeder(&feeder, ft);
    close_fusion(&device);
}

static void test_recenter_does_not_recalibrate(void) {
    device_imu_type device;
    if (!open_fusion(&device)) return;

    feeder_t feeder;
    pthread_t ft;
    start_still_feeder(&feeder, &ft, false);
    rayneo_imu_bridge_run_still_calibration(&device);
    uint32_t after_cal = rayneo_imu_bridge_calibrate_invocations();
    EXPECT(after_cal >= 1, "calibrate should have run before simulated recenter");

    // Recenter is owned by driver.c (reference pose on "Centering screen").
    // RayNeo fusion must not call device_imu_calibrate or SDK Recenter().
    rayneo_imu_bridge_on_recenter();
    EXPECT(rayneo_imu_bridge_calibrate_invocations() == after_cal,
           "recenter must not invoke a second device_imu_calibrate / gyro-bias rebuild");

    stop_feeder(&feeder, ft);
    close_fusion(&device);
}

int main(void) {
    test_next_sample_timeout_waits();
    test_ae1_still_head_walk_after_cal();
    test_ae3_dead_mag_look_around_then_park();
    test_ae4_pitch_tilt_returns_to_level();
    test_recenter_does_not_recalibrate();

    if (g_failures != 0) {
        fprintf(stderr, "%d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("rayneo_fusion_still_test: all passed\n");
    return 0;
}
