#include "device_imu.h"
#include "devices/rayneo_imu_bridge.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

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

static void test_fusion_event(uint64_t timestamp, device_imu_event_type event,
                              const device_imu_ahrs_type* ahrs) {
    (void)timestamp;
    if (event != DEVICE_IMU_EVENT_UPDATE) return;
    g_update_count++;
    g_last_orientation = device_imu_get_orientation(ahrs);
}

static void test_next_sample_empty_returns_zero(void) {
    device_imu_type dummy = {0};
    imu_sample out;
    unsigned char before[sizeof(out)];

    rayneo_imu_bridge_reset();
    memset(&out, 0x7f, sizeof(out));
    memcpy(before, &out, sizeof(out));
    int n = rayneo_imu_bridge_next_sample(&dummy, &out, 0);
    EXPECT(n == 0, "next_sample with no pending data should return 0");
    EXPECT(memcmp(&out, before, sizeof(out)) == 0,
           "next_sample must not invent a sample when nothing is pending");
}

static void test_enqueue_then_next_sample_drains_one(void) {
    device_imu_type dummy = {0};
    rayneo_imu_bridge_reset();
    const float acc[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float mag[3] = {0.0f, 0.0f, 0.0f};
    rayneo_imu_bridge_enqueue(acc, gyro, mag, 2000000ULL);

    imu_sample first = {0};
    int n = rayneo_imu_bridge_next_sample(&dummy, &first, 0);
    EXPECT(n == 1, "pending SDK-shaped sample should produce one next_sample");
    EXPECT(fabsf(first.az - 1.0f) < 1e-5f, "drained sample should keep gravity on Z");
    EXPECT(fabsf(first.gx) < 1e-5f && fabsf(first.gy) < 1e-5f && fabsf(first.gz) < 1e-5f,
           "drained sample should keep zero gyro");
    EXPECT(first.timestamp_ns == 2000000ULL, "drained sample should keep the SDK timestamp");

    imu_sample second;
    memset(&second, 0x7f, sizeof(second));
    n = rayneo_imu_bridge_next_sample(&dummy, &second, 0);
    EXPECT(n == 0, "second next_sample should return 0 after the pending sample is drained");
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

static void test_pending_sample_produces_one_ahrs_update(void) {
    device_imu_type device;
    if (!open_fusion(&device)) return;

    g_update_count = 0;
    memset(&g_last_orientation, 0, sizeof(g_last_orientation));

    const float acc[3] = {0.0f, 0.0f, 1.0f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float mag[3] = {0.0f, 0.0f, 0.0f};
    rayneo_imu_bridge_enqueue(acc, gyro, mag, 2000000ULL);

    device_imu_error_type err = device_imu_read(&device, 0);
    EXPECT(err == DEVICE_IMU_ERROR_NO_ERROR, "device_imu_read should succeed");
    EXPECT(g_update_count == 1, "one pending sample should produce one AHRS UPDATE");
    float qn = sqrtf(g_last_orientation.x * g_last_orientation.x +
                     g_last_orientation.y * g_last_orientation.y +
                     g_last_orientation.z * g_last_orientation.z +
                     g_last_orientation.w * g_last_orientation.w);
    EXPECT(qn > 0.5f, "AHRS update should publish a non-zero orientation");

    imu_sample leftover = {0};
    int n = rayneo_imu_bridge_next_sample(&device, &leftover, 0);
    EXPECT(n == 0, "device_imu_read should drain the single pending sample");

    device_imu_close(&device);
}

static void test_gyro_excess_tracks_still_window_mean(void) {
    rayneo_imu_bridge_reset();
    rayneo_imu_bridge_set_calibrating(true);

    const float acc[3] = {0.0f, 0.0f, 1.0f};
    const float mag[3] = {0.0f, 0.0f, 0.0f};
    const float still_gyro[3] = {0.02f, -0.01f, 0.03f};
    for (int i = 0; i < 20; i++) {
        rayneo_imu_bridge_enqueue(acc, still_gyro, mag, (uint64_t)(i + 1) * 2000000ULL);
        imu_sample drain = {0};
        device_imu_type dummy = {0};
        (void)rayneo_imu_bridge_next_sample(&dummy, &drain, 0);
    }

    EXPECT(!rayneo_imu_bridge_still_gyro_ready(),
           "gyro bias must not be ready until the still window is committed");
    EXPECT(rayneo_imu_bridge_gyro_excess_dps() > 1000.0f,
           "uncommitted gyro excess must stay large so the pose is not frozen during cal");

    rayneo_imu_bridge_commit_still_gyro_bias();
    rayneo_imu_bridge_set_calibrating(false);
    EXPECT(rayneo_imu_bridge_still_gyro_ready(), "committed still-window gyro mean must be ready");

    rayneo_imu_bridge_enqueue(acc, still_gyro, mag, 100000000ULL);
    EXPECT(rayneo_imu_bridge_gyro_excess_dps() < 0.05f,
           "same gyro as the still window must be near-zero excess");

    const float turning_gyro[3] = {0.20f, 0.0f, 0.0f};
    rayneo_imu_bridge_enqueue(acc, turning_gyro, mag, 102000000ULL);
    EXPECT(rayneo_imu_bridge_gyro_excess_dps() > 5.0f,
           "a clear head turn must exceed the still-hold enter threshold");
}

int main(void) {
    test_next_sample_empty_returns_zero();
    test_enqueue_then_next_sample_drains_one();
    test_pending_sample_produces_one_ahrs_update();
    test_gyro_excess_tracks_still_window_mean();

    if (g_failures != 0) {
        fprintf(stderr, "%d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("rayneo_fusion_bridge_test: all passed\n");
    return 0;
}
