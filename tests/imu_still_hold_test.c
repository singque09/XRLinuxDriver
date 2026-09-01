#include "imu.h"

#include <stdio.h>

static int g_failures = 0;

#define EXPECT(cond, msg)                                                          \
    do {                                                                           \
        if (!(cond)) {                                                             \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);          \
            g_failures++;                                                          \
        }                                                                          \
    } while (0)

static imu_quat_type q_yaw(float deg) {
    return euler_to_quaternion_zyx((imu_euler_type){.roll = 0.0f, .pitch = 0.0f, .yaw = deg});
}

static void test_follow_then_freeze_then_ignore_drift(void) {
    imu_still_hold_type st;
    imu_still_hold_reset(&st);

    imu_quat_type looking = q_yaw(20.0f);
    imu_quat_type out = imu_still_hold_update(&st, looking, 10.0f, 0.75f, 2.0f, 3);
    EXPECT(quat_equal(out, looking), "while turning, published pose must follow the live SDK pose");

    imu_quat_type parked = q_yaw(21.0f);
    out = imu_still_hold_update(&st, parked, 0.1f, 0.75f, 2.0f, 3);
    EXPECT(quat_equal(out, parked), "first still samples must still follow until the enter count");
    out = imu_still_hold_update(&st, parked, 0.1f, 0.75f, 2.0f, 3);
    EXPECT(quat_equal(out, parked), "second still sample must still follow until the enter count");
    out = imu_still_hold_update(&st, parked, 0.1f, 0.75f, 2.0f, 3);
    EXPECT(quat_equal(out, parked), "enter-count still sample must freeze at the current SDK pose");

    imu_quat_type drifted = q_yaw(40.0f);
    out = imu_still_hold_update(&st, drifted, 0.1f, 0.75f, 2.0f, 3);
    EXPECT(quat_equal(out, parked), "SDK drift while parked must not move the held pose");
    EXPECT(!quat_equal(out, drifted), "held pose must not equal the drifted SDK pose");
}

static void test_turn_releases_hold(void) {
    imu_still_hold_type st;
    imu_still_hold_reset(&st);

    imu_quat_type parked = q_yaw(5.0f);
    (void)imu_still_hold_update(&st, parked, 0.1f, 0.75f, 2.0f, 1);

    imu_quat_type looking = q_yaw(-15.0f);
    imu_quat_type out = imu_still_hold_update(&st, looking, 8.0f, 0.75f, 2.0f, 1);
    EXPECT(quat_equal(out, looking), "a head turn must release the hold and follow the SDK pose");
}

int main(void) {
    test_follow_then_freeze_then_ignore_drift();
    test_turn_releases_hold();

    if (g_failures != 0) {
        fprintf(stderr, "%d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("imu_still_hold_test: all passed\n");
    return 0;
}
