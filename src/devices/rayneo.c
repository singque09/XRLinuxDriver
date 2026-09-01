#include "devices.h"
#include "devices/rayneo.h"
#include "devices/rayneo_imu_bridge.h"
#include "connection_pool.h"
#include "device_imu.h"
#include "driver.h"
#include "imu.h"
#include "logging.h"
#include "memory.h"
#include "outputs.h"
#include "runtime_context.h"
#include "sdks/rayneo.h"
#include "strings.h"

#include <ctype.h>
#include <dlfcn.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TS_TO_MS_FACTOR 1000000
#define EXPECTED_CYCLES_PER_S 500
#define EXPECTED_CYCLE_TIME_MS (1000.0 / EXPECTED_CYCLES_PER_S)
#define BUFFER_SIZE_TARGET_MS 10 // smooth IMU data over this period of time

#define RAYNEO_ID_VENDOR 0x1bbb
#define RAYNEO_ID_PRODUCT 0xaf50

#define STATE_EVENT_DEVICE_INFO 0x4000
#define RAYNEO_DRIVER_ID "rayneo"

// bare soname, so ld.so resolves it exactly as it did when this was a DT_NEEDED
#define RAYNEO_SDK_SONAME "libRayNeoXRMiniSDK.so"

#define RAYNEO_SDK_SYMBOLS \
    X(RegisterIMUEventCallback) X(UnregisterIMUEventCallback) \
    X(RegisterStateEventCallback) X(UnregisterStateEventCallback) \
    X(EstablishUsbConnection) X(ResetUsbConnection) \
    X(NotifyDeviceConnected) X(NotifyDeviceDisconnected) \
    X(StartXR) X(StopXR) X(SwitchTo2D) X(SwitchTo3D) X(OpenIMU) X(CloseIMU) X(Recenter) \
    X(GetHeadTrackerPose) X(ConvertHostTime2DeviceTime) \
    X(GetDeviceType) X(AcquireDeviceInfo) X(GetSideBySideStatus)

void (*RegisterIMUEventCallback)(IMUEventCallback callback) = NULL;
void (*UnregisterIMUEventCallback)(IMUEventCallback callback) = NULL;
void (*RegisterStateEventCallback)(StateEventCallback callback) = NULL;
void (*UnregisterStateEventCallback)(StateEventCallback callback) = NULL;
int (*EstablishUsbConnection)(int32_t vid, int32_t pid) = NULL;
int (*ResetUsbConnection)() = NULL;
void (*NotifyDeviceConnected)() = NULL;
void (*NotifyDeviceDisconnected)() = NULL;
void (*StartXR)() = NULL;
void (*StopXR)() = NULL;
void (*SwitchTo2D)() = NULL;
void (*SwitchTo3D)() = NULL;
void (*OpenIMU)() = NULL;
void (*CloseIMU)() = NULL;
void (*Recenter)() = NULL;
void (*GetHeadTrackerPose)(float rotation[4], float position[3], uint64_t* timeNsInDevice) = NULL;
uint64_t (*ConvertHostTime2DeviceTime)(uint64_t timeNsInHost) = NULL;
void (*GetDeviceType)(char* device) = NULL;
void (*AcquireDeviceInfo)() = NULL;
int8_t (*GetSideBySideStatus)() = NULL;

static void* sdk_handle = NULL;
static bool sdk_loaded = false;
static bool sdk_load_attempted = false;
static pthread_mutex_t sdk_load_mutex = PTHREAD_MUTEX_INITIALIZER;

// see include/sdks/rayneo.h for why this is dlopen'd rather than linked. Idempotent, thread-safe;
// on failure the entry points stay NULL and the device is reported unsupported.
bool rayneo_sdk_load(void) {
    pthread_mutex_lock(&sdk_load_mutex);
    if (!sdk_load_attempted) {
        sdk_load_attempted = true;

        sdk_handle = dlopen(RAYNEO_SDK_SONAME, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
        if (sdk_handle == NULL) {
            log_error("RayNeo driver, failed to load " RAYNEO_SDK_SONAME ": %s\n", dlerror());
        } else {
            const char* missing = NULL;

            #define X(name) if (!missing) { *(void**)(&name) = dlsym(sdk_handle, #name); if (!name) missing = #name; }
            RAYNEO_SDK_SYMBOLS
            #undef X

            if (missing == NULL) {
                sdk_loaded = true;
                if (config()->debug_device) log_debug("RayNeo driver, loaded " RAYNEO_SDK_SONAME "\n");
            } else {
                log_error("RayNeo driver, " RAYNEO_SDK_SONAME " is missing symbol %s\n", missing);

                #define X(name) *(void**)(&name) = NULL;
                RAYNEO_SDK_SYMBOLS
                #undef X
            }
        }
    }
    bool loaded = sdk_loaded;
    pthread_mutex_unlock(&sdk_load_mutex);

    return loaded;
}

const device_properties_type rayneo_properties = {
    .brand                              = "",
    .model                              = "",
    .hid_vendor_id                      = RAYNEO_ID_VENDOR,
    .hid_product_id                     = RAYNEO_ID_PRODUCT,
    .calibration_setup                  = CALIBRATION_SETUP_AUTOMATIC,
    .pitch_adjustment_degrees           = -14.0,
    .resolution_w                       = RESOLUTION_1080P_W,
    .resolution_h                       = RESOLUTION_1080P_H,
    .fov                                = 43.0,
    .lens_distance_ratio                = 0.05,
    .calibration_wait_s                 = 5,
    .imu_cycles_per_s                   = EXPECTED_CYCLES_PER_S,
    .imu_buffer_size                    = ceil(BUFFER_SIZE_TARGET_MS / EXPECTED_CYCLE_TIME_MS),
    .look_ahead_constant                = 10.0,
    .look_ahead_frametime_multiplier    = 0.45,
    .look_ahead_scanline_adjust         = 7.0,
    .look_ahead_ms_cap                  = 40.0,
    .sbs_mode_supported                 = true,
    .firmware_update_recommended        = false,
    .provides_orientation               = true,
    .provides_position                  = false
};

// hardware connection - device is physically plugged in
static bool hard_connected = false;
// software connection - we're actively in communication, holding open a connection
static bool soft_connected = false;

static bool is_sbs_mode = false;

static device_imu_type rayneo_fusion_imu;
static bool rayneo_fusion_open = false;
static pthread_t rayneo_cal_thread;
static bool rayneo_cal_thread_started = false;
static imu_still_hold_type rayneo_still_hold;

static void rayneo_pose_hold_reset(void) {
    imu_still_hold_reset(&rayneo_still_hold);
}

static imu_quat_type rayneo_sdk_head_quat(void) {
    float rotation[4] = {0};
    float position[3] = {0};
    uint64_t time_ns = 0;
    GetHeadTrackerPose(rotation, position, &time_ns);
    return (imu_quat_type){ .w = rotation[3], .x = rotation[0], .y = rotation[1], .z = rotation[2] };
}

static void* rayneo_calibrate_thread(void* arg) {
    (void)arg;
    device_imu_error_type err = rayneo_imu_bridge_run_still_calibration(&rayneo_fusion_imu);
    if (err != DEVICE_IMU_ERROR_NO_ERROR && err != DEVICE_IMU_ERROR_UNPLUGGED) {
        log_error("RayNeo driver, still calibration failed (%d)\n", err);
    } else if (config()->debug_device && err == DEVICE_IMU_ERROR_NO_ERROR) {
        log_debug("RayNeo driver, still calibration complete (mag %s)\n",
                  rayneo_imu_bridge_mag_is_live() ? "live" : "dead");
    }
    return NULL;
}

static void rayneo_fusion_event(uint64_t timestamp, device_imu_event_type event,
                                const device_imu_ahrs_type* ahrs) {
    (void)ahrs;
    if (event != DEVICE_IMU_EVENT_UPDATE || !soft_connected || driver_disabled()) return;
    if (!GetHeadTrackerPose) return;

    // GetHeadTrackerPose is the look-mapping the wearer confirmed. Fusion remaps
    // (and a one-sample lock onto it) send pitch/yaw to the wrong axes during motion.
    // Hold that SDK pose while the still-cal gyro mean says the head is parked.
    imu_quat_type sdk_nwu = quaternion_eus_to_nwu(rayneo_sdk_head_quat());
    float excess = rayneo_imu_bridge_gyro_excess_dps();
    imu_pose_type pose = (imu_pose_type){0};
    pose.orientation = imu_still_hold_update(
        &rayneo_still_hold, sdk_nwu, excess,
        RAYNEO_STILL_HOLD_ENTER_DPS, RAYNEO_STILL_HOLD_EXIT_DPS, RAYNEO_STILL_HOLD_ENTER_SAMPLES);
    pose.has_orientation = true;
    pose.timestamp_ms = (uint32_t)(timestamp / TS_TO_MS_FACTOR);
    connection_pool_ingest_pose(RAYNEO_DRIVER_ID, pose);
}

static bool rayneo_fusion_start_locked(void) {
    if (rayneo_fusion_open) return true;

    if (!rayneo_imu_bridge_attach(&rayneo_fusion_imu)) {
        log_error("RayNeo driver, failed to attach IMU fusion bridge\n");
        return false;
    }

    device_imu_error_type err = device_imu_open(&rayneo_fusion_imu, rayneo_fusion_event);
    if (err != DEVICE_IMU_ERROR_NO_ERROR) {
        log_error("RayNeo driver, device_imu_open failed for fusion bridge (%d)\n", err);
        device_imu_close(&rayneo_fusion_imu);
        return false;
    }

    rayneo_pose_hold_reset();
    rayneo_imu_bridge_set_calibrating(true);
    rayneo_fusion_open = true;
    if (pthread_create(&rayneo_cal_thread, NULL, rayneo_calibrate_thread, NULL) != 0) {
        log_error("RayNeo driver, failed to start still-calibration thread\n");
        rayneo_fusion_open = false;
        rayneo_imu_bridge_set_calibrating(false);
        device_imu_close(&rayneo_fusion_imu);
        return false;
    }
    rayneo_cal_thread_started = true;
    if (config()->debug_device) {
        log_debug("RayNeo driver, IMU fusion bridge started\n");
    }
    return true;
}

static void rayneo_fusion_stop_locked(void) {
    if (!rayneo_fusion_open && !rayneo_cal_thread_started) return;

    rayneo_imu_bridge_request_stop();
    if (rayneo_cal_thread_started) {
        pthread_join(rayneo_cal_thread, NULL);
        rayneo_cal_thread_started = false;
    }

    if (!rayneo_fusion_open) return;

    device_imu_close(&rayneo_fusion_imu);
    rayneo_fusion_open = false;
    rayneo_pose_hold_reset();
    rayneo_imu_bridge_reset();
    if (config()->debug_device) {
        log_debug("RayNeo driver, IMU fusion bridge stopped\n");
    }
}

void rayneo_imu_callback(const float acc[3], const float gyro[3], const float mag[3], uint64_t timestamp){
    if (!soft_connected || driver_disabled() || !rayneo_fusion_open) return;

    rayneo_imu_bridge_enqueue(acc, gyro, mag, timestamp);
    if (!rayneo_imu_bridge_is_calibrating()) {
        device_imu_read(&rayneo_fusion_imu, 0);
    }
}

static pthread_mutex_t device_name_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t device_name_cond = PTHREAD_COND_INITIALIZER;
static char* device_brand = NULL;
static char* device_model = NULL;
static void rayneo_mcu_callback(uint32_t state, uint64_t timestamp, size_t length, const void* data) {
    uint32_t ts = (uint32_t) (timestamp / TS_TO_MS_FACTOR);
    if (!soft_connected) return;
    if (state == STATE_EVENT_DEVICE_INFO) {
        pthread_mutex_lock(&device_name_mutex);
        if (device_brand == NULL && device_model == NULL) {
            char device_type[64];
            GetDeviceType(device_type);
            if (config()->debug_device) log_debug("RayNeo driver, received device type: %s\n", device_type);

            bool device_found = false;
            if (strlen(device_type) > 0) {
                char compact[64];
                size_t compact_n = 0;
                for (const unsigned char* p = (const unsigned char*)device_type;
                     *p && compact_n + 1 < sizeof(compact); p++) {
                    if (*p == ' ' || *p == '-' || *p == '_' || *p == '/') continue;
                    compact[compact_n++] = (char)tolower(*p);
                }
                compact[compact_n] = '\0';

                if (strstr(compact, "air4pro") != NULL) {
                    device_brand = strdup("RayNeo");
                    device_model = strdup("Air 4 Pro");
                    device_found = (device_brand != NULL && device_model != NULL);
                } else {
                    char device_type_copy[64];
                    strncpy(device_type_copy, device_type, sizeof(device_type_copy) - 1);
                    device_type_copy[sizeof(device_type_copy) - 1] = '\0';

                    char* space = strchr(device_type_copy, ' ');
                    if (space) {
                        *space = '\0';
                        char* model_part = space + 1;
                        while (*model_part == ' ') model_part++;
                        if (strlen(device_type_copy) > 0 && strlen(model_part) > 0) {
                            device_brand = strdup(device_type_copy);
                            device_model = strdup(model_part);
                            device_found = (device_brand != NULL && device_model != NULL);
                        }
                    }
                }
            }
            if (device_found) pthread_cond_signal(&device_name_cond);
        }
        pthread_mutex_unlock(&device_name_mutex);

        is_sbs_mode = GetSideBySideStatus() == 1;
    }
}

static pthread_mutex_t device_connection_mutex = PTHREAD_MUTEX_INITIALIZER;
bool rayneo_device_connect() {
    if (!rayneo_sdk_load()) return false;

    pthread_mutex_lock(&device_connection_mutex);
    if (!soft_connected) {
        if (!hard_connected) {
            RegisterIMUEventCallback(rayneo_imu_callback);
            RegisterStateEventCallback(rayneo_mcu_callback);
            if (EstablishUsbConnection(RAYNEO_ID_VENDOR, RAYNEO_ID_PRODUCT) == 0) {
                NotifyDeviceConnected();
                hard_connected = true;
            }
        }
        if (hard_connected) {
            StartXR();
            OpenIMU();
            if (!rayneo_fusion_start_locked()) {
                log_error("RayNeo driver, IMU fusion bridge failed; leaving device unsupported\n");
                CloseIMU();
                StopXR();
                NotifyDeviceDisconnected();
                ResetUsbConnection();
                UnregisterIMUEventCallback(rayneo_imu_callback);
                UnregisterStateEventCallback(rayneo_mcu_callback);
                free_and_clear(&device_brand);
                free_and_clear(&device_model);
                hard_connected = false;
            } else {
                soft_connected = true;

                // this will trigger the STATE_EVENT_DEVICE_INFO event
                AcquireDeviceInfo();
            }
        } else {
            log_message("RayNeo driver, failed to establish a connection\n");
        }
    }
    pthread_mutex_unlock(&device_connection_mutex);

    return soft_connected;
};

void rayneo_device_disconnect(bool soft, bool is_device_present) {
    pthread_mutex_lock(&device_connection_mutex);
    if (soft_connected) {
        CloseIMU();
        StopXR();
        rayneo_fusion_stop_locked();
        soft_connected = false;
    }

    bool retain_hard_connection = soft && is_device_present;
    if (hard_connected && !retain_hard_connection) {
        NotifyDeviceDisconnected();
        ResetUsbConnection();
        UnregisterIMUEventCallback(rayneo_imu_callback);
        UnregisterStateEventCallback(rayneo_mcu_callback);
        free_and_clear(&device_brand);
        free_and_clear(&device_model);
        hard_connected = false;
    }
    pthread_mutex_unlock(&device_connection_mutex);
};

device_properties_type* rayneo_supported_device(uint16_t vendor_id, uint16_t product_id, uint8_t usb_bus, uint8_t usb_address) {
    if (vendor_id == RAYNEO_ID_VENDOR && product_id == RAYNEO_ID_PRODUCT) {
        if (!rayneo_sdk_load()) return NULL;

        device_properties_type* device = calloc(1, sizeof(device_properties_type));
        *device = rayneo_properties;

        // trying to connect to the device too quickly seems to cause irrecoverable connection issues
        sleep(2);

        // device_connect is actually out-of-turn here, the driver would normally call connect after we return the device 
        // properties, but we kick this off now so we can acquire the device name, which unfortunately comes from the SDK 
        // only after establishing a connection.
        if (rayneo_device_connect()) {
            pthread_mutex_lock(&device_name_mutex);
            while (device_brand == NULL && device_model == NULL) {
                pthread_cond_wait(&device_name_cond, &device_name_mutex);
            }
            pthread_mutex_unlock(&device_name_mutex);
            device->brand = device_brand;
            device->model = device_model;

            // Leave the connection open if we think it'll be used, but if the driver is disabled, disconnect now
            if (driver_disabled()) rayneo_device_disconnect(true, true);

            return device;
        }
    }

    return NULL;
};

void rayneo_block_on_device() {
    device_properties_type* device = device_checkout();
    bool imu_started = false;
    if (soft_connected && device != NULL) {
        // Still-cal holds device_imu_read, so no poses flow until it finishes.
        // wait_for_imu_start() only retries 5s and would disconnect otherwise.
        int cal_wait_s = 0;
        while (soft_connected && rayneo_imu_bridge_is_calibrating() &&
               cal_wait_s < RAYNEO_STILL_CAL_WAIT_S + 15) {
            sleep(1);
            cal_wait_s++;
        }
        imu_started = wait_for_imu_start();
    }
    while (soft_connected && device != NULL && imu_started && is_imu_alive()) {
        sleep(1);
    }

    rayneo_device_disconnect(true, device != NULL);
    device_checkin(device);
};

bool rayneo_device_is_sbs_mode() {
    return is_sbs_mode;
};

bool rayneo_device_set_sbs_mode(bool enabled) {
    if (!rayneo_sdk_load()) return false;

    // don't explicitly change the is_sbs_mode value here, wait for it to come back around from the MCU deviceinfo response
    if (enabled) {
        SwitchTo3D();
    } else {
        SwitchTo2D();
    }

    return true;
};

bool rayneo_is_connected() {
    return soft_connected;
};

void rayneo_disconnect(bool soft) {
    rayneo_device_disconnect(soft, device_present());
};

const device_driver_type rayneo_driver = {
    .id                                 = RAYNEO_DRIVER_ID,
    .supported_device_func              = rayneo_supported_device,
    .device_connect_func                = rayneo_device_connect,
    .block_on_device_func               = rayneo_block_on_device,
    .device_is_sbs_mode_func            = rayneo_device_is_sbs_mode,
    .device_set_sbs_mode_func           = rayneo_device_set_sbs_mode,
    .is_connected_func                  = rayneo_is_connected,
    .disconnect_func                    = rayneo_disconnect
};
