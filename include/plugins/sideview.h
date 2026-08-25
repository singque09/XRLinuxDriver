#pragma once

#include "plugins.h"

#define SIDEVIEW_POSITION_COUNT 9
#define SIDEVIEW_POSITION_MIDDLE_CENTER "middle_center"
extern const char *sideview_position_names[SIDEVIEW_POSITION_COUNT];

struct sideview_ipc_values_t {
    bool *enabled;
    float *position;
};
typedef struct sideview_ipc_values_t sideview_ipc_values_type;

struct sideview_config_t {
    bool enabled;
    int position;
    bool smooth_follow_enabled;
};
typedef struct sideview_config_t sideview_config;

extern const plugin_type sideview_plugin;