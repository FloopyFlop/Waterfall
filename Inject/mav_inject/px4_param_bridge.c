/*
 * PX4 Parameter Bridge - Direct C API Access for Python
 *
 * This shared library provides a simple C interface to PX4's parameter system
 * that can be called from Python using ctypes.
 *
 * NOTE: This library requires special linking to work:
 * - It needs to be loaded into a process that already has PX4's parameter
 *   symbols loaded (i.e., the PX4 process itself)
 * - OR it needs to be statically linked against libparameters.a from PX4
 * - OR we need to use LD_PRELOAD to inject it into PX4's process
 *
 * For now, this will work with MAVLink fallback if C API fails to load.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <dlfcn.h>

// PX4 parameter system types
typedef uintptr_t param_t;

typedef enum {
    PARAM_TYPE_INT32 = 0,
    PARAM_TYPE_FLOAT = 1
} param_type_t;

// Function pointers to PX4 parameter functions
// We'll try to dynamically load these from the running PX4 process
static param_t (*_param_find)(const char *name) = NULL;
static int (*_param_get)(param_t param, void *val) = NULL;
static int (*_param_set)(param_t param, const void *val) = NULL;
static int (*_param_set_no_notification)(param_t param, const void *val) = NULL;
static int (*_param_type)(param_t param) = NULL;

// Initialize function pointers by searching in the current process
static void init_param_functions(void) {
    static int initialized = 0;
    if (initialized) return;

    initialized = 1;

    // Try to find the functions in the current process
    // This will work if we're loaded into PX4's process space
    _param_find = dlsym(RTLD_DEFAULT, "param_find");
    _param_get = dlsym(RTLD_DEFAULT, "param_get");
    _param_set = dlsym(RTLD_DEFAULT, "param_set");
    _param_set_no_notification = dlsym(RTLD_DEFAULT, "param_set_no_notification");
    _param_type = dlsym(RTLD_DEFAULT, "param_type");

    if (!_param_find || !_param_get || !_param_set || !_param_type) {
        fprintf(stderr, "Warning: Could not find PX4 parameter functions in process.\n");
        fprintf(stderr, "This library must be loaded into PX4's process space.\n");
    }
}

// Bridge functions with simple C interface for Python ctypes

/**
 * Get parameter handle by name
 * Returns 0 if parameter not found
 */
param_t px4_param_find(const char *name) {
    init_param_functions();

    if (!_param_find) {
        fprintf(stderr, "px4_param_find: param_find function not available\n");
        return 0;
    }

    if (!name) {
        fprintf(stderr, "px4_param_find: NULL name\n");
        return 0;
    }

    param_t handle = _param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_find: Parameter '%s' not found\n", name);
    }
    return handle;
}

/**
 * Get float parameter value
 * Returns 0 on success, -1 on error
 */
int px4_param_get_float(const char *name, float *value) {
    if (!name || !value) {
        fprintf(stderr, "px4_param_get_float: NULL argument\n");
        return -1;
    }

    param_t handle = param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_get_float: Parameter '%s' not found\n", name);
        return -1;
    }

    if (param_type(handle) != PARAM_TYPE_FLOAT) {
        fprintf(stderr, "px4_param_get_float: Parameter '%s' is not FLOAT type\n", name);
        return -1;
    }

    int ret = param_get(handle, value);
    if (ret != 0) {
        fprintf(stderr, "px4_param_get_float: Failed to get value for '%s' (error %d)\n", name, ret);
        return -1;
    }

    printf("px4_param_get_float: %s = %f\n", name, *value);
    return 0;
}

/**
 * Set float parameter value
 * Returns 0 on success, -1 on error
 */
int px4_param_set_float(const char *name, float value) {
    if (!name) {
        fprintf(stderr, "px4_param_set_float: NULL name\n");
        return -1;
    }

    param_t handle = param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_set_float: Parameter '%s' not found\n", name);
        return -1;
    }

    if (param_type(handle) != PARAM_TYPE_FLOAT) {
        fprintf(stderr, "px4_param_set_float: Parameter '%s' is not FLOAT type\n", name);
        return -1;
    }

    printf("px4_param_set_float: Setting %s = %f\n", name, value);

    int ret = param_set(handle, &value);
    if (ret != 0) {
        fprintf(stderr, "px4_param_set_float: Failed to set value for '%s' (error %d)\n", name, ret);
        return -1;
    }

    printf("px4_param_set_float: Successfully set %s = %f\n", name, value);
    return 0;
}

/**
 * Get int32 parameter value
 * Returns 0 on success, -1 on error
 */
int px4_param_get_int32(const char *name, int32_t *value) {
    if (!name || !value) {
        fprintf(stderr, "px4_param_get_int32: NULL argument\n");
        return -1;
    }

    param_t handle = param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_get_int32: Parameter '%s' not found\n", name);
        return -1;
    }

    if (param_type(handle) != PARAM_TYPE_INT32) {
        fprintf(stderr, "px4_param_get_int32: Parameter '%s' is not INT32 type\n", name);
        return -1;
    }

    int ret = param_get(handle, value);
    if (ret != 0) {
        fprintf(stderr, "px4_param_get_int32: Failed to get value for '%s' (error %d)\n", name, ret);
        return -1;
    }

    printf("px4_param_get_int32: %s = %d\n", name, *value);
    return 0;
}

/**
 * Set int32 parameter value
 * Returns 0 on success, -1 on error
 */
int px4_param_set_int32(const char *name, int32_t value) {
    if (!name) {
        fprintf(stderr, "px4_param_set_int32: NULL name\n");
        return -1;
    }

    param_t handle = param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_set_int32: Parameter '%s' not found\n", name);
        return -1;
    }

    if (param_type(handle) != PARAM_TYPE_INT32) {
        fprintf(stderr, "px4_param_set_int32: Parameter '%s' is not INT32 type\n", name);
        return -1;
    }

    printf("px4_param_set_int32: Setting %s = %d\n", name, value);

    int ret = param_set(handle, &value);
    if (ret != 0) {
        fprintf(stderr, "px4_param_set_int32: Failed to set value for '%s' (error %d)\n", name, ret);
        return -1;
    }

    printf("px4_param_set_int32: Successfully set %s = %d\n", name, value);
    return 0;
}

/**
 * Get parameter type
 * Returns 0 for INT32, 1 for FLOAT, -1 on error
 */
int px4_param_get_type(const char *name) {
    if (!name) {
        fprintf(stderr, "px4_param_get_type: NULL name\n");
        return -1;
    }

    param_t handle = param_find(name);
    if (handle == 0) {
        fprintf(stderr, "px4_param_get_type: Parameter '%s' not found\n", name);
        return -1;
    }

    param_type_t type = param_type(handle);
    printf("px4_param_get_type: %s is type %d\n", name, (int)type);
    return (int)type;
}
