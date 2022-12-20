/*
 * INTEL CONFIDENTIAL
 * Copyright (c) 2012-2022 Intel Corporation
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code ("Material") are owned by Intel Corporation or its
 * suppliers or licensors. Title to the Material remains with Intel
 * Corporation or its suppliers and licensors. The Material may contain trade
 * secrets and proprietary and confidential information of Intel Corporation
 * and its suppliers and licensors, and is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may be
 * used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 *
 * Unless otherwise agreed by Intel in writing, you may not remove or alter
 * this notice or any other notice embedded in Materials by Intel or Intels
 * suppliers or licensors in any way.
 */

#ifdef __ANDROID__
#include <utils/Log.h>
#else
#include <stdio.h>
#endif

#include <stdarg.h>
#include <time.h>
#include "ia_log.h"
// #include "ia_time_meas.h"

#ifndef _MSC_VER

#ifndef ENABLE_CUSTOMIZED_STD_LIB
#include <sys/time.h>
#else
#include "log.h"
#endif //ENABLE_CUSTOMIZED_STD_LIB
#else
#include <Windows.h>
#endif

#ifdef _WIN32
#pragma warning (disable: 4127) // Don't complain about conditional expression is constant
#endif

#ifndef ENABLE_CUSTOMIZED_STD_LIB
#define PRINT_ERR(fmt, args) \
            do { \
                vfprintf(stderr, (fmt), (args)); \
                fprintf(stderr, "\n"); \
            } while(0)

#define PRINT_OUT(fmt, args) \
            do { \
                vfprintf(stdout, (fmt), (args)); \
                fprintf(stdout, "\n"); \
            } while(0)
#else
#define PRINT_ERR(fmt, args) \
            do { \
                vprintf((fmt), (args));\
                printf("\n");\
            } while(0)
#define PRINT_OUT PRINT_ERR
#endif //ENABLE_CUSTOMIZED_STD_LIB

#ifndef _MSC_VER

#ifndef ENABLE_CUSTOMIZED_STD_LIB

static void print_ts()
{
    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char buf[32];

    gettimeofday(&tv, NULL);
    nowtime = tv.tv_sec;
    nowtm = localtime(&nowtime);
    if (nowtm)
    {
        strftime(buf, sizeof buf, "%H:%M:%S", nowtm);
        fprintf(stdout, "%s.%06ld ", buf, tv.tv_usec);
    }
    return;
}
#endif //ENABLE_CUSTOMIZED_STD_LIB

#else

static void print_ts()
{
    LARGE_INTEGER time;
    LARGE_INTEGER frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&time);

    //prints time in microseconds, the last 9 digits
    fprintf(stdout, "%9lld ", time.QuadPart*1000000/frequency.QuadPart%1000000000);
    return;
}
#endif

static ia_env *g_env_ptr = NULL;

ia_err ia_log_init(ia_env *a_env_ptr)
{
#ifdef _MSC_VER
    float temp = 1.2f; /* Force loading float-point support */
    temp = temp;
#endif
    g_env_ptr = (ia_env*)malloc(sizeof(ia_env));

    if (g_env_ptr != NULL)
    {
        *g_env_ptr = *a_env_ptr;
        // ia_time_meas_init();
        return ia_err_none;
    }
    else
    {
        return ia_err_nomemory;
    }
}

#ifndef ENABLE_CUSTOMIZED_STD_LIB
void ia_log(ia_log_level level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    if (level == ia_log_error)
    {
        if (g_env_ptr != NULL)
        {
            if (g_env_ptr->verror != NULL)
            {
                g_env_ptr->verror(fmt, args);
            }
        }
        else
        {
#ifdef __ANDROID__
            __android_log_vprint(ANDROID_LOG_ERROR, "IA_IMAGING", fmt, args);
#else
            PRINT_ERR(fmt, args);
#endif
        }
    }
    else if (level == ia_log_debug)
    {
        if (g_env_ptr != NULL)
        {
            if (g_env_ptr->vdebug != NULL)
            {
                g_env_ptr->vdebug(fmt, args);
            }
        }
        else
        {
#ifdef __ANDROID__
            __android_log_vprint(ANDROID_LOG_DEBUG, "IA_IMAGING", fmt, args);
#else
            PRINT_OUT(fmt, args);
#endif
        }
    }
    else if (level == ia_log_info)
    {
        if (g_env_ptr != NULL)
        {
            if (g_env_ptr->vinfo != NULL)
            {
                g_env_ptr->vinfo(fmt, args);
            }
        }
        else
        {
#ifdef __ANDROID__
            __android_log_vprint(ANDROID_LOG_INFO, "IA_IMAGING", fmt, args);
#else
            print_ts();
            PRINT_OUT(fmt, args);
#endif
        }
    }
    else if (level == ia_log_perf)
    {
        if (g_env_ptr != NULL)
        {
            if (g_env_ptr->vperf != NULL)
            {
                g_env_ptr->vperf(fmt, args);
            }
        }
        else
        {
#ifdef __ANDROID__
            __android_log_vprint(ANDROID_LOG_INFO, "IA_IMAGING", fmt, args);
#else
            print_ts();
            PRINT_OUT(fmt, args);
#endif
        }
    }
    else
    {
        ;//no action required
    }
    va_end(args);
}
#else //ENABLE_CUSTOMIZED_STD_LIB
void ia_log(ia_log_level level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    switch (level)
    {
    case ia_log_error:
        if ((g_env_ptr != NULL) && (g_env_ptr->verror != NULL))
        {
            g_env_ptr->verror(fmt, args);
        }
        else
        {
            if (log_level_enabled((uint32_t)LOG_LVL_ERR) == 0U) {
                break;
            }
            if (log_tag_filtered(LOG_TAG) == 0U) {
                break;
            }
            log_print("", (uint32_t)LOG_LVL_ERR, fmt, args);
        }
        break;
    case ia_log_debug:
        if ((g_env_ptr != NULL) && (g_env_ptr->vdebug != NULL))
        {
            g_env_ptr->vdebug(fmt, args);
        }
        else
        {
            if (log_level_enabled((uint32_t)LOG_LVL_DBG) == 0U) {
                break;
            }
            if (log_tag_filtered(LOG_TAG) == 0U) {
                break;
            }
            log_print("", (uint32_t)LOG_LVL_DBG, fmt, args);
        }
        break;
    case ia_log_info:
        if ((g_env_ptr != NULL) && (g_env_ptr->vinfo != NULL))
        {
            g_env_ptr->vinfo(fmt, args);
        }
        else
        {
            if (log_level_enabled((uint32_t)LOG_LVL_INFO) == 0U) {
                break;
            }
            if (log_tag_filtered(LOG_TAG) == 0U) {
                break;
            }
            log_print("", (uint32_t)LOG_LVL_INFO, fmt, args);
        }
        break;
    case ia_log_perf:
        if ((g_env_ptr != NULL) && (g_env_ptr->vperf != NULL))
        {
            g_env_ptr->vperf(fmt, args);
        }
        else
        {
            if (log_level_enabled((uint32_t)LOG_LVL_PERF) == 0U) {
                break;
            }
            if (log_tag_filtered(LOG_TAG) == 0U) {
                break;
            }
            log_print("", (uint32_t)LOG_LVL_PERF, fmt, args);
        }
        break;
    default:
        ; //no action required
        break;
    }

    va_end(args);
}
#endif //ENABLE_CUSTOMIZED_STD_LIB

const char *ia_log_strerror(ia_err error)
{
    const char *error_string;
    switch (error)
    {
    case ia_err_none:
        error_string = "No error.";
        break;
    case ia_err_general:
        error_string = "General error.";
        break;
    case ia_err_nomemory:
        error_string = "Out of memory.";
        break;
    case ia_err_data:
        error_string = "Corrupted data.";
        break;
    case ia_err_internal:
        error_string = "Error in code.";
        break;
    case ia_err_argument:
        error_string = "Invalid argument for a function.";
        break;
    default:
        error_string = "Undefined error.";
        break;
    }
    return error_string;
}

void ia_log_deinit(void)
{
    // ia_time_meas_uninit();
    if (g_env_ptr != NULL)
    {
        free(g_env_ptr);
        g_env_ptr = NULL;
    }
}