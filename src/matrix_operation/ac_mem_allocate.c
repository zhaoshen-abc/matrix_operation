/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2021-2022 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they
 * were provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit
 * this software or the related documents without Intel's prior written permission.
 * This software and the related documents are provided as is, with no express
 * or implied warranties, other than those that are expressly
 * stated in the License.
 */

#include "ac_mem_allocate.h"
#include "auto_calib_internal.h"

ia_err ac_mem_init(ac_mem_t *ac_mem_alloc_ptr, size_t total_size)
{
    if (ac_mem_alloc_ptr == NULL)
    {
        return ia_err_argument;
    }
    ac_mem_alloc_ptr->start_ptr = NULL;
    ac_mem_alloc_ptr->start_ptr = IA_ALLOC(total_size);
    if (ac_mem_alloc_ptr->start_ptr == NULL)
    {
        return ia_err_nomemory;
    }
    ac_mem_alloc_ptr->total_size = total_size;
    IA_MEMSET(ac_mem_alloc_ptr->var_info, 0, MAX_COCURRENT_OFFSET_NUM * sizeof(var_info_t));
    ac_mem_alloc_ptr->var_in_use_counter = 0U;
    return ia_err_none;
}

ia_err ac_mem_reset(ac_mem_t *ac_mem_alloc_ptr)
{
    if (ac_mem_alloc_ptr == NULL)
    {
        return ia_err_argument;
    }
    ac_mem_alloc_ptr->var_in_use_counter = 0U;
    IA_MEMSET(&ac_mem_alloc_ptr->var_info, 0, MAX_COCURRENT_OFFSET_NUM * sizeof(var_info_t));
    return ia_err_none;
}

static bool is_overlap(size_t x_0, size_t x_1, size_t y_0, size_t y_1)
{
    if (x_0 < y_1 && y_0 < x_1)
    {
        return true;
    }
    return false;
}

ia_err ac_mem_alloc_var(ac_mem_t *ac_mem_alloc_ptr, size_t var_size, void **var_ptr, size_t offset)
{
    if (ac_mem_alloc_ptr == NULL || var_ptr == NULL) {
        return ia_err_argument;
    }
    int32_t empty_index = -1;
    int32_t i = 0;
    for (i = 0; i < (int32_t)MAX_COCURRENT_OFFSET_NUM; i++)
    {
        if (ac_mem_alloc_ptr->var_info[i].in_use == true)
        {
            if (is_overlap(ac_mem_alloc_ptr->var_info[i].offset,
                ac_mem_alloc_ptr->var_info[i].offset + ac_mem_alloc_ptr->var_info[i].size,
                offset,
                offset + var_size) == true &&
                offset + var_size > ac_mem_alloc_ptr->total_size) {
                AC_LOG("USE in-used memory! plsease check the json file and regenerate the header file!");
                return ia_err_nomemory;
            }
        } else {
            empty_index = i;
        }
    }
    if (empty_index >= 0){
        ac_mem_alloc_ptr->var_info[empty_index].in_use = true;
        ac_mem_alloc_ptr->var_info[empty_index].offset = offset;
        ac_mem_alloc_ptr->var_info[empty_index].size = var_size;
        *var_ptr = ac_mem_alloc_ptr->start_ptr + offset;
        ac_mem_alloc_ptr->var_in_use_counter += 1U;
    } else {
        AC_LOG("The large size variable information in code and json file is not synchronized, plsease update the json and regenerate the header file!");
        return ia_err_nomemory;
    }
    return ia_err_none;
}

ia_err ac_mem_free_var(ac_mem_t *ac_mem_alloc_ptr, void **free_var)
{
    if (ac_mem_alloc_ptr == NULL ||
        free_var == NULL) {
        return ia_err_argument;
    }
    size_t offset = (size_t)*free_var - (size_t)ac_mem_alloc_ptr->start_ptr;
    int32_t i = 0;
    for (i = 0; i < (int32_t)MAX_COCURRENT_OFFSET_NUM; i++)
    {
        if (ac_mem_alloc_ptr->var_info[i].in_use == true && ac_mem_alloc_ptr->var_info[i].offset == offset)
        {
            ac_mem_alloc_ptr->var_info[i].in_use = false;
            ac_mem_alloc_ptr->var_info[i].offset = 0U;
            ac_mem_alloc_ptr->var_info[i].size = 0U;
            *free_var = NULL;
            ac_mem_alloc_ptr->var_in_use_counter -= 1U;
            return ia_err_none;
        }
    }
    AC_LOG("The freed pointer have a changed address %p", *free_var);
    return ia_err_internal;
}

ia_err ac_mem_deinit(ac_mem_t *ac_mem_alloc_ptr)
{
    if (ac_mem_alloc_ptr == NULL) {
        return ia_err_argument;
    }
    IA_FREEZ(ac_mem_alloc_ptr->start_ptr);
    ac_mem_alloc_ptr->start_ptr = NULL;
    ac_mem_alloc_ptr->total_size = 0U;
    IA_MEMSET(ac_mem_alloc_ptr->var_info, 0, MAX_COCURRENT_OFFSET_NUM * sizeof(var_info_t));
    ac_mem_alloc_ptr->var_in_use_counter = 0U;
    return ia_err_none;
}