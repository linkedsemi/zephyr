/*
 * Copyright (c) 2025 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LS_SOC_PLATFORM_H_
#define _LS_SOC_PLATFORM_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif


#if defined(CONFIG_CACHE)
#if defined(CONFIG_DCACHE_LINE_SIZE)
#if (CONFIG_DCACHE_LINE_SIZE > 0)
static inline bool soc_check_addr_dcache_aligned(uint32_t addr)
{
    if(!IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE)) {
        return false;
    }
    return true;
}
#else
#error CONFIG_DCACHE_LINE_SIZE is requred
#endif /* (CONFIG_DCACHE_LINE_SIZE > 0) */
#endif /* CONFIG_DCACHE_LINE_SIZE */
#endif /* CONFIG_CACHE */

#ifdef __cplusplus
}
#endif
#endif /* _LS_SOC_PLATFORM_H_ */
