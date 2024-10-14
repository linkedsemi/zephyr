/*
 * Copyright (c) 2022 linkedsemi Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <core_rv32.h>

#define DCACHE_LINE_SIZE 32
#define ICACHE_LINE_SIZE 32

#define ROUND_DOWN_CACHE_LINE(x) (((x) >> 5) << 5)

void cache_data_enable(void)
{
    csi_dcache_enable();
}

void cache_data_disable(void)
{
    csi_dcache_disable();
}

void cache_instr_enable(void)
{
    csi_icache_enable();
}

void cache_instr_disable(void)
{
    csi_icache_disable();
}

int cache_data_invd_all(void)
{
    csi_dcache_invalid();

    return 0;
}

int cache_data_invd_range(void *addr, size_t size)
{
    void *align_addr = (void *)ROUND_DOWN_CACHE_LINE((uint32_t)addr);
    csi_dcache_invalid_range((uint32_t *)align_addr, size);

    return 0;
}

int cache_instr_invd_all(void)
{
    csi_icache_invalid();

    return 0;
}

int cache_instr_invd_range(void *addr, size_t size)
{
    return -ENOTSUP;
}

int cache_data_flush_all(void)
{
    csi_dcache_clean();

    return 0;
}

int cache_data_flush_and_invd_all(void)
{
    csi_dcache_clean_invalid();

    return 0;
}

int cache_data_flush_range(void *addr, size_t size)
{
    void *align_addr = (void *)ROUND_DOWN_CACHE_LINE((uint32_t)addr);
    csi_dcache_clean_range((uint32_t *)align_addr, size);

    return 0;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
    csi_dcache_clean_invalid_range(addr, size);

    return 0;
}

int cache_instr_flush_all(void)
{
    return -ENOTSUP;
}

int cache_instr_flush_and_invd_all(void)
{
    return -ENOTSUP;
}

int cache_instr_flush_range(void *addr, size_t size)
{
    return -ENOTSUP;
}

int cache_instr_flush_and_invd_range(void *addr, size_t size)
{
    return -ENOTSUP;
}

#ifdef CONFIG_DCACHE_LINE_SIZE_DETECT
size_t cache_data_line_size_get(void)
{
    return DCACHE_LINE_SIZE;
}
#endif /* CONFIG_DCACHE_LINE_SIZE_DETECT */

#ifdef CONFIG_ICACHE_LINE_SIZE_DETECT
size_t cache_instr_line_size_get(void)
{
    return ICACHE_LINE_SIZE;
}
#endif /* CONFIG_ICACHE_LINE_SIZE_DETECT */
