/*
 * Copyright (c) 2022 linkedsemi Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <core_rv32.h>

BUILD_ASSERT(CONFIG_ICACHE_LINE_SIZE > 0);
BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE > 0);

__no_optimization bool is_cache_region(uint32_t addr)
{
#if defined(CONFIG_NOCACHE_MEMORY)
    __maybe_unused const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    __maybe_unused const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    __maybe_unused const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
    if ((addr >=__nocache_ram_start) && (addr < __nocache_ram_end)) {
        return false;
    } else {
        return true;
    }
#else
    return true;
#endif
}

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
    if (!is_cache_region((uint32_t)addr)) {
        return 0;
    }

    __ASSERT(IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE),
                        "buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
    csi_dcache_invalid_range(addr, size);

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
    if (!is_cache_region((uint32_t)addr)) {
        return 0;
    }

    __ASSERT(IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE),
                        "buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
    csi_dcache_clean_range(addr, size);

    return 0;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
    if (!is_cache_region((uint32_t)addr)) {
        return 0;
    }

    __ASSERT(IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE),
                        "buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
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
    return CONFIG_DCACHE_LINE_SIZE;
}
#endif /* CONFIG_DCACHE_LINE_SIZE_DETECT */

#ifdef CONFIG_ICACHE_LINE_SIZE_DETECT
size_t cache_instr_line_size_get(void)
{
    return CONFIG_ICACHE_LINE_SIZE;
}
#endif /* CONFIG_ICACHE_LINE_SIZE_DETECT */
