/*
 * Copyright (c) 2022 linkedsemi Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <core_rv32.h>

LOG_MODULE_REGISTER(cache_linkedsemi, CONFIG_CACHE_LOG_LEVEL);

BUILD_ASSERT(CONFIG_ICACHE_LINE_SIZE > 0);
BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE > 0);

#if defined(CONFIG_PSRAM)
extern char __PSRAM_start[];
extern char __PSRAM_end[];
extern char __PSRAM_size[];
#endif /* CONFIG_PSRAM */

__no_optimization bool is_cache_region(uint32_t addr)
{
#if defined(CONFIG_NOCACHE_MEMORY)
    __maybe_unused const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
    __maybe_unused const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
    __maybe_unused const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
    __maybe_unused const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    __maybe_unused const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    __maybe_unused const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
#if defined(CONFIG_PSRAM)
    __maybe_unused const uint32_t ___PSRAM_start = (uint32_t)__PSRAM_start;
    __maybe_unused const uint32_t ___PSRAM_end = (uint32_t)__PSRAM_end;
    __maybe_unused const uint32_t ___PSRAM_size = (uint32_t)__PSRAM_size;
    if (((addr >=__nocache_ram_start) && (addr < __nocache_ram_end))
        || (addr < __image_ram_start)
        || ((addr >= __image_ram_end) && (addr < ___PSRAM_start))
        || (addr >= ___PSRAM_end)) {
        return false;
#else /* defined(CONFIG_PSRAM) */
    if (((addr >=__nocache_ram_start) && (addr < __nocache_ram_end))
        || (addr < __image_ram_start)
        || (addr >= __image_ram_end)) {
        return false;
#endif /* defined(CONFIG_PSRAM) */
    } else {
        return true;
    }
#else /*  defined(CONFIG_NOCACHE_MEMORY) */
    return true;
#endif /*  defined(CONFIG_NOCACHE_MEMORY) */
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

    if(!IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE)) {
        LOG_WRN("buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
    }
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

    if(!IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE)) {
        LOG_WRN("buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
    }
    csi_dcache_clean_range(addr, size);

    return 0;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
    if (!is_cache_region((uint32_t)addr)) {
        return 0;
    }

    if(!IS_ALIGNED(addr, CONFIG_DCACHE_LINE_SIZE)) {
        LOG_WRN("buffer[%p] should be aligned to cache line[%d bytes]",
                        addr, CONFIG_DCACHE_LINE_SIZE);
    }
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
