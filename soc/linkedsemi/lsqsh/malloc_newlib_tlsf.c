/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <errno.h>
#include <zephyr/sys/math_extras.h>
#include <string.h>
#ifdef CONFIG_MULTITHREADING
#include <zephyr/sys/mutex.h>
#endif
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/types.h>
#include "tlsf/tlsf.h"

#define LOG_LEVEL CONFIG_KERNEL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_NEWLIB_LIBC_MALLOC_TLSF_LSQSH

#if (CONFIG_NEWLIB_LIBC_MALLOC_ARENA_2_SIZE != 0)

#define POOL_SECTION __noinit
#define POOL_SECTION_2 __attribute__((section("PSRAM")))

#ifndef HEAP_ALIGN
#define HEAP_ALIGN    sizeof(double)
#endif

#if CONFIG_NEWLIB_LIBC_MALLOC_ARENA_2_SIZE > 0

#define HEAP_STATIC

/* Static allocation of heap in BSS */

#define HEAP_2_SIZE    ROUND_UP(CONFIG_NEWLIB_LIBC_MALLOC_ARENA_2_SIZE, HEAP_ALIGN)
#define HEAP_2_BASE    POINTER_TO_UINT(malloc_arena_2)

POOL_SECTION_2 unsigned char __aligned(HEAP_ALIGN) malloc_arena_2[HEAP_2_SIZE];

# endif /* CONFIG_NEWLIB_LIBC_MALLOC_ARENA_2_SIZE */

struct tlsf_mem_info
{
    tlsf_t tlsf;
    pool_t tlsf_pool;
    void* heap_addr;
    size_t heap_size;
    size_t used_size;
    size_t free_size;
};

Z_LIBC_DATA static struct tlsf_mem_info tlsf_heap_psram;

#ifdef CONFIG_MULTITHREADING
Z_LIBC_DATA SYS_MUTEX_DEFINE(z_malloc_heap_mutex);

static inline void malloc_lock(void)
{
    int lock_ret;
    lock_ret = sys_mutex_lock(&z_malloc_heap_mutex, K_FOREVER);
    __ASSERT_NO_MSG(lock_ret == 0);
}

static inline void malloc_unlock(void)
{
    (void) sys_mutex_unlock(&z_malloc_heap_mutex);
}

#else
#define malloc_lock()
#define malloc_unlock()
#endif

void *_malloc_r (struct _reent *r, size_t size)
{
    void *ptr;
    malloc_lock();
    ptr = tlsf_memalign(tlsf_heap_psram.tlsf,
                        COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                        size);
    if (ptr == NULL && size != 0)
        errno = ENOMEM;
    malloc_unlock();

    return ptr;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t requested_size)
{
    void *ret = NULL;

    if (NULL == ptr)
    {
        ret = _malloc_r(r, requested_size);
    }
    else if (0 == requested_size)
    {
        _free_r(r, ptr);
    }
    else
    {
        malloc_lock();

        ret = tlsf_realloc(tlsf_heap_psram.tlsf, ptr, requested_size);
        if (ret && !IS_ALIGNED(ret, COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t)))) )
        {
             void *align_ptr = _malloc_r(r, requested_size);
             if (align_ptr)
             {
                memcpy(align_ptr, ret, requested_size);
             }
             _free_r(r, ret);
             ret = align_ptr;
        }

        if (ret == NULL && requested_size != 0)
            errno = ENOMEM;

        malloc_unlock();
    }

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
    if (ptr != NULL)
    {
        malloc_lock();
        tlsf_free(tlsf_heap_psram.tlsf, ptr);
        malloc_unlock();
    }
}

void *_calloc_r(struct _reent *r, size_t nmemb, size_t size)
{
    void *ret;

    if (size_mul_overflow(nmemb, size, &size))
    {
        errno = ENOMEM;
        return NULL;
    }
    ret = _malloc_r(r, size);

    if (ret != NULL)
        memset(ret, 0, size);

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    void *ptr;
    malloc_lock();
    ptr = tlsf_memalign(tlsf_heap_psram.tlsf, alignment, size);
    if (ptr == NULL && size != 0) {
        errno = ENOMEM;
    }
    malloc_unlock();

    return ptr;
}

#ifdef CONFIG_GLIBCXX_LIBCPP

/*
 * GCC's libstdc++ may use this function instead of aligned_alloc due to a
 * bug in the configuration for "newlib" environments (which includes picolibc).
 * When toolchains including that bug fix can become a dependency for Zephyr,
 * this work-around can be removed.
 *
 * Note that aligned_alloc isn't defined to work as a replacement for
 * memalign as it requires that the size be a multiple of the alignment,
 * while memalign does not. However, the aligned_alloc implementation here
 * is just a wrapper around sys_heap_aligned_alloc which doesn't have that
 * requirement and so can be used by memalign.
 */

void *memalign(size_t alignment, size_t size)
{
    return aligned_alloc(alignment, size);
}
#endif

static int malloc_prepare(void)
{
    void *heap_base = NULL;
    size_t heap_size;

    heap_base = UINT_TO_POINTER(HEAP_2_BASE);
    heap_size = HEAP_2_SIZE;

    memset(&tlsf_heap_psram, 0, sizeof(struct tlsf_mem_info));

    tlsf_heap_psram.heap_addr = (void *)ROUND_UP(heap_base, tlsf_align_size());
    tlsf_heap_psram.heap_size = heap_size;
    tlsf_heap_psram.tlsf = tlsf_create_with_pool(tlsf_heap_psram.heap_addr, tlsf_heap_psram.heap_size);
    tlsf_heap_psram.tlsf_pool = tlsf_get_pool(tlsf_heap_psram.tlsf);

    return 0;
}
SYS_INIT(malloc_prepare, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_LIBC);

static void tlsf_walker_cb(void* ptr, size_t size, int used, void* user)
{
    struct tlsf_mem_info *info_ptr = user;

    printk("[0x%08x - ", (size_t)ptr);
    if (size < 1024)
        printk("%5d", size);
    else if (size < 1024 * 1024)
        printk("%4dK", size / 1024);
    else
        printk("%4dM", size / (1024 * 1024));

    if (used)
    {
        info_ptr->used_size += size;
        printk("    USED\n");
    }
    else
    {
        info_ptr->free_size += size;
        printk("\n");
    }
}

void tlsf_heap_info(void)
{
    tlsf_walk_pool(tlsf_heap_psram.tlsf_pool, tlsf_walker_cb, &tlsf_heap_psram);
    printk("\n-- used:%d  free:%d --\n", tlsf_heap_psram.used_size, tlsf_heap_psram.free_size);
    tlsf_heap_psram.used_size = 0;
    tlsf_heap_psram.free_size = 0;
}

#else /* No malloc arena */
void *malloc(size_t size)
{
    ARG_UNUSED(size);

    LOG_ERR("CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE is 0");
    errno = ENOMEM;

    return NULL;
}

void free(void *ptr)
{
    ARG_UNUSED(ptr);
}

void *realloc(void *ptr, size_t size)
{
    ARG_UNUSED(ptr);
    return malloc(size);
}
#endif /* else no malloc arena */

#endif /* CONFIG_NEWLIB_LIBC_MALLOC */

#ifdef CONFIG_NEWLIB_LIBC_REALLOCARRAY
void *reallocarray(void *ptr, size_t nmemb, size_t size)
{
    if (size_mul_overflow(nmemb, size, &size)) {
        errno = ENOMEM;
        return NULL;
    }
    return realloc(ptr, size);
}
#endif /* CONFIG_NEWLIB_LIBC_REALLOCARRAY */
