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
#include <zephyr/arch/arch_interface.h>
#include <zephyr/shell/shell.h>
#include "heap_debug.h"
#define LOG_LEVEL CONFIG_KERNEL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_NEWLIB_LIBC_MALLOC_TLSF_LSQSH

#if (CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE != 0)

#define POOL_SECTION __noinit
#define POOL_SECTION_2 __attribute__((section("PSRAM")))

#ifndef HEAP_ALIGN
#define HEAP_ALIGN    sizeof(double)
#endif

#if CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE > 0

#define HEAP_STATIC

/* Static allocation of heap in BSS */

#define HEAP_2_SIZE    ROUND_UP(CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE, HEAP_ALIGN)
#define HEAP_2_BASE    POINTER_TO_UINT(malloc_arena_2)

POOL_SECTION_2 unsigned char __aligned(HEAP_ALIGN) malloc_arena_2[HEAP_2_SIZE];

# endif /* CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE */

struct tlsf_mem_info
{
    tlsf_t tlsf;
    pool_t tlsf_pool;
    void* heap_addr;
    size_t heap_size;
    size_t used_size;
    size_t max_size;
};

Z_LIBC_DATA static struct tlsf_mem_info tlsf_heap_psram;
static int print_callstack = 0;

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

#define TLSF_ENABLE_PRINT_CALLSTACK             (0)

static void tlsf_no_mem_print_info();

static void *tlsf_mem_alloc(size_t align, size_t size)
{
    void *ptr = tlsf_memalign(tlsf_heap_psram.tlsf, align, size);

    if (ptr)
    {
        tlsf_heap_psram.used_size += tlsf_block_size(ptr);
        if (tlsf_heap_psram.used_size > tlsf_heap_psram.max_size)
        {
            tlsf_heap_psram.max_size = tlsf_heap_psram.used_size;
        }
        heap_debug_ptr_push(ptr, size);
    }

    return ptr;
}

static void tlsf_mem_free(void* ptr)
{
    tlsf_heap_psram.used_size -= tlsf_block_size(ptr);
    tlsf_free(tlsf_heap_psram.tlsf, ptr);
    heap_debug_ptr_pop(ptr);
}

void *_malloc_r (struct _reent *r, size_t size)
{
    void *ptr;

    malloc_lock();

    ptr = tlsf_mem_alloc(sizeof(size_t), size);

    if (ptr == NULL && size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_no_mem_print_info();
        errno = ENOMEM;
    }

    heap_debug_callstack("_malloc_r", (size_t)ptr, size);

    malloc_unlock();

    return ptr;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size)
{
    void *ret = NULL;

    malloc_lock();

    if (NULL == ptr)
    {
        ret = tlsf_mem_alloc(sizeof(size_t), size);
    }
    else if (0 == size)
    {
        tlsf_mem_free(ptr);
    }
    else
    {
        size_t old_size = tlsf_block_size(ptr);

        void *align_ptr = tlsf_mem_alloc(sizeof(size_t), size);

        if (align_ptr)
        {
            size_t cp_size = size > old_size ? old_size : size;
            memcpy(align_ptr, ptr, cp_size);
            tlsf_mem_free(ptr);
        }
        ret = align_ptr;
    }

    if (ret == NULL && size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_no_mem_print_info();
        errno = ENOMEM;
    }

    heap_debug_callstack("_realloc_r", (size_t)ret, size);

    malloc_unlock();

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
    if (ptr != NULL)
    {
        malloc_lock();
        heap_debug_callstack("_free_r", (size_t)ptr, tlsf_block_size(ptr));
        tlsf_mem_free(ptr);
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
    malloc_lock();

    ret = tlsf_mem_alloc(sizeof(size_t), size);

    heap_debug_callstack("_calloc_r", (size_t)ret, size);

    malloc_unlock();

    if (ret != NULL)
    {
        memset(ret, 0, size);
    }
    else if (size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_no_mem_print_info();
        errno = ENOMEM;
    }

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    void *ptr;

    malloc_lock();

    ptr = tlsf_mem_alloc(alignment, size);

    if (ptr == NULL && size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_no_mem_print_info();
        errno = ENOMEM;
    }

    heap_debug_callstack("aligned_alloc", (size_t)ptr, size);
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

struct heap_info
{
    size_t used_size;
    size_t free_size;
};

static void tlsf_walker_cb(void* ptr, size_t size, int used, void* user)
{
    struct heap_info *info_ptr = user;

    printf("[0x%08x - ", (size_t)ptr);
    if (size < 1024)
        printf("%5d", size);
    else if (size < 1024 * 1024)
        printf("%4dK", size / 1024);
    else
        printf("%4dM", size / (1024 * 1024));

    if (used)
    {
        info_ptr->used_size += size;
        printf("    USED\n");
    }
    else
    {
        info_ptr->free_size += size;
        printf("\n");
    }
}

void tlsf_heap_info(void)
{
    struct heap_info info = {0};
    tlsf_walk_pool(tlsf_heap_psram.tlsf_pool, tlsf_walker_cb, &info);
    printf("\n-- used:%d  free:%d --\n", info.used_size, info.free_size);
}

static bool print_trace_address(void *arg, unsigned long ra)
{
    printf("ra: %p\n", (void *)ra);
    return true;
}

static void tlsf_no_mem_print_info()
{
    irq_lock();
    printf("alloc fail, irq lock, current thread: %p, name: %s\n", k_current_get(), k_current_get()->name);
    heap_debug_ptr_dump();
    printf("============== call stack start ====================\n");
    printf("thread: %p, name: %s\n", k_current_get(), k_current_get()->name);
    arch_stack_walk(print_trace_address, NULL, k_current_get(), NULL);
    printf("============== call stack end ====================\n");
    tlsf_heap_info();
    while (1);
}

void print_sys_memory_stats(void)
{
    printf("heap size : %u, used : %u, max used : %u\n", tlsf_heap_psram.heap_size, \
            tlsf_heap_psram.used_size, tlsf_heap_psram.max_size);
}

static int tlsf_print_callstack(const struct shell *shell, size_t argc, char **argv, void *data)
{
    if (argc < 2)
    {
        printf("argv invalid\n");
        return 0;
    }

    printf("tlsf print callstack %s.\n", atoi(argv[1]) ? "open" : "close");
    print_callstack = atoi(argv[1]);

    return 0;
}
SHELL_CMD_REGISTER(tlsf_print_callstack, NULL, "tlsf_print_callstack", tlsf_print_callstack);

static int tlsf_print_heap_info(const struct shell *shell, size_t argc, char **argv, void *data)
{
    tlsf_heap_info();
    return 0;
}
SHELL_CMD_REGISTER(tlsf_print_heap_info, NULL, "tlsf_print_heap_info", tlsf_print_heap_info);

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
