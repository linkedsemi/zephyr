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

void tlsf_heap_info(void);

#else
#define malloc_lock()
#define malloc_unlock()
#endif

static void tlsf_print_info();

static bool print_trace_address(void *arg, unsigned long ra)
{
    printf("ra: %p\n", (void *)ra);
    return true;
}

static void get_cur_callstack(const char *func, size_t size)
{
    if (print_callstack == 0)
        return;

    printf("============== call stack start ====================\n");
    printf("thread: %p, name: %s, func: %s, size: %d\n", k_current_get(), k_current_get()->name, func, size);
    arch_stack_walk(print_trace_address, NULL, k_current_get(), NULL);
    printf("============== call stack end ====================\n");
}

static void *tlsf_mem_alloc(size_t align, size_t size)
{
    void *ptr = tlsf_memalign(tlsf_heap_psram.tlsf, align, size);

    if (ptr)
    {
        k_tid_t tid = k_current_get();
        size_t block_size = tlsf_block_size(ptr);
        tid->malloc_size += block_size;
        tlsf_heap_psram.used_size += block_size;
        if (tlsf_heap_psram.used_size > tlsf_heap_psram.max_size)
        {
            tlsf_heap_psram.max_size = tlsf_heap_psram.used_size;
        }
    }

    return ptr;
}

static void tlsf_mem_free(void* ptr)
{
    k_tid_t tid = k_current_get();
    size_t block_size = tlsf_block_size(ptr);

    tid->free_size += block_size;
    tlsf_heap_psram.used_size -= block_size;
    tlsf_free(tlsf_heap_psram.tlsf, ptr);
}

void *_malloc_r (struct _reent *r, size_t size)
{
    void *ptr;
    malloc_lock();

    k_tid_t tid = k_current_get();
    tid->malloc_count += 1;

    get_cur_callstack("_malloc_r", size);

    ptr = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                   size);

    if (ptr == NULL && size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_print_info();
        errno = ENOMEM;
    }

    malloc_unlock();

    return ptr;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t requested_size)
{
    void *ret = NULL;
    k_tid_t tid = k_current_get();
    tid->realloc_count += 1;

    malloc_lock();
    get_cur_callstack("_realloc_r", requested_size);

    if (NULL == ptr)
    {
        ret = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                        (CONFIG_DCACHE_LINE_SIZE),
                                        (__alignof__(z_max_align_t))),
                            requested_size);
    }
    else if (0 == requested_size)
    {
        tlsf_mem_free(ptr);
    }
    else
    {
        size_t old_size = tlsf_block_size(ptr);

        void *align_ptr = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                                    requested_size);

        if (align_ptr)
        {
            size_t cp_size = requested_size > old_size ? old_size : requested_size;
            memcpy(align_ptr, ptr, cp_size);
            tlsf_mem_free(ptr);
        }
        ret = align_ptr;
    }

    if (ret == NULL && requested_size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, requested_size);
        tlsf_print_info();
        errno = ENOMEM;
    }

    malloc_unlock();

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
    if (ptr != NULL)
    {
        k_tid_t tid = k_current_get();
        tid->free_count += 1;
        malloc_lock();
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

    k_tid_t tid = k_current_get();
    tid->calloc_count += 1;

    malloc_lock();

    ret = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                        size);

    malloc_unlock();

    if (ret != NULL)
        memset(ret, 0, size);

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    void *ptr;
    k_tid_t tid = k_current_get();

    malloc_lock();
    get_cur_callstack("aligned_alloc", size);
    tid->aligned_alloc_count += 1;

    ptr = tlsf_mem_alloc(alignment, size);

    if (ptr == NULL && size != 0)
    {
        printf("%s size = %u fail\n", __FUNCTION__, size);
        tlsf_print_info();
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

static void thread_list(const struct k_thread *cthread, void *user_data)
{
    printf("======== thread: %p, name: %s, dynamic memory use info ====== \n", cthread, cthread->name);
    printf("m_c: %u, f_c: %u, r_c: %u, c_c: %u, a_a_c: %u\n", cthread->malloc_count, cthread->free_count, cthread->realloc_count, \
        cthread->calloc_count, cthread->aligned_alloc_count);
    printf("m_size: %u, f_size: %u\n", cthread->malloc_size, cthread->free_size);
}

void tlsf_heap_info(void)
{
    struct heap_info info = {0};
    tlsf_walk_pool(tlsf_heap_psram.tlsf_pool, tlsf_walker_cb, &info);
    printf("\n-- used:%d  free:%d --\n", info.used_size, info.free_size);
}

static void tlsf_print_info()
{
    irq_lock();
    printf("alloc fail, irq lock, current thread: %p, name: %s\n", k_current_get(), k_current_get()->name);
    k_thread_foreach_unlocked(thread_list, NULL);
    printf("============== call stack start ====================\n");
    printf("thread: %p, name: %s, func: %s, size: %d\n", k_current_get(), k_current_get()->name, "tlsf_print_info", 0);
    arch_stack_walk(print_trace_address, NULL, k_current_get(), NULL);
    printf("============== call stack end ====================\n");
    tlsf_heap_info();
    while (1);
}

void print_sys_memory_stats(void)
{
    printf("heap size : %u, used : %u, max used : %u\n", tlsf_heap_psram.heap_size, \
            tlsf_heap_psram.used_size, tlsf_heap_psram.max_size);
    k_thread_foreach_unlocked(thread_list, NULL);
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

static int tlsf_thread_dynamic_memory(const struct shell *shell, size_t argc, char **argv, void *data)
{
    k_thread_foreach_unlocked(thread_list, NULL);
    return 0;
}
SHELL_CMD_REGISTER(tlsf_thread_dynamic_memory, NULL, "tlsf_thread_dynamic_memory", tlsf_thread_dynamic_memory);

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
