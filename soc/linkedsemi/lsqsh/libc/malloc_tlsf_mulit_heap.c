/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <errno.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/kernel.h>
#include "tlsf/tlsf.h"
#include <zephyr/shell/shell.h>
#include "heap_debug.h"
#define LOG_LEVEL CONFIG_KERNEL_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#define POOL_SECTION_PSRAM      __attribute__((section("PSRAM")))
#define HEAP_ALIGN              sizeof(double)

#define SMALL_HEAP_SIZE         ROUND_UP(CONFIG_NEWLIB_LIBC_MALLOC_PSRAM_SMALL_BLOCK_SIZE, HEAP_ALIGN)
#define SMALL_HEAP_BASE         POINTER_TO_UINT(smale_heap)
#define SMALL_HEAP_END          (SMALL_HEAP_BASE + SMALL_HEAP_SIZE)

#define BIG_HEAP_SIZE           ROUND_UP(CONFIG_NEWLIB_LIBC_MALLOC_PSRAM_BIG_BLOCK_SIZE, HEAP_ALIGN)
#define BIG_HEAP_BASE           POINTER_TO_UINT(big_heap)
#define BIG_HEAP_END            (BIG_HEAP_SIZE + BIG_HEAP_BASE)

#define MEM_SIZE_LEVEL          (CONFIG_NEWLIB_LIBC_MALLOC_PSRAM_BLOCK_LEVEL)

POOL_SECTION_PSRAM unsigned char __aligned(HEAP_ALIGN) smale_heap[SMALL_HEAP_SIZE];
POOL_SECTION_PSRAM unsigned char __aligned(HEAP_ALIGN) big_heap[BIG_HEAP_SIZE];

struct tlsf_mem_info
{
    tlsf_t tlsf;
    pool_t tlsf_pool;
    void* heap_addr;
    size_t heap_size;
    size_t used_size;
    size_t max_size;
    const char *name;
    struct k_mutex lock;
};

Z_LIBC_DATA static struct tlsf_mem_info tlsf_small_heap;
Z_LIBC_DATA static struct tlsf_mem_info tlsf_big_heap;

static inline void malloc_lock(struct k_mutex *lock)
{
    int lock_ret;
    lock_ret = k_mutex_lock(lock, K_FOREVER);
    __ASSERT_NO_MSG(lock_ret == 0);
}

static inline void malloc_unlock(struct k_mutex *lock)
{
    (void) k_mutex_unlock(lock);
}

static void tlsf_no_mem_print_info(struct tlsf_mem_info *tlsf);

static struct tlsf_mem_info *tlsf_alloc_obj_get(size_t size)
{
    if (size <= MEM_SIZE_LEVEL)
        return &tlsf_small_heap;
    return &tlsf_big_heap;
}

static struct tlsf_mem_info *tlsf_free_obj_get(void *ptr)
{
    if ((size_t)ptr >= SMALL_HEAP_BASE && (size_t)ptr <= SMALL_HEAP_END)
        return &tlsf_small_heap;
    else if ((size_t)ptr >= BIG_HEAP_BASE && (size_t)ptr <= BIG_HEAP_END)
        return &tlsf_big_heap;

    return NULL;
}

static void *tlsf_mem_alloc(size_t align, size_t size)
{
    struct tlsf_mem_info *tlsf = tlsf_alloc_obj_get(size);

    malloc_lock(&tlsf->lock);

    void *ptr = tlsf_memalign(tlsf->tlsf, align, size);
    if (ptr)
    {
        tlsf->used_size += tlsf_block_size(ptr);
        if (tlsf->used_size > tlsf->max_size)
        {
            tlsf->max_size = tlsf->used_size;
        }
    }

    malloc_unlock(&tlsf->lock);
    heap_debug_ptr_push(ptr, size);

    return ptr;
}

static void tlsf_mem_free(void* ptr)
{
    struct tlsf_mem_info *tlsf = tlsf_free_obj_get(ptr);
    __ASSERT(tlsf, "exception ptr = %p", ptr);

    malloc_lock(&tlsf->lock);

    tlsf->used_size -= tlsf_block_size(ptr);
    tlsf_free(tlsf->tlsf, ptr);

    malloc_unlock(&tlsf->lock);
    heap_debug_ptr_pop(ptr);
}

void *_malloc_r (struct _reent *r, size_t size)
{
    void *ptr;

    ptr = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                         size);

    if (ptr == NULL && size != 0)
    {
        struct tlsf_mem_info *tlsf = tlsf_alloc_obj_get(size);
        printf("%s: %s size = %u fail\n", tlsf->name, __FUNCTION__, size);
        tlsf_no_mem_print_info(tlsf);
        errno = ENOMEM;
    }

    heap_debug_callstack("_malloc_r", (size_t)ptr, size);

    return ptr;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size)
{
    void *ret = NULL;

    if (NULL == ptr)
    {
        ret = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                        (CONFIG_DCACHE_LINE_SIZE),
                                        (__alignof__(z_max_align_t))),
                            size);
    }
    else if (0 == size)
    {
        if (ptr)
        {
            tlsf_mem_free(ptr);
        }
    }
    else
    {
        size_t old_size = tlsf_block_size(ptr);

        void *align_ptr = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                                    (CONFIG_DCACHE_LINE_SIZE),
                                    (__alignof__(z_max_align_t))),
                                    size);

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
        struct tlsf_mem_info *tlsf = tlsf_alloc_obj_get(size);
        printf("%s: %s size = %u fail\n", tlsf->name, __FUNCTION__, size);
        tlsf_no_mem_print_info(tlsf);
        errno = ENOMEM;
    }

    heap_debug_callstack("_realloc_r", (size_t)ret, size);

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
    if (ptr != NULL)
    {
        heap_debug_callstack("_free_r", (size_t)ptr, tlsf_block_size(ptr));
        tlsf_mem_free(ptr);
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

    ret = tlsf_mem_alloc(COND_CODE_1(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay),
                         (CONFIG_DCACHE_LINE_SIZE),
                         (__alignof__(z_max_align_t))),
                         size);
 
    heap_debug_callstack("_calloc_r", (size_t)ret, size);

    if (ret != NULL)
    {
        memset(ret, 0, size);
    }
    else if (size != 0)
    {
        struct tlsf_mem_info *tlsf = tlsf_alloc_obj_get(size);
        printf("%s: %s size = %u fail\n", tlsf->name, __FUNCTION__, size);
        tlsf_no_mem_print_info(tlsf);
        errno = ENOMEM;
    }

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    void *ptr;

    ptr = tlsf_mem_alloc(alignment, size);

    if (ptr == NULL && size != 0)
    {
        struct tlsf_mem_info *tlsf = tlsf_alloc_obj_get(size);
        printf("%s: %s size = %u fail\n", tlsf->name, __FUNCTION__, size);
        tlsf_no_mem_print_info(tlsf);
        errno = ENOMEM;
    }

    heap_debug_callstack("aligned_alloc", (size_t)ptr, size);

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

static void heap_init(struct tlsf_mem_info *tlsf, const char *name, void *heap_base, size_t heap_size)
{
    memset(tlsf, 0, sizeof(struct tlsf_mem_info));
    k_mutex_init(&tlsf->lock);

    tlsf->name = name;
    tlsf->heap_addr = (void *)ROUND_UP(heap_base, tlsf_align_size());
    tlsf->heap_size = heap_size;
    tlsf->tlsf = tlsf_create_with_pool(tlsf->heap_addr, tlsf->heap_size);
    tlsf->tlsf_pool = tlsf_get_pool(tlsf->tlsf);
}

static int malloc_prepare(void)
{
    void *heap_base = NULL;
    size_t heap_size;

    heap_base = UINT_TO_POINTER(SMALL_HEAP_BASE);
    heap_size = SMALL_HEAP_SIZE;
    heap_init(&tlsf_small_heap, "small block heap", heap_base, heap_size);

    heap_base = UINT_TO_POINTER(BIG_HEAP_BASE);
    heap_size = BIG_HEAP_SIZE;
    heap_init(&tlsf_big_heap, "big block heap", heap_base, heap_size);

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
    printf("=========== small block heap use info ===========\n");
    tlsf_walk_pool(tlsf_small_heap.tlsf_pool, tlsf_walker_cb, &info);
    printf("\n-- used:%d  free:%d --\n", info.used_size, info.free_size);

    info.free_size = 0;
    info.used_size = 0;

    printf("=========== big block heap use info ===========\n");
    tlsf_walk_pool(tlsf_big_heap.tlsf_pool, tlsf_walker_cb, &info);
    printf("\n-- used:%d  free:%d --\n", info.used_size, info.free_size);
}

static bool print_trace_address(void *arg, unsigned long ra)
{
    printf("ra: %p\n", (void *)ra);
    return true;
}

static void tlsf_no_mem_print_info(struct tlsf_mem_info *tlsf)
{
    irq_lock();
    printf("%s alloc fail, irq lock, current thread: %p, name: %s\n", tlsf->name, k_current_get(), k_current_get()->name);
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
    printf("small block heap size : %u, used : %u, max used : %u\n", tlsf_small_heap.heap_size, \
            tlsf_small_heap.used_size, tlsf_small_heap.max_size);
    printf("big block heap size : %u, used : %u, max used : %u\n", tlsf_big_heap.heap_size, \
            tlsf_big_heap.used_size, tlsf_big_heap.max_size);
}

static int tlsf_print_heap_info(const struct shell *shell, size_t argc, char **argv, void *data)
{
    tlsf_heap_info();
    return 0;
}
SHELL_CMD_REGISTER(tlsf_print_heap_info, NULL, "tlsf_print_heap_info", tlsf_print_heap_info);

static int tlsf_print_heap_use(const struct shell *shell, size_t argc, char **argv, void *data)
{
    print_sys_memory_stats();
    return 0;
}
SHELL_CMD_REGISTER(tlsf_print_heap_use, NULL, "tlsf_print_heap_use", tlsf_print_heap_use);
