#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/init.h>
#include <errno.h>
#include <zephyr/sys/math_extras.h>
#include <string.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/types.h>
#include "heap.h"
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
#include "heap_debug.h"
#endif

#define LOG_LEVEL CONFIG_KERNEL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_NEWLIB_LIBC_MALLOC
#error please set CONFIG_NEWLIB_LIBC_MALLOC=n
#endif

#ifdef CONFIG_NEWLIB_LIBC_MALLOC_LSQSH

#if (CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE != 0)

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_SECTION_NAME)
#define POOL_SECTION __attribute__((section(CONFIG_NEWLIB_LIBC_MALLOC_SECTION_NAME)))
#else
#define POOL_SECTION __noinit
#endif /* CONFIG_NEWLIB_LIBC_MALLOC_SECTION_NAME */

#ifndef HEAP_ALIGN
#define HEAP_ALIGN    sizeof(double)
#endif

#define HEAP_STATIC

/* Static allocation of heap in BSS */

#define HEAP_SIZE    ROUND_UP(CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE, HEAP_ALIGN)
#define HEAP_BASE    POINTER_TO_UINT(malloc_arena)

#if (CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE > 0)

Z_HEAP_DEFINE_IN_SECT(_app_heap, HEAP_SIZE, POOL_SECTION);
#define _APP_HEAP (&_app_heap)

static void *z_heap_aligned_alloc(struct k_heap *heap, size_t align, size_t size)
{
    void *mem;
    struct k_heap **heap_ref;
    size_t __align;

    /*
     * Adjust the size to make room for our heap reference.
     * Merge a rewind bit with align value (see sys_heap_aligned_alloc()).
     * This allows for storing the heap pointer right below the aligned
     * boundary without wasting any memory.
     */
    if (size_add_overflow(size, sizeof(heap_ref), &size)) {
        return NULL;
    }
    __align = align | sizeof(heap_ref);

    mem = k_heap_aligned_alloc(heap, __align, size, Z_TIMEOUT_TICKS((k_ticks_t)CONFIG_NEWLIB_LIBC_MALLOC_TIMEOUT));
    if (mem == NULL) {
        return NULL;
    }

    heap_ref = mem;
    *heap_ref = heap;
    mem = ++heap_ref;
    __ASSERT(align == 0 || ((uintptr_t)mem & (align - 1)) == 0,
         "misaligned memory at %p (align = %zu)", mem, align);

    return mem;
}

static void *k_aligned_alloc_app(size_t align, size_t size)
{
    __ASSERT(align / sizeof(void *) >= 1
        && (align % sizeof(void *)) == 0,
        "align must be a multiple of sizeof(void *)");

    __ASSERT((align & (align - 1)) == 0,
        "align must be a power of 2");

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, k_aligned_alloc_app, _APP_HEAP);

    void *ret = z_heap_aligned_alloc(_APP_HEAP, align, size);

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, k_aligned_alloc_app, _APP_HEAP, ret);

    return ret;
}

static void *__malloc_r(size_t size)
{
    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _malloc_r, _APP_HEAP);

    void *ret = k_aligned_alloc_app(sizeof(void *), size);

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _malloc_r, _APP_HEAP, ret);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    heap_debug_ptr_push(ret, size);
#endif

    return ret;
}

static void __free_r(void *ptr)
{
    struct k_heap **heap_ref;

    if (ptr != NULL) {
        heap_ref = ptr;
        --heap_ref;
        ptr = heap_ref;

        SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _free_r, *heap_ref, heap_ref);

        k_heap_free(*heap_ref, ptr);

        SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _free_r, *heap_ref, heap_ref);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_ptr_pop(ptr);
#endif
    }
}

void *_malloc_r(struct _reent *r, size_t size)
{
    void *ret = __malloc_r(size);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    heap_debug_callstack(__func__, (size_t)ret, size);
#endif

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    if ((ptr != NULL) && heap_debug_cs_is_enable()) {
        struct k_heap **heap_ref;
        void *_ptr;
        heap_ref = ptr;
        --heap_ref;
        _ptr = heap_ref;

        size_t size = sys_heap_usable_size(&((*heap_ref)->heap), _ptr);
        heap_debug_callstack(__func__, (size_t)_ptr, size);
    }
#endif
    __free_r(ptr);
}

void *_calloc_r(struct _reent *r, size_t nmemb, size_t size)
{
    void *ret;
    size_t bounds;

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _calloc_r, _APP_HEAP);

    if (size_mul_overflow(nmemb, size, &bounds)) {
        SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _calloc_r, _APP_HEAP, NULL);

        return NULL;
    }

    ret = __malloc_r(bounds);
    if (ret != NULL) {
        (void)memset(ret, 0, bounds);
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _calloc_r, _APP_HEAP, ret);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    heap_debug_callstack(__func__, (size_t)ret, size);
#endif

    return ret;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size)
{
    struct k_heap *heap, **heap_ref;
    void *ret;

    if (size == 0) {
        _free_r(NULL, ptr);
        return NULL;
    }
    if (ptr == NULL) {
        ret = __malloc_r(size);
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, (size_t)ret, size);
#endif
        return ret;
    }
    heap_ref = ptr;
    ptr = --heap_ref;
    heap = *heap_ref;

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _realloc_r, heap, ptr);

    if (size_add_overflow(size, sizeof(heap_ref), &size)) {
        SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _realloc_r, heap, ptr, NULL);
        return NULL;
    }

    ret = k_heap_realloc(heap, ptr, size, Z_TIMEOUT_TICKS((k_ticks_t)CONFIG_NEWLIB_LIBC_MALLOC_TIMEOUT));

    if (ret != NULL) {
        heap_ref = ret;
        ret = ++heap_ref;
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _realloc_r, heap, ptr, ret);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    heap_debug_callstack(__func__, (size_t)ret, size);
#endif

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, aligned_alloc, _APP_HEAP);

    void *ret = k_aligned_alloc_app(alignment, size);

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, aligned_alloc, _APP_HEAP, ret);

#if defined(CONFIG_HEAP_DEBUG_LSQSH)
    heap_debug_callstack(__func__, (size_t)ret, size);
#endif

    return ret;
}
#else
#define _APP_HEAP    NULL
#endif /* HEAP_SIZE */

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

#else /* No malloc arena */
void *malloc(size_t size)
{
    ARG_UNUSED(size);

    LOG_ERROR("CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE is 0");
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

#endif /* CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE */


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

void print_sys_memory_stats(void)
{
    struct sys_memory_stats stats;

    sys_heap_runtime_stats_get(&_APP_HEAP->heap, &stats);

    printk("allocated %zu, free %zu, max allocated %zu, heap size %u\n",
        stats.allocated_bytes, stats.free_bytes,
        stats.max_allocated_bytes, CONFIG_NEWLIB_LIBC_MALLOC_ARENA_SIZE);
}

/*
 * Print heap info for debugging / analysis purpose
 */
static void _heap_print_info(struct z_heap *h, bool dump_chunks)
{
    int i, nb_buckets = bucket_idx(h, h->end_chunk) + 1;
    size_t free_bytes, allocated_bytes, total, overhead;

    printf("Heap at %p contains %d units in %d buckets\n\n",
           chunk_buf(h), h->end_chunk, nb_buckets);

    printf("  bucket#min units        total      largest      largest\n"
           "             threshold       chunks      (units)      (bytes)\n"
           "  -----------------------------------------------------------\n");
    for (i = 0; i < nb_buckets; i++) {
        chunkid_t first = h->buckets[i].next;
        chunksz_t largest = 0;
        int count = 0;

        if (first) {
            chunkid_t curr = first;

            do {
                count++;
                largest = MAX(largest, chunk_size(h, curr));
                curr = next_free_chunk(h, curr);
            } while (curr != first);
        }
        if (count) {
            printf("%9d %12d %12d %12d %12zd\n",
                   i, (1 << i) - 1 + min_chunk_size(h), count,
                   largest, chunksz_to_bytes(h, largest));
        }
    }

    if (dump_chunks) {
        printf("\nChunk dump:\n");
        for (chunkid_t c = 0; ; c = right_chunk(h, c)) {
            printf("chunk %4d: [%c] size=%-4d left=%-4d right=%d\n",
                   c,
                   chunk_used(h, c) ? '*'
                   : solo_free_header(h, c) ? '.'
                   : '-',
                   chunk_size(h, c),
                   left_chunk(h, c),
                   right_chunk(h, c));
            if (c == h->end_chunk) {
                break;
            }
        }
    }

    get_alloc_info(h, &allocated_bytes, &free_bytes);
    /* The end marker chunk has a header. It is part of the overhead. */
    total = h->end_chunk * CHUNK_UNIT + chunk_header_bytes(h);
    overhead = total - free_bytes - allocated_bytes;
    printf("\n%zd free bytes, %zd allocated bytes, overhead = %zd bytes (%zd.%zd%%)\n",
           free_bytes, allocated_bytes, overhead,
           (1000 * overhead + total / 2) / total / 10,
           (1000 * overhead + total / 2) / total % 10);
}

static int _sys_heap_print_info(struct sys_heap *heap, bool dump_chunks)
{
    _heap_print_info(heap->heap, dump_chunks);
    return 0;
}

static int cmd_sys_memory_stats(const struct shell *shell, size_t argc, char **argv, void *data)
{
    print_sys_memory_stats();
    return 0;
}
SHELL_CMD_REGISTER(sys_memory_stats, NULL, "sys_memory_stats", cmd_sys_memory_stats);

static int cmd_sys_heap_print_info(const struct shell *shell, size_t argc, char **argv, void *data)
{
    _sys_heap_print_info(&_APP_HEAP->heap, true);
    return 0;
}
SHELL_CMD_REGISTER(sys_heap_print_info, NULL, "sys_heap_print_info", cmd_sys_heap_print_info);
