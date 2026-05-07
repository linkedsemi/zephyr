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
#include <soc.h>

#if defined(CONFIG_HEAP_DEBUG_LSQSH) ||  defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
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

#define POOL_SECTION __attribute__((section(CONFIG_NEWLIB_LIBC_MALLOC_SECTION_NAME)))

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

typedef struct __packed {
    struct k_heap *heap;
    uint8_t mem[];
} k_heap_mem_t;

static void *chunk_mem(struct z_heap *h, chunkid_t c)
{
    chunk_unit_t *buf = chunk_buf(h);
    uint8_t *ret = ((uint8_t *)&buf[c]) + chunk_header_bytes(h);

    CHECK(!(((uintptr_t)ret) & (big_heap(h) ? 7 : 3)));

    return ret;
}

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
enum alloc_type {
    _MALLOC_R,
    _CALLOC_R,
    _REALLOC_R,
    ALIGNED_ALLOC,
    MEMALIGN,
    ACLLOC_TYPE_MAX,
};

const char *const alloc_name[] = {
    "_malloc_r",
    "_calloc_r",
    "_realloc_r",
    "aligned_alloc",
    "memalign",
};

typedef struct {
    uint64_t alloc_moment;
    struct k_thread *owner_tcb;
    uint32_t alloc_size;
    enum alloc_type alloc_type;
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER_RA)
    uintptr_t ra[CONFIG_ARCH_STACKWALK_MAX_FRAMES];
    uint8_t ra_idx;
#endif
#if 0
    void *alloc_ret;
    struct k_heap *heap;
#endif
} k_heap_mem_ext_ftr_t;

#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
#if 0
static size_t k_heap_usable_size(struct k_heap *k_heap, void *mem)
{
    k_heap_mem_t *obj = CONTAINER_OF(mem, k_heap_mem_t, mem);
    struct sys_heap *sys_heap = &k_heap->heap;

    return sys_heap_usable_size(sys_heap, obj);
}
#endif
#endif

static chunkid_t mem_to_chunkid(struct z_heap *h, void *p)
{
    uint8_t *mem = p, *base = (uint8_t *)chunk_buf(h);
    return (mem - chunk_header_bytes(h) - base) / CHUNK_UNIT;
}

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER_RA)
static bool record_trace_address(void *arg, unsigned long ra)
{
    k_heap_mem_ext_ftr_t *ftr = (k_heap_mem_ext_ftr_t *)arg;

    if (ftr->ra_idx < sizeof(ftr->ra)) {
        ftr->ra[ftr->ra_idx++] = ra;
        return true;
    }

    return false;
}
#endif

static int heap_ext_log(enum alloc_type alloc_type, void *alloc_ret, size_t alloc_size)
{
    k_heap_mem_t *obj = CONTAINER_OF(alloc_ret, k_heap_mem_t, mem);
    struct z_heap *h = obj->heap->heap.heap;
    chunkid_t c = mem_to_chunkid(h, (void *)obj);
    uint32_t offset = chunksz_to_bytes(h, chunk_size(h, c)) - sizeof(k_heap_mem_ext_ftr_t);
    k_heap_mem_ext_ftr_t *ftr = (k_heap_mem_ext_ftr_t *)((uint8_t *)chunk_mem(h, c) + offset);

    ftr->alloc_moment = sys_clock_cycle_get_64();
    ftr->owner_tcb = k_current_get();
    ftr->alloc_size = alloc_size;
    ftr->alloc_type = alloc_type;
#if 0
    ftr->alloc_ret = alloc_ret;
    ftr->heap = obj->heap;
#endif
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER_RA)
    ftr->ra_idx = 0;
    arch_stack_walk(record_trace_address, (void *)ftr, NULL, NULL);
#endif

    return 0;
}
#endif

static void *_k_heap_aligned_alloc(struct k_heap *heap, size_t align, size_t size)
{
    void *mem;
    k_heap_mem_t *obj;
    size_t __align;
    size_t ext_size = sizeof(struct k_heap *);

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
    ext_size += sizeof(k_heap_mem_ext_ftr_t);
#endif
    /*
     * Adjust the size to make room for our heap reference.
     * Merge a rewind bit with align value (see sys_heap_aligned_alloc()).
     * This allows for storing the heap pointer right below the aligned
     * boundary without wasting any memory.
     */
    if (size_add_overflow(size, ext_size, &size)) {
        return NULL;
    }
    __align = align | sizeof(struct k_heap *);

    obj = (k_heap_mem_t *)k_heap_aligned_alloc(heap, __align, size, Z_TIMEOUT_MS(CONFIG_NEWLIB_LIBC_MALLOC_TIMEOUT_MS));
    if (obj == NULL) {
        return NULL;
    }

    obj->heap = heap;
    mem = (void *)obj->mem;
    __ASSERT(align == 0 || ((uintptr_t)mem & (align - 1)) == 0,
         "misaligned memory at %p (align = %zu)", mem, align);

    return mem;
}

static void *k_aligned_alloc_app(size_t align, size_t size)
{
    __ASSERT(align / CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT >= 1
        && (align % CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT) == 0,
        "align must be a multiple of CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT");

    __ASSERT((align & (align - 1)) == 0,
        "align must be a power of 2");

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, k_aligned_alloc_app, _APP_HEAP);

    void *ret = _k_heap_aligned_alloc(_APP_HEAP, align, size);

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, k_aligned_alloc_app, _APP_HEAP, ret);

    return ret;
}

static void __free_r(void *ptr)
{
    k_heap_mem_t *obj;
    struct k_heap *heap;

    if (ptr == NULL) {
        return;
    }

    obj = CONTAINER_OF(ptr, k_heap_mem_t, mem);
    heap = obj->heap;

#if 0
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
    do {
        struct z_heap *h = obj->heap->heap.heap;
        chunkid_t c = mem_to_chunkid(h, (void *)obj);
        uint32_t offset = chunksz_to_bytes(h, chunk_size(h, c)) - sizeof(k_heap_mem_ext_ftr_t);
        k_heap_mem_ext_ftr_t *ftr = (k_heap_mem_ext_ftr_t *)((uint8_t *)chunk_mem(h, c) + offset);
        __ASSERT_NO_MSG(heap == ftr->heap);
    } while(0);
#endif
#endif

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _free_r, heap);

    k_heap_free(heap, (void *)obj);

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _free_r, heap);
}

void *_malloc_r(struct _reent *r, size_t size)
{
    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _malloc_r, _APP_HEAP);

    void *ret = k_aligned_alloc_app(CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT, size);
    if (ret != NULL) {
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
        heap_ext_log(_MALLOC_R, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_push(__func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ret, size);
#endif
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _malloc_r, _APP_HEAP, ret);

    return ret;
}

void _free_r(struct _reent *r, void *ptr)
{
    if (ptr != NULL) {
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_pop(ptr, 0);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ptr, 0);
#endif
    }
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

    ret = k_aligned_alloc_app(CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT, bounds);
    if (ret != NULL) {
        (void)memset(ret, 0, bounds);

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
        heap_ext_log(_CALLOC_R, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_push(__func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ret, size);
#endif
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _calloc_r, _APP_HEAP, ret);

    return ret;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size)
{
    k_heap_mem_t *obj;
    struct k_heap *heap;
    void *ret;
    size_t ext_size = sizeof(struct k_heap *);

    if (size == 0) {
        _free_r(NULL, ptr);
        return NULL;
    }
    if (ptr == NULL) {
        ret = k_aligned_alloc_app(CONFIG_NEWLIB_LIBC_MALLOC_ALIGNMENT, size);
        if (ret != NULL) {
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
            heap_ext_log(_REALLOC_R, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
            heap_debug_ptr_push(__func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
            heap_debug_callstack(__func__, ret, size);
#endif
        }
        return ret;
    }
    obj = CONTAINER_OF(ptr, k_heap_mem_t, mem);
    heap = obj->heap;

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
    ext_size += sizeof(k_heap_mem_ext_ftr_t);
#endif

    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, _realloc_r, heap, (void *)obj);

    if (size_add_overflow(size, ext_size, &size)) {
        SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _realloc_r, heap, (void *)obj, NULL);
        return NULL;
    }

    ret = k_heap_realloc(heap, ptr, size, Z_TIMEOUT_MS(CONFIG_NEWLIB_LIBC_MALLOC_TIMEOUT_MS));

    if (ret != NULL) {
        obj = (k_heap_mem_t *)ret;
        obj->heap = heap;
        ret = (void *)obj->mem;

#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
        heap_ext_log(_REALLOC_R, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_replace(ptr, 0, __func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ret, size);
#endif
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, _realloc_r, heap, (void *)obj, ret);

    return ret;
}

void *aligned_alloc(size_t alignment, size_t size)
{
    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, aligned_alloc, _APP_HEAP);

    void *ret = k_aligned_alloc_app(alignment, size);
    if (ret != NULL) {
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
        heap_ext_log(ALIGNED_ALLOC, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_push(__func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ret, size);
#endif
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, aligned_alloc, _APP_HEAP, ret);


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
    SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_heap_app, memalign, _APP_HEAP);

    void *ret = k_aligned_alloc_app(alignment, size);
    if (ret != NULL) {
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
        heap_ext_log( MEMALIGN, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_RD_LSQSH)
        heap_debug_ptr_push(__func__, ret, size);
#endif
#if defined(CONFIG_HEAP_DEBUG_LSQSH)
        heap_debug_callstack(__func__, ret, size);
#endif
    }

    SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_heap_app, memalign, _APP_HEAP, ret);

    return ret;
}
#endif

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
static void heap_print_info(struct z_heap *h, bool dump_chunks, const struct k_thread *thread)
{
    int nb_buckets = bucket_idx(h, h->end_chunk) + 1;
    size_t free_bytes, allocated_bytes, total, overhead;

    printf("Heap at %p contains %d units in %d buckets\n\n",
           chunk_buf(h), h->end_chunk, nb_buckets);

    printf("  bucket#min units        total      largest      largest\n"
           "             threshold       chunks      (units)      (bytes)\n"
           "  -----------------------------------------------------------\n");
    for (int i = 0; i < nb_buckets; i++) {
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
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER)
            if (thread) {
                printf("thread: %p: %s\n", thread, thread->name);
            }
            size_t used_chunk_count = 0;
            size_t used_chunk_size_total = 0;
            size_t used_chunk_alloc_size_total = 0;
            size_t alloc_size_total[ACLLOC_TYPE_MAX] = {};
            for (chunkid_t c = 0; ; c = right_chunk(h, c)) {
                bool used = chunk_used(h, c);
                size_t thread_chunk_size = chunk_size(h, c);
                if (used && (c != 0) && (c != h->end_chunk)) {
                    uint32_t offset = chunksz_to_bytes(h, chunk_size(h, c)) - sizeof(k_heap_mem_ext_ftr_t);
                    void *const mem = chunk_mem(h, c);
#if 1
                    /* k_heap_aligned_alloc(): */
                    /*     obj may be an address of a CHUNK_UNIT within the range [(uintptr_t)mem, (uintptr_t)mem + CHUNK_UNIT). */
                    k_heap_mem_t *obj = mem;
                    while ((((uintptr_t)obj - (uintptr_t)mem) < CHUNK_UNIT) && (_APP_HEAP != obj->heap)) {
                        obj++;
                    }
                    if ((_APP_HEAP != obj->heap)) {
                        printf("raw mem: %p maybe invalid\n", mem);
                        continue;
                    }
#endif
                    k_heap_mem_ext_ftr_t *ftr = (k_heap_mem_ext_ftr_t *)((uint8_t *)mem + offset);
                    const char *thread_name = "";
                    if (thread) {
                        if (thread != ftr->owner_tcb) {
                            continue;
                        }
                    } else {
                        thread_name = ftr->owner_tcb ? ftr->owner_tcb->name : "na";
                    }

                    used_chunk_count++;
                    used_chunk_size_total += thread_chunk_size;
                    used_chunk_alloc_size_total += ftr->alloc_size;
                    alloc_size_total[ftr->alloc_type] += ftr->alloc_size;
#if 0
                    __ASSERT_NO_MSG(ftr->alloc_ret == obj->mem);
#endif
                    printf("[%d] %s chunk %4d: [%c] size=%-4d left=%-4d right=%d "
                           "%s: ptr: %8p alloc_size: %-10u alloc_moment: %-20llu",
                        used_chunk_count,
                        thread_name,
                        c,
                        used ? '*'
                        : solo_free_header(h, c) ? '.'
                        : '-',
                        thread_chunk_size,
                        left_chunk(h, c),
                        right_chunk(h, c),
                        alloc_name[ftr->alloc_type],
#if 0
                        ftr->alloc_ret,
#else
                        obj->mem,
#endif
                        ftr->alloc_size,
                        ftr->alloc_moment
                    );
#if defined(CONFIG_NEWLIB_LIBC_MALLOC_DEBUG_FOOTER_RA)
                    size_t ra_sum = 0;
                    for (int i = 0; i < ftr->ra_idx; i++) {
                        ra_sum += ftr->ra[i];
                    }
                    printf(" ra_sum: %#-8x", ra_sum);
                    printf(" ra:");
                    for (int i = 0; i < ftr->ra_idx; i++) {
                        printf(" %#8lx", ftr->ra[i]);
                    }
#endif
                    printf("\n");
                } else {
                    if (!thread) {
                        printf("chunk %4d: [%c] size=%-4d left=%-4d right=%d\n",
                            c,
                            chunk_used(h, c) ? '*'
                            : solo_free_header(h, c) ? '.'
                            : '-',
                            thread_chunk_size,
                            left_chunk(h, c),
                            right_chunk(h, c));
                    }
                }
                if (c == h->end_chunk) {
                    break;
                }
            }
            printf("\n");
            for (int i = 0; i < ACLLOC_TYPE_MAX; i++) {
                printf("%s: alloc_size_total: %u\n", alloc_name[i], alloc_size_total[i]);
            }
            printf("\n"
                    "used_chunk_count: %d\n"
                    "used_chunk_size_total: %d\n"
                    "used_chunk_alloc_size_total: %d\n",
                    used_chunk_count,
                    used_chunk_size_total,
                    used_chunk_alloc_size_total);
#else
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
#endif
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

static int cmd_sys_memory_stats(const struct shell *shell, size_t argc, char **argv, void *data)
{
    print_sys_memory_stats();
    return 0;
}
SHELL_CMD_REGISTER(sys_memory_stats, NULL, "sys_memory_stats", cmd_sys_memory_stats);

struct k_thread_name_to_ptr_arg {
    const char *name;
    const struct k_thread *thread;
};

void k_thread_name_to_ptr(const struct k_thread *thread, void *user_data)
{
    struct k_thread_name_to_ptr_arg *arg = user_data;
    if (arg->name) {
        if (!(strcmp(thread->name, arg->name))) {
            arg->thread = thread;
            return;
        }
    }
}

static int cmd_sys_heap_print_info(const struct shell *shell, size_t argc, char **argv, void *data)
{
    const struct k_thread *thread = NULL;
    if (argc > 1) {
        struct k_thread_name_to_ptr_arg arg;
        arg.name = argv[1];
        arg.thread = NULL;
        k_thread_foreach(k_thread_name_to_ptr, (void *)&arg);
        if (NULL == arg.thread) {
            printf("thread not found: %s\n", arg.name);
            return -EINVAL;
        }
        thread = arg.thread;
    }
    heap_print_info(((_APP_HEAP->heap).heap), true, thread);
    return 0;
}
SHELL_CMD_ARG_REGISTER(sys_heap_print_info, NULL, "sys_heap_print_info", cmd_sys_heap_print_info, 1, 1);
