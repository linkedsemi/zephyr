#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/shell/shell.h>
#include "heap_debug.h"

enum cmp_op
{
    CMP_NONE = 0,
    CMP_LT,
    CMP_LE,
    CMP_GT,
    CMP_GE,
    CMP_EQ,
};

#define HEAP_DEBUG_RECORX_MAX_NUM                   (4096)

struct heap_debug_cs_range
{
    enum cmp_op op1;
    enum cmp_op op2;
    size_t val1;
    size_t val2;
    bool has_two;
    bool is_enable;
};

struct heap_debug_cs_name
{
    char name_filter[CONFIG_THREAD_MAX_NAME_LEN];
    bool is_enable;
};

struct heap_debug_record
{
    const char *name;
    size_t ptr;
    size_t size;
};

struct heap_debug
{
    struct heap_debug_cs_range rang;
    struct heap_debug_cs_name name;
    struct heap_debug_record record[HEAP_DEBUG_RECORX_MAX_NUM];
    int rd_index;
    int rd_enable;
    int cs_enable;
};

static struct heap_debug heap_debug;
static K_MUTEX_DEFINE(heap_rd_lock);

static int heap_debug_help(const struct shell *sh, size_t argc, char **argv)
{
    return 0;
}

static int heap_debug_cs_enable(const struct shell *sh, size_t argc, char **argv)
{
    printf("heap debug enable\n");
    heap_debug.cs_enable = 1;
    return 0;
}

static int heap_debug_cs_disable(const struct shell *sh, size_t argc, char **argv)
{
    printf("heap debug disable\n");
    heap_debug.cs_enable = 0;
    return 0;
}

bool heap_debug_cs_is_enable(void)
{
    return heap_debug.cs_enable;
}

static enum cmp_op parse_cmp(const char *s)
{
    if (!strcmp(s, ">"))  return CMP_GT;
    if (!strcmp(s, ">=")) return CMP_GE;
    if (!strcmp(s, "<"))  return CMP_LT;
    if (!strcmp(s, "<=")) return CMP_LE;
    if (!strcmp(s, "==")) return CMP_EQ;
    return CMP_NONE;
}

static bool heap_debug_range_match(size_t size)
{
    bool ret = true;

    switch (heap_debug.rang.op1)
    {
        case CMP_GT: ret = size >  heap_debug.rang.val1; break;
        case CMP_GE: ret = size >= heap_debug.rang.val1; break;
        case CMP_LT: ret = size <  heap_debug.rang.val1; break;
        case CMP_LE: ret = size <= heap_debug.rang.val1; break;
        case CMP_EQ: ret = size == heap_debug.rang.val1; break;
        default: return false;
    }

    if (!ret)
        return false;

    if (!heap_debug.rang.has_two)
    {
        return true;
    }

    switch (heap_debug.rang.op2)
    {
        case CMP_GT: ret = size >  heap_debug.rang.val2; break;
        case CMP_GE: ret = size >= heap_debug.rang.val2; break;
        case CMP_LT: ret = size <  heap_debug.rang.val2; break;
        case CMP_LE: ret = size <= heap_debug.rang.val2; break;
        case CMP_EQ: ret = size == heap_debug.rang.val2; break;
        default: ret = false;
    }

    return ret;
}

static bool heap_debug_name_match(const char *name)
{
    return !strcmp(name, heap_debug.name.name_filter);
}

static bool heap_debug_match(size_t size, const char *name)
{
    if (heap_debug.name.is_enable && heap_debug.rang.is_enable)
    {
        return heap_debug_name_match(name) && heap_debug_range_match(size);
    }
    else if (heap_debug.rang.is_enable)
    {
        return heap_debug_range_match(size);
    }
    else if (heap_debug.name.is_enable){
        return heap_debug_name_match(name);
    }

    return true;
}

static bool heap_debug_cs_match(size_t size, const char *name)
{
    if (!heap_debug.cs_enable)
        return false;
    return heap_debug_match(size, name);
}

/*
    argv:
    [0] = op
    [1] = <cmp1>
    [2] = <val1>
    [3] = <cmp2> (optional)
    [4] = <val2> (optional)
*/
static int heap_debug_cs_range(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 2)
    {
        printf("disable range filter\n");
        heap_debug.rang.is_enable = 0;
        return 0;
    }
    
    if (argc != 3 && argc != 5)
    {
        printf("Usage:\n"
                "  heap_debug cs range <cmp> <val>\n"
                "  heap_debug cs range <cmp1> <val1> <cmp2> <val2>\n");
        return -EINVAL;
    }

    memset(&heap_debug.rang, 0, sizeof(heap_debug.rang));

    const char *cmp1 = argv[1];
    const char *val1 = argv[2];

    heap_debug.rang.op1 = parse_cmp(cmp1);
    if (heap_debug.rang.op1 == CMP_NONE)
    {
        printf("Invalid cmp: %s\n", cmp1);
        heap_debug.rang.is_enable = 0;
        return -EINVAL;
    }
    heap_debug.rang.val1 = strtoul(val1, NULL, 0);
    heap_debug.rang.is_enable = 1;

    if (argc == 5)
    {
        const char *cmp2 = argv[3];
        const char *val2 = argv[4];

        heap_debug.rang.op2 = parse_cmp(cmp2);
        if (heap_debug.rang.op2 == CMP_NONE)
        {
            printf("Invalid second cmp: %s\n", cmp2);
            heap_debug.rang.is_enable = 0;
            return -EINVAL;
        }

        heap_debug.rang.val2 = strtoul(val2, NULL, 0);
        heap_debug.rang.has_two = true;
    }
    printf("heap_debug: cs range set:\n");
    printf("  cond1: size %s %u\n", cmp1, heap_debug.rang.val1);

    if (heap_debug.rang.has_two)
        printf("  cond2: size %s %u\n", argv[3], heap_debug.rang.val2);

    return 0;
}

static int heap_debug_cs_name(const struct shell *sh, size_t argc, char **argv)
{
    if (strcmp(argv[1], "close") == 0)
    {
        printf("disable name filter\n");
        heap_debug.name.is_enable = 0;
        return 0;
    }

    snprintf(heap_debug.name.name_filter, CONFIG_THREAD_MAX_NAME_LEN, "%s", argv[1]);
    printf("heap_debug: cs name set: %s\n", heap_debug.name.name_filter);\
    heap_debug.name.is_enable = 1;

    return 0;
}

static bool print_trace_address(void *arg, unsigned long ra)
{
    printf("ra: %p\n", (void *)ra);
    return true;
}

void heap_debug_callstack(const char *func, size_t ptr, size_t size)
{
    int match = heap_debug_cs_match(size, k_current_get()->name);
    if (match)
    {
        printf("============== call stack start ====================\n");
        printf("thread: %p, name: %s, func = %s, ptr = %x, size = %u\n", k_current_get(), k_current_get()->name, func, ptr, size);
        arch_stack_walk(print_trace_address, NULL, k_current_get(), NULL);
        printf("============== call stack end ====================\n");
    }
}

void heap_debug_ptr_push(void *ptr, size_t size)
{
    if (!heap_debug.rd_enable || heap_debug.rd_index == HEAP_DEBUG_RECORX_MAX_NUM)
        return;

    struct heap_debug_record *record = heap_debug.record;

    k_mutex_lock(&heap_rd_lock, K_FOREVER);
    for (int i = 0; i < HEAP_DEBUG_RECORX_MAX_NUM; i++)
    {
        if (record[i].name == NULL)
        {
            record[i].ptr = (size_t)ptr;
            record[i].name = k_current_get()->name;
            record[i].size = size;
            heap_debug.rd_index++;
            break;
        }
    }
    k_mutex_unlock(&heap_rd_lock);
}

void heap_debug_ptr_pop(void *ptr)
{
    if (!heap_debug.rd_enable)
        return;

    struct heap_debug_record *record = heap_debug.record;

    k_mutex_lock(&heap_rd_lock, K_FOREVER);
    for (int i = 0; i < HEAP_DEBUG_RECORX_MAX_NUM; i++)
    {
        if (record[i].name)
        {
            if (record[i].ptr == (size_t)ptr)
            {
                record[i].name = NULL;
                heap_debug.rd_index--;
                break;
            }
        }
    }
    k_mutex_unlock(&heap_rd_lock);
}

void heap_debug_ptr_dump(void)
{
    struct heap_debug_record *record = heap_debug.record;
    printf("============ heap ptr record dump =================\n");
    for (int i = 0; i < HEAP_DEBUG_RECORX_MAX_NUM; i++)
    {
        if (record[i].name)
        {
            printf("thread: %s, ptr: %x, size: %u\n", record[i].name, record[i].ptr, record[i].size);
        }
    }
}

static int heap_debug_rd_enable(const struct shell *sh, size_t argc, char **argv)
{
    printf("heap debug record enable\n");
    heap_debug.rd_enable = 1;
    return 0;
}

static int heap_debug_rd_disable(const struct shell *sh, size_t argc, char **argv)
{
    printf("heap debug record disable\n");
    heap_debug.rd_enable = 0;
    return 0;
}

static int heap_debug_rd_dump(const struct shell *sh, size_t argc, char **argv)
{
    heap_debug_ptr_dump();
    return 0;
}

static int heap_debug_rd_clear(const struct shell *sh, size_t argc, char **argv)
{
    printf("heap debug record clear\n");
    k_mutex_lock(&heap_rd_lock, K_FOREVER);
    memset(heap_debug.record, 0, sizeof(struct heap_debug_record) * HEAP_DEBUG_RECORX_MAX_NUM);
    heap_debug.rd_index = 0;
    k_mutex_unlock(&heap_rd_lock);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_heap_debug_rd,
            SHELL_CMD_ARG(enable, NULL,
                        "heap_debug rd enable",
                         heap_debug_rd_enable, 0, 0),
            SHELL_CMD_ARG(disable, NULL,
                        "heap_debug rd disable\n",
                         heap_debug_rd_disable, 0, 0),
            SHELL_CMD_ARG(dump, NULL,
                        "heap_debug rd dump\n",
                         heap_debug_rd_dump, 0, 0),
            SHELL_CMD_ARG(clear, NULL,
                        "heap_debug rd clear\n",
                         heap_debug_rd_clear, 0, 0),
            SHELL_SUBCMD_SET_END );

SHELL_STATIC_SUBCMD_SET_CREATE(sub_heap_debug_cs,
            SHELL_CMD_ARG(enable, NULL,
                    "Usage:\n"
                    "heap_debug cs enable\n",
                    heap_debug_cs_enable, 0, 0),
            SHELL_CMD_ARG(disable, NULL,
                    "Usage:\n"
                    "heap_debug cs disable\n",
                    heap_debug_cs_disable, 0, 0),

            SHELL_CMD_ARG(range, NULL,
                        "heap_debug cs range <cmp> <size> [<cmp> <size>]\n \
                         heap_debug cs range close\n",
                         heap_debug_cs_range, 2, 5),

            SHELL_CMD_ARG(name, NULL,
                        "heap_debug cs name <thread_name>\n\
                         heap_debug cs name close\n",
                         heap_debug_cs_name, 2, 0),

            SHELL_SUBCMD_SET_END );

SHELL_STATIC_SUBCMD_SET_CREATE(sub_heap_debug,
            SHELL_CMD(cs, &sub_heap_debug_cs,
                    "heap_debug cs <range|name> ...",
                    NULL),
            SHELL_CMD(rd, &sub_heap_debug_rd,
                    "heap_debug rd <enable|disable|dump>",
                    NULL),
            SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(heap_debug, &sub_heap_debug,
           "heap debug command\n",
           heap_debug_help);
