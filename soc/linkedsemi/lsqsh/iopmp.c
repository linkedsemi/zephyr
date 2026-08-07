#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/sys/util.h>
#if defined(CONFIG_RESET)
#include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#endif
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(iopmp, CONFIG_IOPMP_LOG_LEVEL);

#include <soc.h>
#include <platform.h>
#include <iopmp.h>

#define PMP_DEBUG_INIT_DUMP  IS_ENABLED(CONFIG_IOPMP_DEBUG_INIT_DUMP)
#define PMP_DEBUG_WRITE_DUMP IS_ENABLED(CONFIG_IOPMP_DEBUG_WRITE_DUMP)
#define DT_DRV_COMPAT        linkedsemi_iopmp

#ifdef CONFIG_64BIT
#define PR_ADDR "0x%016lx"
#else
#define PR_ADDR "0x%08lx"
#endif

#define PMP_TOR_SUPPORTED   !IS_ENABLED(CONFIG_IOPMP_NO_TOR)
#define PMP_NA4_SUPPORTED   !IS_ENABLED(CONFIG_IOPMP_NO_NA4)
#define PMP_NAPOT_SUPPORTED !IS_ENABLED(CONFIG_IOPMP_NO_NAPOT)

#define PMPCFG_STRIDE sizeof(unsigned long)

#define PMP_ADDR(addr)             ((addr) >> 2)
#define NAPOT_RANGE(size)          (((size)-1) >> 3)
#define PMP_ADDR_NAPOT(addr, size) (PMP_ADDR(addr) | NAPOT_RANGE(size))

#define PMP_NONE 0

struct iopmp_partition_attr {
    uint32_t attr;
    uint32_t base;
    uint64_t size;
};

struct iopmp_config {
    mm_reg_t base;
    struct iopmp_partition_attr *attr;
    uint8_t attr_num;
};

static const char *iopmp_mode_str(uint32_t mode)
{
    switch (mode) {
    case PMP_TOR:
        return "TOR";
    case PMP_NA4:
        return "NA4";
    case PMP_NAPOT:
        return "NAPOT";
    default:
        return "NONE";
    }
}

static void print_pmp_entries(const struct device *dev,
                              unsigned int pmp_start,
                              unsigned int pmp_end,
                              unsigned long *pmp_addr,
                              unsigned long *pmp_cfg,
                              const char *banner)
{
    uint8_t *pmp_n_cfg = (uint8_t *)pmp_cfg;
    unsigned int index;

    DEV_DBG(dev, "PMP %s:", banner);
    for (index = pmp_start; index < pmp_end; index++) {
        unsigned long start, end, tmp;

        switch (pmp_n_cfg[index] & PMP_A) {
        case PMP_TOR:
            start = (index == 0) ? 0 : pmp_addr[index - 1];
            end = pmp_addr[index] - 1;
            break;
        case PMP_NA4:
            start = pmp_addr[index] << 2;
            end = start + 3;
            break;
        case PMP_NAPOT:
            tmp = ((pmp_addr[index] ^ (pmp_addr[index] + 1)) >> 1);
            start = (pmp_addr[index] & ~tmp) << 2;
            end = start + ((tmp << 3) | 0x7);
            break;
        default:
            start = 0;
            end = 0;
            break;
        }

        if (end == 0) {
            DEV_DBG(dev, "%3d: " PR_ADDR " 0x%02x",
                index,
                pmp_addr[index],
                pmp_n_cfg[index]);
        } else {
            DEV_DBG(dev, "%3d: " PR_ADDR " 0x%02x --> " PR_ADDR "-" PR_ADDR " %c%c%c%s %s",
                index,
                pmp_addr[index],
                pmp_n_cfg[index],
                start,
                end,
                (pmp_n_cfg[index] & PMP_R) ? 'R' : '-',
                (pmp_n_cfg[index] & PMP_W) ? 'W' : '-',
                (pmp_n_cfg[index] & PMP_X) ? 'X' : '-',
                (pmp_n_cfg[index] & PMP_L) ? " LOCKED" : "",
                iopmp_mode_str(pmp_n_cfg[index] & PMP_A));
        }
    }
}

static void dump_pmp_regs(const struct device *dev, const char *banner)
{
    const struct iopmp_config *dev_config = dev->config;
    unsigned long pmp_addr[CONFIG_IOPMP_SLOTS];
    unsigned long pmp_cfg[CONFIG_IOPMP_SLOTS / PMPCFG_STRIDE];

#define PMPADDR_READ(x) pmp_addr[x] = iopmp_get_pmpaddrx(dev_config->base, x)

    FOR_EACH (PMPADDR_READ, (;), 0, 1, 2, 3, 4, 5, 6, 7);
#if CONFIG_IOPMP_SLOTS > 8
    FOR_EACH (PMPADDR_READ, (;), 8, 9, 10, 11, 12, 13, 14, 15);
#endif

#undef PMPADDR_READ

#ifdef CONFIG_64BIT
    pmp_cfg[0] = csr_read(pmpcfg0);
#if CONFIG_IOPMP_SLOTS > 8
    pmp_cfg[1] = csr_read(pmpcfg2);
#endif
#else
    pmp_cfg[0] = iopmp_get_pmpxcfg(dev_config->base, 0);
    pmp_cfg[1] = iopmp_get_pmpxcfg(dev_config->base, 1);
#if CONFIG_IOPMP_SLOTS > 8
    pmp_cfg[2] = iopmp_get_pmpxcfg(dev_config->base, 2);
    pmp_cfg[3] = iopmp_get_pmpxcfg(dev_config->base, 3);
#endif
#endif

    print_pmp_entries(dev, 0, CONFIG_IOPMP_SLOTS, pmp_addr, pmp_cfg, banner);
}

static bool set_pmp_entry(const struct device *dev,
                          unsigned int *index_p,
                          uint8_t perm,
                          uintptr_t start,
                          uint64_t size,
                          unsigned long *pmp_addr,
                          unsigned long *pmp_cfg,
                          unsigned int index_limit)
{
    uint8_t *pmp_n_cfg = (uint8_t *)pmp_cfg;
    unsigned int index = *index_p;
    bool ok = true;

    __ASSERT((start & (CONFIG_IOPMP_GRANULARITY - 1)) == 0, "misaligned start address");
    __ASSERT((size & (CONFIG_IOPMP_GRANULARITY - 1)) == 0, "misaligned size");

    if (index >= index_limit) {
        DEV_ERR(dev, "out of PMP slots");
        ok = false;
    } else if (PMP_TOR_SUPPORTED
            && ((index == 0 && start == 0)
            || (index != 0 && pmp_addr[index - 1] == PMP_ADDR(start)))) {
        /* We can use TOR using only one additional slot */
        pmp_addr[index] = start;
        pmp_n_cfg[index] = perm | PMP_TOR;
        index += 1;
    } else if (PMP_NA4_SUPPORTED && size == 4) {
        pmp_addr[index] = PMP_ADDR(start);
        pmp_n_cfg[index] = perm | PMP_NA4;
        index += 1;
    } else if (PMP_NAPOT_SUPPORTED && ((size & (size - 1)) == 0) /* power of 2 */
            && ((start & (size - 1)) == 0) /* naturally aligned */
            && (PMP_NA4_SUPPORTED || (size != 4))) {
        pmp_addr[index] = PMP_ADDR_NAPOT(start, size);
        pmp_n_cfg[index] = perm | PMP_NAPOT;
        index += 1;
    } else if (PMP_TOR_SUPPORTED && index + 1 >= index_limit) {
        DEV_ERR(dev, "out of PMP slots");
        ok = false;
    } else if (PMP_TOR_SUPPORTED) {
        pmp_addr[index] = start;
        pmp_n_cfg[index] = 0;
        index += 1;
        pmp_addr[index] = start + size;
        pmp_n_cfg[index] = perm | PMP_TOR;
        index += 1;
    } else {
        DEV_ERR(dev, "inappropriate PMP range (start=%#lx size=%#llx)", start, size);
        ok = false;
    }

    *index_p = index;
    return ok;
}

static void __write_pmp_entries(const struct device *dev,
                                unsigned int start,
                                unsigned int end,
                                bool clear_trailing_entries,
                                const unsigned long *pmp_addr,
                                const unsigned long *pmp_cfg)
{
    const struct iopmp_config *dev_config = dev->config;
    for (unsigned int i = start; i < end; i++) {
        iopmp_set_pmpaddrx(dev_config->base, i, pmp_addr[i]);
    }
    int pmpxcfg_start = ROUND_DOWN(start, PMPCFG_STRIDE) / PMPCFG_STRIDE;
    int pmpxcfg_end = ROUND_UP(end, PMPCFG_STRIDE) / PMPCFG_STRIDE;
    for (int i = pmpxcfg_start; i < pmpxcfg_end; i++) {
        iopmp_set_pmpxcfg(dev_config->base, i, pmp_cfg[i]);
    }
}

static void write_pmp_entries(const struct device *dev,
                              unsigned int start,
                              unsigned int end,
                              bool clear_trailing_entries,
                              unsigned long *pmp_addr,
                              unsigned long *pmp_cfg,
                              unsigned int index_limit)
{
    __ASSERT(start < end && end <= index_limit && index_limit <= CONFIG_IOPMP_SLOTS,
             "bad PMP range (start=%u end=%u)",
             start,
             end);

    /* Be extra paranoid in case assertions are disabled */
    if (start >= end || end > index_limit) {
        k_panic();
    }

    if (clear_trailing_entries) {
        /*
         * There are many config entries per pmpcfg register.
         * Make sure to clear trailing garbage in the last
         * register to be written if any. Remaining registers
         * will be cleared in __write_pmp_entries().
         */
        uint8_t *pmp_n_cfg = (uint8_t *)pmp_cfg;
        unsigned int index;

        for (index = end; index % PMPCFG_STRIDE != 0; index++) {
            pmp_n_cfg[index] = 0;
        }
    }

    if (PMP_DEBUG_WRITE_DUMP) {
        print_pmp_entries(dev, start, end, pmp_addr, pmp_cfg, "register write");
    }

    __write_pmp_entries(dev, start, end, clear_trailing_entries, pmp_addr, pmp_cfg);
}

static int iopmp_init(const struct device *dev)
{
    const struct iopmp_config *dev_config = dev->config;

    if (iopmp_is_enable(dev_config->base)) {
        return 0;
    }

    uint32_t slot_idx = 0;
    unsigned long pmp_addr[CONFIG_IOPMP_SLOTS] = {};
    unsigned long pmp_cfg[CONFIG_IOPMP_SLOTS / PMPCFG_STRIDE] = {};
    for (uint8_t addr_idx = 0; addr_idx < dev_config->attr_num; addr_idx++) {
        uint32_t last_slot_idx = slot_idx;
        bool ok = set_pmp_entry(dev,
                                &slot_idx,
                                dev_config->attr[addr_idx].attr,
                                dev_config->attr[addr_idx].base,
                                dev_config->attr[addr_idx].size,
                                pmp_addr,
                                pmp_cfg,
                                CONFIG_IOPMP_SLOTS);
        if (!ok) {
            DEV_ERR(dev, "set_pmp_entry failed");
            return -1;
        }
        write_pmp_entries(dev,
                        last_slot_idx,
                        slot_idx,
                        true,
                        pmp_addr,
                        pmp_cfg,
                        CONFIG_IOPMP_SLOTS);
    }
    iopmp_config_enable(dev_config->base, true);

    if (PMP_DEBUG_INIT_DUMP) {
        dump_pmp_regs(dev, "initial register dump");
    }

    return 0;
}

#define PARTITION_CHILD(node_id)        \
    {                                   \
        .base = DT_REG_ADDR(node_id),   \
        .size = DT_REG_SIZE(node_id),   \
        .attr = DT_PROP(node_id, attr), \
    },

#define IOPMP_INIT(idx)                                                                 \
    BUILD_ASSERT(CONFIG_IOPMP_SLOTS >= DT_CHILD_NUM(DT_INST_PHANDLE(idx, partitions))); \
    static struct iopmp_partition_attr attr_partition_##idx[] = {                       \
        DT_FOREACH_CHILD(DT_INST_PHANDLE(idx, partitions), PARTITION_CHILD)             \
    };                                                                                  \
    static const struct iopmp_config iopmp_cfg_##idx = {                                \
        .base = DT_INST_REG_ADDR(idx),                                                  \
        .attr = attr_partition_##idx,                                                   \
        .attr_num = DT_CHILD_NUM(DT_INST_PHANDLE(idx, partitions)),                     \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(idx,                                                          \
                          iopmp_init,                                                   \
                          NULL,                                                         \
                          NULL,                                                         \
                          &iopmp_cfg_##idx,                                             \
                          POST_KERNEL,                                                  \
                          CONFIG_APPLICATION_INIT_PRIORITY,                             \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(IOPMP_INIT)
