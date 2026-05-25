#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <float.h>
#include <inttypes.h>
#include <stdbool.h>
#include <platform.h>
#include <ls_hal_cache.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#define UINT64_DOUBLE_SAFE_MAX (UINT64_C(1) << DBL_MANT_DIG)

extern void cpu_sysmap_show(void);

static bool u64_to_double_safe(uint64_t val, double *out)
{
    if (val > UINT64_DOUBLE_SAFE_MAX) {
        return false;
    }

    *out = (double)val;
    return true;
}

static void pmu_counter_init(int hw_id, int event_code)
{
    csi_pmu_hpmcounter_disable(hw_id);
    csi_pmu_hpmcounter_disable_interrupt(hw_id);
    csi_pmu_hpmcounter_write_value(hw_id, 0);
    csi_pmu_hpmcounter_write_event(hw_id, event_code);
    csi_pmu_hpmcounter_enable(hw_id);
}

static void print_qspi_cache_stat(const struct shell *sh, const char *name, uint32_t hit, uint32_t miss)
{
    if ((hit + miss) > 0U) {
        shell_print(sh, "%s cache hit: %u miss: %u rate: %lf",
                    name, hit, miss, ((double)hit * 100.0) / ((double)hit + (double)miss));
    } else {
        shell_print(sh, "%s cache rate: 0.00", name);
    }
}

static bool read_cpu_icache_counter(const struct shell *sh, int hw_id, const char *kind, double *out)
{
    uint64_t val = csi_pmu_hpmcounter_read_value(hw_id);

    if (!u64_to_double_safe(val, out)) {
        shell_print(sh, "cpu icache counter %d, %s counter_value overflow for double: %llu",
                    hw_id, kind, val);
        return false;
    }

    shell_print(sh, "cpu icache counter %d, %s counter_value = %llu", hw_id, kind, val);
    return true;
}

static int cmd_cache_init(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    __set_MCYCLE(0);
    pmu_counter_init(3, PERF_HARDWARE_CACHE_L1I_RD_ACCESS);
    pmu_counter_init(4, PERF_HARDWARE_CACHE_L1I_RD_MISS);

    __set_MCOUNTEREN(0xffffffff);

    if (ls_clock_control_is_on(CACHE1_CLOCK)) {
        lscache_cachex_stat_enable(LSCACHE);
    }
    if (ls_clock_control_is_on(CACHE2_CLOCK)) {
        lscache_cachex_stat_enable(LSCACHE2);
    }

    return 0;
}

static int cmd_cache_rate_show(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "\n");
    double miss;
    double access;

    bool access_ok = read_cpu_icache_counter(sh, 3, "access", &access);
    bool miss_ok = read_cpu_icache_counter(sh, 4, "miss", &miss);

    if (!miss_ok) {
        shell_print(sh, "cpu icache rate: N/A");
    } else if (access_ok && access > 0.0) {
        shell_print(sh, "cpu icache rate: %lf", ((1.0 - (miss / access)) * 100.0));
    } else {
        shell_print(sh, "cpu icache rate: 0.00");
    }

    if (ls_clock_control_is_on(CACHE1_CLOCK)) {
        uint32_t hit = READ_REG(LSCACHE->CSHR);
        uint32_t cache_miss = READ_REG(LSCACHE->CSMR);
        print_qspi_cache_stat(sh, "qspi1", hit, cache_miss);
    }
    if (ls_clock_control_is_on(CACHE2_CLOCK)) {
        uint32_t hit = READ_REG(LSCACHE2->CSHR);
        uint32_t cache_miss = READ_REG(LSCACHE2->CSMR);
        print_qspi_cache_stat(sh, "qspi2", hit, cache_miss);
    }

    return 0;
}

static int cmd_cache_config_show(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    cpu_sysmap_show();

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cache_cmds,
    SHELL_CMD_ARG(init,
                  NULL,
                  "Usage: init",
                  cmd_cache_init,
                  1,
                  0),
    SHELL_CMD_ARG(rate_show,
                  NULL,
                  "Usage: show",
                  cmd_cache_rate_show,
                  1,
                  0),
    SHELL_CMD_ARG(config_show,
                  NULL,
                  "Usage: show",
                  cmd_cache_config_show,
                  1,
                  0),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(soc_cache, &sub_cache_cmds, "cache util commands", NULL);
