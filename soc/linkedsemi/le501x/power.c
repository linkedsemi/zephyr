#include <zephyr/kernel.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>

#include <le501x/sleep.h>

__weak const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
    static const struct pm_state_info idle = PM_STATE_INFO_DT_INIT(DT_NODELABEL(idle));
    static const struct pm_state_info lp0 = PM_STATE_INFO_DT_INIT(DT_NODELABEL(lp0));
    if(mac_sleep_check())
    {
        return &lp0;
    }else
    {
        return &idle;
    }
}

__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    if(state == PM_STATE_STANDBY)
    {
        deep_sleep();
    }
}

__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    irq_unlock(0);
}

void arch_cpu_idle()
{
    irq_unlock(0);
}