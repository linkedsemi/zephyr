#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/state.h>
#include "platform.h"
#include "sleep.h"
#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define TIMEOUT_MIN_TICKS 5
struct hw_time
{
    uint32_t hs;
    uint32_t hus;
};
struct hw_time last_anchor;
struct hw_time target;
int32_t os_sleep_ticks;
static struct k_spinlock lock;

struct hw_time ble_time_get();
void ble_timer_set(struct hw_time target);
void sys_init_ll();
void ble_ll_init();
void ke_timer_os_wrapper_init(void (*start)(void **,uint32_t),void (*stop)(void **),void (*tick_handler)(),void (*wakeup_compensate)(uint32_t));
void os_wrapped_timer_callback(uint32_t param);
void eif_init(void *read,void *write,void *flow_on,void *flow_off);
void ll_hci_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *,uint8_t), void* dummy);
void ll_hci_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *,uint8_t), void* dummy);
void ll_hci_flow_on();
bool ll_hci_flow_off();
void ble_ll_task_event_set();
void func_post(void (*func)(void *),void *param);

static void timeout_handler(void *param)
{
    struct k_timer **timer = param;
    os_wrapped_timer_callback((uint32_t)timer);
    k_free(*timer);
    *timer = NULL;
}

static void timer_expiry_callback(struct k_timer *timer)
{
    void **p_timer = k_timer_user_data_get(timer);
    func_post(timeout_handler,p_timer);
    ble_ll_task_event_set();
}

static void timer_start(void **timer,uint32_t delay)
{
    *timer = k_malloc(sizeof(struct k_timer));
    k_timer_init(*timer,timer_expiry_callback,NULL);
    k_timer_user_data_set(*timer,timer);
    k_timer_start(*timer,K_MSEC(delay),K_NO_WAIT);
}

static void timer_stop(void **timer)
{
    k_timer_stop(*timer);
    k_free(*timer);
    *timer = NULL;
}

static void os_tick_handler()
{
    struct hw_time current_match = target;
    sys_clock_announce((((target.hs - last_anchor.hs) & 0xfffffff)*625 - last_anchor.hus + target.hus)/CYC_PER_TICK);
    last_anchor = current_match;
}

static void wakeup_compenstation(uint32_t hus)
{
    // int32_t ticks = ceiling_fraction(hus,2*1000000/CONFIG_SYS_CLOCK_TICKS_PER_SEC);
    // sys_clock_announce(ticks);
}

static int32_t os_sleep_duration_get()
{
    int32_t dur = OSTICK_HS_STEP_INC(CONFIG_SYS_CLOCK_TICKS_PER_SEC,(uint64_t)os_sleep_ticks);
    return dur>=0?dur:INT_MAX;
}

static int le501x_init(const struct device *arg)
{
    sys_init_ll();
    extern int32_t (*os_sleep_duration_get_fn)();
    os_sleep_duration_get_fn = os_sleep_duration_get;
    eif_init(ll_hci_read,ll_hci_write,ll_hci_flow_on,ll_hci_flow_off);
    ke_timer_os_wrapper_init(timer_start,timer_stop,os_tick_handler,wakeup_compenstation);
    ble_ll_init();
    return 0;
}

SYS_INIT(le501x_init, PRE_KERNEL_1, 0);

void ll_stack_reset_hook()
{
    last_anchor.hs = 0;
    last_anchor.hus = 0;
};

uint32_t sys_clock_elapsed(void)
{
    struct hw_time current = ble_time_get();
    return (((current.hs-last_anchor.hs)&0xfffffff)*625 + current.hus - last_anchor.hus)/CYC_PER_TICK;
}

void sys_clock_set_timeout(int32_t ticks,bool idle)
{
    if(ticks<TIMEOUT_MIN_TICKS) ticks = TIMEOUT_MIN_TICKS;
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t step = sys_clock_elapsed()+ticks;
    target.hus = last_anchor.hus + OSTICK_HUS_STEP_INC(CONFIG_SYS_CLOCK_TICKS_PER_SEC,step);
    if(target.hus >= 625)
    {
        target.hus -= 625;
        target.hs = (1 + last_anchor.hs + OSTICK_HS_STEP_INC(CONFIG_SYS_CLOCK_TICKS_PER_SEC,step)) & 0xfffffff;
    }else
    {
        target.hs = (last_anchor.hs + OSTICK_HS_STEP_INC(CONFIG_SYS_CLOCK_TICKS_PER_SEC,step)) & 0xfffffff;
    }
    ble_timer_set(target);
	k_spin_unlock(&lock, key);
}

uint64_t sys_clock_cycle_get_64(void)
{
    struct hw_time tim = ble_time_get();
    return tim.hs*625+tim.hus;
}

uint32_t sys_clock_cycle_get_32()
{
    return sys_clock_cycle_get_64();
}

uint32_t ble_isr();
void ble_stack_isr()
{
    uint32_t irq_stat = ble_isr();
    if(irq_stat != 0x40)
    {
        ble_ll_task_event_set();
    }
    z_arm_int_exit();
}

const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
    static const struct pm_state_info idle = PM_STATE_INFO_DT_INIT(DT_NODELABEL(idle));
    static const struct pm_state_info lp0 = PM_STATE_INFO_DT_INIT(DT_NODELABEL(lp0));
    os_sleep_ticks = ticks;
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