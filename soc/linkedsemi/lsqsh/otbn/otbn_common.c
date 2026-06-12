#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "reg_otbn_type.h"
#include "ls_msp_otbn.h"
#include "ls_hal_otbn.h"
#include "co_math.h"
#include "qsh.h"
#include "ls_otbn_config.h"
#include <stdio.h>
#include <math.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ls_otbn, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

#define MBEDTLS_ERR_LS_OTBN_BUSY -0x135
#define OTBN_OPERATION_TIMEOUT_MS 10000

static struct k_mutex otbn_lock;
static struct k_sem wait_complete;
static bool otbn_inited;
volatile struct k_thread *otbn_owner_thread = NULL;
static otbn_firmware_t current_obtn_firmware = OTBN_FIRMWARE_UNUSED;
/*
#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN)&&defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_ENABLE)
BUILD_ASSERT(false,"otbn is reused");
#endif
*/

static uint32_t ls_otbn_default_prng_cb(void)
{
    return (uint32_t)rand();
}

static void *s_otbn_param = NULL;
static otbn_done_callback_t s_otbn_done_handler  = NULL;
static otbn_rand_cb s_trng_cb = ls_otbn_default_prng_cb;
static otbn_rand_cb s_prng_cb = ls_otbn_default_prng_cb;
static uint32_t EDN_URND_BUS_IN;


static void LS_OTBN_IRQHandler(void)
{
    if (LSOTBN->INTR_STATE)
    {
        LSOTBN->INTR_STATE = OTBN_INTR_STATE_DONE_MASK;
        k_sem_give(&wait_complete);
    }
    if(s_otbn_done_handler)
    {
        s_otbn_done_handler(s_otbn_param);
    }
}

static void LS_OTBN_SYSC_IRQHandler(void)
{
    uint32_t intr = SYSC_SEC_CPU->OTBN_INTR_STT;
    if (intr & CO_BIT(0))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(0);
    }
    if (intr & CO_BIT(1))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(1);
    }
    if (intr & CO_BIT(2))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(2);
    }
    if (intr & CO_BIT(3))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(3);
    }
    if (intr & 0xf0)
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = 0xf0;
    }
    if (intr & CO_BIT(8))
    {
        SYSC_SEC_CPU->EDN_RND_BUS = s_trng_cb();
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(8)|CO_BIT(9);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(8);
    }
    if (intr & CO_BIT(9))
    {
        SYSC_SEC_CPU->EDN_URND_BUS = s_prng_cb();
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(10)|CO_BIT(11);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(9);
    }
    if (intr & CO_BIT(10))
    {
        SYSC_SEC_CPU->OTBN_OTP_KEY_0 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_1 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_2 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_3 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(12);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(10);
    }
}

void ls_otbn_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb)
{
    s_trng_cb = trng_cb ? trng_cb : ls_otbn_default_prng_cb;
    s_prng_cb = prng_cb ? prng_cb : ls_otbn_default_prng_cb;
}

void ls_otbn_done_callback_register(otbn_done_callback_t handler, void *param)
{
    s_otbn_done_handler = handler;
    s_otbn_param = param;
}

void ls_otbn_done_callback_unregister(void)
{
    s_otbn_done_handler = NULL;
    s_otbn_param = NULL;
}

void ls_otbn_module_init(void)
{
    if (otbn_inited) {
        return;
    }
    REG_FIELD_WR(SYSC_SEC_CPU->INTR_CTRL_INTR_MSK, SYSC_SEC_CPU_I_EDN_URND_REQ, 0);
    SYSC_SEC_CPU->PD_CPU_CLKG[1] = SYSC_SEC_CPU_CLKG_CLR_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_SRST[1] = SYSC_SEC_CPU_SRST_CLR_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_SRST[1] = SYSC_SEC_CPU_SRST_SET_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_CLKG[1] = SYSC_SEC_CPU_CLKG_SET_OTBN_MASK;
    for (uint8_t i = 0; i < 16; i++)
    {
        while (!REG_FIELD_RD(SYSC_SEC_CPU->OTBN_INTR_RAW, SYSC_SEC_CPU_I_EDN_URND_REQ)) ;
        SYSC_SEC_CPU->EDN_URND_BUS = ++EDN_URND_BUS_IN;
        REG_FIELD_WR(SYSC_SEC_CPU->OTBN_CTRL2, SYSC_SEC_CPU_EDN_URND_ACK, 1);
        REG_FIELD_WR(SYSC_SEC_CPU->OTBN_CTRL2, SYSC_SEC_CPU_EDN_URND_ACK, 0);
        SYSC_SEC_CPU->INTR_CLR_MSK = SYSC_SEC_CPU_I_EDN_URND_REQ_MASK;
    }
    SYSC_SEC_CPU->INTR_CLR_MSK = FIELD_BUILD(SYSC_SEC_CPU_I_EDN_RND_REQ, 1) |
                            FIELD_BUILD(SYSC_SEC_CPU_I_EDN_URND_REQ, 1) |
                            FIELD_BUILD(SYSC_SEC_CPU_I_OTBN_OTP_REQ, 1);
    SYSC_SEC_CPU->INTR_CTRL_INTR_MSK = FIELD_BUILD(SYSC_SEC_CPU_I_EDN_RND_REQ, 1) |
                              FIELD_BUILD(SYSC_SEC_CPU_I_EDN_URND_REQ, 1) |
                              FIELD_BUILD(SYSC_SEC_CPU_I_OTBN_OTP_REQ, 1);

    // otbn system irq
    IRQ_CONNECT(OTBN_SYSC_IRQN, 3, LS_OTBN_SYSC_IRQHandler,NULL, 0);
    irq_enable(OTBN_SYSC_IRQN);
    // otbn status irq
    IRQ_CONNECT(OBTN_IRQN, 3, LS_OTBN_IRQHandler,NULL, 0);
    irq_enable(OBTN_IRQN);

    k_sem_init(&wait_complete,0,1);
    k_mutex_init(&otbn_lock);
    otbn_inited = true;
}

int ls_otbn_module_deinit(void)
{
    LOG_DBG("%s", __func__);

    /* 如果当前线程是 owner，先正常释放 session */
    if (otbn_owner_thread == k_current_get()) {
        otbn_owner_thread = NULL;
        current_obtn_firmware = OTBN_FIRMWARE_UNUSED;
        k_mutex_unlock(&otbn_lock);
    }

    /* 阻塞等待其他线程释放锁 */
    int ret = k_mutex_lock(&otbn_lock, K_FOREVER);
    if (ret != 0) {
        LOG_ERR("%s: mutex lock failed (%d)", __func__, ret);
        return ret;
    }

    /* 如果 OTBN 正在执行，等待完成 */
    if (!HAL_OTBN_In_Idle_State()) {
        k_sem_reset(&wait_complete);
        ret = k_sem_take(&wait_complete, K_MSEC(OTBN_OPERATION_TIMEOUT_MS));
        if (ret != 0) {
            LOG_ERR("%s: wait for OTBN completion timeout", __func__);
            k_mutex_unlock(&otbn_lock);
            return ret;
        }
    }

    /* 执行 deinit */
    int key = irq_lock();
    HAL_LSOTBN_MSP_DeInit();
    otbn_inited = false;
    otbn_owner_thread = NULL;
    current_obtn_firmware = OTBN_FIRMWARE_UNUSED;
    irq_unlock(key);
    k_sem_reset(&wait_complete);
    k_mutex_unlock(&otbn_lock);

    return 0;
}

int ls_otbn_session_acquire(otbn_firmware_t firmware_id, otbn_timeout_s timeout)
{
    LOG_DBG("%s, firmware_id=%u, timeout=%u", __func__, firmware_id, timeout);

    struct k_thread *current_thread = k_current_get();

    /* 同线程重入直接拒绝，避免非递归 mutex 死锁 */
    if (otbn_owner_thread == current_thread) {
        LOG_ERR("%s: reentrant acquire rejected (firmware=%u)",
                __func__, firmware_id);
        return -EBUSY;
    }

    if (current_obtn_firmware != OTBN_FIRMWARE_UNUSED) {
        LOG_ERR("%s: busy, current firmware=%u, requested=%u",
                __func__, current_obtn_firmware, firmware_id);
        return -EBUSY;
    }
    current_obtn_firmware = firmware_id;
    ls_otbn_module_init();

    int ret = k_mutex_lock(&otbn_lock, K_SECONDS(timeout));
    if (ret != 0) {
        LOG_ERR("%s: mutex lock timeout (firmware=%u, timeout=%u)",
                __func__, firmware_id, timeout);
        current_obtn_firmware = OTBN_FIRMWARE_UNUSED;
        return -EBUSY;
    }

    __ASSERT(otbn_owner_thread == NULL,
             "OTBN owner not NULL after lock");

    otbn_owner_thread = current_thread;
    LOG_INF("start otbn session, firmware_id = %u",firmware_id);
    return 0;
}

void ls_otbn_session_release(void)
{
    LOG_DBG("%s", __func__);

    __ASSERT(otbn_owner_thread != NULL,
             "OTBN release without owner");
    __ASSERT(otbn_owner_thread == k_current_get(),
             "OTBN release by non-owner");

    otbn_owner_thread = NULL;
    current_obtn_firmware = OTBN_FIRMWARE_UNUSED;
    k_mutex_unlock(&otbn_lock);
}

bool ls_otbn_session_is_owner(void)
{
    if (otbn_owner_thread != k_current_get()) {
        LOG_DBG("%s: not owner, owner=%p, current=%p",
                __func__, (void *)otbn_owner_thread, (void *)k_current_get());
        return false;
    }
    return true;
}

int ls_otbn_cmd(enum otbn_cmd_t cmd)
{
    int rc;
    // LOG_DBG("ls_otbn_cmd : 0x%x",cmd);
    if (LSOTBN->INTR_STATE)
        LSOTBN->INTR_STATE = OTBN_INTR_STATE_DONE_MASK;
    k_sem_reset(&wait_complete);
    LSOTBN->INTR_ENABLE = OTBN_INTR_ENABLE_EN_MASK;
    LSOTBN->CMD = cmd;
    rc = k_sem_take(&wait_complete, K_MSEC(OTBN_OPERATION_TIMEOUT_MS));
    if (rc != 0) {
        if (rc == -EAGAIN) {
            LOG_ERR("%s timeout", __func__);
            rc = -ETIMEDOUT;
        }
        return rc;
    }
    return 0;
}

static int ls_otbn_hal_status_to_errno(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return 0;
    case HAL_BUSY:
        return -EBUSY;
    case HAL_INVALIAD_PARAM:
        return -EINVAL;
    default:
        return -EIO;
    }
}

static int ls_otbn_check_idle(void)
{
    if (!HAL_OTBN_In_Idle_State()) {
        LOG_DBG("%s: OTBN not idle", __func__);
        return -EBUSY;
    }
    return 0;
}

static int ls_otbn_mem_param_check(uint32_t offset, uint32_t size)
{
    if ((offset & 0x3) || (size & 0x3)) {
        LOG_ERR("%s: offset=%u size=%u not 4-byte aligned",
                __func__, offset, size);
        return -EINVAL;
    }
    return 0;
}

int ls_otbn_imem_write(uint32_t offset, const uint32_t *src, uint32_t size)
{
    int ret = ls_otbn_mem_param_check(offset, size);
    if (ret != 0) {
        return ret;
    }
    ret = ls_otbn_check_idle();
    if (ret != 0) {
        return ret;
    }
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }
    return ls_otbn_hal_status_to_errno(
        HAL_OTBN_IMEM_Write(offset, (uint32_t *)src, size));
}

int ls_otbn_imem_read(uint32_t offset, uint32_t *dst, uint32_t size)
{
    int ret = ls_otbn_mem_param_check(offset, size);
    if (ret != 0) {
        return ret;
    }
    ret = ls_otbn_check_idle();
    if (ret != 0) {
        return ret;
    }
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }
    return ls_otbn_hal_status_to_errno(
        HAL_OTBN_IMEM_Read(offset, dst, size));
}

int ls_otbn_dmem_write(uint32_t offset, const uint32_t *src, uint32_t size)
{
    int ret = ls_otbn_mem_param_check(offset, size);
    if (ret != 0) {
        return ret;
    }
    ret = ls_otbn_check_idle();
    if (ret != 0) {
        return ret;
    }
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }
    return ls_otbn_hal_status_to_errno(
        HAL_OTBN_DMEM_Write(offset, (uint32_t *)src, size));
}

int ls_otbn_dmem_read(uint32_t offset, uint32_t *dst, uint32_t size)
{
    int ret = ls_otbn_mem_param_check(offset, size);
    if (ret != 0) {
        return ret;
    }
    ret = ls_otbn_check_idle();
    if (ret != 0) {
        return ret;
    }
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }
    return ls_otbn_hal_status_to_errno(
        HAL_OTBN_DMEM_Read(offset, dst, size));
}

int ls_otbn_dmem_set(uint32_t offset, uint32_t data, uint32_t size)
{
    int ret = ls_otbn_mem_param_check(offset, size);
    if (ret != 0) {
        return ret;
    }
    ret = ls_otbn_check_idle();
    if (ret != 0) {
        return ret;
    }
    if (!ls_otbn_session_is_owner()) {
        return -EACCES;
    }
    return ls_otbn_hal_status_to_errno(
        HAL_OTBN_DMEM_Set(offset, data, size));
}