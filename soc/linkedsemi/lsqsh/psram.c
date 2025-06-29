#include <stdint.h>
#include <zephyr/kernel.h>
#include <reg_base_addr.h>
#include <per_func_mux.h>
#include <ls_soc_gpio.h>
#include <DWC_ssi_v2_header.h>
#include <ls_hal_ssi.h>
#include <core_rv32.h>

#if !defined(DW_FIELD_BUILD)
#define DW_FIELD_BUILD(field,val) \
    (((unsigned int)(val)<<(field##_BitAddressOffset))&(((1 << field##_RegisterSize) - 1)<<(field##_BitAddressOffset)))
#endif

#define CONFIG_PSRAM_CACHE
#define SSI_CLK_DIV              3
#define SSIC_VERSION_ID          0x3130332a
#define CMD_RESET_ENABLE         0x66
#define CMD_RESET                0x99
#define CMD_WRAP_BOUNDARY_TOGGLE 0xc0

void psram_pin_init(void)
{
    pinmux_cfg_pin_func_alt(PSRAM_CSN0_FUNC3_PG07_PIN, PSRAM_CSN0_FUNC3_PG07_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_CLK_FUNC3_PF12_PIN , PSRAM_CLK_FUNC3_PF12_FUNC , 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT0_FUNC3_PF01_PIN, PSRAM_DAT0_FUNC3_PF01_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT1_FUNC3_PT09_PIN, PSRAM_DAT1_FUNC3_PT09_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT3_FUNC3_PH12_PIN, PSRAM_DAT3_FUNC3_PH12_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT2_FUNC3_PF00_PIN, PSRAM_DAT2_FUNC3_PF00_FUNC, 0);
    io_cfg_input(PSRAM_DAT0_FUNC3_PF01_PIN);
    io_cfg_input(PSRAM_DAT1_FUNC3_PT09_PIN);
    io_cfg_input(PSRAM_DAT3_FUNC3_PH12_PIN);
    io_cfg_input(PSRAM_DAT2_FUNC3_PF00_PIN);
}

void psram_reset(void)
{
    SSI_HandleTypeDef SsiHandle = {0};
    uint8_t ssi_tx_buf[] = {CMD_RESET_ENABLE, CMD_RESET, CMD_WRAP_BOUNDARY_TOGGLE};
    SsiHandle.REG = (reg_ssi_t *)APP_PSRAM_CFG_ADDR;
#if 1
    SsiHandle.Init.clk_div = SSI_CLK_DIV << 1;
#else
    SsiHandle.Init.clk_div = SSI_CLK_DIV;
#endif
    SsiHandle.Init.rxsample_dly = 0;
    SsiHandle.Init.ctrl.cph = SCLK_Toggle_In_Middle;
    SsiHandle.Init.ctrl.cpol = Inactive_Low;
    SsiHandle.Init.ctrl.data_frame_size = DFS_32_8_bits;

    if (HAL_SSI_Init(&SsiHandle) != HAL_OK) {
        /* Initialization Error */
        while(1);
    }

    if (HAL_SSI_Transmit(&SsiHandle, (uint8_t *)&ssi_tx_buf[0], 1) != HAL_OK) {
        while(1);
    }
    // k_msleep(1);
    for(uint32_t i = 0; i < 10000; i++) __NOP();
    if (HAL_SSI_Transmit(&SsiHandle, (uint8_t *)&ssi_tx_buf[1], 1) != HAL_OK) {
        while(1);
    }
    // k_msleep(1);
    for(uint32_t i = 0; i < 10000; i++) __NOP();
    if (HAL_SSI_Transmit(&SsiHandle, (uint8_t *)&ssi_tx_buf[2], 1) != HAL_OK) {
        while(1);
    }
}

void psram_init(void) {
    uint32_t val = 0;

    if (SSIC_VERSION_ID != sys_read32(APP_PSRAM_CFG_ADDR + SSIV2_SSIC_VERSION_ID)) {
        return; /* it has been initialized */
    }
    psram_pin_init();
    psram_reset();

    val = DW_FIELD_BUILD(SSIV2_CTRLR0_SSI_IS_MST, 0x1)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_SPI_FRF, 0x2)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_CFS, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_TMOD, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_FRF, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_DFS, 0x1f);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_CTRLR0);

    val = DW_FIELD_BUILD(SSIV2_BAUDR_SCKDV, SSI_CLK_DIV);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_BAUDR);

    val = DW_FIELD_BUILD(SSIV2_TXFTLR_TFT, 0x8);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_TXFTLR);

    val = DW_FIELD_BUILD(SSIV2_RXFTLR_RFT, 0x8);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_RXFTLR);

    val = 0;
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_IMR);
 
    val = DW_FIELD_BUILD(SSIV2_XIP_MODE_BITS_XIP_MD_BITS, 0x0);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_MODE_BITS);
 
    val = DW_FIELD_BUILD(SSIV2_XIP_INCR_INST_INCR_INST, 0xeb);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_INCR_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRAP_INST_WRAP_INST, 0xeb);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRAP_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES, 0x0)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_INST_L, 0x2)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_ADDR_L, 0x6)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_TRANS_TYPE, 0x1)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_FRF, 0x2);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_CTRL);

    val = DW_FIELD_BUILD(SSIV2_XIP_CTRL_XIP_PREFETCH_EN, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_XIP_MBL, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_INST_EN, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_DFS_HC, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_WAIT_CYCLES, 0x6)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_MD_BITS_EN, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_INST_L, 0x2)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_ADDR_L, 0x6)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_TRANS_TYPE, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_FRF, 0x2);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_CTRL);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_INCR_INST_INCR_WRITE_INST, 0x38);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_INCR_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_WRAP_INST_WRAP_WRITE_INST, 0x38);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_WRAP_INST);

    val = DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_PREFETCH_EN, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_MBL, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_WAIT_CYCLES, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_INST_L, 0x2)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_MD_BIT_EN, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_TRANS_TYPE, 0x1);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_SPI_CTRLR0);

    val = DW_FIELD_BUILD(SSIV2_SSIENR_SSIC_EN, 1);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_SSIENR);

    /* cache enable */
    val = 0x00070001;
    sys_write32(val, APP_PSRAM_CCH_ADDR + 0x10);
    while(sys_read32(APP_PSRAM_CCH_ADDR + 0x28) != 0x101);

    /* ahb config */
#if defined(CONFIG_PSRAM_CACHE)
    val = 0x4c2; /* cacheable && bufferble */
#else
    val = 0x402; /* non-cacheable && non-bufferble */
#endif
    sys_write32(val, APP_SYSC_CPU_APP_ADDR + 0x84);

    /* xip enable */
    val = BIT(0) | BIT(2);
    sys_write32(val, APP_SYSC_CPU_APP_ADDR + 0x80);
}
