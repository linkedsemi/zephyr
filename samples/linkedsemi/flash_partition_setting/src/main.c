/*
 * Copyright (c) 2025 linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/drivers/flash/soc_flash_ls.h>

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    uint32_t base = 0;
    uint32_t size = 0;
    uint32_t attr = 0;

    /* show static layout info */
    for (int idx = 0; idx < flash_ls_get_part_num(dev); idx++) {
        flash_ls_get_part_info(dev, idx, &base, &size, &attr);
        printf("idx: %d\n", idx);
        printf("    base: %x\n", base);
        printf("    size: %x\n", size);
        printf("    attr: %x\n", attr);
    }

    /* runtime set layout attr to 0 that app cpu can not read/write flash any more */
    for (int idx = 0; idx < flash_ls_get_part_num(dev); idx++) {
        flash_ls_set_part_attr(dev, idx, 0);
        flash_ls_get_part_info(dev, idx, &base, &size, &attr);
        printf("idx: %d\n", idx);
        printf("    base: %x\n", base);
        printf("    size: %x\n", size);
        printf("    attr: %x\n", attr);
    }

    /* runtime set layout attr to PMP_R | PMP_W that app cpu can normally read/write flash */
    for (int idx = 0; idx < flash_ls_get_part_num(dev); idx++) {
        flash_ls_set_part_attr(dev, idx, PMP_R | PMP_W);
        flash_ls_get_part_info(dev, idx, &base, &size, &attr);
        printf("idx: %d\n", idx);
        printf("    base: %x\n", base);
        printf("    size: %x\n", size);
        printf("    attr: %x\n", attr);
    }

    return 0;
}
