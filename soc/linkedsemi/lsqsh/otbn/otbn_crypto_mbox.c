
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "stdio.h"

#include <zephyr/drivers/mbox.h>


// const struct device *otbn_mbox_dev = DEVICE_DT_GET(DT_NODELABEL(mbox_consumer_mbedtls));
// struct mbox_dt_spec ls_otbn_tx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_mbedtls),tx);
// struct mbox_dt_spec ls_otbn_rx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_mbedtls),rx);

// void ls_otbn_mbox_chanels_init(void)
// {
//     printf("ls_otbn_tx = 0x%x",ls_otbn_tx);
//     printf("ls_otbn_rx = 0x%x",ls_otbn_rx);
// }