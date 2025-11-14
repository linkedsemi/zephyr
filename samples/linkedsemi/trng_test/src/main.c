#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include "ls_hal_iwdgv2.h"
#include "ls_hal_trng.h"
#include "platform.h"
#include "reg_sec_pmu_rg.h"
static void dump_hex_limited(const uint8_t *buf, size_t len, size_t max_show)
{
	size_t show = len < max_show ? len : max_show;
	for (size_t i = 0; i < show; ++i) {
		printf("%02x%s", buf[i], ((i + 1) % 16 == 0) ? "\n" : " ");
	}
	if (show % 16 != 0) {
		printf("\n");
	}
	if (len > show) {
		printf("... (%zu bytes shown of %zu)\n", show, len);
	}
}



void main(void)
{
	HAL_IWDG_DeInit(SEC_PMU_IWDG);
  HAL_IWDG_DeInit(SEC_IWDG);
  SEC_PMU->SFT_CTRL[2] &= ~0xf;
	printf("TRNG device testing start\n");

	const struct device *trng = DEVICE_DT_GET(DT_NODELABEL(trng1));
	if (!device_is_ready(trng)) {
		printf("TRNG device not ready\n");
		return;
	}
	printf("TRNG ready.\n");


	k_sleep(K_MSEC(10));

	int rc;
	static uint8_t buf[12288];


	size_t want1 = 2048;
	printf("\n[TEST 1] thread get_entropy blocking: %zu bytes\n", want1);
	rc = entropy_get_entropy(trng, buf, want1);
	if (rc != 0) {
		printf("get_entropy failed: %d\n", rc);
	} else {
		printf("get_entropy ok\n");
		dump_hex_limited(buf, want1, 2048);
	}

	size_t want2 = 1024;
	printf("\n[TEST 2] ISR get_entropy_isr non-busy single call: request %zu bytes\n", want2);
	rc = entropy_get_entropy_isr(trng, buf, want2, 0);
	if (rc < 0) {
		printf("get_entropy_isr(non-busy) failed: %d\n", rc);
	} else {
		printf("get_entropy_isr(non-busy) got %d/%zu bytes\n", rc, want2);
		dump_hex_limited(buf, (size_t)rc, 1024);
	}


	size_t want3 = 2048;
	printf("\n[TEST 3] ISR get_entropy_isr BUSYWAIT: %zu bytes\n", want3);
	rc = entropy_get_entropy_isr(trng, buf, want3, ENTROPY_BUSYWAIT);
	if (rc < 0) {
		printf("get_entropy_isr(busywait) failed: %d\n", rc);
	} else {
		printf("get_entropy_isr(busywait) returned %d bytes (expect %zu)\n", rc, want3);
		dump_hex_limited(buf, (size_t)rc, 2048);
	}


	printf("\n[TEST 4] multiple non-busy calls (16B x 16)\n");
	for (int i = 0; i < 16; i++) {
		rc = entropy_get_entropy_isr(trng, buf, 16, 0);
		if (rc < 0) {
			printf("call %d failed: %d\n", i + 1, rc);
		} else {
			printf("call %d returned %d bytes\n", i + 1, rc);
		}
		k_sleep(K_MSEC(20));
	}
	size_t want5 = 2048;
	printf("\n[TEST 1] thread get_entropy blocking: %zu bytes\n", want5);
	rc = entropy_get_entropy(trng, buf, want5);
	if (rc != 0) {
		printf("get_entropy failed: %d\n", rc);
	} else {
		printf("get_entropy ok\n");
		dump_hex_limited(buf, want5, 2048);
	}
	printf("\nTRNG tests done.\n");
}
