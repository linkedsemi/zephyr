/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_LINKEDSEMI_LS_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_LINKEDSEMI_LS_GPIO_H_

#define LS_GPIO_DS_POS                  9
#define LS_GPIO_DS_MASK                 (0x3U << LS_GPIO_DS_POS)

/* GPIO driver will use 1/4 out drive capability if DS is 0 or 1 */
#define LS_GPIO_DS_DEFAULT_DRIVE        (0x0U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_QUARTER_DRIVE        (0x1U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_HALF_DRIVE           (0x2U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_MAX_DRIVE            (0x3U << LS_GPIO_DS_POS)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_LINKEDSEMI_LS_GPIO_H_ */
