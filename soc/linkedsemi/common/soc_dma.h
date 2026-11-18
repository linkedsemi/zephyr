/*
 * Copyright (c) 2025 Linkedsemi.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_DMA_H_
#define _SOC_DMA_H_

#include <stdint.h>

void soc_dma_channel_handshake_set(uint32_t base, uint8_t ch_idx, uint8_t handshake);

#endif /* _SOC_H_ */
