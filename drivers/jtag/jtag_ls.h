/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _JTAG_MASTER_H_
#define _JTAG_MASTER_H_

#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

enum jtag_ls_tap_states {
	LS_TAP_RESET,      /* 0: Test-Logic Reset */
	LS_TAP_IDLE,       /* 1: Run-Test/ Idle */

	LS_TAP_SELECT_DR,  /* 2: Select-DR/ Scan */
	LS_TAP_CAPTURE_DR, /* 3: Capture-DR */
	LS_TAP_SHIFT_DR,   /* 4: Shift-DR */
	LS_TAP_EXIT1_DR,   /* 5: Exit1-DR */
	LS_TAP_PAUSE_DR,   /* 6: Pause-DR */
	LS_TAP_EXIT2_DR,   /* 7: Exit2-DR */
	LS_TAP_UPDATE_DR,  /* 8: Update-DR */

	LS_TAP_SELECT_IR,  /* 9: Select-IR/ Scan */
	LS_TAP_CAPTURE_IR, /* 10: Capture-IR */
	LS_TAP_SHIFT_IR,   /* 11: Shift-IR */
	LS_TAP_EXIT1_IR,   /* 12: Exit1-IR */
	LS_TAP_PAUSE_IR,   /* 13: Pause-IR */
	LS_TAP_EXIT2_IR,   /* 14: Exit2-IR */
	LS_TAP_UPDATE_IR,  /* 15: Update-IR */

	LS_TAP_CURRENT_STATE
};

/* this structure represents a TMS cycle, as expressed in a set of bits and
 * a count of bits (note: there are no start->end state transitions that
 * require more than 1 byte of TMS cycles)
 */
struct tms_cycle {
	uint8_t tms_bits;
	uint8_t count;
};

/* this is the complete set TMS cycles for going from any TAP state to
 * any other TAP state, following a ｡ｧshortest path｡ｨ rule
 */
const struct tms_cycle _tms_cycle_lookup[][16] = {
/*      RESET       IDLE       SELECT_DR  CAPTURE_DR SHIFT_DR   */
/*      EXIT1_DR    PAUSE_DR   EXIT2_DR   UPDATE_DR  SELECT_IR  */
/*      CAPTURE_IR  SHIFT_IR   EXIT1_IR   PAUSE_IR   EXIT2_IR   */
/*      UPDATE_IR                                               */
/* RESET */
	{
		{ 0x01, 1 }, { 0x00, 1 }, { 0x02, 2 }, { 0x02, 3 }, { 0x02, 4 },
		{ 0x0a, 4 }, { 0x0a, 5 }, { 0x2a, 6 }, { 0x1a, 5 }, { 0x06, 3 },
		{ 0x06, 4 }, { 0x06, 5 }, { 0x16, 5 }, { 0x16, 6 }, { 0x56, 7 },
		{ 0x36, 6 }
	},
/* IDLE */
	{
		{ 0x07, 3 }, { 0x00, 1 }, { 0x01, 1 }, { 0x01, 2 }, { 0x01, 3 },
		{ 0x05, 3 }, { 0x05, 4 }, { 0x15, 5 }, { 0x0d, 4 }, { 0x03, 2 },
		{ 0x03, 3 }, { 0x03, 4 }, { 0x0b, 4 }, { 0x0b, 5 }, { 0x2b, 6 },
		{ 0x1b, 5 }
	},
/* SELECT_DR */
	{
		{ 0x03, 2 }, { 0x03, 3 }, { 0x00, 0 }, { 0x00, 1 }, { 0x00, 2 },
		{ 0x02, 2 }, { 0x02, 3 }, { 0x0a, 4 }, { 0x06, 3 }, { 0x01, 1 },
		{ 0x01, 2 }, { 0x01, 3 }, { 0x05, 3 }, { 0x05, 4 }, { 0x15, 5 },
		{ 0x0d, 4 }
	},
/* CAPTURE_DR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x00, 0 }, { 0x00, 1 },
		{ 0x01, 1 }, { 0x01, 2 }, { 0x05, 3 }, { 0x03, 2 }, { 0x0f, 4 },
		{ 0x0f, 5 }, { 0x0f, 6 }, { 0x2f, 6 }, { 0x2f, 7 }, { 0xaf, 8 },
		{ 0x6f, 7 }
	},
/* SHIFT_DR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x07, 4 }, { 0x00, 0 },
		{ 0x01, 1 }, { 0x01, 2 }, { 0x05, 3 }, { 0x03, 2 }, { 0x0f, 4 },
		{ 0x0f, 5 }, { 0x0f, 6 }, { 0x2f, 6 }, { 0x2f, 7 }, { 0xaf, 8 },
		{ 0x6f, 7 }
	},
/* EXIT1_DR */
	{
		{ 0x0f, 4 }, { 0x01, 2 }, { 0x03, 2 }, { 0x03, 3 }, { 0x02, 3 },
		{ 0x00, 0 }, { 0x00, 1 }, { 0x02, 2 }, { 0x01, 1 }, { 0x07, 3 },
		{ 0x07, 4 }, { 0x07, 5 }, { 0x17, 5 }, { 0x17, 6 }, { 0x57, 7 },
		{ 0x37, 6 }
	},
/* PAUSE_DR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x07, 4 }, { 0x01, 2 },
		{ 0x05, 3 }, { 0x00, 1 }, { 0x01, 1 }, { 0x03, 2 }, { 0x0f, 4 },
		{ 0x0f, 5 }, { 0x0f, 6 }, { 0x2f, 6 }, { 0x2f, 7 }, { 0xaf, 8 },
		{ 0x6f, 7 }
	},
/* EXIT2_DR */
	{
		{ 0x0f, 4 }, { 0x01, 2 }, { 0x03, 2 }, { 0x03, 3 }, { 0x00, 1 },
		{ 0x02, 2 }, { 0x02, 3 }, { 0x00, 0 }, { 0x01, 1 }, { 0x07, 3 },
		{ 0x07, 4 }, { 0x07, 5 }, { 0x17, 5 }, { 0x17, 6 }, { 0x57, 7 },
		{ 0x37, 6 }
	},
/* UPDATE_DR */
	{
		{ 0x07, 3 }, { 0x00, 1 }, { 0x01, 1 }, { 0x01, 2 }, { 0x01, 3 },
		{ 0x05, 3 }, { 0x05, 4 }, { 0x15, 5 }, { 0x00, 0 }, { 0x03, 2 },
		{ 0x03, 3 }, { 0x03, 4 }, { 0x0b, 4 }, { 0x0b, 5 }, { 0x2b, 6 },
		{ 0x1b, 5 }
	},
/* SELECT_IR */
	{
		{ 0x01, 1 }, { 0x01, 2 }, { 0x05, 3 }, { 0x05, 4 }, { 0x05, 5 },
		{ 0x15, 5 }, { 0x15, 6 }, { 0x55, 7 }, { 0x35, 6 }, { 0x00, 0 },
		{ 0x00, 1 }, { 0x00, 2 }, { 0x02, 2 }, { 0x02, 3 }, { 0x0a, 4 },
		{ 0x06, 3 }
	},
/* CAPTURE_IR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x07, 4 }, { 0x07, 5 },
		{ 0x17, 5 }, { 0x17, 6 }, { 0x57, 7 }, { 0x37, 6 }, { 0x0f, 4 },
		{ 0x00, 0 }, { 0x00, 1 }, { 0x01, 1 }, { 0x01, 2 }, { 0x05, 3 },
		{ 0x03, 2 }
	},
/* SHIFT_IR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x07, 4 }, { 0x07, 5 },
		{ 0x17, 5 }, { 0x17, 6 }, { 0x57, 7 }, { 0x37, 6 }, { 0x0f, 4 },
		{ 0x0f, 5 }, { 0x00, 0 }, { 0x01, 1 }, { 0x01, 2 }, { 0x05, 3 },
		{ 0x03, 2 }
	},
/* EXIT1_IR */
	{
		{ 0x0f, 4 }, { 0x01, 2 }, { 0x03, 2 }, { 0x03, 3 }, { 0x03, 4 },
		{ 0x0b, 4 }, { 0x0b, 5 }, { 0x2b, 6 }, { 0x1b, 5 }, { 0x07, 3 },
		{ 0x07, 4 }, { 0x02, 3 }, { 0x00, 0 }, { 0x00, 1 }, { 0x02, 2 },
		{ 0x01, 1 }
	},
/* PAUSE_IR */
	{
		{ 0x1f, 5 }, { 0x03, 3 }, { 0x07, 3 }, { 0x07, 4 }, { 0x07, 5 },
		{ 0x17, 5 }, { 0x17, 6 }, { 0x57, 7 }, { 0x37, 6 }, { 0x0f, 4 },
		{ 0x0f, 5 }, { 0x01, 2 }, { 0x05, 3 }, { 0x00, 1 }, { 0x01, 1 },
		{ 0x03, 2 }
	},
/* EXIT2_IR */
	{
		{ 0x0f, 4 }, { 0x01, 2 }, { 0x03, 2 }, { 0x03, 3 }, { 0x03, 4 },
		{ 0x0b, 4 }, { 0x0b, 5 }, { 0x2b, 6 }, { 0x1b, 5 }, { 0x07, 3 },
		{ 0x07, 4 }, { 0x00, 1 }, { 0x02, 2 }, { 0x02, 3 }, { 0x00, 0 },
		{ 0x01, 1 }
	},
/* UPDATE_IR */
	{
		{ 0x07, 3 }, { 0x00, 1 }, { 0x01, 1 }, { 0x01, 2 }, { 0x01, 3 },
		{ 0x05, 3 }, { 0x05, 4 }, { 0x15, 5 }, { 0x0d, 4 }, { 0x03, 2 },
		{ 0x03, 3 }, { 0x03, 4 }, { 0x0b, 4 }, { 0x0b, 5 }, { 0x2b, 6 },
		{ 0x00, 0 }
	},
};

#ifdef __cplusplus
}
#endif


#endif /* _JTAG_MASTER_H_ */
