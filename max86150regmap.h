#ifndef	MAX86150REGMAP_H
#define	MAX86150REGMAP_H

#include "max86150cfgmap.h"

#define NUM_REGS			22
#define NUM_READ_ONLY_REGS	4
#define NUM_REGS_WR			(NUM_REGS - NUM_READ_ONLY_REGS)
#define FIFO_LENGTH			32
#define NUM_DATA_SLOTS		4

enum max86150_regs {
	INTERRUPT_STATUS_1,				// 0x00 RO
	INTERRUPT_STATUS_2,				// 0x01 RO
	INTERRUPT_ENABLE_1,				// 0x02 RW
	INTERRUPT_ENABLE_2,				// 0x03 RW

	FIFO_WRITE_POINTER,				// 0x04 RW
	OVERFLOW_COUNTER,				// 0x05 RW
	FIFO_READ_POINTER,				// 0x06 RW
	FIFO_DATA_REGISTER,				// 0x07 RO
	FIFO_CONFIGURATION,				// 0x08 RW

	FIFO_DATA_CONTROL_REGISTER_1,	// 0x09 RW
	FIFO_DATA_CONTROL_REGISTER_2,	// 0x0A RW

	SYSTEM_CONTROL = 0x0D,			// 0x0D RW

	PPG_CONFIGURATION_1,			// 0x0E RW
	PPG_CONFIGURATION_2,			// 0x0F RW
	PROX_INTERRUPT_TRESHOLD,		// 0x10 RW

	LED1_PA,						// 0x11 RW
	LED2_PA,						// 0x12 RW
	LED_RANGE = 0x14,				// 0x14 RW
	LED_PILOT_PA,					// 0x15 RW

	ECG_CONFIGURATION_1 = 0x3C,		// 0x3C RW
	ECG_CONFIGURATION_3 = 0x3E,		// 0x3E RW

	PART_ID = 0xFF					// 0xFF RO
};

static const uint8_t read_reg_map_jumps[][2] = {
	{INTERRUPT_STATUS_1,	FIFO_DATA_REGISTER},
	{FIFO_CONFIGURATION,	FIFO_DATA_CONTROL_REGISTER_2},
	{SYSTEM_CONTROL,		LED2_PA},
	{LED_RANGE,				LED_PILOT_PA},
	{ECG_CONFIGURATION_1,	ECG_CONFIGURATION_1},
	{ECG_CONFIGURATION_3,	ECG_CONFIGURATION_3},
	{PART_ID,				PART_ID}
};

static const uint8_t write_reg_map_jumps[][2] = {
	{INTERRUPT_ENABLE_1,	FIFO_READ_POINTER},
	{FIFO_CONFIGURATION,	FIFO_DATA_CONTROL_REGISTER_2},
	{SYSTEM_CONTROL,		LED2_PA},
	{LED_RANGE,				LED_PILOT_PA},
	{ECG_CONFIGURATION_1,	ECG_CONFIGURATION_1},
	{ECG_CONFIGURATION_3,	ECG_CONFIGURATION_3}
};

#endif
