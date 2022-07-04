#ifndef MAX86150BITMAP_H
#define MAX86150BITMAP_H

// INTERRUPT_STATUS_1
#define A_FULL_BIT				0x80	// FIFO Almost Full Flag
#define PPG_RDY_BIT				0x40	// New PPG FIFO Data Ready
#define ALC_OVF_BIT				0x20	// Ambient Light Cancellation Overflow
#define PROX_INT_BIT			0x10	// Proximity interrupt
#define PWR_RDY_BIT				0x01	// Power Ready Flag
// INTERRUPT_STATUS_2
#define VDD_OOR_BIT				0x80	// VDD Out-of-Range Flag
#define ECG_RDY_BIT				0x04	// New ECG FIFO Data Ready

// INTERRUPT_ENABLE_1
#define A_FULL_EN_BIT			0x80	// FIFO Almost Full Flag Enable
#define PPG_RDY_EN_BIT			0x40	// New PPG FIFO Data Ready Interrupt Enable
#define ALC_OVF_EN_BIT			0x20	// Ambient Light Cancellation (ALC) Overflow Interrupt Enable
#define PROX_INT_EN_BIT			0x10	// Proximity interrupt Enable
// INTERRUPT_ENABLE_2
#define VDD_OOR_EN_BIT			0x80	// VDD Out-of-Range Indicator Enable
#define ECG_RDY_EN_BIT			0x04	// New ECG FIFO Data Redy Enable

// FIFO_WRITE_POINTER
#define FIFO_WR_PTR_MASK		0x1F	// FIFO Write Pointer
// OVERFLOW_COUNTER
#define OVF_COUNTER_MASK		0x1F	// FIFO Overflow Counter
// FIFO_READ_POINTER
#define FIFO_RD_PTR_MASK		0x1F	// FIFO Read Pointer

// FIFO_DATA_REGISTER
//#define FIFO_DATA_MASK		0xFF	// FIFO Data Register

// FIFO_CONFIGURATION
#define A_FULL_CLR_BIT			0x40	// FIFO Almost Full Interrupt Options
#define A_FULL_TYPE_BIT			0x20	// FIFO Almost Full Flag Options
#define FIFO_ROLLS_ON_FULL_BIT	0x10	// FIFO Rolls on Full Options
#define FIFO_A_FULL_MASK		0x0F	// FIFO Almost Full Value

// FIFO_DATA_CONTROL_REGISTER_1
#define FD1_MASK				0x0F	// FIFO Data Time Slot 1
#define FD2_MASK				0xF0	// FIFO Data Time Slot 2
// FIFO_DATA_CONTROL_REGISTER_2
#define FD3_MASK				0x0F	// FIFO Data Time Slot 3
#define FD4_MASK				0xF0	// FIFO Data Time Slot 4

// SYSTEM_CONTROL
#define FIFO_EN_BIT				0x04	// FIFO Enable
#define SHDN_BIT				0x02	// Shutdown Control
#define RESET_BIT				0x01	// Reset Control

// PPG_CONFIGURATION_1
#define PPG_ADC_RGE_MASK		0xC0	// SpO2 ADC Range Control
#define PPG_SR_MASK				0x3C	// SpO2 Sample Rate Control
#define PPG_LED_PW_MASK			0x03	// LED Pulse Width Control

// PPG_CONFIGURATION_2
#define SMP_AVE_MASK			0x07	// Sample Averaging Options

// PROX_INTERRUPT_TRESHOLD
//#define PROX_INT_THRESH_MASK	0xFF	// Proximity Mode Interrupt Threshold

// LED1_PA
//#define LED1_PA_MASK			0xFF	// IR Current Pulse Amplitude
// LED2_PA
//#define LED2_PA_MASK			0xFF	// RED Current Pulse Amplitude

// LED_RANGE
#define LED1_RGE_MASK			0x03	// IR Current Range Control
#define LED2_RGE_MASK			0x0C	// RED Current Range Control

// LED_POLOT_PA
//#define PILOT_PA_MASK			0xFF	// Proximity Mode LED Pulse Amplitude

// ECG_IMP_CONFIGURATION_1
#define ECG_ADC_CLK_BIT			0x04	// ECG ADC Oversampling Ratio  [2] {2:0}
#define ECG_ADC_OSR_MASK		0x03	// ECG ADC Oversampling Ratio [1:0]{2:0}

// ECG_IMP_CONFIGURATION_3
#define PGA_ECG_GAIN_MASK		0x0C	// ECG PGA Gain Options
#define IA_GAIN_MASK			0x03	// Instrumental Amplifier Gain Options

// PART_ID
//#define PART_ID_MASK			0xFF	// Part Identifier

#endif
