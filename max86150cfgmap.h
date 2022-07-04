#ifndef	MAX86150CFGMAP_H
#define	MAX86150CFGMAP_H

#include "max86150bitmap.h"

#define ECG_DATA_BIT_MASK	0x3FFFF
#define PPG_DATA_BIT_MASK	0x7FFFF
#define BYTES_PER_ELEMENT	3

enum ppg_leds {
    IR_LED_PPG,
    RED_LED_PPG
};
#define PILOT_LED_PPG		IR_LED_PPG

enum ppg_leds_in_num {
    LED1_PPG,
    LED2_PPG
};

// FIFO_A_FULL
#define FIFO_A_FULL_IF_SAMPLES(n)	((FIFO_LENGTH - (n)) & FIFO_A_FULL_MASK)	// default 17
#define DEFAULT_FULL_SAMPLES		17

// FDn
#define FDn_DATA(n, data_bit_cfg)	(((data_bit_cfg) & 0x0F) << (4 * (~(n) & 1)))	// n=1,2,3,4  data_bit_cfg - fifo_data_type
enum fifo_data_slot {
	FD1 = 1,
	FD2,
	FD3,
	FD4
};
enum fifo_data_type {
	UNUSED,
	PPG_IR_DATA,
	PPG_RED_DATA,
	PILOT_IR_DATA = 5,
	PILOT_RED_DATA,
	ECG_DATA = 9
};

// PPG_ADC_RGE
enum ppg_adc_range_nA {
    PPG_ADC_RGE_nA_4096 = 0b00 << 6,	// LSB 7.8125 pA, default
    PPG_ADC_RGE_nA_8192 = 0b01 << 6,	// LSB 15.625 pA
    PPG_ADC_RGE_nA_16384 = 0b10 << 6,	// LSB 32.25 pA
    PPG_ADC_RGE_nA_32768 = 0b11 << 6	// LSB 62.5 pA
};

// PPG_SR
enum ppg_sampling_rate {
    PPG_SR_N1_10 = 0b0000 << 2,
    PPG_SR_N1_20 = 0b0001 << 2,
    PPG_SR_N1_50 = 0b0010 << 2,
    PPG_SR_N1_84 = 0b0011 << 2,
    PPG_SR_N1_100 = 0b0100 << 2,
    PPG_SR_N1_200 = 0b0101 << 2,
    PPG_SR_N1_400 = 0b0110 << 2,
    PPG_SR_N1_800 = 0b0111 << 2,
    PPG_SR_N1_1000 = 0b1000 << 2,
    PPG_SR_N1_1600 = 0b1001 << 2,
    PPG_SR_N1_3200 = 0b1010 << 2,
    PPG_SR_N2_10 = 0b1011 << 2,
    PPG_SR_N2_20 = 0b1100 << 2,
    PPG_SR_N2_50 = 0b1101 << 2,
    PPG_SR_N2_84 = 0b1110 << 2,
    PPG_SR_N2_100 = 0b1111 << 2
};

/*
>>> Maximum sample rates supported for all pulse widths and number of LEDs <<<
NUM OF | (see enum above) |		  (integration time)
ACTIVE | NUM OF ADC CONV. |		   LED PULSE WIDTH
 LEDs  |    PER SAMPLE 	  |	50us	100us	200us	400us
----------------------------------------------------------
 1 LED		   N=1			3200	1600	1000	1000
 2 LEDs		   N=1			1600	800		800		400
 1 LED		   N=2			100		100		100		100
 2 LEDs		   N=2			100		100		100		84
*/

// PPG_LED_PW
enum ppg_led_pulse_width {
	PPG_LED_PW_50us,
	PPG_LED_PW_100us,
	PPG_LED_PW_200us,
	PPG_LED_PW_400us
};

// SMP_AVE
enum ecppg_sample_averaging {
	SMP_AVE_1,
	SMP_AVE_2,
	SMP_AVE_4,
	SMP_AVE_8,
	SMP_AVE_16,
	SMP_AVE_32
};

// LEDn_PA
enum ppg_led_pulse_amplitude {
	LEDn_PA_0mA_0mA = 0,
	LEDn_PA_0mA2_0mA4 = 1,	// LSB
	LEDn_PA_1mA_2mA = 5,
	LEDn_PA_5mA_10mA = 25,
	LEDn_PA_10mA_20mA = 50,
	LEDn_PA_15mA_30mA = 75,
	LEDn_PA_20mA_40mA = 100,
	LEDn_PA_25mA_50mA = 125,
	LEDn_PA_30mA_60mA = 150,
	LEDn_PA_35mA_70mA = 200,
	LEDn_PA_40mA_80mA = 225,
	LEDn_PA_45mA_90mA = 250,
	LEDn_PA_51mA_102mA = 255
};

// LEDn_RGE
enum ppg_leds_range {
    IR_LED_RGE_50mA = 0,
    IR_LED_RGE_100mA = 1,
    RED_LED_RGE_50mA = 0 << 2,
    RED_LED_RGE_100mA = 1 << 2
};

// {ECG_ADC_CLK, ECG_ADC_OSR}
enum ecg_sampling_rate {
	ECG_SR_1600,
	ECG_SR_800,
	ECG_SR_400,
	ECG_SR_200,
	ECG_SR_ck_3200,
	ECG_SR_ck_1600,
	ECG_SR_ck_800,
	ECG_SR_ck_400
};

// PGA_ECG_GAIN
enum ecg_pga_gain {
	PGA_ECG_GAIN_1 = 0b00 << 2,		// default
	PGA_ECG_GAIN_2 = 0b01 << 2,
	PGA_ECG_GAIN_4 = 0b10 << 2,
	PGA_ECG_GAIN_8 = 0b11 << 2		// most calibrated
};

// IA_GAIN
enum ecg_ia_gain {
	IA_ECG_GAIN_5,
	IA_ECG_GAIN_9p5,	// most calibrated
	IA_ECG_GAIN_20,		// default
	IA_ECG_GAIN_50
};

// enable bits and flag bits
enum max86150_interrupt_bit_positions {  // enable bits and flag bits
    PWR_RDY,
    PROX_INT = 4,
    ALC_OVF,
    PPG_RDY,
    A_FULL,

    ECG_RDY = 8 + 2,
    VDD_OOR = 8 + 7
};

#endif
