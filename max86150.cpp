#include "max86150.h"

/**
 * @brief max86150 constructor, running without interrupts
 * @param i2c address of I2C object to be used for communication with max86150
*/
max86150::max86150(PinName sda, PinName scl) : max86150::max86150(sda, scl, NC) { }

/**
 * @brief max86150 constructor, running with interrupts
 * @param i2c address of I2C object to be used for communication with max86150
 * @param inpterrupt_pin name of pin to be used for interrupt
*/
max86150::max86150(PinName sda, PinName scl, PinName interrupt_pin) : max86150::max86150(sda, scl, NC, NULL) { }

/**
 * @brief max86150 constructor, running with interrupts
 * @param i2c address of I2C object to be used for communication with max86150
 * @param inpterrupt_pin name of pin to be used for interrupt
 * @param interrupt_handler pointer to function to be called at the end of internal interrupt routine, needs to be interrupt save!
*/
max86150::max86150(PinName sda, PinName scl, PinName interrupt_pin, void(*_interrupt_handler)(uint32_t status)) : intIn(interrupt_pin, PullUp) {
	if (sda == NC || scl == NC) {
		return;
	}
	i2c_init(&i2c, sda, scl);
	//i2c->frequency(400000);
	i2c_frequency(&i2c, 400000);
	queue_init(&q, q_buf, sizeof(ecppg_data_t), sizeof(q_buf));
	wait_us(1000);  // ThisThread::sleep_for(1ms);
	reset();
	if (interrupt_pin != NC) {
		if (_interrupt_handler) {
			interrupt_handler = _interrupt_handler;
		}
		internal_interrupt_handler();
		intIn.fall(callback(this, &max86150::internal_interrupt_handler));
	}
	no_interrupts = (interrupt_pin == NC);
}

//------------------------------------------------------------------------
// STRICT FDn ORDER:  HAVE TO START FROM FD1  (it is not completely necessary, but ppg IR and RED data need to be in slots 1 and 2)
/*
 .			  S					v			  v
	|	IR + RED + ECG	|	RED + IR	|	IR		|
	|	RED + IR + ECG	|	IR + ECG	|	RED		|
	|	IR + RED		|	RED + ECG	|	ECG		|
 .			  v					v			  E
*/

/**
 * @brief initialize simple ecg mode, (using data slot 3)
 * @param sampling_rate enum value (bits) for ecg sampling rate configuration
 * @param ia_gain enum value (bits) for ecg ia gain configuration
 * @param pga_gain enum value (bits) for ecg pga gain configuration
 * @return 0 - success, (-1) - sampling rate not supported (different sampling rate selected), other nonzero - failure
*/
int max86150::ecg_mode(ecg_sampling_rate sampling_rate, ecg_ia_gain ia_gain, ecg_pga_gain pga_gain) {
	fifo_enable(false);
	read_fifo_to_queue();  // to not lost anything or // clear_queue(); // to lost everything
	
	int ret = set_ecg_sampling_rate(sampling_rate);
	int ret_sr = ret;
	if (ret == -1)  {
		ret = 0;
	}
	if (ret) {
		return ret;
	}
	ret = set_ecg_gain(ia_gain, pga_gain);
	if (ret) {
		return ret;
	}
  set_data_slot(FD3, ECG_DATA);	// find free or or set always same one ?
	ret = enable_interrupt(A_FULL);
	if (ret) {
		return ret;
	}
	ret = fifo_enable();
	if (ret) {
		return ret;
	}
	return ret_sr;
}

/**
 * @brief initialize simple ppg mode, (using data slots 1 and 2, LEDs currents are set same - can by adjusted separately later, not using proximity mode (can be activated separately later))
 * @param sampling_rate enum value (bits) for ppg sampling rate configuration
 * @param range enum value (bits) for ppg (ADC) range configuration
 * @param ranges enum value (bits) for LEDs current range configuration
 * @param currents value representing pulse amplitude configuration for both LEDs (IR (in normal mode) and RED),
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range
 * @param pulse_width enum value (bits) for pulse width configuration
 * @return 0 - success, (-1) - sampling rate not supported (different sampling rate selected), other nonzero - failure
*/
int max86150::ppg_mode(ppg_sampling_rate sampling_rate, ppg_adc_range_nA range, ppg_leds_range ranges, ppg_led_pulse_amplitude currents, ppg_led_pulse_width pulse_width) {
	fifo_enable(false);
	read_fifo_to_queue();  // to not lost anything or // clear_queue(); // to lost everything

	int ret = write_reg(PPG_CONFIGURATION_1, range | sampling_rate | pulse_width);
	if (ret) {
		return ret;
	}
	int ret_sr = ((read_reg(PPG_CONFIGURATION_1) & PPG_SR_MASK) == sampling_rate) ? 0 : -1;

	uint8_t led_ranges = ranges;
	if (led_ranges > 3) {
		led_ranges >>= 2;
	}
	uint8_t data[3] = {currents, currents, set_reg_bits(read_reg(LED_RANGE), LED1_RGE_MASK | LED2_RGE_MASK, (led_ranges << 2) | led_ranges)};
	ret = write_regs(LED1_PA, 3, data);
	if (ret) {
		return ret;
	}
  set_data_slot(FD1, PPG_IR_DATA);	// find free or or set always same one ?
  set_data_slot(FD2, PPG_RED_DATA);	// find free or or set always same one ?
	ret = enable_interrupt(A_FULL);
	if (ret) {
		return ret;
	}
	ret = fifo_enable();
	if (ret) {
		return ret;
	}
	return ret_sr;
}

/**
 * @brief initialize simple combined ecg and ppg mode, (using data slots 1, 2 and 3, LEDs currents are set same - can by adjusted separately later, not using proximity mode (can be activated separately later))
 * @param sampling_rate enum value (bits) for ecg sampling rate configuration (sampling rate is also applied for PPG)
 * @param ia_gain enum value (bits) for ecg ia gain configuration
 * @param pga_gain enum value (bits) for ecg pga gain configuration
 * @param range enum value (bits) for ppg (ADC) range configuration
 * @param ranges enum value (bits) for LEDs current range configuration
 * @param currents value representing pulse amplitude configuration for both LEDs (IR (in normal mode) and RED),
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range
 * @param pulse_width enum value (bits) for pulse width configuration
 * @return 0 - success, (-1) - sampling rate not supported (different sampling rate selected), other nonzero - failure
*/
int max86150::ecppg_mode(ecg_sampling_rate sampling_rate, ecg_ia_gain ia_gain, ecg_pga_gain pga_gain, ppg_adc_range_nA range, ppg_leds_range ranges, ppg_led_pulse_amplitude currents, ppg_led_pulse_width pulse_width) {
	const uint16_t ppg_srs[] = {10, 20, 50, 84, 100, 200, 400, 800, 1000, 1600, 3200, 10, 20, 50, 84, 100};
	const uint16_t ecg_srs[] = {1600, 800, 400, 200, 3200, 1600, 800, 400};
	
	sampling_rate = (ecg_sampling_rate) (sampling_rate & (ECG_ADC_CLK_BIT | ECG_ADC_OSR_MASK));
	uint8_t ppg_sr = PPG_SR_N1_200;  // default
	// determine PPG sampling rate
	for (uint8_t i = 0; i < (sizeof(ppg_srs) / sizeof(ppg_srs[0])); i++) {
		if (ppg_srs[i] == ecg_srs[sampling_rate]) {
			ppg_sr = i << 2;
			break;
		}
	}

	int ret = ppg_mode((ppg_sampling_rate) ppg_sr, range, ranges, currents, pulse_width);
	fifo_enable(false);
	bool different_sr = false;
	if (ret == -1) {
		different_sr = true;
		ppg_sr = read_reg(PPG_CONFIGURATION_1) & PPG_SR_MASK;
		ret = 0;
	}
	if (ret) {
		return ret;
	}

	uint8_t confirmed_sr = sampling_rate;
	if (different_sr) {
		// determine new ECG sampling rate
		for (uint8_t i = 0; i < (sizeof(ecg_srs) / sizeof(ecg_srs[0])); i++) {
			if (ecg_srs[i] == ppg_srs[ppg_sr >> 2]) {
				confirmed_sr = i;
				break;
			}
			if (i == (sizeof(ecg_srs) / sizeof(ecg_srs[0]))) {
				confirmed_sr = ECG_SR_200;
			}
		}
	}
	ret = ecg_mode((ecg_sampling_rate) confirmed_sr, ia_gain, pga_gain);
	return (different_sr) ? -1 : ret;
}

/**
 * @brief disable ecg mode by deactivating it's FIFO data slots
 * @return 0 - success, nonzero - failure
*/
int max86150::disable_ecg() {
	uint8_t ecg_off[2] = {0, 0};
	const uint8_t ecg_off_types[] = {ECG_DATA};
	int ret = read_regs(FIFO_DATA_CONTROL_REGISTER_1, 2, ecg_off);
	if (ret) {
		return ret;
	}
	for (uint8_t i = 0; i < (sizeof(ecg_off_types) / sizeof(ecg_off_types[0])); i++) {
		clear_slots_in_regs_with_type(ecg_off, ecg_off_types[i]);
	}
	//read_fifo_to_queue();  // because FIFO will FLUSH
	return write_regs(FIFO_DATA_CONTROL_REGISTER_1, 2, ecg_off);
}

/**
 * @brief disable ppg mode by deactivating it's FIFO data slots
 * @return 0 - success, nonzero - failure
*/
int max86150::disable_ppg() {
	uint8_t ppg_off[2] = {0, 0};
	const uint8_t ppg_off_types[] = {PPG_IR_DATA, PPG_RED_DATA, PILOT_IR_DATA, PILOT_RED_DATA};
	int ret = read_regs(FIFO_DATA_CONTROL_REGISTER_1, 2, ppg_off);
	if (ret) {
		return ret;
	}
	for (uint8_t i = 0; i < (sizeof(ppg_off_types) / sizeof(ppg_off_types[0])); i++) {
		clear_slots_in_regs_with_type(ppg_off, ppg_off_types[i]);
	}
	//read_fifo_to_queue();  // because FIFO will FLUSH
	return write_regs(FIFO_DATA_CONTROL_REGISTER_1, 2, ppg_off);
}

/**
 * @brief disable ecg and ppg modes by deactivating all FIFO data slots
 * @return 0 - success, nonzero - failure
*/
int max86150::disable_ecppg() {
	uint8_t ecppg_off[2] = {0, 0};
	//read_fifo_to_queue();  // because FIFO will FLUSH
	return write_regs(FIFO_DATA_CONTROL_REGISTER_1, 2, ecppg_off);
}

/**
 * @brief convert ecg raw value to mV
 * @param raw_ecg_value signed value
 * @return ecg measured voltage value in mV
*/
float max86150::get_ecg_mV(int32_t raw_ecg_value) {  // (E-6)[uV_orig] + E+3[mV] + E-1[10timesIaGain] = E-4
	static uint16_t ia_gains_10[] = {50, 95, 200, 500};
	return raw_ecg_value * 12.247E-4 / ia_gains_10[ecg_gain.ia_gain] / (1 << (ecg_gain.pga_gain >> 2));
}

/**
 * @brief convert ppg raw value to nA
 * @param raw_ppg_value unsigned value
 * @return ppg measured current value in nA
*/
float max86150::get_ppg_nA(uint32_t raw_ppg_value) {
	return (uint64_t) (raw_ppg_value * (4096 * (1 << (ppg_range >> 6)))) / (float) PPG_DATA_BIT_MASK;
}

/**
 * @brief enable interrupt by setting its enable bit
 * @param interrupt enum value or position of enable bit of interrupt (add 8 to bits from INTERRUPT_ENABLE_REGISTER_2)
 * @return 0 - success, nonzero - failure ((-1) - wrong interrupt bit position or interrupt are not allowed)
*/
int max86150::enable_interrupt(max86150_interrupt_bit_positions interrupt) {
	if (!no_interrupts) {
		if (interrupt > 0 && interrupt < 8) {
			return write_reg(INTERRUPT_ENABLE_1, read_reg(INTERRUPT_ENABLE_1) | (1 << interrupt));
		} else if (interrupt > 7 && interrupt < 16) {
			return write_reg(INTERRUPT_ENABLE_2, read_reg(INTERRUPT_ENABLE_2) | (1 << (interrupt - 8)));
		}
	}
	return -1;
}

/**
 * @brief disable interrupt by clearing its enable bit
 * @param interrupt position of enable bit of interrupt (add 8 to bits from INTERRUPT_ENABLE_REGISTER_2)
 * @return 0 - success, nonzero - failure ((-1) - wrong interrupt bit position or interrupt are not allowed)
*/
int max86150::disable_interrupt(max86150_interrupt_bit_positions interrupt) {
	if (!no_interrupts) {
		if (interrupt > 0 && interrupt < 8) {
			return write_reg(INTERRUPT_ENABLE_1, read_reg(INTERRUPT_ENABLE_1) & ~(1 << interrupt));
		} else if (interrupt > 7 && interrupt < 16) {
			return write_reg(INTERRUPT_ENABLE_2, read_reg(INTERRUPT_ENABLE_2) & ~(1 << (interrupt - 8)));
		}
	}
	return -1;
}

/**
 * @brief enter shutdown (low power sleep mode), all registers are accessible during shutdown
 * @return 0 - success, nonzero - failure
*/
int max86150::shutdown() {
	return write_reg(SYSTEM_CONTROL, read_reg(SYSTEM_CONTROL) | SHDN_BIT);
}

/**
 * @brief wake up device (clear shutdown bit), everything is like before shutdown
 * @return 0 - success, nonzero - failure
*/
int max86150::wakeup() {
	return write_reg(SYSTEM_CONTROL, read_reg(SYSTEM_CONTROL) & ~SHDN_BIT);
}

/**
 * @brief execute software reset of the device
 * @return 0 - success, nonzero - failure ((-1) - device reset complete but queue failure)
*/
int max86150::reset() {
	int ret = write_reg(SYSTEM_CONTROL, read_reg(SYSTEM_CONTROL) | RESET_BIT);
	if (!ret && !(ret = clear_queue())) {
		fifo_full_num_samples = DEFAULT_FULL_SAMPLES;
		used_data_slots = 0;
		for (uint8_t i = 0; i < NUM_DATA_SLOTS; i++) {
			data_slots_type[i] = UNUSED;
		}
		ecg_gain = {
			.ia_gain = IA_ECG_GAIN_20,
			.pga_gain = PGA_ECG_GAIN_1
		};
		ppg_range = PPG_ADC_RGE_nA_4096;
	}
	return ret;
}

/**
 * @brief set ecg ia (instrumentation amplifier) and pga (programmable gain amplifier) gain
 * @param ia_gain enum value (bits) for ecg ia gain configuration
 * @param pga_gain enum value (bits) for ecg pga gain configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::set_ecg_gain(ecg_ia_gain ia_gain, ecg_pga_gain pga_gain) {
	return write_reg(
		ECG_CONFIGURATION_3,
		set_reg_bits(
			read_reg(ECG_CONFIGURATION_3),
			PGA_ECG_GAIN_MASK | IA_GAIN_MASK,
			(ia_gain & IA_GAIN_MASK) | (pga_gain & PGA_ECG_GAIN_MASK)
		)
	);
}

/**
 * @brief set current range and pulse amplitude of ppg LED
 * @param led enum value representing ppg LED (IR / RED)
 * @param range enum value (bits) for current range configuration
 * @param current value representing pulse amplitude configuration,
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range
 * @return 0 - success, nonzero - failure
*/
int max86150::set_led_current(ppg_leds led, ppg_leds_range range, ppg_led_pulse_amplitude current) {
	if (int ret = set_led_current_range(led, range)) {
		return ret;
	}
	return set_led_pulse_amplitude_current(led, current);
}

/**
 * @brief compute number of samples in FIFO from read and write FIFO pointers positions
 * @return number of samples in FIFO
*/
uint8_t max86150::get_num_samples_in_fifo() {
	uint8_t data[3] = {0};
	if (read_regs(FIFO_WRITE_POINTER, 3, data)) {
		return 0;
	}
	uint8_t write_pointer = data[0] & FIFO_WR_PTR_MASK;
	uint8_t overflow_counter = data[1] & OVF_COUNTER_MASK;
	uint8_t read_pointer = data[2] & FIFO_RD_PTR_MASK;
	if (overflow_counter || ((fifo_full_num_samples == 32) && (write_pointer == read_pointer))) {
		return FIFO_LENGTH;
	}
	// if (overflow_counter == 0 && read_pointer == write_pointer) num_samples could be 32 or 0 ... consider 0
	//  (only if FIFO_A_FULL == 32 than consider 32, but if the truth is 0, undefined samples are read)
	return (read_pointer > write_pointer ? FIFO_LENGTH : 0) + write_pointer - read_pointer;
}

/**
 * @brief read FIFO write pointer position
 * @return FIFO write pointer value
*/
uint8_t max86150::read_fifo_write_pointer() {
	return read_reg(FIFO_WRITE_POINTER) & FIFO_WR_PTR_MASK;
}

/**
 * @brief read FIFO read pointer position
 * @return FIFO read pointer value
*/
uint8_t max86150::read_fifo_read_pointer() {
	return read_reg(FIFO_READ_POINTER) & FIFO_RD_PTR_MASK;
}

/**
 * @brief read FIFO overflow conter value (saturated at 0x1F (31))
 * @return number of samples lost
*/
uint8_t max86150::read_ovf_counter() {
	return read_reg(OVERFLOW_COUNTER) & OVF_COUNTER_MASK;
}

/**
 * @brief enqueue all samples currently in FIFO (to prevent lost of data when FIFO FLUSH)
 *  ; note: FIFO is flushed when enabling FIFO or writing fifo data control and ppg / ecg configuration register
 * @return number of lost samples (due to full queue buffer): 0 - success, <1; FIFO_LENGTH> - failure, (-1) - communication failure
*/
int max86150::read_fifo_to_queue_now() {
	return read_fifo_to_queue();
}

// TODO - maybe implement software "proximity" mode (hardware one causes FIFO FLUSH)
/**
 * @brief enable / disable proximity mode,
 *  Warning: (when proximity mode enabled) transition between proximity and normal ppg mode will FLUSH FIFO
 * @param state true - enable proximity mode, false - disable proximity mode
 * @return 0 - success, nonzero - failure
*/
int max86150::proximity_mode(bool state) {
	return state ? enable_interrupt(PROX_INT) : disable_interrupt(PROX_INT);
}

/**
 * @brief enable / disable proximity mode and it's LED pulse width and current
 *  Warning: (when proximity mode enabled) transition between proximity and normal ppg mode will FLUSH FIFO
 * @param state true - enable proximity mode, false - disable proximity mode
 * @param pilot_led_current value representing pulse amplitude configuration,
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range,
 *  (IR) LED's range is used also in proximity mode
 * @param prox_thresh 88 bit value representing 8 MSB of minimal ppg ADC data value that will trigger proximity interrupt flag
 * @return 0 - success, nonzero - failure
*/
int max86150::proximity_mode(bool state, ppg_led_pulse_amplitude pilot_led_current, uint8_t prox_thresh) {
	int ret = write_reg(PROX_INTERRUPT_TRESHOLD, prox_thresh);
	if (!ret) {
		ret = write_reg(LED_PILOT_PA, pilot_led_current);
		if (!ret) {
			ret = proximity_mode(state);
		}
	}
	return ret;
}

/**
 * @brief set proximity interrupt threshold value
 * @param threshold 8 bit value representing 8 MSB of minimal ppg ADC data value that will trigger proximity interrupt flag
 * @return 0 - success, nonzero - failure
*/
int max86150::set_prox_int_threshold(uint8_t threshold) {
	return write_reg(PROX_INTERRUPT_TRESHOLD, threshold);
}

/**
 * @brief set pulse amplitude current of ppg LED used in proximity mode, used (IR) LED's range (and pulse width) is used also in proximity mode
 * @param current value representing pulse amplitude configuration,
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range
 * @return 0 - success, nonzero - failure
*/
int max86150::set_pilot_led_pulse_amplitude_current(ppg_led_pulse_amplitude current) {
	return write_reg(LED_PILOT_PA, current);
}

/**
 * @brief enable FIFO functionality
 * @param state true - enable, false - disable
 * @return 0 - success, nonzero - failure
*/
int max86150::fifo_enable(bool state) {
	return write_reg(SYSTEM_CONTROL, read_reg(SYSTEM_CONTROL) | (state ? FIFO_EN_BIT : 0));
}

/**
 * @brief enable PPG / ECG functionality by setting source of content of specified FIFO element
 * @param slot number (enum value) of FIFO element
 * @param type enum value of FIFO element type (ppg IR, ppg RED, ecg, pilot IR, pilot RED)
 * @return 0 - success, nonzero - failure ((-1) - wrong data slot or data type)
*/
int max86150::set_data_slot(fifo_data_slot slot, fifo_data_type type) {
	uint8_t data_control_register = 0, mask = 0, data_type = 0;
	switch(type) {
		case UNUSED:
		case PPG_IR_DATA:
		case PPG_RED_DATA:
		case PILOT_IR_DATA:
		case PILOT_RED_DATA:
		case ECG_DATA:
			data_type = FDn_DATA(slot, type);
			break;
		default:
			return -1;
	}
	switch(slot) {
		case FD1:
			data_control_register = FIFO_DATA_CONTROL_REGISTER_1;
			mask = FD1_MASK;
			break;
		case FD2:
			data_control_register = FIFO_DATA_CONTROL_REGISTER_1;
			mask = FD2_MASK;
			break;
		case FD3:
			data_control_register = FIFO_DATA_CONTROL_REGISTER_2;
			mask = FD3_MASK;
			break;
		case FD4:
			data_control_register = FIFO_DATA_CONTROL_REGISTER_2;
			mask = FD4_MASK;
			break;
		default:
			return -1;
	}
	return write_reg(data_control_register, set_reg_bits(read_reg(data_control_register), mask, data_type));
}

/**
 * @brief specify if FIFO new samples: (true) rewrite old samples when FIFO is full or (false) are lost
 * @param state true - old samples are lost, false - new samples are lost
 * @return 0 - success, nonzero - failure
*/
int max86150::fifo_rolls_on_fun(bool state) {
	return write_reg(FIFO_CONFIGURATION, read_reg(FIFO_CONFIGURATION) | (state ? FIFO_ROLLS_ON_FULL_BIT : 0));
}

/**
 * @brief specify number of samples stored in FIFO when FIFO almost full interrupt flag is set, (range <17; 32>),
 *  Warning: 32 is not recommended, if there is no oveflow (lost samples), it can not be distinguished from 0 (FIFO empty)
 * @param samples_stored number of samples stored when FIFO_A_FULL interrupt flag is set
 * @return 0 - success, nonzero - failure
*/
int max86150::fifo_almost_full_interrupt_at_samples(uint8_t samples_stored) {
	return write_reg(FIFO_CONFIGURATION, set_reg_bits(read_reg(FIFO_CONFIGURATION), FIFO_A_FULL_MASK, FIFO_A_FULL_IF_SAMPLES(samples_stored)));
}

/**
 * @brief specify if FIFO almost full interrupt flag is cleared when FIFO data register is read
 * @param state true - FIFO_A_FULL interrupt flag is cleared by reading FIFO_DATA_REG, false - FIFO_A_FULL interrupt flag is not cleared by reading FIFO_DATA_REG
 * @return 0 - success, nonzero - failure
*/
int max86150::fifo_almost_full_interrupt_clear_on_data_read(bool state) {
	return write_reg(FIFO_CONFIGURATION, read_reg(FIFO_CONFIGURATION) | (state ? A_FULL_CLR_BIT : 0));
}

/**
 * @brief specify if FIFO almost full interrupt flag is set by number of samples (just) equal or (also) greater than set number of samples (e.g.: if a_full @ 17 : true - 17,18,19,... ; false - 17)
 * @param state true - FIFO_A_FULL interrupt flag is set when number of samples is equal or greater than specified number, false - FIFO_A_FULL interrupt flag is set only when number of samples equals specified number
 * @return 0 - success, nonzero - failure
*/
int max86150::fifo_almost_full_interrupt_flag_repeat(bool state){
	return write_reg(FIFO_CONFIGURATION, read_reg(FIFO_CONFIGURATION) | (!state ? A_FULL_TYPE_BIT : 0));
}

/**
 * @brief set ecg ia (instrumentation amplifier) gain
 * @param gain enum value (bits) for ecg ia gain configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::set_ecg_ia_gain(ecg_ia_gain gain) {
	return write_reg(ECG_CONFIGURATION_3, set_reg_bits(read_reg(ECG_CONFIGURATION_3), IA_GAIN_MASK, gain));
}

/**
 * @brief set ecg pga (programmable gain amplifier) gain
 * @param gain enum value (bits) for ecg pga gain configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::set_ecg_pga_gain(ecg_pga_gain gain) {
	return write_reg(ECG_CONFIGURATION_3, set_reg_bits(read_reg(ECG_CONFIGURATION_3), PGA_ECG_GAIN_MASK, gain));
}

/**
 * @brief set ecg sampling rate
 * @warning LED pulse width cfg changes: ppg SR cfg, but can not change ecg SR in ecppg mode
 * @warning if ecg SR > ppg SR, ecg SR cfg changes ppg SR cfg, but if ppg SR > ecg SR, ppg SR cfg can not change ecg SR cfg
 * @param sampling_rate enum value (bits) for ecg sampling rate configuration
 * @return 0 - success, (-1) - sampling rate not supported (different sampling rate selected), other nonzero - failure
*/
int max86150::set_ecg_sampling_rate(ecg_sampling_rate sampling_rate) {
	if (int ret = write_reg(ECG_CONFIGURATION_1, set_reg_bits(read_reg(ECG_CONFIGURATION_1), ECG_ADC_CLK_BIT | ECG_ADC_OSR_MASK, sampling_rate))) {
		return ret;
	}
	uint8_t reg = read_reg(ECG_CONFIGURATION_1);
	reg &= ECG_ADC_CLK_BIT | ECG_ADC_OSR_MASK;
	return reg == sampling_rate ? 0 : -1;
}

/**
 * @brief tries to set ppg sampling rate (might not be supported <- ECG sampling rate, LEDs pulse width), see table below ppg_sampling_rate definition
 * @warning LED pulse width cfg changes: ppg SR cfg, but can not change ecg SR in ecppg mode
 * @warning if ecg SR > ppg SR, ecg SR cfg changes ppg SR cfg, but if ppg SR > ecg SR, ppg SR cfg can not change ecg SR cfg
 * @param sampling_rate enum value (bits) for ppg sampling rate configuration
 * @return 0 - success, (-1) - sampling rate not supported (different sampling rate selected), other nonzero - failure
*/
int max86150::set_ppg_sampling_rate(ppg_sampling_rate sampling_rate) {
	if (int ret = write_reg(PPG_CONFIGURATION_1, set_reg_bits(read_reg(PPG_CONFIGURATION_1), PPG_SR_MASK, sampling_rate))) {
		return ret;
	}
	uint8_t reg = read_reg(PPG_CONFIGURATION_1);
	reg &= PPG_SR_MASK;
	return reg == sampling_rate ? 0 : -1;
}

/**
 * @brief set range of ppg ADC (in nA)
 * @param range enum value (bits) for ppg (ADC) range configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::set_ppg_range_nA(ppg_adc_range_nA range) {
	return write_reg(PPG_CONFIGURATION_1, set_reg_bits(read_reg(PPG_CONFIGURATION_1), PPG_ADC_RGE_MASK, range));
}

/**
 * @brief set current range of ppg LED
 * @param led enum value representing ppg LED (IR / RED)
 * @param range enum value (bits) for current range configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::set_led_current_range(ppg_leds led, ppg_leds_range range) {
	if (led == IR_LED_PPG) {
		return write_reg(LED_RANGE, set_reg_bits(read_reg(LED_RANGE), LED1_RGE_MASK, range));
	} else if (led == RED_LED_PPG) {
		return write_reg(LED_RANGE, set_reg_bits(read_reg(LED_RANGE), LED2_RGE_MASK, range));
	}
	return -1;
}

/**
 * @brief set pulse amplitude of ppg LED
 * @param led enum value representing ppg LED (IR / RED)
 * @param current value representing pulse amplitude configuration,
 *  could be enum value (bits) for pulse amplitude configuration or
 *  8 bit value - resolution : LSB ~ 0.2mA @ 50mA range / LSB ~ 0.4mA @ 100mA range
 * @return 0 - success, nonzero - failure ((-1) - wrong LED)
*/
int max86150::set_led_pulse_amplitude_current(ppg_leds led, ppg_led_pulse_amplitude current) {
	if (led == IR_LED_PPG) {
		return write_reg(LED1_PA, current);
	} else if (led == RED_LED_PPG) {
		return write_reg(LED2_PA, current);
	}
	return -1;
}

/**
 * @brief set pulse width of (all) ppg LEDs, Warning: it could change (ppg / ecg / ecppg) sampling rate
 *  (if pulse will be too long for so fast sampling rate), see table above ppg_led_pulse_width definition
 * @param pulse_width enum value (bits) for pulse width configuration
 * @return 0 - success, (-1) - success but sampling rate changed, nonzero - failure
*/
int max86150::set_leds_pulse_width(ppg_led_pulse_width pulse_width) {
	uint8_t ppg_sr = read_reg(PPG_CONFIGURATION_1) & PPG_SR_MASK;
	uint8_t ecg_sr = read_reg(ECG_CONFIGURATION_1) & (ECG_ADC_CLK_BIT | ECG_ADC_OSR_MASK);
	int ret = write_reg(PPG_CONFIGURATION_1, set_reg_bits(read_reg(PPG_CONFIGURATION_1), PPG_LED_PW_MASK, pulse_width));
	if (!ret) {
		if ((read_reg(PPG_CONFIGURATION_1) & PPG_SR_MASK) != ppg_sr || (read_reg(ECG_CONFIGURATION_1) & (ECG_ADC_CLK_BIT | ECG_ADC_OSR_MASK)) != ecg_sr) {
			return -1;
		}
	}
	return ret;
}

/**
 * @brief averaging all active (ECG / PPC / ECG & PPG) samples
 * @param num_samples_averaged enum value (bits) for sample averaging configuration
 * @return 0 - success, nonzero - failure
*/
int max86150::enable_sample_averaging(ecppg_sample_averaging num_samples_averaged) {
	return write_reg(PPG_CONFIGURATION_2, set_reg_bits(read_reg(PPG_CONFIGURATION_2), SMP_AVE_MASK, num_samples_averaged));
}

/**
 * @brief read all used register addresses in device (jumping unused addresses)
 * @param reg_map pointer to array where read registers will be written
 * @return 0 - success, nonzero - failure
*/
int max86150::read_reg_map(uint8_t *reg_map) {
	int ret;
	for (int i = 0, len = 0; len < NUM_REGS; i++) {
		int continuous_regs = read_reg_map_jumps[i][1] - read_reg_map_jumps[i][0] + 1;
		if ((ret = read_regs(read_reg_map_jumps[i][0], continuous_regs, &reg_map[len]))) {
			break;
		}
		len += continuous_regs;
	}
	return ret;
}

/**
 * @brief write all used register addresses in device (jumping unused addresses)
 * @param reg_map pointer to array of values to be written into registers
 * @return 0 - success, nonzero - failure
*/
int max86150::write_reg_map(uint8_t *reg_map) {
	int ret;
	for (int i = 0, len = 0; len < NUM_REGS_WR; i++) {
		int continuous_regs = write_reg_map_jumps[i][1] - write_reg_map_jumps[i][0] + 1;
		if ((ret = write_regs(write_reg_map_jumps[i][0], continuous_regs, &reg_map[len]))) {
			break;
		}
		len += continuous_regs;
	}
	check_if_reg_is_config_reg(FIFO_CONFIGURATION, reg_map[6]);
	check_if_reg_is_config_reg(FIFO_DATA_CONTROL_REGISTER_1, reg_map[7]);
	check_if_reg_is_config_reg(FIFO_DATA_CONTROL_REGISTER_2, reg_map[8]);
	check_if_reg_is_config_reg(PPG_CONFIGURATION_1, reg_map[10]);
	check_if_reg_is_config_reg(ECG_CONFIGURATION_3, reg_map[18]);
	return ret;
}

/**
 * @brief read register in device using I2C
 * @param start_reg_address address of register in the device
 * @return read register value
*/
uint8_t max86150::read_reg(uint8_t reg_address) {
	uint8_t reg_value = 0x00;  // 0xFF
	i2c_write(&i2c, MAX86150_ADDRESS, (char *) &reg_address, 1, false);
	i2c_read(&i2c, MAX86150_ADDRESS, (char *) &reg_value, 1, true);
	return reg_value;
}

/**
 * @brief read subsequent registers in device using I2C
 * @param start_reg_address address of first register in the device
 * @param num_regs number of registers
 * @param in_values pointer to array where read register values will be written
 * @return 0 - success, nonzero - failure
*/
int max86150::read_regs(uint8_t start_reg_address, uint16_t num_regs, uint8_t *out_buffer) {
	i2c_write(&i2c, MAX86150_ADDRESS, (char *) &start_reg_address, 1, false);
	return !(i2c_read(&i2c, MAX86150_ADDRESS, (char *) out_buffer, num_regs, true) == num_regs);
}

/**
 * @brief write register in device using I2C
 * @param reg_address address of register in the device
 * @param values value to be written into register
 * @return 0 - success, nonzero - failure
*/
int max86150::write_reg(uint8_t reg_address, uint8_t value) {
	uint8_t data[] = {reg_address, value};
	int ret = !(i2c_write(&i2c, MAX86150_ADDRESS, (char *) data, sizeof(data), true) == sizeof(data));
	if (!ret) {
		check_if_reg_is_config_reg(reg_address, value);
	}
	return ret;
}

/**
 * @brief write subsequent registers in device using I2C
 * @param start_reg_address address of first register in the device
 * @param num_regs number of registers
 * @param in_values pointer to array of values to be written into registers
 * @return 0 - success, nonzero - failure
*/
int max86150::write_regs(uint8_t start_reg_address, uint8_t num_regs, uint8_t *in_values) {
	uint8_t data[1 + NUM_REGS] = {start_reg_address, };
	uint8_t length = 1 + min(num_regs, (uint8_t) NUM_REGS);
	memcpy(&data[1], in_values, length);
	int ret = !(i2c_write(&i2c, MAX86150_ADDRESS, (char *) data, length, true) == length);
	if (!ret) {
		for (int i = 0; i < length - 1; i++) {
			check_if_reg_is_config_reg(start_reg_address + i, in_values[i]);
		}
	}
	return ret;
}

/**
 * @brief read all data from queue
 * @param data pointer to array where read data will be written
 * @return number of data read
*/
int max86150::read_queue_data(ecppg_data_t *data) {
	int i = 0;
	while (!dequeue(&q, (uint8_t *) data)) {
		data++;
		i++;
	}
	return i;
}

/**
 * @brief read data from queue (up to maximal specified number of data)
 * @param data pointer to array where read data will be written
 * @param max_length maximal number of data to read
 * @return number of data read
*/
int max86150::read_queue_data(ecppg_data_t *data, int max_length) {
	if (max_length > 0) {
		for (int i = 0; i < max_length; i++) {
			if (dequeue(&q, (uint8_t *) data)) {
				return i;
			}
			data++;
		}
	}
	return max_length;
}

/**
 * @brief dele data stored in queue
 * @return 0 - success, (-1) - failure (queue corrupted)
*/
int max86150::clear_queue() {
	return queue_reset(&q);
}

// v - - - private - - - v //

/**
 * @brief change bits under mask to specified value in register
 * @param reg value of register
 * @param mask possitive mask (masking required bits)
 * @param bits required bits in register under mask
 * @return changed register value
*/
uint8_t max86150::set_reg_bits(uint8_t reg, uint8_t mask, uint8_t bits) {
	reg &= ~mask;
	bits &= mask;
	return reg | bits;
}

/**
 * @brief read, process and enqueue all samples currently in FIFO (to prevent lost of data when FIFO FLUSH)
 *  ; note: FIFO is flushed when enabling FIFO or writing fifo data control and ppg / ecg configuration register
 * @return number of lost data: 0 - success, <1; FIFO_LENGTH> - failure, (-1) - communication failure
*/
int max86150::read_fifo_to_queue() {
	return read_fifo_to_queue(get_num_samples_in_fifo());
}

/**
 * @brief read specified number of data from FIFO, process them and enqueue them
 * @param num_samples number of data to be read from FIFO
 * @return number of lost samples (due to full queue buffer): 0 - success, <1; FIFO_LENGTH> - failure, (-1) - communication failure
*/
int max86150::read_fifo_to_queue(uint8_t num_samples) {
	uint16_t byte_length = min(num_samples, (uint8_t) FIFO_LENGTH) * BYTES_PER_ELEMENT * used_data_slots;
	uint8_t raw_data[(FIFO_LENGTH * BYTES_PER_ELEMENT * NUM_DATA_SLOTS)];
	if (read_regs(FIFO_DATA_REGISTER, byte_length, raw_data)) {
		return -1;
	}
	return process_raw_data_to_queue(raw_data, num_samples);
}

/**
 * @brief process raw register values to valid numbers and copy them to queue
 * @param raw_data pointer to array of data to be enqueued
 * @param num_samples number of data to be enqueued
 * @return number of data left to be enqueued, 0 - succes
*/
int max86150::process_raw_data_to_queue(uint8_t *raw_data, int num_samples) {
	for (int i = 0; i < num_samples; i++) {
		ecppg_data_t sample;
		memset(&sample, 0, sizeof(sample));
		int sample_offset = BYTES_PER_ELEMENT * used_data_slots * i;
		int empty_slot_compenzation = 0;
		for (int j = 0; j < NUM_DATA_SLOTS; j++) {  // if [UNUSED, LED1, LED2, ECG] while using "used_data_slots" will fail ?
			if (data_slots_type[j] == UNUSED) {
				empty_slot_compenzation++;
				break;
			}  // check data slot used
			int element_offset = BYTES_PER_ELEMENT * (j - empty_slot_compenzation);
			int offset = sample_offset + element_offset;
			int tmp = ((int) raw_data[offset] << 16) | ((int) raw_data[offset + 1] << 8) | ((int) raw_data[offset + 2]);
			if (data_slots_type[j] == ECG_DATA) {
				tmp &= ECG_DATA_BIT_MASK;
				if (tmp & (1 << 17)) { // ECG - two's complement
					tmp -= (1 << 18);
				}
				sample.ecg = tmp;
			} else if (data_slots_type[j] == PPG_RED_DATA) {
				tmp &= PPG_DATA_BIT_MASK;
				sample.red = tmp;
			} else if (data_slots_type[j] == PPG_IR_DATA) {
				tmp &= PPG_DATA_BIT_MASK;
				sample.ir = tmp;
			}
		}  // element
		if (enqueue(&q, (uint8_t *) &sample)) {
			return num_samples - i;
		}
	}  // sample
	return 0;
}

/**
 * @brief adjust internal values (for used data slot and computing ECG mV and ppg nA) if user rewrite configuration settings
 * @param reg register address being rewritten
 * @param val value being written into register
*/
void max86150::check_if_reg_is_config_reg(uint8_t reg, uint8_t val) {
	if (reg == FIFO_CONFIGURATION) {
		fifo_full_num_samples = FIFO_LENGTH - (val & FIFO_A_FULL_MASK);
	} else if (reg == FIFO_DATA_CONTROL_REGISTER_1) {
		update_slot_type(1, val);
		update_slot_type(2, val);
	} else if (reg == FIFO_DATA_CONTROL_REGISTER_2) {
		update_slot_type(3, val);
		update_slot_type(4, val);
	} else if (reg == PPG_CONFIGURATION_1) {
		ppg_range = val & PPG_ADC_RGE_MASK;
	} else if (reg == ECG_CONFIGURATION_3) {
		ecg_gain.ia_gain = val & IA_GAIN_MASK;
		ecg_gain.pga_gain = val & PGA_ECG_GAIN_MASK;
	}
}

/**
 * @brief set all positions (bits) of slots in "regs" which contain value "data"
 * @param regs pointer to array of read FIFO data control register values
 * @param data value of slot configuration (fifo_data_type) to find in "regs"
*/
void max86150::clear_slots_in_regs_with_type(uint8_t *regs, uint8_t data) {
	for (uint8_t i = 0; i < NUM_DATA_SLOTS; i++) {
		uint8_t FDx_MASK = FDn_DATA(i, 0x0F);
		if ((regs[i >> 1] & FDx_MASK) == FDn_DATA(i, data)) {
			regs[i >> 1] &= ~FDx_MASK;
			//regs[i >> 1] |= FDn_DATA(i, UNUSED);  // redundant ... UNUSED == 0
		}
	}
}

/**
 * @brief update internal value used for data slot (1, 2, 3 or 4) type information
 * @param slot number of data slot
 * @param type data type stored in slot
*/
void max86150::update_slot_type(uint8_t slot, /*fifo_data_type*/uint8_t type) {
	slot = (slot < 1) ? 1 : ((slot > 4) ? 4 : slot);  // max((uint8_t) 1, min((uint8_t) 4, slot));
	type >>= (~slot & 1)? 4 : 0;  // slot 2 or 4 are 4 MSB
	type &= 0x0F;
	if (type) {
		if (data_slots_type[slot - 1] == UNUSED) {
			used_data_slots++;
		}
		data_slots_type[slot - 1] = type;
	} else {
		if (data_slots_type[slot - 1] != UNUSED) {
			used_data_slots--;
			data_slots_type[slot - 1] = UNUSED;
		}
	}
}

/**
 * @brief check interrupt flags from status registers and call their routines, FIFO is read to queue,
 *  optionally if (external) handler is initalized, it is called at the end of this function
*/
void max86150::internal_interrupt_handler() {
	uint8_t int_status[2] = {0, 0};
	read_regs(INTERRUPT_STATUS_1, 2, int_status);

	if (no_interrupts) {  // if unauthorised write to interrupt enable registers occures in no interrupt mode
		uint8_t clear_int[2] = {0, 0};
		write_regs(INTERRUPT_ENABLE_1, 2, clear_int);
		return;
	}

	if (int_status[0] & A_FULL_BIT) {
		read_fifo_to_queue();
	}
	if (int_status[0] & PPG_RDY_BIT) {
		read_fifo_to_queue();
	}
	if (int_status[0] & ALC_OVF_BIT) {	// ambient light cancellation overflow
		// PPG data affected by ambient light
	}
	if (int_status[0] & PROX_INT_BIT) {	// proximity interrupt
		// FIFO gets FLUSHed
		// max86xxx driver reads FIFO, but why ?
	}
	if (int_status[0] & PWR_RDY_BIT) {	// power ready
		// sensor is ready
	}
	if (int_status[1] & VDD_OOR_BIT) {
		// supply voltage out of range
	}
	if (int_status[1] & ECG_RDY_BIT) {
		read_fifo_to_queue();
	}

	if (interrupt_handler) {
		interrupt_handler(((uint32_t) int_status[1] << 8) | (uint32_t) int_status[0]);
	}
}
