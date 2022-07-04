#ifndef MAX86150_H
#define MAX86150_H

#include <mbed.h>
#include "max86150regmap.h"
extern "C" {
	#include "max_queue.h"
}

#define MAX86150_ADDRESS    0xBC
typedef struct ecppg_data ecppg_data_t;

struct ecppg_data {
	uint32_t ir;
	uint32_t red;
	int32_t ecg;
};

/*
	Start (constructor) takes 1ms.

	Write to any configuration register will flush FIFO, to store samples 
	call read_fifo_to_queue_now() before writing configuration registers.

	Current implementation store all samples to internal queue, so the queue needs to be read 
	(primarily) in user defined interrupt handler (which is called at the end of internal interrupt handler) 
	or in loop (ideally faster then it is filled to not lose samples).

  >Hardware "limitations":
	ECG sampling rate is master sampling rate:
	ECG SR > PPG SR:
	-	ppg sr set to same sr as ecg, if it not possible (LED PW),
	-	maximal allowd ppg sr is set and ppg data in fifo is redundant (repeating same value)
	ECG SR == PPG SR:
	-	everything OK:
	ECG SR < PPG SR:
	-	ppg sr stays same, but only every n-th value is stored in fifo (other values are lost)
*/
class max86150 {
  public:
	max86150(PinName sda, PinName scl);
	max86150(PinName sda, PinName scl, PinName interrupt_pin);
	max86150(PinName sda, PinName scl, PinName interrupt_pin, void(*interrupt_handler)(uint32_t status));

	int ecg_mode(ecg_sampling_rate sampling_rate, ecg_ia_gain ia_gain, ecg_pga_gain pga_gain);
	int ppg_mode(ppg_sampling_rate sampling_rate, ppg_adc_range_nA adc_range, ppg_leds_range ranges, ppg_led_pulse_amplitude currents, ppg_led_pulse_width pulse_width);
	int ecppg_mode(ecg_sampling_rate sampling_rate, ecg_ia_gain ia_gain, ecg_pga_gain pga_gain, ppg_adc_range_nA range, ppg_leds_range ranges, ppg_led_pulse_amplitude currents, ppg_led_pulse_width pulse_widths);
	int disable_ecg();
	int disable_ppg();
	int disable_ecppg();  // FDn = 0000
	float get_ecg_mV(int32_t raw_ecg_value);
	float get_ppg_nA(uint32_t raw_ppg_value);

	int set_ecg_gain(ecg_ia_gain ia_gain, ecg_pga_gain pga_gain);
	int set_led_current(ppg_leds led, ppg_leds_range range, ppg_led_pulse_amplitude current);
	uint8_t get_num_samples_in_fifo();
	uint8_t read_fifo_write_pointer();
	uint8_t read_fifo_read_pointer();
	uint8_t read_ovf_counter();
	int read_fifo_to_queue_now();
	
	int enable_interrupt(max86150_interrupt_bit_positions interrupt);
	int disable_interrupt(max86150_interrupt_bit_positions interrupt);
	int shutdown();
	int wakeup();
	int reset();

	int proximity_mode(bool state);
	int proximity_mode(bool state, /*ppg_leds pilot_led,*/ ppg_led_pulse_amplitude pilot_led_current/* = LEDn_PA_0mA_0mA*/, uint8_t prox_thresh/* = 0x00*/);
	int set_prox_int_threshold(uint8_t threshold);
	int set_pilot_led_pulse_amplitude_current(ppg_led_pulse_amplitude current);

	int fifo_enable(bool state = true);
	int set_data_slot(fifo_data_slot slot, fifo_data_type type);
	int fifo_rolls_on_fun(bool state);
	int fifo_almost_full_interrupt_at_samples(uint8_t samples_stored = DEFAULT_FULL_SAMPLES);
	int fifo_almost_full_interrupt_clear_on_data_read(bool state);
	int fifo_almost_full_interrupt_flag_repeat(bool state = false);  // if a_full @ 17 : true - 17,18,19,... ; false - 17

	int set_ecg_ia_gain(ecg_ia_gain gain);
	int set_ecg_pga_gain(ecg_pga_gain gain);
	int set_ecg_sampling_rate(ecg_sampling_rate sampling_rate);
	int set_ppg_sampling_rate(ppg_sampling_rate sampling_rate);
	int set_ppg_range_nA(ppg_adc_range_nA range);
	int set_led_current_range(ppg_leds led, ppg_leds_range range);
	int set_led_pulse_amplitude_current(ppg_leds led, ppg_led_pulse_amplitude current);
	int set_leds_pulse_width(ppg_led_pulse_width pulse_width);
	int enable_sample_averaging(ecppg_sample_averaging num_samples_averaged);
	
	uint8_t read_reg(uint8_t reg_address);
	int read_regs(uint8_t start_reg_address, uint16_t num_regs, uint8_t *out_buffer);
	int read_reg_map(uint8_t *reg_map);
	int write_reg(uint8_t reg_address, uint8_t value);
	int write_regs(uint8_t start_reg_address, uint8_t num_regs, uint8_t *in_values);
	int write_reg_map(uint8_t *reg_map);

	int read_queue_data(ecppg_data_t *data);
	int read_queue_data(ecppg_data_t *data, int max_length);
	int clear_queue();

  private:
	uint8_t set_reg_bits(uint8_t reg, uint8_t mask, uint8_t bits);
  	int read_fifo_to_queue();
	int read_fifo_to_queue(uint8_t num_samples);
	int process_raw_data_to_queue(uint8_t *raw_data, int num_samples);
	void check_if_reg_is_config_reg(uint8_t reg, uint8_t val);
	void clear_slots_in_regs_with_type(uint8_t *regs, uint8_t data);
	void update_slot_type(uint8_t slot, uint8_t type);
	void internal_interrupt_handler();

	i2c_t i2c;
	InterruptIn intIn;

	void (*interrupt_handler)(uint32_t status) = NULL;
	bool no_interrupts = false;
	struct _ecg_gain {
		uint8_t ia_gain;  // ecg_ia_gain
		uint8_t pga_gain;  // ecg_pga_gain
	} ecg_gain = {
		.ia_gain = IA_ECG_GAIN_20,
		.pga_gain = PGA_ECG_GAIN_1
	};
	uint8_t ppg_range = PPG_ADC_RGE_nA_4096;  // ppg_adc_range_nA
	uint8_t fifo_full_num_samples = 17;  // default
	uint8_t used_data_slots = 0;
	uint8_t data_slots_type[NUM_DATA_SLOTS];  // fifo_data_type
	struct queue_t q = {
		.head = -1,
		.tail = -1,
		.element_size = 1,
		.buffer = NULL,
		.buffer_size = 0
	};
	uint8_t q_buf[sizeof(struct ecppg_data) * 100];  // 1200
};

#endif
