#include <mbed.h>
#include <max32630fthr.h>
#include <max86150.h>

void my_handler(uint32_t status);
void enable_button();	// button debounce
void sleep_control();	// ["shutdown" (sleep) sensor and stop sending data] / [wake up and start sending data] on button press
void print_sample_in_bytes(ecppg_data_t *sample);

MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);			// board
PwmOut b(LED_BLUE);										// board runnig indicator
DigitalOut g(LED_GREEN);								// change state every "samples read" interrupt
InterruptIn sw(SW1, PullUp);							// user button on board
Timeout to;												// button debounce
max86150 sens(I2C2_SDA, I2C2_SCL, P5_5, my_handler);	// sensor driver

FileHandle *mbed::mbed_override_console(int) {
    static BufferedSerial uart(USBTX, USBRX, 115200);	// default 9600 is too slow
    return &uart;
}

ecppg_data_t samples[64] = {0};
volatile int samples_buffer_length = 0;

#define TEXT_PRINT

int main() {
	b.period(1);
	b = .5f;
	g = 1;
	sw.fall(sleep_control);
	memset(samples, 0, sizeof(samples));
	//int ret = sens.ecg_mode(ECG_SR_200, IA_ECG_GAIN_9p5, PGA_ECG_GAIN_8);
    //int ret = sens.ppg_mode(PPG_SR_N1_200, PPG_ADC_RGE_nA_4096, IR_LED_RGE_50mA, LEDn_PA_5mA_10mA, PPG_LED_PW_50us);
	int ret = sens.ecppg_mode(ECG_SR_200, IA_ECG_GAIN_9p5, PGA_ECG_GAIN_8, PPG_ADC_RGE_nA_4096, IR_LED_RGE_50mA, LEDn_PA_1mA_2mA, PPG_LED_PW_50us);
	printf("ret = %d\n", ret);

	while (1) {
		if (samples_buffer_length>0) {
			for (int i = 0; i < samples_buffer_length; i++) {
#ifdef TEXT_PRINT
				printf("%lu;%lu;%ld\n", samples[i].ir, samples[i].red, samples[i].ecg);
#else
				print_sample_in_bytes(&samples[i]);
#endif
			}
			samples_buffer_length = 0;
		}
	}
}

void my_handler(uint32_t status) {
	uint16_t statusInt = status;
	if (statusInt & A_FULL_BIT) {
		g = !g;
		samples_buffer_length = sens.read_queue_data(samples, (sizeof(samples) / sizeof(samples[0])));
	}
}

void enable_button() {
	to.detach();
	sw.enable_irq();
}

void sleep_control() {
	static uint8_t sleep;
	sw.disable_irq();
	sleep = !sleep;
	if (sleep) {
		sens.shutdown();
		b.period(2);
		g = 1;
	} else {
		sens.wakeup();
		b.period(1);
	}
	to.attach(enable_button, 500ms);
}

void print_sample_in_bytes(ecppg_data_t *sample) {
	for (int j = 0; j < 4; j++) {
		putchar(*(((uint8_t *) &(sample->ir)) + j));
	}
	for (int j = 0; j < 4; j++) {
		putchar(*(((uint8_t *) &(sample->red)) + j));
	}
	for (int j = 0; j < 4; j++) {
		putchar(*(((uint8_t *) &(sample->ecg)) + j));
	}
	putchar('\n');
}
