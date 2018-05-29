#include "types.h"


volatile tir_tx_buffer_cursor ir_tx_buffer_cursor; //курсор передающего ИК буфера
volatile uint8_t tx_buffer[TX_BUFFER_SIZE] __attribute__ ((section (".noinit"))); 	//Буффер ИК-передатчика


volatile uint32_t leds_on_counter;

volatile uint32_t vibro_on_counter;

volatile uint32_t adc_test_counter;

volatile bool leds_off_flag;

volatile bool vibro_off_flag;

volatile bool adc_test_now_flag = false;

volatile bool bluetooth_connection_was_already = false;
