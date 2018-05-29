#ifndef __GLOBAL_VARIABLES_H
#define __GLOBAL_VARIABLES_H

extern volatile tir_tx_buffer_cursor ir_tx_buffer_cursor; //курсор передающего ИК буфера
extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE] __attribute__ ((section (".noinit"))); 	//Буффер ИК-передатчика

extern volatile uint32_t leds_on_counter;
extern volatile uint32_t vibro_on_counter;
extern volatile uint32_t adc_test_counter;

extern volatile bool leds_off_flag;
extern volatile bool vibro_off_flag;
extern volatile bool adc_test_now_flag;
extern volatile bool bluetooth_connection_was_already;
#endif
