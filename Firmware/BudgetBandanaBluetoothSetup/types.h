#ifndef __TYPES_H
#define __TYPES_H
#include "stm32f0xx.h"
#ifndef bool

                        #define bool unsigned char

                        #define true 1

                        #define false 0

#endif

#define TX_BUFFER_SIZE   40 //Размер буффера передатчика

typedef struct IR_Tx_Buffer_Cursor {
uint8_t byte_pos; //номен байта в буфере передатчика
uint8_t bit_mask;//битовая маска для получения текущего бита
uint16_t bits_for_tx;//сколько бит нужно ещё передать
bool heder_been_sent;//флаг, указывающий, отправлен ли заголовок пакета

} tir_tx_buffer_cursor;








#endif /* __TYPES_H */
