#ifndef __TYPES_H
#define __TYPES_H
#include "stm32f0xx.h"
#ifndef bool

                        #define bool unsigned char

                        #define true 1

                        #define false 0

#endif

#define TX_BUFFER_SIZE   40 //������ ������� �����������

typedef struct IR_Tx_Buffer_Cursor {
uint8_t byte_pos; //����� ����� � ������ �����������
uint8_t bit_mask;//������� ����� ��� ��������� �������� ����
uint16_t bits_for_tx;//������� ��� ����� ��� ��������
bool heder_been_sent;//����, �����������, ��������� �� ��������� ������

} tir_tx_buffer_cursor;








#endif /* __TYPES_H */
