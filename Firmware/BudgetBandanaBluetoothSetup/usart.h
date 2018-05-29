#ifndef __USART_H
#define __USART_H
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_rcc.h"
#define BAUDRATE 9600//38400
#define SIZE_BUF 128//256       //� ������ ��������� ������� - <255


//������� ��� ���������� � ���������� ���������� usart`a
#define usart1_EnableRxInt()   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#define usart1_DisableRxInt()  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
#define usart1_EnableTxInt()   USART_ITConfig(USART1, USART_IT_TC, ENABLE);
#define usart1_DisableTxInt()   USART_ITConfig(USART1, USART_IT_TC, DISABLE);
#define usart2_EnableRxInt()   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#define usart2_DisableRxInt()  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
#define usart2_EnableTxInt()   USART_ITConfig(USART2, USART_IT_TC, ENABLE);
#define usart2_DisableTxInt()   USART_ITConfig(USART2, USART_IT_TC, DISABLE);

void InitCOM1(uint32_t boudRate);
void InitCOM1_FullDuplex(uint32_t boudRate);
void DeInitCOM1(void);

unsigned char USART1_GetTxCount(void); //����� ����� �������� ����������� ������
unsigned char USART1_GetRxCount(void);
void USART1_FlushTxBuf(void); //�������� ���������� �����
void USART1_FlushRxBuf(void); //�������� �������� �����
void USART1_PutChar(unsigned char sym); //�������� ������ � �����
unsigned char USART1_GetChar(void);
void USART1_SendStr(char * data);

#endif



