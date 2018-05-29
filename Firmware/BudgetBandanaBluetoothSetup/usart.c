#include "usart.h"

#ifndef bool

                        #define bool unsigned char

                        #define true 1

                        #define false 0

#endif

//передающий буфер
 volatile unsigned char usart1_TxBuf[SIZE_BUF] __attribute__ ((section (".noinit")));
volatile unsigned char usart1_txBufTail = 0;
volatile unsigned char usart1_txBufHead = 0;
volatile unsigned char usart1_txCount = 0;

//приемный буфер
volatile unsigned char usart1_RxBuf[SIZE_BUF]; //__attribute__ ((section (".noinit")));
volatile unsigned char usart1_rxBufTail = 0;
volatile unsigned char usart1_rxBufHead = 0;
volatile unsigned char usart1_rxCount = 0;

volatile char usartData;
extern volatile bool cr_received;

void InitCOM1(uint32_t boudRate)
{
	GPIO_InitTypeDef s;
	USART_InitTypeDef usart;
	//Включаем тактирование
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    s.GPIO_Pin   = GPIO_Pin_2;       // Настройка вывода PB1.
    s.GPIO_Mode  = GPIO_Mode_AF;     // Вывод настроен на одну из альтернативных функций.
    s.GPIO_OType = GPIO_OType_OD;    // Открытый сток.
    s.GPIO_PuPd  = GPIO_PuPd_UP; // Подтягивающие резисторы к питанию.
    s.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &s);            // Инициализация порта.

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = boudRate;//BAUDRATE;
    USART_Init(USART1, &usart);
    USART_HalfDuplexCmd(USART1,ENABLE);

    //Включаем прерывания по приему байта и по окончанию передачи
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    //Запускаем сам USART

	USART_Cmd(USART1, ENABLE);
	//Разрешаем прерывания
	NVIC_SetPriority(USART1_IRQn, 12);
	NVIC_EnableIRQ(USART1_IRQn);
}
void DeInitCOM1(void){

    //Выключаем прерывания по приему байта и по окончанию передачи
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    //Останавливаем сам USART

	USART_Cmd(USART1, DISABLE);
	//Запрещаем прерывания
//	NVIC_SetPriority(USART1_IRQn, 12);
	NVIC_DisableIRQ(USART1_IRQn);
}
void InitCOM1_FullDuplex(uint32_t boudRate)
{
	GPIO_InitTypeDef s;
	USART_InitTypeDef usart;
	//Включаем тактирование
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    s.GPIO_Pin   = GPIO_Pin_2|GPIO_Pin_3;       // Настройка вывода PB1.
    s.GPIO_Mode  = GPIO_Mode_AF;     // Вывод настроен на одну из альтернативных функций.
    s.GPIO_OType = GPIO_OType_OD;    // Открытый сток.
    s.GPIO_PuPd  = GPIO_PuPd_UP; // Подтягивающие резисторы к питанию.
    s.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &s);            // Инициализация порта.

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = boudRate;//BAUDRATE;
    USART_Init(USART1, &usart);
    USART_HalfDuplexCmd(USART1,DISABLE/*ENABLE*/);

    //Включаем прерывания по приему байта и по окончанию передачи
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    //Запускаем сам USART

	USART_Cmd(USART1, ENABLE);
	//Разрешаем прерывания
	NVIC_SetPriority(USART1_IRQn, 12);
	NVIC_EnableIRQ(USART1_IRQn);
}



void USART1_IRQHandler(void){
	volatile int i;
	i++;
	static volatile char ch_tmp;
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
    	if (usart1_txCount > 0)
    	{              //если буфер не пустой
    	    USART_SendData(USART1, usart1_TxBuf[usart1_txBufHead]);//записываем в UDR символ из буфера
    	    usart1_txCount--;                   //уменьшаем счетчик символов
    	    usart1_txBufHead++;                 //инкрементируем индекс головы буфера
    	    if (usart1_txBufHead == SIZE_BUF) usart1_txBufHead = 0;

    	}

		USART_ClearITPendingBit(USART1, USART_IT_TC);

	}
	//Проверяем, действительно ли прерывание вызвано приемом нового байта
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		  if (usart1_rxCount < SIZE_BUF){                //если в буфере еще есть место
		      usart1_RxBuf[usart1_rxBufTail] = USART_ReceiveData(USART1);        //считать символ из UDR в буфер
		      if (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5)){//если блютус модль в режиме AT-команд
		      		      	  if(usart1_RxBuf[usart1_rxBufTail]=='\r')
		      		      	  {
		      		      		cr_received=true;
		      		      	  }
		      		      }//[ if (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5))]

		      usart1_rxBufTail++;                             //увеличить индекс хвоста приемного буфера
    	      if (usart1_rxBufTail == SIZE_BUF) usart1_rxBufTail = 0;
    	      usart1_rxCount++;                                 //увеличить счетчик принятых символов

		  }

	}
	 if (USART1->ISR & USART_FLAG_ORE )
	    {
	    	USART_ClearITPendingBit(USART1, USART_IT_ORE);
	    }
}




//возвращает колличество символов передающего буфера
unsigned char USART1_GetTxCount(void)
{
  return usart1_txCount;
}

//______________________________________________________________________________
//возвращает колличество символов находящихся в приемном буфере
unsigned char USART1_GetRxCount(void)
{
  return usart1_rxCount;
}

//"очищает" передающий буфер
void USART1_FlushTxBuf(void)
{
	usart1_txBufTail = 0;
	usart1_txBufHead = 0;
	usart1_txCount = 0;
}


//"очищает" приемный буфер
void USART1_FlushRxBuf(void)
{
  usart1_DisableRxInt();  //запрещаем прерывание по заверщению приема
  usart1_rxBufTail = 0;
  usart1_rxBufHead = 0;
  usart1_rxCount = 0;
  usart1_EnableRxInt();
}


void USART1_PutChar(unsigned char sym) //положить символ в буфер
{
	 if (/*(USART_GetITStatus(USART1, USART_IT_TXE) == SET)*/(USART1->ISR & USART_FLAG_TXE/*USART_SR_TXE*/)&& (usart1_txCount == 0))
	 {
//		 USART_GetFlagStatus(USART1,USART_FLAG_TXE);
		 USART_SendData(USART1, sym);
	 }
	 else {
		 if (usart1_txCount < SIZE_BUF){    //если в буфере еще есть место
			 usart1_TxBuf[usart1_txBufTail] = sym; //помещаем в него символ
			 usart1_txCount++;                   //инкрементируем счетчик символов
			 usart1_txBufTail++;                 //и индекс хвоста буфера
			 if (usart1_txBufTail == SIZE_BUF) usart1_txBufTail = 0;
			 }
		 }
}


//чтение буфера
unsigned char USART1_GetChar(void)
{
  unsigned char sym;
  if (usart1_rxCount > 0){                     //если приемный буфер не пустой
    sym = usart1_RxBuf[usart1_rxBufHead];        //прочитать из него символ
    usart1_rxCount--;                          //уменьшить счетчик символов
    usart1_rxBufHead++;                        //инкрементировать индекс головы буфера
    if (usart1_rxBufHead == SIZE_BUF) usart1_rxBufHead = 0;
    return sym;                         //вернуть прочитанный символ
  }
  return 0;
}





//функция посылающая строку по usart`у
void USART1_SendStr(char * data)
{
volatile  unsigned char sym;
//  while(*data)
  while((sym = *data++)){

    USART1_PutChar(sym);
  }
}
