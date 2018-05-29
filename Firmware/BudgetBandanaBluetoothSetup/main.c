#include "stm32f0xx.h"
#include "stm32f0xx_crc.h"
#include "stm32f0xx_dma.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_flash.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_rtc.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_adc.h"

#include "ws2812b.h"
//#include "math.h"
#include <stdlib.h>
#include "miles_protocol.h"
#include "types.h"
#include "global_variables.h"

#define NUM_LEDS    4
#define LEDS_ON_TIME 1000
#define ASCET_LEDS_ON_TIME (300/8)
#define VIBRO_ON_TIME 1000
#define ADC_TEST_INTERVAL 10000

RGB_t leds[NUM_LEDS] __attribute__ ((section (".noinit")));
//HSV_t hsv_leds[NUM_LEDS] __attribute__ ((section (".noinit")));



volatile int i=0;
GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef timer1,timer3;
GPIO_InitTypeDef port;
TIM_ICInitTypeDef TIM_ICStructure;

volatile bool cr_received=false;
const char numbers[]={"0123456789"};

char* int_to_str(uint8_t x, uint8_t digits){

static volatile char str[6];



volatile uint8_t celoe, ostatok;
celoe=x;

int digits_tmp;
digits_tmp=digits;
if (digits == 0) digits_tmp=3;
      for (int i=(digits_tmp-1); i>=0; i--)
      {

      ostatok= celoe%10;
	  celoe  = celoe/10;
	  str[i]= numbers[ostatok];
      }
      str[digits_tmp]='\0';



if (digits == 0)
{
        while ((str[0]=='0')&&(str[1] !='\0')) for (int i=0; i < 6; i++) str[i]=str[i+1];
}

      return (char*) &str;

}

char* long_int_to_str(uint16_t x, uint8_t digits){

static volatile char str[6];



volatile uint16_t celoe, ostatok;
celoe=x;

int digits_tmp;
digits_tmp=digits;
if (digits == 0) digits_tmp=5;
      for (int i=(digits_tmp-1); i>=0; i--)
      {

      ostatok= celoe%10;
	  celoe  = celoe/10;
	  str[i]= numbers[ostatok];
      }
      str[digits_tmp]='\0';



if (digits == 0)
{
        while ((str[0]=='0')&&(str[1] !='\0')) for (int i=0; i < 6; i++) str[i]=str[i+1];
}

      return (char* )&str;

}


void set_bt_buffer_bit(uint8_t index, bool value){	//Задаем значение биту в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(value)
		{
			tx_buffer[byte_index] |= (1<<(7-bit_index));
		}
else	{
			tx_buffer[byte_index] &= ~(1<<(7-bit_index));
		}
}



/**************************************************************************************
* Функця производит "выстрел"
* устанавлвает курсор на позицию начала блока данных data_packet.data[0]
* и разрешает передачу данных
* функция возвращает управление  только после отправки всех данных
***************************************************************************************/
void send_ir_shot_package(void){ //Отправляем пакет ("стреляем")

//	FLASH_LED_ON;
	ir_tx_buffer_cursor.byte_pos = 0;
	ir_tx_buffer_cursor.bit_mask = (1<<7);
	ir_tx_buffer_cursor.bits_for_tx=14;//"выстрел" состоит из 14 бит
	ir_tx_buffer_cursor.heder_been_sent = false;
	TIM_DeInit(TIM16);//сбрасываем настройки таймера на дефолтовые
//	TIM4->CCR1 = 50;
	Tim16_Conf();
}



void init_SysTic(){
	/* SysTick закончит счет каждые 1 мс */
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}

volatile uint32_t timestamp = 0;
volatile uint32_t countdown_timer =0;



void SysTick_Handler (void)
{
timestamp++;
if(countdown_timer) countdown_timer--;
if(leds_on_counter)
	{
		leds_on_counter--;
		if(leds_on_counter==0)
		{
			leds_off_flag=1;
		}
	}
if(vibro_on_counter)
	{
		vibro_on_counter--;
		if(vibro_on_counter==0)
		{
			vibro_off_flag=1;
		}
	}

if(adc_test_counter)
	{
		adc_test_counter--;
		if(adc_test_counter==0)
		{
			adc_test_now_flag=1;
		//	adc_test_counter=ADC_TEST_INTERVAL;
		}
	}
else{
		adc_test_counter=ADC_TEST_INTERVAL;
	}
}

void delay_mS(uint32_t mS){
	countdown_timer = mS;
	while (countdown_timer){};
}
void Tim16_Conf(void) // Конфигурация таймера.
{

	TIM_TimeBaseInitTypeDef Tim;
    TIM_OCInitTypeDef Pwm;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE); // Включаем тактирование таймера 14.
    Tim.TIM_ClockDivision = TIM_CKD_DIV1; // Выключаем предварительный делитель частоты таймера.
    Tim.TIM_CounterMode   = TIM_CounterMode_Up; // Значение в счетном регистре увеличивается.
    Tim.TIM_Period        = IR_START_BIT_DURATION + IR_SPACE_DURATION;; // Значение до которого досчитав таймер обнулится (максимум 65535).
    Tim.TIM_Prescaler     =  (SystemCoreClock/1000000)-1;//считаем миллисекунды
    Tim.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM16, &Tim); // Инициализация таймера.
    // Настройка ШИМ.
    TIM_OCStructInit(&Pwm); // Загрузка в структуру настроек таймера по умолчанию.
    Pwm.TIM_OCMode = TIM_OCMode_PWM1; // Режим ШИМ 1.
    Pwm.TIM_OutputState = TIM_OutputState_Disable;
    Pwm.TIM_Pulse = IR_START_BIT_DURATION; //32768;
    TIM_OC1Init(TIM16, &Pwm);

    /* Разрешаем таймеру генерировать прерывание по захвату */
    TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
    // и по таймауту третьего канала
    TIM_ITConfig(TIM16, TIM_DIER_UIE, ENABLE);

    TIM_ClearFlag(TIM16, TIM_FLAG_CC1);
    TIM_ClearFlag(TIM16, TIM_FLAG_Update);

    TIM_Cmd(TIM16, ENABLE); // Запуск таймера.
    //устанавливаем приоритет, долен быть больше 11 для корректной работы FreeRTOS
    NVIC_SetPriority(TIM16_IRQn, 12);
    // разрешаем прерывания
    NVIC_EnableIRQ(TIM16_IRQn);

}
void Tim14_Conf(void) // Конфигурация таймера.
{
	 GPIO_InitTypeDef s;

	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // Включаем тактирование порта PB.

	    s.GPIO_Pin   = GPIO_Pin_1;       // Настройка вывода PB1.
	    s.GPIO_Mode  = GPIO_Mode_AF;     // Вывод настроен на одну из альтернативных функций.
	    s.GPIO_OType = GPIO_OType_PP;    // Двухтактный выход (т. е. не открытый сток).
	    s.GPIO_PuPd  = GPIO_PuPd_NOPULL; // Подтягивающие резисторы отключены.
	    s.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOB, &s);            // Инициализация порта.

	    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_0); // PB1 это выход таймера TIM 14.


	TIM_TimeBaseInitTypeDef Tim;
    TIM_OCInitTypeDef Pwm;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); // Включаем тактирование таймера 14.

    Tim.TIM_ClockDivision = TIM_CKD_DIV1; // Выключаем предварительный делитель частоты таймера.
    Tim.TIM_CounterMode   = TIM_CounterMode_Up; // Значение в счетном регистре увеличивается.
    Tim.TIM_Period        =  100; // Значение до которого досчитав таймер обнулится (максимум 65535).
    Tim.TIM_Prescaler     =  (SystemCoreClock/(56000*100))-1;//(SystemCoreClock/10000/*00000*/)-1;//считаем микросекунды;
    Tim.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM14, &Tim); // Инициализация таймера.

    // Настройка ШИМ.
    TIM_OCStructInit(&Pwm); // Загрузка в структуру настроек таймера по умолчанию.

    Pwm.TIM_OCMode = TIM_OCMode_PWM1; // Режим ШИМ 1.
    Pwm.TIM_OutputState = TIM_OutputState_Enable;
    Pwm.TIM_Pulse = 0; //32768;
    TIM_OC1Init(TIM14, &Pwm);

    TIM_Cmd(TIM14, ENABLE); // Запуск таймера.
}



void init_pwm_TIM3(void){

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF;//GPIO_Mode_IN;
	port.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &port);

//	GPIO_PinAFConfig(GPIOA,GPIO_Pin_6,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_1);
	TIM_TimeBaseStructInit(&timer3);
	timer3.TIM_Prescaler = /*72*/(SystemCoreClock/1000000)-1;//считаем микросекунды
    TIM_TimeBaseInit(TIM3, &timer3);

    TIM_ICStructure.TIM_Channel = TIM_Channel_1;//первый канал
    TIM_ICStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; // по заднему фронту
    TIM_ICStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // прямо с ножки
    TIM_ICStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // без делителя
    TIM_ICStructure.TIM_ICFilter = 0; // без фильтра

    // эта функция и включает режим PWM input - автоматически настраивает комплементарный канал
              // правда в стандартной библиотеке работает на 1 и 2 канале, на 3 и 4 - не умеет
    TIM_PWMIConfig(TIM3, &TIM_ICStructure);

    /* Выбираем источник для триггера: вход 1 */
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    /* По событию от триггера счётчик будет сбрасываться. */
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    /* Включаем события от триггера */
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

    // это третий канал, для таймаута. Таймаут 15мс, поскольку максимальный бит (старт) 13.5мс
    TIM_OCInitTypeDef TIM_OCStructure;
    TIM_OCStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCStructure.TIM_Pulse = 15000;
    TIM_OC3Init(TIM3, &TIM_OCStructure);

    /* Разрешаем таймеру генерировать прерывание по захвату */
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    // и по таймауту третьего канала
    TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE /*ENABLE*/);

    TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
    TIM_ClearFlag(TIM3, TIM_FLAG_CC3);

    /* Включаем таймер */
     TIM_Cmd(TIM3, ENABLE);
     //устанавливаем приоритет, долен быть больше 11 для корректной работы FreeRTOS
     NVIC_SetPriority(TIM3_IRQn, 12);
     // разрешаем прерывания
     NVIC_EnableIRQ(TIM3_IRQn);
}


//Определим перечисляемый тип для событий ИК-приемника
enum Rx_Event {
				NOT_EVENT, 		//нет событий
				RX_COMPLETE, 	//принят пакет
				RX_MESSAGE_COMPLETE,//принято сообщение
				RX_ERROR		//ошибка приема пакета
				};
typedef enum Rx_Event trx_event; //Определим перечисляемый тип для событий ИК-приемника


enum damage_zone{
	zone_4,
	zone_3,
	zone_2,
	zone_1
};
typedef enum damage_zone TDamageZone;
#define ZONE_RX_BUFFER_SIZE   40 //Размер буффера приемника зон поражения
volatile uint8_t zone3_rx_buffer[ZONE_RX_BUFFER_SIZE]; 	//Буффер ИК-приемника

void set_zone_buffer_bit (TDamageZone zone, uint8_t index, bool value){
	uint8_t byte_index;
	uint8_t bit_index;
	byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
	bit_index = index - (byte_index*8);//Определяем номер бита в байте


				switch(zone)
				{
					case zone_1:
					{
//						if(value) zone1_rx_buffer[byte_index]|= (1<<(7-bit_index));

//						else zone1_rx_buffer[byte_index] &= ~(1<<(7-bit_index));

					}
					break;
					case zone_2:
					{

//						if(value) zone2_rx_buffer[byte_index]|= (1<<(7-bit_index));

//						else zone2_rx_buffer[byte_index] &= ~(1<<(7-bit_index));
					}
					break;
					case zone_3:
					{

						if(value) zone3_rx_buffer[byte_index]|= (1<<(7-bit_index));

						else zone3_rx_buffer[byte_index] &= ~(1<<(7-bit_index));
					}
					break;
					case zone_4:
					{

//						if(value) zone4_rx_buffer[byte_index]|= (1<<(7-bit_index));

//						else zone4_rx_buffer[byte_index] &= ~(1<<(7-bit_index));
					}
					break;
				}

}



enum recStatus{
	REC_Idle,
	REC_Recording,
	REC_Captured
};
typedef enum recStatus TRecStatus;

volatile TRecStatus tim3_rec_state=REC_Idle;
volatile TRecStatus tim1_rec_state=REC_Idle;
volatile uint16_t tim3_ir_rx_count=0;
#define IR_RAW_BUFFER_SIZE 256 //размер буфера ИК-приемника
#define IR_START_BIT_DURATION 2400	// Длительность Старт-Бита (в микросекундах)
#define IR_ONE_BIT_DURATION 1200	// Длительность Бита, соотретствующего единичке (в микросекундах)
#define IR_ZERO_BIT_DURATION 600	// Длительность Бита, соотретствующего нулю (в микросекундах)
#define IR_SPACE_DURATION 600		// Длительность Бита, соотретствующего интервалу между битами (в микросекундах)

#define IR_TOL                      20 //Допустимая погрешность при приеме ИК пакета (в процентах


volatile trx_event Zone3RxEvent;//события 3-ей зоны

// этот дефайн просто сравнивает значения с заданной точностью в процентах
#define checkVal(var,val,tol) (var>(val*(100-tol)/100) && var<(val*(100+tol)/100))

void TIM16_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET) {
	  TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
	  TIM14->CCR1 = 0;
	  return;
  }

  if(TIM_GetITStatus(TIM16,  TIM_SR_UIF) != RESET){
     TIM_ClearITPendingBit(TIM16,  TIM_SR_UIF);
     if (ir_tx_buffer_cursor.bits_for_tx>0) //если указатель указывает не на пустую ячейку
     {
    	 TIM14->CCR1 = 50; // включаем несущую ИК
    	 if(!ir_tx_buffer_cursor.heder_been_sent)
    	 {
    	 	ir_tx_buffer_cursor.heder_been_sent = true;
    	 	return;
    	 }
    	 if(ir_tx_buffer_cursor.bit_mask == 0)//все биты текущего байта уже переданы
    	 {
    	 	ir_tx_buffer_cursor.byte_pos++;//переходим к следующему байту
    	 	ir_tx_buffer_cursor.bit_mask = (1<<7); //старший бит уходит первым
    	 }
    	 if (tx_buffer[ir_tx_buffer_cursor.byte_pos]&ir_tx_buffer_cursor.bit_mask)//если текущий бит равен "1"
    	 {
    		 //timer3.TIM_Period = IR_ONE_BIT_DURATION + IR_SPACE_DURATION;
			TIM16->ARR = IR_ONE_BIT_DURATION + IR_SPACE_DURATION;
			TIM16->CCR1 = IR_ONE_BIT_DURATION;
				//ir_pulse_counter = IR_ONE;//отправим "1" (помигаем 1200 микросекунд)
    	 }
		else //текущий бит равен "0"
		{
			//timer3.TIM_Period = IR_ZERO_BIT_DURATION + IR_SPACE_DURATION;
			TIM16->ARR = IR_ZERO_BIT_DURATION + IR_SPACE_DURATION;
			TIM16->CCR1 = IR_ZERO_BIT_DURATION;
				//				ir_pulse_counter = IR_ZERO;//отправим "0" (помигаем 600 микросекунд)
		}
		ir_tx_buffer_cursor.bit_mask = (ir_tx_buffer_cursor.bit_mask >> 1); //следующий бит
		ir_tx_buffer_cursor.bits_for_tx--; //уменьшаем количество оставшихся бит

     }//[if (ir_tx_buffer_cursor.bits_for_tx>0)]
     else //все биты передали
		{
			TIM14->CCR1 = 0; // включаем несущую ИК
			TIM_ClearFlag(TIM16, TIM_SR_UIF/*TIM_DIER_UIE*//*TIM_FLAG_CC1OF*/);
			TIM_Cmd(TIM16, DISABLE);//выключаем прерывание (передачу)
//			FLASH_LED_OFF;

		}




  }
}


void TIM3_IRQHandler(void)//3-я зона
{

	uint16_t cnt1, cnt2;
	        if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
	                TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	                TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	                cnt1 = TIM_GetCapture1(TIM3);
	                cnt2 = TIM_GetCapture2(TIM3);

	                if (tim3_rec_state == REC_Recording)
	                {

	                	if (tim3_ir_rx_count  < IR_RAW_BUFFER_SIZE) {
	                		if(tim3_ir_rx_count==0)//первым должен идти заголовок
	                		{
	                			if(checkVal(cnt2,IR_START_BIT_DURATION,IR_TOL))//проверяем заголовок на ошибку
	                			{
	                				USART1_PutChar('h');
	                				tim3_ir_rx_count++;
	                			}
	                			else //заголовок "битый"
	                			{
	                				// запрещаем прерывания по переполнению третьего канала
//	                				USART1_PutChar('e');
	                				TIM_ITConfig(TIM3, TIM_IT_CC3, /*ENABLE*/DISABLE);
	                				tim3_rec_state = REC_Idle;
	                				tim3_ir_rx_count=0;
								}
	                		}
	                		else //заголовок уже получен, идет прием данных
	                		{
//	                			TIM3_IR_RX_RAW_BUFFER[tim3_ir_rx_count-1].Period=cnt1;
//	                	        TIM3_IR_RX_RAW_BUFFER[tim3_ir_rx_count-1].Pulse=cnt2;

	                			if(checkVal(cnt2,IR_ONE_BIT_DURATION,IR_TOL)&&checkVal(cnt1,IR_ONE_BIT_DURATION+IR_SPACE_DURATION, IR_TOL))
	                			{
	                				USART1_PutChar('1');
	                				set_zone_buffer_bit(zone_3,tim3_ir_rx_count-1,true);//это единица
	                				tim3_ir_rx_count++;
	                			}
	                			else if (checkVal(cnt2, IR_ZERO_BIT_DURATION,IR_TOL)&&checkVal(cnt1,IR_ZERO_BIT_DURATION+IR_ZERO_BIT_DURATION,IR_TOL))
	                			{
	                				USART1_PutChar('0');
	                				set_zone_buffer_bit(zone_3,tim3_ir_rx_count-1,false);//это ноль
	                				tim3_ir_rx_count++;
	                			} else {//ошибка
	                				if(tim3_ir_rx_count!=0)USART1_PutChar('e');
	                				TIM_ITConfig(TIM3, TIM_IT_CC3, /*ENABLE*/DISABLE);
	                				tim3_rec_state = REC_Idle;
	                				tim3_ir_rx_count=0;
								}



	                		}
	                        }
	                	else {//слишком длинный пакет - ошибка
	                		 // запрещаем прерывания по переполнению третьего канала
	                		  	TIM_ITConfig(TIM3, TIM_IT_CC3, /*ENABLE*/DISABLE);
	                		   	tim3_rec_state = REC_Idle;
	                		   	tim3_ir_rx_count=0;
						}

	                }


	                if (tim3_rec_state == REC_Idle) {
	                        //Пришел первый фронт, начинаем запись
	                	//разрешаем прерывания по 3 каналу, предварительно очистив флаг
	                	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
	                	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	                	tim3_rec_state = REC_Recording;
	//                        captCount = 0;

	                }

	        }//[if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) ]

	        if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {//если таймаут по приёму
	                TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

	                if (tim3_rec_state == REC_Recording) {

		              //  if (tim3_ir_rx_count!=0)USART1_PutChar('t');

	                	if((tim3_ir_rx_count==14)||(tim3_ir_rx_count==24))
	                	{
	                		cnt2 = TIM_GetCapture2(TIM3);
	                		if(checkVal(cnt2,IR_ONE_BIT_DURATION,IR_TOL))
	                		{
	                			USART1_PutChar('1');
	                			set_zone_buffer_bit(zone_3,tim3_ir_rx_count-1,true);//это единица
	                			USART1_PutChar('t');
//	                			tim3_ir_rx_count++;
	                		}
	                		else if (checkVal(cnt2, IR_ZERO_BIT_DURATION,IR_TOL))
	                		{
	                			USART1_PutChar('0');
	                		set_zone_buffer_bit(zone_3,tim3_ir_rx_count-1,false);//это ноль
	                		USART1_PutChar('t');
//	                		tim3_ir_rx_count++;
	                		} else {//ошибка
	                	   				TIM_ITConfig(TIM3, TIM_IT_CC3, /*ENABLE*/DISABLE);
	                	   				tim3_rec_state = REC_Idle;
	                            		tim3_ir_rx_count=0;
	        	                		Zone3RxEvent=RX_ERROR;
//	        	                		xSemaphoreGiveFromISR( xZone3Semaphore, &xHigherPriorityTaskWoken);

	                				}

	                		tim3_rec_state = REC_Captured;
	                		if(tim3_ir_rx_count==14)Zone3RxEvent=RX_COMPLETE;
	                		else Zone3RxEvent=RX_MESSAGE_COMPLETE;
//	                		tim3_ir_rx_count=0;
//		                	xSemaphoreGiveFromISR( xZone3Semaphore, &xHigherPriorityTaskWoken );

	                	}
	                	else {//ошибка
	                		tim3_rec_state = REC_Idle;
	                		Zone3RxEvent=RX_ERROR;
//	                		xSemaphoreGiveFromISR( xZone3Semaphore, &xHigherPriorityTaskWoken );
						}

	                	// запрещаем прерывания по переполнению третьего канала
	                	TIM_ITConfig(TIM3, TIM_IT_CC3, /*ENABLE*/DISABLE);
	   //             	tim5_rec_state = REC_Idle;

	                	tim3_ir_rx_count=0;

	                	//xSemaphoreGiveFromISR( xZone4Semaphore, &xHigherPriorityTaskWoken );
	                }

	        }



}

/*
uint32_t getTrueRandomNumber(void) {

  ADC_InitTypeDef ADC_InitStructure;

  //enable ADC1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  // Initialize ADC 14MHz RC
  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
  RCC_HSI14Cmd(ENABLE);
  while (!RCC_GetFlagStatus(RCC_FLAG_HSI14RDY))
    ;

  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO; //default
  ADC_Init(ADC1, &ADC_InitStructure);

  //enable internal channel
  ADC_TempSensorCmd(ENABLE);

  // Enable ADCperipheral
  ADC_Cmd(ADC1, ENABLE);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET)
    ;

  ADC1->CHSELR = 0; //no channel selected
  //Convert the ADC1 temperature sensor, user shortest sample time to generate most noise
  ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor, ADC_SampleTime_1_5Cycles);

  // Enable CRC clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

  uint8_t i;
  for (i = 0; i < 8; i++) {
    //Start ADC1 Software Conversion
    ADC_StartOfConversion(ADC1);
    //wait for conversion complete
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
    }

    CRC_CalcCRC(ADC_GetConversionValue(ADC1));
    //clear EOC flag
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  }

  //disable ADC1 to save power
  ADC_Cmd(ADC1, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

  return CRC_CalcCRC(0xBADA55E5);
}
*/
static inline void wait_for_data_transfer_is_complete(void){
	while(USART1_GetTxCount()){};
	/* polling idle frame Transmission */
	  while(!(USART1->ISR & USART_ISR_TXE))
	  {

		  /* add time out here for a robust application */
	  }

	delay_mS(3);
	USART1_FlushRxBuf();
}

enum adc_source{
	batt,
	ref
};
typedef enum adc_source tadc_source;


battary_adc_init(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_StructInit(&port);
	port.GPIO_Mode =  GPIO_Mode_AN; //GPIO_Mode_AF;//GPIO_Mode_IN;
	port.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &port);


	  ADC_InitTypeDef ADC_InitStructure;

	  //enable ADC1 clock
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	  // Initialize ADC 14MHz RC
	  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
	  RCC_HSI14Cmd(ENABLE);
	  while (!RCC_GetFlagStatus(RCC_FLAG_HSI14RDY));

	  ADC_DeInit(ADC1);
	  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO; //default
	  ADC_Init(ADC1, &ADC_InitStructure);
	  /* ADC Calibration */
	  	  ADC_GetCalibrationFactor(ADC1);
	  	  asm volatile("nop; nop;");
	  // Enable ADCperipheral
	  ADC_Cmd(ADC1, ENABLE);
	  /*
	  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET)
	    ;
	 */
	  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)){

	  }

//	  ADC1->CHSELR = 0; //no channel selected
	   //Convert the ADC1 temperature sensor, user shortest sample time to generate most noise




}

uint16_t get_adc_value(tadc_source source){
	uint16_t result;

	if(source==batt)   ADC_ChannelConfig(ADC1, ADC_Channel_7, ADC_SampleTime_1_5Cycles);
	else ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint, ADC_SampleTime_239_5Cycles /*ADC_SampleTime_1_5Cycles*/);

	//Start ADC1 Software Conversion
	    ADC_StartOfConversion(ADC1);
	    //wait for conversion complete
	    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
	    }

	    result = ADC_GetConversionValue(ADC1);
	    //clear EOC flag
	    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	    //disable ADC1 to save power
//	     ADC_Cmd(ADC1, DISABLE);
//	     RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
	    return result;
}
/*
uint16_t get_vref_adc_value(void){
	uint16_t result;
	   ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint, ADC_SampleTime_1_5Cycles);

		//Start ADC1 Software Conversion
		    ADC_StartOfConversion(ADC1);
		    //wait for conversion complete
		    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
		    }

		    result = ADC_GetConversionValue(ADC1);
		    //clear EOC flag
		    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		    return result;
}
*/

#define RANDOM_MIN (1)
#define RANDOM_MAX (7)

volatile uint32_t random_num;



#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_PIN GPIO_Pin_0

#define BUZZER_RCC_AHBPeriphClockCmd RCC_AHBPeriphClockCmd
#define BUZZER_RCC_AHBPeriph RCC_AHBPeriph_GPIOA



void buzzerInit(void){
	BUZZER_RCC_AHBPeriphClockCmd(BUZZER_RCC_AHBPeriph,ENABLE);
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_OUT;
	port.GPIO_Pin = BUZZER_PIN;
	port.GPIO_OType=GPIO_OType_PP;
	port.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init(BUZZER_GPIO_PORT, &port);
}


void buzzer_on(void){

	GPIO_SetBits(BUZZER_GPIO_PORT,BUZZER_PIN);

}

void buzzer_off(void){
	GPIO_ResetBits(BUZZER_GPIO_PORT,BUZZER_PIN);

}





#define VIBRO_GPIO_PORT GPIOA
#define VIBRO_PIN GPIO_Pin_10

#define VIBRO_RCC_AHBPeriphClockCmd RCC_AHBPeriphClockCmd
#define VIBRO_RCC_AHBPeriph RCC_AHBPeriph_GPIOA

void vibroInit(void){
	BUZZER_RCC_AHBPeriphClockCmd(VIBRO_RCC_AHBPeriph,ENABLE);
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_OUT;
	port.GPIO_Pin = VIBRO_PIN;
	port.GPIO_OType=GPIO_OType_PP;
	port.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init(VIBRO_GPIO_PORT, &port);
}


void vibro_on(void){

	GPIO_SetBits(VIBRO_GPIO_PORT,VIBRO_PIN);

}

void vibro_off(void){
	GPIO_ResetBits(VIBRO_GPIO_PORT,VIBRO_PIN);

}


#define BLUETOOTH_GPIO_PORT GPIOA

#define BLUETOOTH_RESET_PIN GPIO_Pin_1
#define BLUETOOTH_STATE_PIN GPIO_Pin_4
#define BLUETOOTH_AT_MODE_PIN GPIO_Pin_5
#define BLUETOOTH_RCC_AHBPeriphClockCmd RCC_AHBPeriphClockCmd
#define BLUETOOTH_RCC_AHBPeriph RCC_AHBPeriph_GPIOA

void bluetooth_gpio_init (void){

	BLUETOOTH_RCC_AHBPeriphClockCmd(BLUETOOTH_RCC_AHBPeriph, ENABLE);
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_OUT;
	port.GPIO_Pin = BLUETOOTH_RESET_PIN| BLUETOOTH_AT_MODE_PIN;
	port.GPIO_OType=GPIO_OType_PP;
	port.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init( BLUETOOTH_GPIO_PORT, &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_IN;
	port.GPIO_Pin = BLUETOOTH_STATE_PIN;
	port.GPIO_PuPd= GPIO_PuPd_DOWN;
	port.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init( BLUETOOTH_GPIO_PORT, &port);

}


void bluetooth_reset_low(void){
	GPIO_ResetBits(BLUETOOTH_GPIO_PORT, BLUETOOTH_RESET_PIN);

}


void bluetooth_reset_high(void){
	GPIO_SetBits(BLUETOOTH_GPIO_PORT, BLUETOOTH_RESET_PIN);

}


void bluetooth_at_mode_low(void){
	GPIO_ResetBits(BLUETOOTH_GPIO_PORT, BLUETOOTH_AT_MODE_PIN);

}


void bluetooth_at_mode_high(void){
	GPIO_SetBits(BLUETOOTH_GPIO_PORT, BLUETOOTH_AT_MODE_PIN);

}


void bt_set_at_commands_mode(bool mode)//переводим блютус модуль в режим at-команд
{
	if (mode)bluetooth_at_mode_high();
	else bluetooth_at_mode_low();
}


void bt_reset(void)//аппаратный сброс блютус-модуля
{
	bluetooth_reset_low();
	delay_mS(500);
	bluetooth_reset_high();
	delay_mS(500);
}


void bt_init(void){
	bluetooth_gpio_init();
	bt_set_at_commands_mode(false);
	bt_reset();
}


bool bt_connect_status(void){

	return GPIO_ReadInputDataBit(BLUETOOTH_GPIO_PORT, BLUETOOTH_STATE_PIN);
}

extern volatile unsigned char usart1_rxBufTail;
extern volatile unsigned char usart1_rxBufHead;
extern volatile unsigned char usart1_RxBuf[];

bool send_test_at_command(void){
	USART1_FlushTxBuf(); //очистить передающий буфер
	USART1_FlushRxBuf(); //очистить приемный буфер
	cr_received = false;
	USART1_SendStr("AT");
	USART1_SendStr("\r\n");

	countdown_timer=1000;//будем ждать ответа 1 секунду
	while ((cr_received==false)&&(countdown_timer));
	if(countdown_timer==0) return false;
	if (memmem(&usart1_RxBuf[usart1_rxBufHead],USART1_GetRxCount(),"OK",2)!='\0'/*NULL*/)
		{
			return true;
		}
	else
		{
			return false;
		}


}

bool send_set_at_command(char* cmd, char* param){
	USART1_FlushTxBuf(); //очистить передающий буфер
	USART1_FlushRxBuf(); //очистить приемный буфер
	cr_received = false;
	USART1_SendStr(cmd);
	USART1_SendStr("=");
	USART1_SendStr(param);
	USART1_SendStr("\r\n");
	countdown_timer=1000;//будем ждать ответа 1 секунду
	while ((cr_received==false)&&(countdown_timer));
	if(countdown_timer==0) return false;
	if (memmem(&usart1_RxBuf[usart1_rxBufHead],USART1_GetRxCount(),"OK",2)!='\0'/*NULL*/)
		{
			return true;
		}
	else
		{
			return false;
		}


}
bool send_at_command(char* cmd){
	USART1_FlushTxBuf(); //очистить передающий буфер
	USART1_FlushRxBuf(); //очистить приемный буфер
	cr_received = false;
	USART1_SendStr(cmd);
	USART1_SendStr("\r\n");
	countdown_timer=1000;//будем ждать ответа 1 секунду
	while ((cr_received==false)&&(countdown_timer));
	if(countdown_timer==0) return false;
	if (memmem(&usart1_RxBuf[usart1_rxBufHead],USART1_GetRxCount(),"OK",2)!='\0'/*NULL*/)
		{
			return true;
		}
	else
		{
			return false;
		}


}

#define IR_CONTROL_ENABLE

const unsigned char at_rmaad[] = "AT+RMAAD";
const unsigned char at_name[] = "AT+NAME";
const unsigned char at_bind[] = "AT+BIND";
const unsigned char at_bind_param[] = "AT+BIND";
const unsigned char at_uart[] = "AT+UART";
const unsigned char at_role[] = "AT+ROLE";
#ifdef IR_CONTROL_ENABLE
const unsigned char at_role_param[] = "0";//0-Slave; 1-Master
#else
const unsigned char at_role_param[] = "1";//0-Slave; 1-Master
#endif
const unsigned char at_cmode[] = "AT+CMODE";
const unsigned char at_cmode_param[] = "1";//0 - connect fixed address, 1 - connect any address, 2 - slave-Loop
const unsigned char at_pswd[] = "AT+PSWD";
const unsigned char at_pswd_param[] = "1234";

bool configure_bluetooth(void)//настраиваем блютус модуль
{
	volatile bool at_res;
	bluetooth_gpio_init();
//	USART_DeInit(USART1);
	InitCOM1_FullDuplex(38400);
//	 InitCOM1_FullDuplex(9600);
	bt_set_at_commands_mode(true);
	delay_mS(500);
	bt_reset();
	delay_mS(500);
	at_res = send_test_at_command();
	if(!at_res){
		bt_set_at_commands_mode(false);
		delay_mS(500);
		bt_reset();
		delay_mS(500);
		DeInitCOM1();
		InitCOM1_FullDuplex(9600);
		return false;
	}
//	at_res = send_test_at_command();

#ifndef  	IR_CONTROL_ENABLE
	at_res = send_at_command((char*)at_rmaad);
	if(!at_res){
			bt_set_at_commands_mode(false);
			delay_mS(500);
			bt_reset();
			delay_mS(500);
			DeInitCOM1();
			InitCOM1_FullDuplex(9600);
			return false;
		}
#endif
	at_res = send_set_at_command((char*)at_role,(char*)at_role_param);
	if(!at_res){
		bt_set_at_commands_mode(false);
		delay_mS(500);
		bt_reset();
		delay_mS(500);
		DeInitCOM1();
		InitCOM1_FullDuplex(9600);
		return false;
	}
	at_res = send_set_at_command((char*)at_cmode,(char*)at_cmode_param);
	if(!at_res){
		bt_set_at_commands_mode(false);
		delay_mS(500);
		bt_reset();
		delay_mS(500);
		DeInitCOM1();
		InitCOM1_FullDuplex(9600);
		return false;
	}

/*
	at_res = send_set_at_command((char*)at_pswd,(char*)at_pswd_param);
	if(!at_res){
		bt_set_at_commands_mode(false);
		delay_mS(500);
		bt_reset();
		delay_mS(500);
		DeInitCOM1();
		InitCOM1_FullDuplex(9600);
		return false;
	}
*/


	bt_set_at_commands_mode(false);
	delay_mS(500);
	bt_reset();
	delay_mS(500);
	DeInitCOM1();
	InitCOM1_FullDuplex(9600);
return true;

}


int main(void)
{
volatile uint32_t bt_is_configure_flag;
	/*
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode =GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	init_pwm_TIM3();
	*/
//	unsigned char char_tmp;
	init_SysTic();
//	init_pwm_TIM1();
	Tim14_Conf();
	init_pwm_TIM3();
	ws2812b_Init();
	Tim16_Conf();
//	InitCOM1(9600);
//	InitCOM1_FullDuplex(9600);
//	 InitCOM1_FullDuplex(38400);
//	bt_init();
	configure_bluetooth();

	/*
	bt_is_configure_flag = FLASH_Read(1024*16-4);
	if (bt_is_configure_flag!=0x05050505){
		if (configure_bluetooth()) {
			bt_is_configure_flag=0x05050505;
			FLASH_unlock();
			FLASH_write((uint32_t)(1024*16-4),(uint32_t*)&bt_is_configure_flag,1);
			FLASH_lock();
			bt_is_configure_flag = FLASH_Read(1024*16-4);
			}
		}

	else InitCOM1(9600);
*/

	buzzerInit();
	vibroInit();
	battary_adc_init();
	for(uint8_t i=0;i<NUM_LEDS; i++)
	{
	leds[i].r=(uint8_t)0;
	leds[i].g=(uint8_t)0;
	leds[i].b=(uint8_t)0;
	}


	set_player_id(45);
	set_team_color(Red);
	set_gun_damage(Damage_4);
/*
	hsv_leds[0].h =0;
	hsv_leds[0].s =100;
	hsv_leds[0].v =100;
*/
//	tx_buffer[1]=1;
//	USART_SendData(USART1, 'I');//записываем в UDR символ из буфера

	while(1)
    {


		while(! bt_connect_status()){

			leds[0].r=(uint8_t)255;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[1].g=(uint8_t)255;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[2].b=(uint8_t)255;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[3].r=(uint8_t)255;
			leds[3].g=(uint8_t)255;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(500);
			leds[0].r=(uint8_t)0;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[1].g=(uint8_t)0;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[2].b=(uint8_t)0;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			delay_mS(250);
			leds[3].r=(uint8_t)0;
			leds[3].g=(uint8_t)0;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);

			buzzer_on();
			delay_mS(50);
			buzzer_off();
//			delay_mS(500);

		}
		if (!bluetooth_connection_was_already){
			USART1_PutChar('f');
			bluetooth_connection_was_already=true;
		}




/////		while (!ws2812b_IsReady()); // wait
/////		random_num = RANDOM_MIN + /*getTrueRandomNumber()*/rand()%RANDOM_MAX;
//		rand();
/*
		if(leds[0].r<255)leds[0].r=leds[0].r+(uint8_t)1;
		else leds[0].r=(uint8_t)0;
		if(hsv_leds[0].v<100)hsv_leds[0].v=hsv_leds[0].v+(uint8_t)1;
				else hsv_leds[0].v=(uint8_t)0;
*/
//		delay_mS(500);
//		send_ir_shot_package();





//		USART1_PutChar('O');
//	    USART_HalfDuplexCmd(USART1,DISABLE);


		/*
		USART1_SendStr("Hello!\r\n");
		USART1_PutChar('\r');
		USART1_PutChar('\n');
		wait_for_data_transfer_is_complete();
*/

//	    USART_HalfDuplexCmd(USART1,ENABLE);
//		USART_SendData(USART1, 'S');//записываем в UDR символ из буфера
/////		tx_buffer[0]++;
//		send_ir_shot_package();
/////		delay_mS(30);
		            //
		            // Fill leds buffer
           //
		if (leds_off_flag){
			leds[0].r=0;
			leds[0].g=0;
			leds[0].b=0;
			leds[1].r=0;
			leds[1].g=0;
			leds[1].b=0;
			leds[2].r=0;
			leds[2].g=0;
			leds[2].b=0;
			leds[3].r=0;
			leds[3].g=0;
			leds[3].b=0;
			while (!ws2812b_IsReady()); // wait
			ws2812b_SendRGB(leds, NUM_LEDS);
			leds_off_flag=false;

		}
		if (vibro_off_flag){
			vibro_off();
			vibro_off_flag=false;
		}
		while ( USART1_GetRxCount())//пока не считаем все символы из буфера
			{
			//	char_tmp == USART1_GetChar();
				//int i;
			//	i++;
				static volatile bool bt_header_receive = false;
				static volatile uint8_t bt_bits_receive=0;
				if (bt_header_receive){
					switch (USART1_GetChar()){
					case '1': set_bt_buffer_bit(bt_bits_receive++, true);
					break;
					case '0': set_bt_buffer_bit(bt_bits_receive++, false);
					break;
					case 't': {//now send package

						ir_tx_buffer_cursor.byte_pos = 0;
						ir_tx_buffer_cursor.bit_mask = (1<<7);
						ir_tx_buffer_cursor.bits_for_tx=bt_bits_receive;//"выстрел" состоит из 14 бит
						ir_tx_buffer_cursor.heder_been_sent = false;
						TIM_DeInit(TIM16);//сбрасываем настройки таймера на дефолтовые
						//	TIM4->CCR1 = 50;
						Tim16_Conf();

						bt_header_receive = false;
						bt_bits_receive=0;

					}
					break;
					case 'h': {
						bt_bits_receive=0;
					}
					break;
					default:{//error
						bt_header_receive = false;
						bt_bits_receive=0;
					}
					break;
					//continue;
				}//[switch (USART1_GetChar())]
			}//[if (bt_header_receive)]

				else{
				switch (USART1_GetChar()){
				case 'h':{
					bt_header_receive = true;
					bt_bits_receive=0;
				}
				break;
				case '1':{
					leds[0].r=(uint8_t)255;
					leds[0].g=0;
					leds[0].b=0;
					leds[1].r=(uint8_t)255;
					leds[1].g=0;
					leds[1].b=0;
					leds[2].r=(uint8_t)255;
					leds[2].g=0;
					leds[2].b=0;
					leds[3].r=(uint8_t)255;
					leds[3].g=0;
					leds[3].b=0;
					while (!ws2812b_IsReady()); // wait
					ws2812b_SendRGB(leds, NUM_LEDS);
					leds_on_counter+=ASCET_LEDS_ON_TIME;
				}
				break;
				case 'r':{
					leds[0].r=(uint8_t)255;
					leds[0].g=0;
					leds[0].b=0;
					leds[1].r=(uint8_t)255;
					leds[1].g=0;
					leds[1].b=0;
					leds[2].r=(uint8_t)255;
					leds[2].g=0;
					leds[2].b=0;
					leds[3].r=(uint8_t)255;
					leds[3].g=0;
					leds[3].b=0;
					while (!ws2812b_IsReady()); // wait
					ws2812b_SendRGB(leds, NUM_LEDS);
					leds_on_counter=LEDS_ON_TIME;
				}
				break;
				case 'g':{
					leds[0].r=0;
					leds[0].g=(uint8_t)255;
					leds[0].b=0;
					leds[1].r=0;
					leds[1].g=(uint8_t)255;
					leds[1].b=0;
					leds[2].r=0;
					leds[2].g=(uint8_t)255;
					leds[2].b=0;
					leds[3].r=0;
					leds[3].g=(uint8_t)255;
					leds[3].b=0;
					while (!ws2812b_IsReady()); // wait
					ws2812b_SendRGB(leds, NUM_LEDS);
					leds_on_counter=LEDS_ON_TIME;
				}
				break;
				case 'b':{
					leds[0].r=0;
					leds[0].g=0;
					leds[0].b=(uint8_t)255;
					leds[1].r=0;
					leds[1].g=0;
					leds[1].b=(uint8_t)255;
					leds[2].r=0;
					leds[2].g=0;
					leds[2].b=(uint8_t)255;
					leds[3].r=0;
					leds[3].g=0;
					leds[3].b=(uint8_t)255;
					while (!ws2812b_IsReady()); // wait
					ws2812b_SendRGB(leds, NUM_LEDS);
					leds_on_counter=LEDS_ON_TIME;

				}
				break;
				case 'y':{
					leds[0].r=(uint8_t)255;
					leds[0].g=(uint8_t)255;
					leds[0].b=0;
					leds[1].r=(uint8_t)255;
					leds[1].g=(uint8_t)255;
					leds[1].b=0;
					leds[2].r=(uint8_t)255;
					leds[2].g=(uint8_t)255;
					leds[2].b=0;
					leds[3].r=(uint8_t)255;
					leds[3].g=(uint8_t)255;
					leds[3].b=0;
					while (!ws2812b_IsReady()); // wait
					ws2812b_SendRGB(leds, NUM_LEDS);
					leds_on_counter=LEDS_ON_TIME;
				}
				break;
				case 'v':{
					vibro_on();
					vibro_on_counter=VIBRO_ON_TIME;
				}
				break;
				default: break;

				}
			}//else

		}
		switch(Zone3RxEvent){
						case RX_COMPLETE:
							{
				                tim3_rec_state = REC_Idle;//разрешаем прием на 3-ей зоне
/*
				                leds[0].r=255;
								leds[1].r=255;
								leds[2].r=255;
								leds[3].r=255;
								buzzer_on();
								vibro_on();
								while (!ws2812b_IsReady()); // wait
								ws2812b_SendRGB(leds, NUM_LEDS);
//								delay_mS(20);
								leds[0].r=0;
								leds[1].r=0;
								leds[2].r=0;
								leds[3].r=0;
								while (!ws2812b_IsReady()); // wait
								ws2812b_SendRGB(leds, NUM_LEDS);
								buzzer_off();
								vibro_off();
*/
								Zone3RxEvent = NOT_EVENT;//сбрасываем событие

							}
						break;
						case RX_MESSAGE_COMPLETE:
							{
/*
								leds[0].g=255;
								leds[1].g=255;
								leds[2].g=255;
								leds[3].g=255;
								while (!ws2812b_IsReady()); // wait
								ws2812b_SendRGB(leds, NUM_LEDS);
								delay_mS(1000);
								leds[0].g=0;
								leds[1].g=0;
								leds[2].g=0;
								leds[3].g=0;
								while (!ws2812b_IsReady()); // wait
								ws2812b_SendRGB(leds, NUM_LEDS);
*/
								Zone3RxEvent = NOT_EVENT;//сбрасываем событие
				                tim3_rec_state = REC_Idle;//разрешаем прием на 3-ей зоне
							}
						break;
						case RX_ERROR:
							{
				                Zone3RxEvent = NOT_EVENT;//сбрасываем событие
				 //               tim3_rec_state = REC_Idle;//разрешаем прием на 3-ей зоне
							}
						break;
						case NOT_EVENT:
							{

							}
						break;
						default: break;




						}



/*
		ws2812b_SendRGB(leds, NUM_LEDS);

		leds[0].r+=10;
		leds[1].r-=10;
	//	int i;
		for(int i=2;i<9;i++)
		{
		leds[i].r = rand()%127;
		leds[i].g = rand()%127;
		leds[i].b = rand()%127;
		}

*/
//		   ws2812b_SendHSV(hsv_leds, NUM_LEDS);
 /*
    	GPIO_SetBits(GPIOA, GPIO_Pin_4);
    	i++;
    	i=i+2;
    	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    	i--;
*/

		volatile uint16_t batt_adc_val,vref_adc_val,voltage;
if(adc_test_now_flag)
		{
				adc_test_now_flag = false;
//				battary_adc_init();
				batt_adc_val=get_adc_value(batt);
				//если батарея разрядилась ниже 3400 мВ, посылаем символ 'L'
				if(batt_adc_val < 2110) USART1_PutChar('L');
//				battary_adc_init();
//				vref_adc_val = get_adc_value(ref);
		//		voltage = (2400*(uint16_t)batt_adc_val)/vref_adc_val;
////				voltage = (6600*(uint16_t)batt_adc_val)/4095;
////				USART1_SendStr("batt_adc_val: ");
////				USART1_SendStr(long_int_to_str(batt_adc_val,0));
//				wait_for_data_transfer_is_complete();
////				USART1_PutChar('\n');
////				USART1_PutChar('\r');
//				USART1_SendStr("vref_adc_val: ");
//				USART1_SendStr(long_int_to_str(vref_adc_val,0));
//				wait_for_data_transfer_is_complete();
//				USART1_PutChar('\n');
//				USART1_PutChar('\r');
////				USART1_SendStr("voltage: ");
////				USART1_SendStr(long_int_to_str(voltage,0));
////				wait_for_data_transfer_is_complete();
////				USART1_PutChar('\n');
////				USART1_PutChar('\r');
				//delay_mS(3000);
		}

    }
}


