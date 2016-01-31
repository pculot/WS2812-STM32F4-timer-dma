#include <math.h>
#include "stm32f4xx.h"

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

#define d2r (3.14159265/180)

// Use update signal
// PAP: This is better than the CC1 signal; using the CC1 signal causes the timing to shift
#define PWM_TIMER	TIM3
#define DMA_STREAM	DMA1_Stream2
#define DMA_TCIF	DMA_FLAG_TCIF2
#define DMA_CHANNEL	DMA_Channel_5
#define DMA_SOURCE	TIM_DMA_Update


#define TIM_PERIOD			(25)
#define TIM_COMPARE_LOGIC_1	(14)
#define TIM_COMPARE_LOGIC_0	(7)


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

/* Buffer that holds one complete DMA transmission
 *
 * Ensure that this buffer is big enough to hold
 * all data bytes that need to be sent
 *
 * The buffer size can be calculated as follows:
 * number of LEDs * 24 bytes + 42 bytes
 *
 * This leaves us with a maximum string length of
 * (2^16 bytes per DMA stream - 42 bytes)/24 bytes per LED = 2728 LEDs
 */
uint16_t LED_BYTE_Buffer[24*24]={TIM_COMPARE_LOGIC_0, TIM_COMPARE_LOGIC_0, TIM_COMPARE_LOGIC_0, TIM_COMPARE_LOGIC_0, TIM_COMPARE_LOGIC_1, TIM_COMPARE_LOGIC_1, TIM_COMPARE_LOGIC_1, TIM_COMPARE_LOGIC_1, 0, 0};

void Timer3_init(void)
{
	uint16_t PrescalerValue;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* GPIOB Configuration: PWM_TIMER Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Compute the prescaler value */
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 20000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD; // 800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
	TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//TIM_COMPARE_LOGIC_1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(PWM_TIMER, &TIM_OCInitStructure);

	/***
	 * Must enable reload for PWM (STMicroelectronicd RM0090 section 18.3.9
	 * "PWM mode":
	 *
	 *   You must enable the corresponding preload register by setting the
	 *   OCxPE bit in the TIMx_CCMRx register.
	 *
	 * This is part of the fix for the pulse corruption (the runt pulse at
	 * the start and the extra pulse at the end).
	 */
	TIM_OC1PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);


	/* configure DMA */
	/* DMA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* DMA1 Channel6 Config */
	DMA_DeInit(DMA_STREAM);

	DMA_InitStructure.DMA_BufferSize = 24*24;//42;
	DMA_InitStructure.DMA_Channel = DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// data shifted from memory to peripheral
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&LED_BYTE_Buffer;		// this is the buffer memory
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// stop DMA feed after buffer size is reached

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&PWM_TIMER->CCR1;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

	DMA_Init(DMA_STREAM, &DMA_InitStructure);
	/* PWM_TIMER CC1 DMA Request enable */
		TIM_DMACmd(PWM_TIMER, DMA_SOURCE, ENABLE);

	DMA_Cmd(DMA_STREAM, ENABLE);


}

/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 *
 * This will result in the RGB triplet passed by argument 1 being sent to
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 */
void WS2812_send(const uint8_t (*color)[3], const uint16_t _len)
{
	int i, j;
	uint8_t led;
	uint16_t memaddr;
	uint16_t buffersize;
	uint16_t len = _len;

	// Byte order mapping. 0 is red, 1 is green, 2 is blue
	const uint8_t pix_map[3] = {0, 2, 1};

	buffersize = (len*24);	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	led = 0;					// reset led index

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 8; j++)					// GREEN data
			{
				if ( (color[led][pix_map[i]]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOGIC_1;	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOGIC_0;		// compare value for logical 0
				}
				memaddr++;
			}
		}

		led++;
		len--;
	}

	LED_BYTE_Buffer[memaddr++] = 0;
	LED_BYTE_Buffer[memaddr++] = 0;
	// add needed delay at end of byte cycle, pulsewidth = 0
//	while(memaddr < buffersize)
//	{
//		LED_BYTE_Buffer[memaddr] = 0;
//		memaddr++;
//	}

	DMA_SetCurrDataCounter(DMA_STREAM, buffersize + 2); 	// load number of bytes to be transferred

	// PAP: Clear the timer's counter and set the compare value to 0. This
	// sets the output low on start and gives us a full cycle to set up DMA.
	TIM_SetCounter(PWM_TIMER, 0);
	TIM_SetCompare1(PWM_TIMER, 0);
	TIM_Cmd(PWM_TIMER, ENABLE); 						// enable Timer 3

	// PAP: Start DMA transfer after starting the timer. This prevents the
	// DMA/PWM from dropping the first bit.
	DMA_Cmd(DMA_STREAM, ENABLE); 			// enable DMA channel 6
	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
	TIM_Cmd(PWM_TIMER, DISABLE); 					// disable Timer 3
	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag
}


/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 *
 * This will result in the RGB triplet passed by argument 1 being sent to
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 */
void WS2812_send2(const uint8_t (*color)[3], const uint16_t _len)
{
	int i, j;
	uint8_t led;
	uint16_t memaddr;
	uint16_t buffersize;
	uint16_t len = _len;

	// Byte order mapping. 0 is red, 1 is green, 2 is blue
	const uint8_t pix_map[3] = {0, 2, 1};

	buffersize = (len*24);	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	led = 0;					// reset led index

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 8; j++)					// GREEN data
			{
				if ( (color[led][pix_map[i]]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOGIC_1;	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOGIC_0;		// compare value for logical 0
				}
				memaddr++;
			}
		}

		led++;
		len--;
	}

	LED_BYTE_Buffer[memaddr++] = 0;
	LED_BYTE_Buffer[memaddr++] = 0;


	DMA_SetCurrDataCounter(DMA_STREAM, buffersize + 2); 	// load number of bytes to be transferred

	// PAP: Clear the timer's counter and set the compare value to 0. This
	// sets the output low on start and gives us a full cycle to set up DMA.
	TIM_SetCounter(PWM_TIMER, 0);
	TIM_SetCompare1(PWM_TIMER, 0);
	TIM_Cmd(PWM_TIMER, ENABLE); 						// enable Timer 3

	// PAP: Start DMA transfer after starting the timer. This prevents the
	// DMA/PWM from dropping the first bit.
	DMA_Cmd(DMA_STREAM, ENABLE); 			// enable DMA channel 6
	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
	TIM_Cmd(PWM_TIMER, DISABLE); 					// disable Timer 3
	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag
}

void myWS2812(void)
{
	DMA_SetCurrDataCounter(DMA_STREAM, 10); 	// load number of bytes to be transferred

	TIM_SetCounter(PWM_TIMER, 0);
	TIM_SetCompare1(PWM_TIMER, 0);
	TIM_Cmd(PWM_TIMER, ENABLE); 						// enable Timer 3

	DMA_Cmd(DMA_STREAM, ENABLE); 			// enable DMA channel 6

	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
	TIM_Cmd(PWM_TIMER, DISABLE); 					// disable Timer 3
	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6


	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag
}
/**
 *
 * @return
 */
int main(void) {
	int i, j=0;
	uint8_t colourbuf[24][3];//={255,0,127,255,0,127};
	Timer3_init();

	while(1)
	{
		for (i = 0; i < 24; i++)
		{
			colourbuf[i][0] = 0;	//R

			if (i == j)				//G
				colourbuf[i][1] = 255;
			else
				colourbuf[i][1] = 0;

			colourbuf[i][2] = 0;	//B
		}
		WS2812_send2(/*(const uint8_t(*)[]) &*/colourbuf, 24);
		Delay(2000000L);

		if (++j > 24)
			j = 0;
	}

	while (1)
	{
		myWS2812();
		Delay(200000);

	}
}
