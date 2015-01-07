#include "stm32l1xx.h"
#include "discover_board.h"
#include <math.h>
#define SAMPLES 3

volatile uint16_t ADC_ConvertedValues[SAMPLES];
double thermistor(int RawADC);

void init_gpio(void);
void init_adc(void);
void init_dma(void);
void init_nvic(void);
void init_rcc(void);

int main(void) {
	init_gpio();
	init_nvic();
	init_rcc();
	init_adc();
	init_dma();

	/* Infinite loop */
	while (1)
		;
}

//******************************************************************************

void DMA1_Channel1_IRQHandler(void) {
	/* Test on DMA Channel Half Transfer interrupt */
	if (DMA_GetITStatus(DMA1_IT_HT1)) {
		/* Clear DMA Channel Half Transfer interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_IT_HT1);
		//you can do additional stuff here if you want to check on the halftime conversion
	}

	/* Test on DMA Channel Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC1)) {
		/* Clear DMA Channel Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_IT_TC1);

		//read the first value that is the light sensor
		uint16_t lightSensorValue = ADC_ConvertedValues[0];
		if (lightSensorValue < 150) {
			//when it gets dark, put led on
			GPIO_HIGH(LD_PORT, LD_GREEN);
		} else {
			//when it gets light put led off
			GPIO_LOW(LD_PORT, LD_GREEN);
		}

		//read the second value that is the moisture sensor
		uint16_t value2 = ADC_ConvertedValues[1];
		if (value2 < 100) {
			//put led on for water
			GPIO_HIGH(LD_PORT, LD_BLUE);
		} else {
			//put led off for water
			GPIO_LOW(LD_PORT, LD_BLUE);
		}

		//read the third value that is the thermistor
		uint16_t value3 = ADC_ConvertedValues[2];
		double temp = thermistor(value3);
		if (temp < 26) {
			//put led on for heater
			GPIO_HIGH(GPIOB, GPIO_Pin_8);
		} else {
			//put led off for heater
			GPIO_LOW(LD_PORT, GPIO_Pin_8);
		}
	}
}

void init_gpio(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	//enables clock for gpio
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

	/* Configure PB6 (BLUE LED) and PB7 (yellow LED) and PB8(red) for own led */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//configure pin PA1 PA2 PA3 for analog inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
*This function initialises the ADC
*/
void init_adc(void) {
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Deinitialize ADC */
	ADC_DeInit(ADC1);

	ADC_CommonStructInit(&ADC_CommonInitStructure);
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b; // the analog value will be 8 bits (0 - 255)
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Single channel
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Continuously back-to-back
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = SAMPLES; //this is the number of conversions that are made
	ADC_Init(ADC1, &ADC_InitStructure);

	//configures the 3 adc channels that we use
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_192Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_192Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_192Cycles);

	/* Enable ADC1 Power Down during Delay */
	ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);
}

void init_dma(void) {
	DMA_InitTypeDef DMA_InitStructure;

	/* DMA1 channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = SAMPLES; // tells the dma how many values should be generated
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //keeps converting the analog values
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC_ConvertedValues; //puts the values in the array
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA Stream Half / Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);

	/* Enable DMA1 channel1 + channel2*/
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until ADC1 ON status */
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
		;

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);

}

void init_nvic(void) {
	//initialises the interrupts channel
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; //listen to interrupts from the dma
	NVIC_Init(&NVIC_InitStructure);
}

void init_rcc(void) {
	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

//read the temperature with a thermistor
double thermistor(int RawADC) {
	double temp;
	temp = log(10000.0 * ((1024.0 / RawADC - 1)));
	temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp)) * temp);
	temp = temp - 190.15; // Convert Kelvin to Celcius
	return temp;
}
