#include <ctype.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"

#include "string.h"

//#include "cbor.h"
#include "Mpu6050.h"


extern "C" {
#include "printf.h"
#include "systimer.h"
}



uint64_t get_current_ms() {

	return mstimer_get();
}

#define USE_RADIO_

void init() {
	mstimer_init();
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);

	GPIO_InitTypeDef gpioStructureButton;
	gpioStructureButton.GPIO_Pin = GPIO_Pin_0;
	gpioStructureButton.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioStructureButton.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioStructureButton);

	GPIO_InitTypeDef gpioStructure1;
	gpioStructure1.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_10;
	gpioStructure1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioStructure1);

	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7
			| GPIO_Pin_9;
	gpioStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioStructure);

	GPIO_InitTypeDef gpioStructure2;
	gpioStructure2.GPIO_Pin = GPIO_Pin_4;
	gpioStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioStructure2);

	//USART
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

//
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
//	EXTI_InitTypeDef EXTI_InitStructure;
//
//	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

}

typedef enum{
	X_POS,
	Y_POS,
	Z_POS,

	X_NEG,
	Y_NEG,
	Z_NEG,

	NONE


}Orientation;


char* getOrientation(uint8_t orientation){

	if (orientation == X_POS) return "X_POS";
	if (orientation ==Y_POS)return "Y_POS";
	if (orientation ==Z_POS)return "Z_POS";
	if (orientation ==X_NEG)return "X_NEG";
	if (orientation ==Y_NEG)return "Y_NEG";
	if (orientation ==Z_NEG)return "Z_NEG";
	if (orientation ==NONE)return "NONE";



}



int main() {
	init();

	Mpu6050 mpu;
	mpu.init();
	mpu.calibrate();

	if (!mpu.test()){
		printf_("mpu has problem\n");

	}
	float sum = 0;
	int16_t data[6];



	int16_t detectionPoint = 3900;


	uint8_t orientation = Z_POS;

	float trust = 0.95;

	uint8_t len = 0;
	uint8_t changeOrientationLen =0;

	printf_("start loop\n");
	while(1){
		mpu.GetRawAccelGyro(data);

		int16_t x = data[0];
		int16_t y = data[1];
		int16_t z = data[2];


		if (orientation != NONE){
			int16_t val;

			if (orientation == X_POS){
				val = x;
			}else if (orientation ==Y_POS){
				val = y;
			}else if (orientation ==Z_POS){
				val = z;
			}else if (orientation ==X_NEG){
				val = -x;
			}else if (orientation ==Y_NEG){
				val = -y;
			}else if (orientation ==Z_NEG){
				val = -z;
			}

			if (val < 2000 ){
				changeOrientationLen++;
				if (changeOrientationLen > 4){
					printf_("orientation change \n");
					orientation = NONE;
					changeOrientationLen = 0;
				}
			}else{
				changeOrientationLen = 0;
			}

			if(val < -500){
				len++;
			}else{
				if (len >0 && len < 10){
					printf_("event %d %d %s\n",mstimer_get(), len, getOrientation(orientation));
				}
				len = 0;
			}
		}

		if (orientation == NONE){
			if (x > detectionPoint){
				orientation = X_POS;
			}else if (x < -detectionPoint){
				orientation = X_NEG;
			}else if (y > detectionPoint){
				orientation = Y_POS;
			}else if (y < -detectionPoint){
				orientation = Y_NEG;
			}else if (z > detectionPoint){
				orientation = Z_POS;
			}else if (z < -detectionPoint){
				orientation = Z_NEG;
			}else{
				orientation = NONE;
			}
			printf_("orientation changed to %s\n", getOrientation(orientation));
			delay_ms(500);
			changeOrientationLen = 0;
			len =0;
		}




		delay_ms(20);
	}



		//printf_("start\n");
#ifdef USE_RADIO
		rfm69.reset();

		rfm69.init();
		rfm69.sleep();

		rfm69.setPowerDBm(13); // +10 dBm

		memset(packet, 0, 10);

#endif


	return 0;
}
