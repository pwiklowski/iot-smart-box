#include <ctype.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"

#include "string.h"

//#include "cbor.h"
#include "MPU9250.h"
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
#ifdef USE_RADIO
Rfm69 rfm69;
uint8_t packet[300];
uint32_t setTime;
extern "C" void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0);
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET) {
		setTime = mstimer_get();
	} else {
		if (setTime !=0 ) {
			uint32_t pressTime = mstimer_get() - setTime;
			setTime = 0;
			if (pressTime >50) {
				printf_("release %d\n", pressTime);
				uint32_t t = mstimer_get();
				List<uint8_t> data;

//			  cbor initial(CBOR_TYPE_MAP);
//			  initial.append("rt", "oic.r.switch.binary");
//			  initial.append("value", 1);
//
//			  initial.dump(&data);
//
//			  for (uint16_t i=0; i<data.size(); i++){
//				  packet[i] = data.at(i);
//			  }
//
//			  printf_("prep %d\n", mstimer_get() -t);
//			  t = mstimer_get();
//			  rfm69.send(packet, data.size(), 0);
//			  printf_("send %d\n", mstimer_get() -t);
//			  rfm69.sleep();
			}
		}
	}
}

#endif
static void reverse(char* s) {
	int i, j;
	char c;

	for (i = 0, j = strlen(s) - 1; i < j; i++, j--) {
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}

static void itoa(int n, char* s) {
	int i, sign;

	if ((sign = n) < 0) /* записываем знак */
		n = -n; /* делаем n положительным числом */
	i = 0;
	do { /* генерируем цифры в обратном порядке */
		s[i++] = n % 10 + '0'; /* берем следующую цифру */
	} while ((n /= 10) > 0); /* удаляем */
	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';
	reverse(s);
}
#define M9250_

#define M6050

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

#ifdef M6050
	Mpu6050 mpu;
	mpu.init();
	mpu.calibrate();

	if (!mpu.test()){
		printf_("mpu has problem\n");

	}
	float sum = 0;
	int16_t data[6];

	int16_t baseX = 0;
	int16_t baseY = 0;
	int16_t baseZ = 0;


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


		uint8_t pos;
		baseX = baseX*trust+ x*(1-trust);
		baseY = baseY*trust+ y*(1-trust);
		baseZ = baseZ*trust+ z*(1-trust);


		//printf_("%d %d %d\n",mstimer_get(), z,baseZ);



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
		}




		delay_ms(20);
	}


#endif

		//printf_("start\n");
#ifdef USE_RADIO
		rfm69.reset();

		rfm69.init();
		rfm69.sleep();

		rfm69.setPowerDBm(13); // +10 dBm

		memset(packet, 0, 10);

#endif

#endif

		return 0;
	}
