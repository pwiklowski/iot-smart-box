
#include <ctype.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "systimer.h"
#include "string.h"
#include "spiL1.hpp"
#include "rfm69.hpp"

#include "COAPServer.h"
#include "cbor.h"
#include "OICServer.h"

extern "C"{
	#include "printf.h"
}


void init(){
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
	gpioStructure1.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_10;
	gpioStructure1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioStructure1);



	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 |  GPIO_Pin_7 | GPIO_Pin_9;
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
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

}




static void reverse(char* s)
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

static void itoa(int n, char* s)
{
     int i, sign;

     if ((sign = n) < 0)  /* записываем знак */
         n = -n;          /* делаем n положительным числом */
     i = 0;
     do {       /* генерируем цифры в обратном порядке */
         s[i++] = n % 10 + '0';   /* берем следующую цифру */
     } while ((n /= 10) > 0);     /* удаляем */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}
#define DEVICE

int main()
{
	uint8_t buf[1024];
	init();
	printf_("start\n");

	SPI spiRF(SPI1);
	spiRF.setPrescaler(SPI_BaudRatePrescaler_8);
	spiRF.init();
	//SPI_RxFIFOThresholdConfig (SPI1, SPI_RxFIFOThreshold_HF);


	for(int i=0; i<255; i++){
		buf[i] = i;

	}


	RFM69 rfm69(&spiRF, GPIOA, GPIO_Pin_4, false); // false = RFM69W, true = RFM69HW
	rfm69.reset();

	rfm69.init();
	rfm69.sleep();

	rfm69.setPowerDBm(10); // +10 dBm

	char* test = "czesc";
	char* ok = "ok";


	rfm69.setMode(RFM69_MODE_RX);
	rfm69.waitForModeReady();


	buf[0] = 200 >>8;
	buf[1] = 200;

	while(1){


//		int res = rfm69._receive(buf, 64);
//		if (res >0){
//			printf_("received packet with len=%d\n", res);
//			rfm69.send((uint8_t*)ok, 2);
//		}

		int res = rfm69.receivePacket(buf, 1024);
		if (res >0){
			printf_("received packet with len=%d\n", res);
		}



//
//		printf_("send packet\n");
//		if (rfm69.sendPacket(buf, 202) <0){
//			printf_("send packet FAIL\n");
//		}
//
//		delay_ms(5000);
//
//
//


	}



	uint8_t status;

	OICServer server("Buttun PC","0685B960-736F-46F7-BEC0-9E6CBD61ADC1", [&](COAPPacket* packet){
		size_t response_len;
		packet->build(buf, &response_len);
		rfm69.send(buf, response_len);
		rfm69.sleep();
		printf_("send packet\n");

	});

	cbor initial(CBOR_TYPE_MAP);
	initial.append("rt", "oic.r.switch.binary");
	initial.append("value", 1);

	OICResource* button = new OICResource("/switch", "oic.r.switch.binary","oic.if.r", [&](cbor* data){


	}, initial);

	server.addResource(button);
	server.start();



    COAPPacket* p = new COAPPacket();
	p->setType(COAP_TYPE_CON);
	p->setResonseCode(COAP_METHOD_GET);
	p->addOption(new COAPOption(COAP_OPTION_URI_PATH, "oic"));
	p->addOption(new COAPOption(COAP_OPTION_URI_PATH, "res"));

	p->setMessageId(0);
	p->setAddress("224.0.1.187 5683"); //TODO: move it to application


#if 1
	server.getCoapServer()->sendPacket(p, [&](COAPPacket* p){
	int a = 4;
	});
#endif

	int c=0;

	while(1){
		int bytesReceived = rfm69._receive(buf, sizeof(buf));
		c += bytesReceived;

		if (bytesReceived>0){
			printf_("Handle bytes\n");
			COAPPacket* p = COAPPacket::parse(buf+1, bytesReceived, "afs");
			if (p !=0){
				//send("Handle packet\n");
				//server.getCoapServer()->handleMessage(p);
			}
		}
	}

	return 0;
}

