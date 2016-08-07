#include <ctype.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"

#include "string.h"

//#include "cbor.h"
#include "MPU9250.h"

extern "C" {
#include "printf.h"
#include "systimer.h"
}

#include "math.h"

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
#define DEVICE

int main() {
	init();

	MPU9250 mpu9250;
	mpu9250.initI2c();

	uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
	printf_("I AM 0x%x\n\r", whoami);
	printf_("I SHOULD BE 0x71\n\r");

	//whoami = mpu9250.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); // Read WHO_AM_I register for MPU-9250
	//printf_("I AM 0x%x\n\r", whoami);
	//printf_("I SHOULD BE 0x71\n\r");


	mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
	mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	delay_ms(200);
	mpu9250.initMPU9250();
	mpu9250.initAK8963(magCalibration);
	delay_ms(200);

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity

    magbias[0] = -26;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = 110;  // User environmental x-axis correction in milliGauss
    magbias[2] = -157;  // User environmental x-axis correction in milliGauss

#define MAG_CALIBRATION_

#ifdef MAG_CALIBRATION

  	printf_("mag calibration\n");
    for(int i=0; i<3000; i++){
    	mpu9250.readMagData(magCount);  // Read the x/y/z adc values
    	printf_("%d %d %d\n", magCount[0], magCount[1], magCount[2]);
    	delay_ms(10);
    }
  	printf_("mag calibration end\n");
  	return 0;

#endif


  	float sum = 0;

  	mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values



  	mpu9250.waitForWakeUp();
	while (1) {
		uint8_t interupt = mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS);
		//printf_("int %d\n", interupt);
		if(interupt & (1<<6)) {
			uint8_t status= mpu9250.readByte(MPU9250_ADDRESS, MOT_DETECT_STATUS);

			printf_("%d %d wakeup\n", mstimer_get(), status);
			mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
			printf_("%d %d %d\n", accelCount[0], accelCount[1], accelCount[2]);
			delay_ms(100);
		}


		mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
		gx = (float) gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
		gy = (float) gyroCount[1] * gRes - gyroBias[1];
		gz = (float) gyroCount[2] * gRes - gyroBias[2];


		mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
		ax = (float) accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) accelCount[1] * aRes - accelBias[1];
		az = (float) accelCount[2] * aRes - accelBias[2];

		Now = mstimer_get();
		deltat = (float) ((Now - lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		sum += gz*deltat;

		//printf_("Yaw, Pitch, Roll: %d %d %d %d\n",mstimer_get(), (int)gx*10, (int)gy*10, (int)gz*10);
		//printf_("%d %d\n",(int)accelCount[2], (int)sum);


		delay_ms(10);
		continue;
	}


	while (1) {
		mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
		ax = (float) accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) accelCount[1] * aRes - accelBias[1];
		az = (float) accelCount[2] * aRes - accelBias[2];

		mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
		gx = (float) gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
		gy = (float) gyroCount[1] * gRes - gyroBias[1];
		gz = (float) gyroCount[2] * gRes - gyroBias[2];

		mpu9250.readMagData(magCount);  // Read the x/y/z adc values
		mx = (float) (magCount[0]- magbias[0]) * mRes * magCalibration[0]; // get actual magnetometer value, this depends on scale being set
		my = (float) (magCount[1]- magbias[1]) * mRes * magCalibration[1];
		mz = (float) (magCount[2]- magbias[2]) * mRes * magCalibration[2];

		Now = mstimer_get();
		deltat = (float) ((Now - lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		printf_("Yaw, Pitch, Roll: %d %d %d\n", (int)gx*10, (int)gy*10, (int)gz*10);

		continue;

		mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);
		//mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


		tempCount = mpu9250.readTempData();  // Read the adc values
		temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade

		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / PI;
		yaw *= 180.0f / PI;
		//yaw -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll *= 180.0f / PI;

		printf_("Yaw, Pitch, Roll: %d %d %d %d\n", (int)yaw, (int)pitch, (int)roll, (int)temperature);
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
