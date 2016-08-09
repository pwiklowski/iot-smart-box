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

	//mpu.setMotionDetection();


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
//		uint8_t interupt;
//		mpu.BufferRead(MPU6050_DEFAULT_ADDRESS, &interupt, MPU6050_RA_INT_STATUS, 1);
//
//		//printf_("int %d \n", interupt);
//		if(interupt & (1<<6)) {
//
//			uint8_t status;
//			mpu.BufferRead(MPU6050_DEFAULT_ADDRESS, &status, MPU6050_RA_MOT_DETECT_STATUS, 1);
//
//			if (( status & (1<<7)) == (1<<7)) printf_("X NEG\n");
//			if (( status & (1<<6)) == (1<<6)) printf_("X POS\n");
//			if (( status & (1<<5)) == (1<<5)) printf_("Y NEG\n");
//			if (( status & (1<<4)) == (1<<4)) printf_("Y POS\n");
//			if (( status & (1<<3)) == (1<<3)) printf_("Z NEG\n");
//			if (( status & (1<<2)) == (1<<2)) printf_("Z POS\n");
//			if (( status & (1<<0)) == (1<<0)) printf_("ZERO\n");
//
//			printf_("int %d %d\n", interupt, status);
//		}


//
//		gx = (float)data[3]*mpu.getGyroScale();
//		gy = (float)data[4]*mpu.getGyroScale();
//		gz = (float)data[5]*mpu.getGyroScale();
//
//		Now = mstimer_get();
//		deltat = (float) ((Now - lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
//		lastUpdate = Now;
//
//		sum += gz*deltat;



		//printf_("%d %d %d\n", base, (int) data[2], base - currentSample);

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

			if (val < 3000 ){
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




//		if (baseX > detectionPoint){
//			pos = X_POS;
//		}else if (baseX < -detectionPoint){
//			pos = X_NEG;
//		}else if (baseY > detectionPoint){
//			pos = Y_POS;
//		}else if (baseY < -detectionPoint){
//			pos = Y_NEG;
//		}else if (baseZ > detectionPoint){
//			pos = Z_POS;
//		}else if (baseZ < -detectionPoint){
//			pos = Z_NEG;
//		}else{
//			pos = NONE;
//		}
//
//		if (orientation != pos){
//			printf_("nowa orientacja %d\n", pos);
//			orientation = pos;
//		}
//
//		if (orientation == Z_POS){
//			if (data[2] <0){
//				printf_("jest klik Z_POS\n");
//				delay_ms(20);
//			}
//		}
//		if (orientation == Z_NEG){
//			if (data[2] > 0){
//				printf_("jest klik Z_NEG\n");
//				delay_ms(20);
//			}
//		}

		delay_ms(20);


	//printf_("%d %d %d %d\n", (int)gx, (int)gy,(int)gz, (int)sum);
	}


#endif

#ifdef M9250
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

#endif

		return 0;
	}
