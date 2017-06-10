/**
 Cansat Code  (Master Code)
Yahya M. Gilany
Space Systems and Technology Lab (SSTLab)

TODO: Do I need to ? set pin 10 as output (SD card) > who's SS
TODO: Temp from mpu 
TODO: Line 107
TODO: two batteries summation (DONE)
TODO: change comments for the mpu 117


Mission: This project is to measure the temperature, pressure, humidity , time, date , altitude , longitude , latitude ,

// ==============================================================
// =               Used Sensors and components                  =
// ==============================================================
- 2 ATMEGA 328P
- DHT11 - Humidity and Temperature Sensor
- NEO-6M - GPS Module
- MPU6050 - Accelerometer and Gyroscope (Temperature Also)
- BMP085 - Pressure sensor  (can also measure Temperature)
- SD Card reader module
- RF Module - Wireless module

// ==============================================================
// = 			Pin connections 			=
// ==============================================================

**I2C Devices:(MPU6050, BMP085, Slave Arduino Uno)
----------------------------------------
Arduino    		I2C Devices
----------------------------------------
A4         		SDA (Serial DAta)
A5       		SCL (Serial CLock)
Gnd				AD0 (Only in MPU-6050)
+3V3      		VCC
Gnd       		Gnd

*Don't forget the 10K resistances between Vcc & SDA and SCL

----------------------------------------
Arduino  	  	DHT11
----------------------------------------
+3V3			Vin (1st Pin from right)
D5				Data (2nd)
...	       		(3rd pin) left unused
Gnd       		Gnd

----------------------------------------
Arduino    		SD Card
----------------------------------------
D10       		CS (chip select)
D11       		MOSI
D12     		MISO
D13        		CLK (clock)
+5V        		5V
Gnd        		Gnd

----------------------------------------
Arduino    		GPS Module
----------------------------------------
D4       		TX
D3       		RX
+3V3        	Vin
Gnd        		Gnd

----------------------------------------
Arduino    		Other Components
----------------------------------------
A3       		Battery Life
D9       		Buzzer

// =========
// = Notes =
// =========


// ================
// = Team Members =
// ================
Due Thanks To my amazing team who was able to work under pressure and overcome all the obstacles we've gotten through
-Eng. Ahmed Adel Al Rewiny (PCB and Power System)
-Eng. Ahmad Mostafa (Ground Station)
-Eng. Hussein Junior (Structure)
-Eng. Samar El Diary (Recovery System)
-Eng. Yahya Gilany (On-board processor and coding)

Also Due thanks to all those who've extended their hand for help throughout
-Eng. Hassan Ali
-Eng. Ahmad Abd El-Moniem
-Eng. Mostafa Sayed
-Eng. Ahmad El-Khateeb
-Eng. Ahmad Hamada for his support

and all other team for their sport manner

// =========
// = Notes =
// =========
- Haven't put the option to measure the tempreture from the BMP085 Sensor due to the delay it would have beent taken to switch between 
  the pressure and tempreture.

**/

// ================================================================
// ===                     Working Features                     ===
// ================================================================
/* Set to Zero to disable the component */

// ==================================
// = Working Sensors and components =
// ==================================

#define MPUSENS 	1 	// MPU6050 Sensor
#define SDCARD_ON 	0 	// SD card Module
#define DHTSENS 	1	// DHT11 Sensor 
#define BMPSENS 	1	// BMP085 Sensor
#define GPS_ON 		1	// NEO-6M GPS Module
#define BATTERYLIFE 1	// BatteryLife Measurment

// ======================================
// = Working Features of the components =
// ======================================
#define TEMP_FROM_DHT 1	// To read Temperature from the DHT11 Sensor
#define TEMP_FROM_MPU 0	// To read temperature from the MPU6050 Sensor
#define WAIT_TO_START 0 // Wait for serial input in setup() to activate the SD Card

#if SDCARD_ON
#define PRINTSD 	1	// Print the data to the SD Card 
#endif

#define PRINTSERIAL 1	// Send through Serial
#define MPUSD 		0	// Print the MPU6050 data to the SD card (No need for this )

// Set to Zero "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO 1

// Set to Zero "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL 1


// ================================================================
// ===              PIN and constants Definition                ===
// ================================================================

#define LED_PIN 13		// will be used to show activity 
#define SD_CS 10 		// can be removed i guess 
#define BUZZER 9		// Buzzer Pin
#define DHT11_PIN 5		// DHT11 Pin
#define SLAVEGPS 9		// Slave address of the 2nd Atmega that reads the data from the GPS
#define BATT1_PIN A3	// Battery 1 Pin
#define BATT2_PIN A2	// Battery 2 Pin
#define V_OUTPUT 3.7	// Voltage output from single battry

/*SD Card*/
#define CHIP_SELECT SS 		// SD chip select pin
#define LOG_INTERVAL 1000 	// mills between entries
#define SENSOR_COUNT 3 		// number of analog pins to log
#define SYNC_INTERVAL 100 	// mills between calls to sync()
#define LOGFILE "CANSAT01.CSV" //This is the initial Name for the Logging Sheet, it's self-updated

#define SERIAL_BAUD_RATE 115200	//Set the Baud Rate from here
#define ACSCALE		  16100.0	//dividing the raw accelerometer values by 16100 convert the raw data into multiples of g ..(page 30 of DataSheet1)
#define GYSCALE		  131.0		//dividing the raw gyroscope values by 131 gives angular velocity in degrees per second .. page 32 assuming FS_SEL = 0
#define SEA_LEVEL_PRESSURE 92540.0 // Set this to the ground pressure in order to fix the altitude calibration

// ================================================================
// ===                   Included Libraries                     ===
// ================================================================

#include "I2Cdev.h"
#include "Wire.h"

#if GPS_ON
#include "SoftwareSerial.h" // To be put in the slave Atmega that will be connected to the GPS Module
#endif //GPS ON

#if MPUSENS
#include "MPU6050_6Axis_MotionApps20.h" //To get advantage from the stable data coming from the Digital Motion Processing That comes with the MPU6050 
#endif//MPUSENS

#if SDCARD_ON
#include <Fat16.h>		// Lighter library for SD Card (The SD card should be formatted to FAT16 File System)
#include <Fat16util.h> 	// use functions to print strings from flash memory
#endif //SDCARD_ON

#if BMPSENS
#include "BMP085.h"		// Pressure Sensor library
#endif//BMPSENS

#if DHTSENS
#include <dht.h>		// DHT11 Sensor library
#endif//DHTSENS

#if GPS_ON
#include <I2C_Anything.h>	// make the transfer from the slave to the master 
#endif // GPS ON 
//This can be added to the first one 

// ================================================================
// ===                   Global Variables                       ===
// ================================================================
bool blinkState = false; //for the led that would be indicative of activity

int16_t temperature; //Either from the DHT11 or the MPU6050

/*MPU*/
#if MPUSENS
int16_t ax, ay, az;		// Accelerometer data
int16_t gx, gy, gz;		// Gyroscope data

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;    // [x, y, z]            gravity vector

#endif //MPUSENS

/*SD Card*/ //try to minimize that
#if SDCARD_ON
uint32_t syncTime = 0;     // time of last sync()
uint32_t logTime  = 0;     // time data was logged
char name[] = LOGFILE;

#endif //SDCARD_ON

/*DHT11 Sensor*/
#if DHTSENS
int16_t humidity;			// humidity data
int chk;
#endif //DHTSENS

/*BMP085*/
#if BMPSENS
int32_t pressure;			// pressure data
int32_t altitude;			// altitude data
int32_t lastMicros;			// To avoid the delay of the sensor
#endif //BMPSENS

#if GPS_ON
volatile float latit, longit;	// Longitude and latitude data
volatile float tym, Date;		// time and date data
#endif //GPS ON

// ================================================================
// ===                     Global Objects                       ===
// ================================================================
#if MPUSENS
MPU6050 mpu; 		// class default I2C address is 0x68
#endif //MPUSENS

#if SDCARD_ON
SdCard card;
Fat16 file;
#endif //SDCARD_ON

#if BMPSENS
BMP085 barometer; // class default I2C address is 0x77
#endif //BMPSENS

#if DHTSENS
dht DHT;
#endif //DHTSENS

#if GPS_ON

#endif // GPS ON

// ================================================================
// ===                 Function Declaraton                      ===
// ================================================================

#if SDCARD_ON
/* store error strings in flash to save RAM*/
#define error(s) error_P(PSTR(s))
//------------------------------------------------------------------------------
void error_P(const char* str) {
	PgmPrint("error: ");
	SerialPrintln_P(str);
	if (card.errorCode) {
		PgmPrint("SD error: ");
		Serial.println(card.errorCode, HEX);
	}
	while (1);
}
//------------------------------------------------------------------------------
#endif //SDCARD_ON


/* Temperature from the MPU6050 */
#if MPUSENS
#if TEMP_FROM_MPU
void ReadTmp(uint16_t* Tmp_out ) {
	Wire.beginTransmission(0x68);
	Wire.write(MPU6050_RA_TEMP_OUT_H);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 2);
	if (Wire.available () == 2) {
		*Tmp_out   = (Wire.read() << 8 | Wire.read()) / 340.00 + 36.53;
	}
}
#endif //TEMP_FROM_MPU

/* INTERRUPT DETECTION ROUTINE */
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void ReadTheData(int16_t* AcX_out, int16_t* AcY_out, int16_t* AcZ_out, int16_t* GyroX_out, int16_t* GyroY_out, int16_t* GyroZ_out, int16_t* Tmp_out ) {
	Wire.beginTransmission(0x68);
	Wire.write(MPU6050_RA_ACCEL_XOUT_H);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 14);
	if (Wire.available () == 14) {
		*AcX_out   = (Wire.read() << 8 | Wire.read());
		*AcY_out   = (Wire.read() << 8 | Wire.read());
		*AcZ_out   = (Wire.read() << 8 | Wire.read());
		*Tmp_out   = (Wire.read() << 8 | Wire.read()) / 340.00 + 36.53;
		*GyroX_out = (Wire.read() << 8 | Wire.read());
		*GyroY_out = (Wire.read() << 8 | Wire.read());
		*GyroZ_out = (Wire.read() << 8 | Wire.read());
	}
}
#endif //MPUSENS

#if BATTERYLIFE

float mapFloat (float in , float a , float b , float c , float d) {
   return ((in-a)/(b-a)*(d-c)+c);  
}

#endif // battery life


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {

	// initialize serial communication
	Serial.begin(SERIAL_BAUD_RATE);			// change the baud rate from above Line number (170)

	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)



	// initialize I2C devices
	Serial.println(F("Initializing I2C devices..."));
	delay(700);
#if MPUSENS
	mpu.initialize();
	Serial.println(F("Testing MPU6050 device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	delay (500);
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
#endif //MPUSENS

#if BMPSENS
	barometer.initialize();
	Serial.println(F("Testing BMP085 device connections..."));
	Serial.println(barometer.testConnection() ? F("BMP085 connection successful") : F("BMP085 connection failed"));
	// request pressure (3x oversampling mode, high detail, 23.5ms delay)
	barometer.setControl(BMP085_MODE_PRESSURE_3);
	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
#endif //BMPSENS
	
#if SDCARD_ON
#if WAIT_TO_START
	PgmPrintln("Type any character to start");
	while (!Serial.available());
#endif //WAIT_TO_START

	// initialize the SD card
	if (!card.begin(CHIP_SELECT)) error("card.begin");

	// initialize a FAT16 volume
	if (!Fat16::init(&card)) error("Fat16::init");

	// create a new file ((This renames the file if it was repeated before))
	// char name[] = LOGFILE;
	for (uint8_t i = 0; i < 100; i++) {
		name[6] = i / 10 + '0';
		name[7] = i % 10 + '0';
		// O_CREAT - create the file if it does not exist
		// O_EXCL - fail if the file exists
		// O_WRITE - open for write only
		if (file.open(name, O_CREAT | O_EXCL | O_WRITE))break;
	}
	if (!file.isOpen()) error ("create");
	PgmPrint("Logging to: ");
	Serial.println(name);

#endif //SDCARD_ON


#if SDCARD_ON
	/* write data header */
	file.writeError = false; // clear write error
#if PRINTSD
	file.print("Date,Time,Latitude,Longitude,Humidity%,Temperature,Pressure,Altitude");
#endif //PRINTSD


#if PRINTSD
	file.println();
#endif //PRINTSD


	if (file.writeError || !file.sync()) {
		error("write header");
	}
	/* End Of FILE Header*/
#endif // SDCARD_ON

#if PRINTSERIAL
	Serial.print("Date\t\tTime\t\tLat\t\tlong\t\tHum\tTmp\tP\tAlt\tYaw\tPitch\tRoll\tax\tay\taz\tgx\tgy\tgz");
	Serial.println();
#endif //PRINTSERIAL

	// configure LED to indicate activity as an output
	pinMode(LED_PIN, OUTPUT);
        pinMode (BUZZER, OUTPUT);
} //void Setup


/*
BMP085 connection successful
Date		Time		Lat			long		Hum	Tmp	P		Alt	Yaw		Pitch	Roll	ax		ay		az		gx		gy		gz
130415.00	17594300.00	31.295413	29.961160	35	23	94737	563	0.00	0.00	0.00	0.00	0.00	0.00	0.00	0.00	0.00
130415.00	17594400.00
*/

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
#if SDCARD_ON
	file.open(name, O_CREAT | O_EXCL | O_WRITE);
	uint16_t m = logTime;
	// wait till time is an exact multiple of LOG_INTERVAL
	do {
		logTime = millis();
	} while (m == logTime || logTime % LOG_INTERVAL);
	// log time to file
#if PRINTSD
	file.print(logTime);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(logTime);
	Serial.write('\t');
#endif //PRINTSERIAL
#endif//sd card


	/* This is to be subtituted with the time and date coming from the GPS , hopefully */

#if MPUSENS
	//delay (500);
#if TEMP_FROM_MPU
	//ReadTmp (&temperature);
#endif //temp from mpu

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.print(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

#if OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif//yaw roll pitch

#if OUTPUT_READABLE_ACCELGYRO
		//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		ReadTheData(&ax, &ay, &az, &gx, &gy, &gz, &temperature);
#endif //readable accelgyrp

		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	} //if ((mpuIntStatus & 0x10) || fifoCount == 1024)
#endif//MPUSENS



#if DHTSENS
	// READ DATA
	chk = DHT.read11(DHT11_PIN);
	switch (chk)
	{
		case DHTLIB_OK:
		humidity = DHT.humidity;

#if TEMP_FROM_DHT
		temperature = DHT.temperature;

#endif// Temp from DHT
		break;
		case DHTLIB_ERROR_CHECKSUM:
		break;
		case DHTLIB_ERROR_TIMEOUT:
		break;
		case DHTLIB_ERROR_CONNECT:
		break;
		case DHTLIB_ERROR_ACK_L:
		break;
		case DHTLIB_ERROR_ACK_H:
		break;
		default:
		break;
	} //by doing this i've avoided the use of the delay, and will only get the new data whenever is possible
#endif //DHTSENS


#if BMPSENS
	pressure = barometer.getPressure();
	altitude = barometer.getAltitude(pressure,SEA_LEVEL_PRESSURE);

	if (altitude < 3 ) { //meteres
		digitalWrite(BUZZER, HIGH);
	}

#endif //BMPSENS

#if GPS_ON
	Wire.requestFrom (SLAVEGPS, 4);
	I2C_readAnything(latit);
	Wire.requestFrom (SLAVEGPS, 4);
	I2C_readAnything(longit);
	Wire.requestFrom (SLAVEGPS, 4);
	I2C_readAnything(tym);
	Wire.requestFrom (SLAVEGPS, 4);
	I2C_readAnything(Date);
#endif  //gps on


	/** Printing Data **/

#if GPS_ON
	// ========
	// = Date =
	// ========
#if PRINTSD
	file.print(Date);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(Date);
	Serial.write('\t');
#endif //PRINTSERIAL
	// ========
	// = Time =
	// ========
#if PRINTSD
	file.print(tym);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(tym);
	Serial.write('\t');
#endif //PRINTSERIAL
	// =============
	// = Longitude =
	// =============
#if PRINTSD
	file.print(longit, 6);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(longit, 6);
	Serial.write('\t');
#endif //PRINTSERIAL
	// ============
	// = Latitude =
	// ============
#if PRINTSD
	file.print(latit, 6);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(latit, 6);
	Serial.write('\t');
#endif //PRINTSERIAL
#endif//GPS_ON

	// ==========
	// = Humidity =
	// ==========
#if DHTSENS
#if PRINTSD
	file.print(humidity);
	file.write(',');
#endif //PRINTSD
#if PRINTSERIAL
	Serial.print(humidity);
	Serial.write('\t');
#endif //PRINTSERIAL
#endif // DHT SENS

	// ===============
	// = Temperature =
	// ===============
#if PRINTSD
	file.print(temperature);
	file.write(',');
#endif // PRINTSD
#if PRINTSERIAL
	Serial.print(temperature);
	Serial.write('\t');
#endif //PRINTSERIAL

#if BMPSENS
	// ============
	// = Pressure =
	// ============
#if PRINTSD
	file.print(pressure);
	file.write(',');
#endif // PRINTSD
#if PRINTSERIAL
	Serial.print(pressure);
	Serial.write('\t');
#endif //PRINTSERIAL
	// ============
	// = Altitude =
	// ============
#if PRINTSD
	file.print(altitude);
	file.write(',');
#endif // PRINTSD
#if PRINTSERIAL
	Serial.print(altitude);
	Serial.write('\t');
#endif //PRINTSERIAL
#endif // BMP SENS	

#if MPUSENS
	// ==========
	// =   Yaw  =
	// ==========
#if MPUSD
#if PRINTSD
	file.print(ypr[0] * 180 / M_PI);
	file.write(',');
#endif //PRINTSD
#endif // MPUSD
#if PRINTSERIAL
	Serial.print(ypr[0] * 180 / M_PI);
	Serial.write('\t');
#endif //PRINTSERIAL	

	// ==========
	// = Pitch  =
	// ==========
#if MPUSD
#if PRINTSD
	file.print(ypr[1] * 180 / M_PI);
	file.write(',');
#endif //PRINTSD
#endif // MPUSD
#if PRINTSERIAL
	Serial.print(ypr[1] * 180 / M_PI);
	Serial.write('\t');
#endif //PRINTSERIAL

	// ==========
	// =  Roll  =
	// ==========
#if MPUSD
#if PRINTSD
	file.print(ypr[2] * 180 / M_PI);
	file.write(',');
#endif //PRINTSD
#endif // MPUSD
#if PRINTSERIAL
	Serial.print(ypr[2] * 180 / M_PI);
	Serial.write('\t');
#endif //PRINTSERIAL

	// =====================
	// = Acceleration in X =
	// =====================
#if MPUSD
#if PRINTSD
	file.print(ax / ACSCALE);
	file.write(',');
#endif //PRINTSD
#endif // MPUSD
#if PRINTSERIAL
	Serial.print(ax / ACSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL

	// =====================
	// = Acceleration in Y =
	// =====================
#if MPUSD
#if PRINTSD
	file.print(ay / ACSCALE);
	file.write(',');
#endif //PRINTSD
#endif // MPUSD
#if PRINTSERIAL
	Serial.print(ay / ACSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL

	// =====================
	// = Acceleration in Z =
	// =====================
#if MPUSD
#if PRINTSD
	file.print(az / ACSCALE);
	file.write(',');
#endif //PRINTSD
#endif //MPUSD
#if PRINTSERIAL
	Serial.print(az / ACSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL	

	// ==================
	// = Gyroscope in X =
	// ==================
#if MPUSD
#if PRINTSD
	file.print(gx / GYSCALE);
	file.write(',');
#endif //PRINTSD
#endif//MPUSD
#if PRINTSERIAL
	Serial.print(gx / GYSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL

	// ==================
	// = Gyroscope in Y =
	// ==================
#if MPUSD
#if PRINTSD
	file.print(gy / GYSCALE);
	file.write(',');
#endif //PRINTSD
#endif //MPUSD
#if PRINTSERIAL
	Serial.print(gy / GYSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL

	// ==================
	// = Gyroscope in Z =
	// ==================
#if MPUSD
#if PRINTSD
	file.print(gz / GYSCALE);
	file.write(',');
#endif //PRINTSD
#endif //MPUSD
#if PRINTSERIAL
	Serial.print(gz / GYSCALE);
	Serial.write('\t');
#endif //PRINTSERIAL
#endif // MPUSENS
	
	// ================
	// = Battery Life =
	// ================
#if BATTERYLIFE
#if PRINTSERIAL
	Serial.print(mapFloat (analogRead(BATT1_PIN), 0.0, 1023.0, 0.0, V_OUTPUT) + mapFloat (analogRead(BATT2_PIN), 0.0, 1023.0, 0.0, V_OUTPUT));
	Serial.write('\t');
#endif //PRINTSERIAL
#endif //BATTERYLIFE
	
#if PRINTSD
	file.println();
#endif //PRINTSD
#if PRINTSERIAL
	Serial.println();
#endif //PRINTSERIAL

#if SDCARD_ON
	if (file.writeError) error("write data");
	//don't sync too often - requires 2048 bytes of I/O to SD card
	//	if ((millis() - syncTime) <  SYNC_INTERVAL) return;
	//	syncTime = millis();
	//	if (!file.sync()) error("sync");
#endif // SDCARD_ON

} // void Loop