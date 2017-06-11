# CanSat
SSTLab 2015 | CanSat Competition

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
