/* GPS Module Library on Arduino
	Space System Technology Labratory [SSTLab]
	Created By: Yahya M. Gilany 
	
	Due Thanks to Eng. Ahmad elkhateeb for his insightful help 
	___________________________________________________
	This code is an interface code for Arduino UNO that works on (NEO-6 u-blox 6 GPS Modules) using UART as communication protocol
	It follows the NMEA Reference Manuel for the data packaging protocol 
	It extracts the GPGGA which is for the (Time, position and fix type data.) info and the GPRMC which is for (Time, date, position, course and speed data.)info 
*/
#include <Arduino.h>


#ifndef GPSGILANY_H
#define GPSGILANY_H

// ************ Variables Declerations ********************// 


void GPSReadNPrint(char MsgID[], char time[], char Status[], char latitude [], char NS[], char longitude[],	char EW[], char SpGnd[],char CrsGnd[],char date[], char PFx[], char SatUsed[], char MSL[],char GSeperat[],char skip []);
void Prnt (char array [] , int noElements);  // takes in the array and its number of elements as input 		

#endif