/* GPS Module Library on Arduino
	Space System Technology Labratory [SSTLab]
	Created By: Yahya M. Gilany 
	
	Due Thanks to Eng. Ahmad elkhateeb for his insightful help 
	___________________________________________________________________
	This code is an interface code for Arduino UNO that works on (NEO-6 u-blox 6 GPS Modules) using UART as communication protocol
	It follows the NMEA Reference Manuel for the data packaging protocol 
	It extracts the GPGGA which is for the (Time, position and fix type data.) info and the GPRMC which is for (Time, date, position, course and speed data.)info 
*/
//_______________________________________________________________________________________________________________________________________________________

#include "gpsgilany.h"

void GPSReadNPrint(char MsgID[], char time[], char Status[], char latitude [], char NS[], char longitude[],	char EW[], char SpGnd[],char CrsGnd[],char date[], char PFx[], char SatUsed[], char MSL[],char GSeperat[],char skip []){
	if (Serial.available()>0){             			//check if there's a new data received
		while (Serial.read() == '$') {       		//new line starts with & before any new message header
			Serial.println ("I'm Inn");		 		//for the sake of debugging, this is printed at the beginning of every line starting with $ and recognized by the microcontroller 
			Serial.readBytesUntil(',',MsgID,7);       //read the MsgID until it finds a comma which separates the data received in one msg package
			
			//we only need to check for the third index of the msgID to distinguish the msg Protocol header (refer to page 11 of NMEA )
			if (MsgID[3]== 'G'){						//for GGA message protocol which may look as follows ($GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M, , , ,0000*18)
				Serial.print("ID: ");					
				Prnt(MsgID,sizeof(MsgID)/sizeof(char));	//Print the msgID before it extracts the data from it.
				Serial.println("#######");				//not of a significant importance other than starting a new line after printing the ID
				
				Serial.readBytesUntil(',',time,11);		//get time and store it in the time Array
				Serial.print("Time: ");					//printing the value 
				Prnt ( time , sizeof(time)/sizeof(char));
				
				Serial.readBytesUntil(',',latitude,11);	//get latitude 
				Serial.print("     Latitude: ");
				Prnt ( latitude , sizeof(latitude)/sizeof(char));
				
				Serial.readBytesUntil(',',NS,3);		//get North/South indicator
				Serial.print("    NS: ");
				Prnt ( NS , sizeof(NS)/sizeof(char));
				
				Serial.readBytesUntil(',',longitude,12);// get longitude
				Serial.print("    Longitude: ");
				Prnt ( longitude , sizeof(longitude)/sizeof(char));
				
				Serial.readBytesUntil(',',EW,3);		//get East/west indicator
				Serial.print("    EW: ");
				Prnt ( EW , sizeof(EW)/sizeof(char));
				
				Serial.readBytesUntil(',',PFx,3);		//Positin Fix indicator
				Serial.print("  Position Fix: ");
				Prnt ( PFx , sizeof(PFx)/sizeof(char));
				
				Serial.readBytesUntil(',',SatUsed,4);	//number of Satalites used
				Serial.print("    Satalites used: ");
				Prnt ( SatUsed , sizeof(SatUsed)/sizeof(char));
				
				Serial.readBytesUntil(',',skip,12);		//to skip the readings of HDOP
				
				Serial.readBytesUntil(',',MSL,5);		//MSL altitude
				Serial.print("    MSL altitude: ");
				Prnt ( MSL , sizeof(MSL)/sizeof(char));
				
				Serial.readBytesUntil(',',skip,12);		//to skip the readings of Units
				
				Serial.readBytesUntil(',',GSeperat,5);	//Geoid separation
				Serial.print("    Geoid Seperation ");
				Prnt ( GSeperat , sizeof(GSeperat)/sizeof(char));
				
				for (int i=0 ; i<4 ; i++){				//for loop to skip (Units , Age of Diff Corr. , Diff Ref. Station ID , Checksum)
					Serial.readBytesUntil(',',skip,12);
				}
			}
			else if (MsgID[3] == 'M'){				//for RMC message protocol which may look as follows ($GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10)
				Serial.print("ID: ");					//Print the msgID before it extracts the data from it.
				Prnt(MsgID,sizeof(MsgID)/sizeof(char));	//not of a significant importance other than starting a new line after printing the ID
				Serial.println("#######");
				Serial.readBytesUntil(',',skip,12);		//to skip the readings of the time as we've already gotten it
				Serial.readBytesUntil(',',Status,3);	// get the Status A= Valid , V = non valid
				Serial.print("    Status: ");
				Prnt ( Status , sizeof(Status)/sizeof(char));
				for (int i=0 ; i<4 ; i++){				//to skip the readings of (latitude , N/S indicator , Longitude , E/W indicator) as we've already gotten them
					Serial.readBytesUntil(',',skip,12);
				}
				
				Serial.readBytesUntil(',',SpGnd,6);		//Speed over ground 
				Serial.print("    Speed over Ground: ");
				Prnt ( SpGnd , sizeof(SpGnd)/sizeof(char));
				
				Serial.readBytesUntil(',',CrsGnd,8);	//Course over ground
				Serial.print("    course over Ground: ");
				Prnt ( CrsGnd , sizeof(CrsGnd)/sizeof(char));
				
				Serial.readBytesUntil(',',date,8);		//date
				Serial.print("    Date: ");
				Prnt ( date , sizeof(date)/sizeof(char));
				
				for (int i=0 ; i<3 ; i++){				// to skip (Magnetic Variation , Mode, and Checksum)
					Serial.readBytesUntil(',',skip,12);
				}
			}
			Serial.println ("***"); 					//to be printed at the end of every message 
		}//while $
	}//if available
}//GPSReadNPrint

void Prnt (char array [] , int noElements){  // takes in the array and its number of elements as input 		
	for (int i=0 ; i<noElements ; i++){
		Serial.print (array[i]);				//prints every element of the array 
	}//for
} //void print

/* Notes : 
	- for all the calls of the Prnt function you used a method to calculate the number of elements of Every Array since we wanted to generalize the function so it fits any called array
	we know that Size of ( Array ) = Number of Elements  *  Size of every element ( char ) -->> no. Elements = Size of (Array)/size of (char)
	-you may notice when running the code that you're getting so many ("I'm in" and "***") without data being written after and that's because the microcontroller
	sees the starting * of the other msg Protocols we haven't used 
	
	
	-for Further info or questions, Please contact the SSTLab or myself on yahya.gilany@live.com
*/







