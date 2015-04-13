#ifndef data_logging_h
#define data_logging_h
#include "global.h"



void Data_logging() {
	delay(100);
	//code taken from the arduino example of SD card datalogger, with mayor modifications

	/*
	SD card datalogger

	This example shows how to log data from three analog sensors
	to an SD card using the SD library.

	The circuit:
	* analog sensors on analog ins 0, 1, and 2
	* SD card attached to SPI bus as follows:
	** MOSI - pin 11
	** MISO - pin 12
	** CLK - pin 13
	** CS - pin 4

	created  24 Nov 2010
	modified 9 Apr 2012
	by Tom Igoe

	This example code is in the public domain.

	*/
	
	const int chipSelect = 4;
	

	// make sure that the default chip select pin is set to
	// output, even if you don't use it:
	pinMode(10, OUTPUT);

	// see if the card is present and can be initialized:
	while (1){ //modified the code so it constantly tries to establish a connection, but dosnt lock up the board in an infinte loop, or return the program.
		
	
		if (!SD.begin(chipSelect)) {
			/*if (global.debug_handler.data_logging_debug)*/ Serial.println("Card failed, or not present"); //we want to know this no matter what
			// don't do anything more:
			delay(5000); //wait 5 seconds and try again
			continue;
			
		}
		break;
	}
	if (global.debug_handler.data_logging_debug) Serial.println("card initialized.");

	//in this part i am going to make the filename to use
	//it takes the form "yy.mm.dd.hh.mm.ss"
	//i also wait for the gps to have a fix, since i get the timestamp used in the filename from the gps
	delay(5000); //wait til everything is set up
	while (!global.gps_data.fix && global.GPS_module.year==80){ //when the module is booted up from fresh, it thinks we are in the year 2080, as soon as it know we are not, then it knows when we are.
		if(global.debug_handler.data_logging_debug)  Serial.println("datalogger waiting for fix");
		if (global.debug_handler.data_logging_debug) Serial.println(global.GPS_module.year); //temp TODO remove
		delay(100); //wait till gps fix, that way we have the current time

	}
	if(global.debug_handler.data_logging_debug)	Serial.println("datalogger got current UTC time");

	char filename[] = "mmddhhmm.csv"; //Arduino supports a max filename of 8 characters, in this case its formated as month,date,hour and minuttes
	filename[0] = global.GPS_module.month / 10 + '0';
	filename[1] = global.GPS_module.month % 10 + '0';
	filename[2] = global.GPS_module.day / 10 + '0';
	filename[3] = global.GPS_module.day % 10 + '0';
	filename[4] = global.GPS_module.hour / 10 + '0';
	filename[5] = global.GPS_module.hour % 10 + '0';
	filename[6] = global.GPS_module.minute / 10 + '0';
	filename[7] = global.GPS_module.minute % 10 + '0';

	String dataString;
	// make a string for assembling the data to log:
	dataString = "";
	//Then make the string containing all the data
	dataString += String("millis()"); //timestamp in ms
	dataString += ";";
	dataString += String("global.GPS_module.latitude");
	dataString += ";";
	dataString += String("global.GPS_module.longitude");
	dataString += ";";
	dataString += String("global.gps_data.fix");
	dataString += ";";
	dataString += String("global.gps_data.gps_bearing");
	dataString += ";";
	dataString += String("(global.gps_data.location.latitude) * 1000000"); //to get the number of digits i want, i multiply with 1.000.000, it needs to be divided by 1.000.000 in the other end
	dataString += ";";
	dataString += String("(global.gps_data.location.longtitude) * 1000000");
	dataString += ";";
	dataString += String("global.gps_data.speed");
	dataString += ";";
	dataString += String("global.bearing_container.compass_bearing");
	dataString += ";";
	dataString += String("global.bearing_container.pitch");
	dataString += ";";
	dataString += String("global.bearing_container.roll");
	dataString += ";";
	dataString += String("global.wind_bearing");
	dataString += ";";
	dataString += String("global.global_wind_bearing");
	dataString += ";";
	dataString += String("global.waypoints.count()");
	dataString += ";";


	//The below part comes from guidance.h
	dataString += String("global.data_from_navigation_to_log.Boat1_Data_X_T_b_real");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.Boat1_Data_X_T_b_imag");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.global_Rudder_Desired_Angle");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.waypoints_count");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.desired_heading");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.distance_to_target");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.bearing_to_target");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.bearing_to_target_relative_to_wind");
	dataString += ";";
	dataString += String("global.data_from_navigation_to_log.current_state");


	File dataFile = SD.open(filename, FILE_WRITE);

	// if the file is available, write to it:
	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close();
		// print to the SerialUSB port too:
		if (global.debug_handler.data_logging_debug) Serial.println(dataString);
	}
	// if the file isn't open, pop up an error:
	else {
		if (global.debug_handler.data_logging_debug) Serial.println("error opening datalog.txt");
	}

	while (1) {
		// make a string for assembling the data to log:
		dataString = "";
		//Then make the string containing all the data
		dataString += String(millis()); //timestamp in ms
		dataString += ";";
		dataString += String(global.GPS_module.latitude);
		dataString += ";";
		dataString += String(global.GPS_module.longitude);
		dataString += ";";
		dataString += String(global.gps_data.fix);
		dataString += ";";
		dataString += String(global.gps_data.gps_bearing);
		dataString += ";";
		dataString += String((global.gps_data.location.latitude)*1000000); //to get the number of digits i want, i multiply with 1.000.000, it needs to be divided by 1.000.000 in the other end
		dataString += ";";
		dataString += String((global.gps_data.location.longtitude) * 1000000);
		dataString += ";";
		dataString += String(global.gps_data.speed);
		dataString += ";";
		dataString += String(global.bearing_container.compass_bearing);
		dataString += ";";
		dataString += String(global.bearing_container.pitch);
		dataString += ";";
		dataString += String(global.bearing_container.roll);
		dataString += ";";
		dataString += String(global.wind_bearing);
		dataString += ";";
		dataString += String(global.global_wind_bearing);
		dataString += ";";
		dataString += String(global.waypoints.count());
		dataString += ";";
		//dataString += global.data_from_path_to_log;

		/*	double Boat1_Data_X_T_b_real;
	double Boat1_Data_X_T_b_imag;
	double global_Rudder_Desired_Angle;
	int waypoints_count;
	float desired_heading;
	float distance_to_target;
	float bearing_to_target;
	float bearing_to_target_relative_to_wind;
	int current_state;*/

		dataString += String(global.data_from_navigation_to_log.Boat1_Data_X_T_b_real);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.Boat1_Data_X_T_b_imag);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.global_Rudder_Desired_Angle);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.waypoints_count);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.desired_heading);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.distance_to_target);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.bearing_to_target);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.bearing_to_target_relative_to_wind);
		dataString += ";";
		dataString += String(global.data_from_navigation_to_log.current_state);





		// open the file. note that only one file can be open at a time,
		// so you have to close this one before opening another.
		File dataFile = SD.open(filename, FILE_WRITE);

		// if the file is available, write to it:
		if (dataFile) {
			dataFile.println(dataString);
			//dataFile.println(global.data_from_path_to_log);

			dataFile.close();
			// print to the SerialUSB port too:
			if (global.debug_handler.data_logging_debug){
				Serial.println(dataString);
				//Serial.println(global.data_from_path_to_log);
			}
		}
		// if the file isn't open, pop up an error:
		else {
			if (global.debug_handler.data_logging_debug) Serial.println("error opening datalog.txt");
		}
		dataString = "";
		delay(30); //log 2 times a seconds, supject to change //with 30ms delay, it logs 20 times a second
	}

	
}

#endif