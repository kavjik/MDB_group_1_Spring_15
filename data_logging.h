#ifndef data_logging_h
#define data_logging_h
#include "global.h"
#define DO_WE_HAVE_CONTROLL_OF_SERVOS_PIN 50
#define BATTERY_CURRENT_READ_PIN A0
#define BATTERY_VOLTAGE_READ_PIN A1
#include "path_finding.h"

class Sd_card_data_logging { //this contains the data one could need from the gps
public:
	int chipSelect = 4;
	char filename[12]; //Arduino supports a max filename of 8 characters, in this case its formated as month,date,hour,minuttes
	Sd_card_data_logging() :chipSelect(4) {
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
		
		chipSelect = 4;
		


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
		delay(200); //wait til everything is set up
		while (!global.gps_data.fix && global.GPS_module.year == 80 && SIMULATOR_MODE == false){ //when the module is booted up from fresh, it thinks we are in the year 2080, as soon as it know we are not, then it knows when we are.
			if (global.debug_handler.data_logging_debug)  Serial.println("datalogger waiting for fix");
			if (global.debug_handler.data_logging_debug) Serial.println(global.GPS_module.year); //temp TODO remove
			delay(100); //wait till gps fix, that way we have the current time

		}
		if (global.debug_handler.data_logging_debug)	Serial.println("datalogger got current UTC time");
		
		if (SIMULATOR_MODE){
			filename[0] = 's';
			filename[1] = 'i';
			filename[2] = 'm';
			filename[3] = 'u';
			filename[4] = 'l';
			filename[5] = 'a';
			filename[6] = 't';
			filename[7] = 'e';
		}
		else {
			filename[0] = global.GPS_module.month / 10 + '0';
			filename[1] = global.GPS_module.month % 10 + '0';
			filename[2] = global.GPS_module.day / 10 + '0';
			filename[3] = global.GPS_module.day % 10 + '0';
			filename[4] = global.GPS_module.hour / 10 + '0';
			filename[5] = global.GPS_module.hour % 10 + '0';
			filename[6] = global.GPS_module.minute / 10 + '0';
			filename[7] = global.GPS_module.minute % 10 + '0';
		}
		filename[8] = '.';
		filename[9] = 'c';
		filename[10] = 's';
		filename[11] = 'v';

		
		// make a string for assembling the data to log:
		//Then make the string containing all the data
		write_to_SD_card("millis())"); //timestamp in ms
		write_to_SD_card("global.GPS_module.latitude, 10");
		write_to_SD_card("global.GPS_module.longitude, 10");
		write_to_SD_card("global.gps_data.fix");
		write_to_SD_card("global.gps_data.gps_bearing");
		write_to_SD_card("(global.gps_data.location.latitude) * 1000000"); //to get the number of digits i want, i multiply with 1.000.000, it needs to be divided by 1.000.000 in the other end
		write_to_SD_card("(global.gps_data.location.longtitude) * 1000000)");
		write_to_SD_card("global.gps_data.location.speed");
		write_to_SD_card("global.bearing_container.compass_bearing");
		write_to_SD_card("global.bearing_container.pitch");
		write_to_SD_card("global.bearing_container.roll");
		write_to_SD_card("global.wind_bearing");
		write_to_SD_card("global.global_wind_bearing");
		write_to_SD_card("global.waypoints.actual_size");
		write_to_SD_card("digitalRead(DO_WE_HAVE_CONTROLL_OF_SERVOS_PIN)");
		write_to_SD_card("analogRead(BATTERY_CURRENT_READ_PIN)");
		write_to_SD_card("analogRead(BATTERY_VOLTAGE_READ_PIN)");
		write_to_SD_card("global.data_from_navigation_to_log.Boat1_Data_X_T_b_real");
		write_to_SD_card("global.data_from_navigation_to_log.Boat1_Data_X_T_b_imag)");
		write_to_SD_card("global.data_from_navigation_to_log.global_Rudder_Desired_Angle");
		write_to_SD_card("global.data_from_navigation_to_log.waypoints_count");
		write_to_SD_card("global.data_from_navigation_to_log.desired_heading");
		write_to_SD_card("global.data_from_navigation_to_log.distance_to_target");
		write_to_SD_card("global.data_from_navigation_to_log.bearing_to_target)");
		write_to_SD_card("global.data_from_navigation_to_log.bearing_to_target_relative_to_wind");
		write_to_SD_card("global.data_from_navigation_to_log.current_state");
		write_to_SD_card("global.data_from_navigation_to_log.theta_A");
		write_to_SD_card("global.data_from_navigation_to_log.theta_B");
		write_to_SD_card("global.data_from_navigation_to_log.theta_AB");
		write_to_SD_card("global.data_from_navigation_to_log.theta_BA");
		write_to_SD_card("global.data_from_navigation_to_log.x");
		write_to_SD_card("global.data_from_navigation_to_log.collision_avoidance_active");
		write_to_SD_card("global.data_from_navigation_to_log.collision_avoidance_did_evasion");
		write_to_SD_card("\n");


	
	
	} //initialize to dummy values, they are overwritten when real world data makes sense

	void loop(void) {
		// make a string for assembling the data to log:
		//Then make the string containing all the data
		write_to_SD_card(String(millis())); //timestamp in ms
		write_to_SD_card(String(global.GPS_module.latitude, 10));
		write_to_SD_card(String(global.GPS_module.longitude, 10));
		write_to_SD_card(String(global.gps_data.fix));
		write_to_SD_card(String(global.gps_data.gps_bearing));
		write_to_SD_card(String((global.gps_data.location.latitude) * 1000000)); //to get the number of digits i want, i multiply with 1.000.000, it needs to be divided by 1.000.000 in the other end
		write_to_SD_card(String((global.gps_data.location.longtitude) * 1000000));
		write_to_SD_card(String(global.gps_data.location.speed));
		write_to_SD_card(String(global.bearing_container.compass_bearing));
		write_to_SD_card(String(global.bearing_container.pitch));
		write_to_SD_card(String(global.bearing_container.roll));
		write_to_SD_card(String(global.wind_bearing));
		write_to_SD_card(String(global.global_wind_bearing));
		write_to_SD_card(String(global.waypoints.actual_size));
		write_to_SD_card(String(digitalRead(DO_WE_HAVE_CONTROLL_OF_SERVOS_PIN)));
		write_to_SD_card(String(analogRead(BATTERY_CURRENT_READ_PIN)));
		write_to_SD_card(String(analogRead(BATTERY_VOLTAGE_READ_PIN)));
		write_to_SD_card(String(guidance_object.real_part_of_complex_from_old_code));
		write_to_SD_card(String(guidance_object.imaginary_part_of_complex_from_old_code));
		write_to_SD_card(String(global.Rudder_Desired_Angle));
		write_to_SD_card(String(global.waypoints.actual_size));
		write_to_SD_card(String(global.desired_heading));
		write_to_SD_card(String(guidance_object.distance_to_target));
		write_to_SD_card(String(guidance_object.bearing_to_target));
		write_to_SD_card(String(guidance_object.bearing_to_target_relative_to_wind));
		write_to_SD_card(String(guidance_object.state));
		write_to_SD_card(String(guidance_object.theta_A));
		write_to_SD_card(String(guidance_object.theta_B));
		write_to_SD_card(String(guidance_object.theta_AB));
		write_to_SD_card(String(guidance_object.theta_BA));
		write_to_SD_card(String(guidance_object.x));
		write_to_SD_card(String(guidance_object.collision_avoidance_active));
		write_to_SD_card(String(guidance_object.collision_avoidance_did_evasion));
		write_to_SD_card("\n");


	}

	void write_to_SD_card(String Input){
		Input = ";" + Input;
		File dataFile = SD.open(filename, FILE_WRITE);

		// if the file is available, write to it:
		if (dataFile) {
			dataFile.print(Input);
			//dataFile.println(global.data_from_path_to_log);

			dataFile.close();
			// print to the SerialUSB port too:
			if (global.debug_handler.data_logging_debug){
				Serial.print(Input);
				//Serial.println(global.data_from_path_to_log);
			}
		}
		// if the file isn't open, pop up an error:
		else {
			if (global.debug_handler.data_logging_debug) Serial.println("error opening datalog.txt");
		}
	}

	

};





void Data_logging() {
	
	Sd_card_data_logging data_logger;
	while (1) {
		data_logger.loop();
		delay(30);
	}



}


#endif