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
	char filename[13];
	 //Arduino supports a max filename of 8 characters, in this case its formated as month,date,hour,minuttes
	Sd_card_data_logging() :chipSelect(4) {
		delay(20);
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
		
		//filename = "simulate.csv";

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
		delay(50); //wait til everything is set up
		while (!global.gps_data.fix && SIMULATOR_MODE == false){ //when the module is booted up from fresh, it thinks we are in the year 2080, as soon as it know we are not, then it knows when we are.
			if (global.debug_handler.data_logging_debug)  Serial.println("datalogger waiting for fix");
			if (global.debug_handler.data_logging_debug) Serial.println(global.GPS_module.year); //temp TODO remove
			delay(100); //wait till gps fix, that way we have the current time

		}
		delay(1000);
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

		//char in_function_filename[] = "simulate.csv";
		//for (int i = 0; i < 8; i++){
		//	in_function_filename[i] = filename[i]; //very ugly work around
		//}
		//dataFile = SD.open(in_function_filename, FILE_WRITE);
		//if (dataFile) {
			// make a string for assembling the data to log:
			//Then make the string containing all the data
			write_to_SD_card(";millis())"); //timestamp in ms
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
			write_to_SD_card("guidance_object.real_part_of_complex_from_old_code");
			write_to_SD_card("guidance_object.imaginary_part_of_complex_from_old_code");
			write_to_SD_card("global.Rudder_Desired_Angle");
			write_to_SD_card("global.waypoints.actual_size");
			write_to_SD_card("global.desired_heading");
			write_to_SD_card("guidance_object.distance_to_target");
			write_to_SD_card("guidance_object.bearing_to_target");
			write_to_SD_card("guidance_object.bearing_to_target_relative_to_wind");
			write_to_SD_card("guidance_object.state");
			write_to_SD_card("guidance_object.theta_A");
			write_to_SD_card("guidance_object.theta_B");
			write_to_SD_card("guidance_object.theta_AB");
			write_to_SD_card("guidance_object.theta_BA");
			write_to_SD_card("guidance_object.x");
			write_to_SD_card("guidance_object.alpha");
			write_to_SD_card("guidance_object.beta");
			write_to_SD_card("guidance_object.theta_A_AB");
			write_to_SD_card("guidance_object.theta_B_BA");
			write_to_SD_card("guidance_object.theta_B_AB");
			write_to_SD_card("guidance_object.collision_avoidance_active");
			write_to_SD_card("guidance_object.collision_avoidance_did_evasion");
			write_to_SD_card("guidance_object.a");
			write_to_SD_card("longtitude_fix_triggered");
			write_to_SD_card("lattitude_fix_triggered");
			write_to_SD_card("\n");
			/*
			dataFile.close();
		}
		else {
			if (global.debug_handler.data_logging_debug) {
				Serial.println("error opening datalog.txt");
				Serial.println(in_function_filename);
			}
		}
		*/
	
	
	} //initialize to dummy values, they are overwritten when real world data makes sense
	File dataFile;
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
			write_to_SD_card(String(guidance_object.alpha));
			write_to_SD_card(String(guidance_object.beta));
			write_to_SD_card(String(guidance_object.theta_A_AB));
			write_to_SD_card(String(guidance_object.theta_B_BA));
			write_to_SD_card(String(guidance_object.theta_B_AB));
			write_to_SD_card(String(guidance_object.collision_avoidance_active));
			write_to_SD_card(String(guidance_object.collision_avoidance_did_evasion));
			write_to_SD_card(String(guidance_object.a));
			write_to_SD_card(String(global.longtitude_fix_triggered));
			write_to_SD_card(String(global.lattitude_fix_triggered));
			write_to_SD_card("\n");

			delay(50);
		


	}

	void write_to_SD_card(String Input){
		static int i = 0;
		static String write_to_file = "";
		write_to_file += Input;
		write_to_file += ";";

		if (i == 5){
			char in_function_filename[] = "simulate.csv";
			for (int i = 0; i < 8; i++){
				in_function_filename[i] = filename[i]; //very ugly work around
			}
			dataFile = SD.open(in_function_filename, FILE_WRITE);
			if (dataFile) {

				// if the file is available, write to it:

				dataFile.print(write_to_file);
			}
			else {
				if (global.debug_handler.data_logging_debug) {
					Serial.println("error opening datalog.txt");
					Serial.println(in_function_filename);
				}
			}
			dataFile.close();
			//dataFile.println(global.data_from_path_to_log);



			// print to the SerialUSB port too:
			if (global.debug_handler.data_logging_debug){
				Serial.print(write_to_file);
				//Serial.println(global.data_from_path_to_log);
			}
			i = 0;
			write_to_file = "";
		}
		i++;
		
		delay(5);
		
	}

	

};





void Data_logging() {


	delay(1500); //wait till everything has been setup
	Sd_card_data_logging data_logger;
	while (1) {
		data_logger.loop();
		yield();
	}



}


#endif