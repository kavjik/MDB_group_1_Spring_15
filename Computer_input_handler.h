

#ifndef computer_input_handler_h
#define computer_input_handler_h

#include "global.h"

//this thread takes inputs from the computer attached, if such a thing is attached
//at the moment its only used for switching debug messages on or off, but later its supposed to also control things like calibration.
//se list of commands in the switch case below.


void computer_input_handler() 
{

	
	while (1){

	
		yield();
		if (Serial.available()){
			//we got a command from the computer, lets react to it
			char ch = Serial.read();
			switch (ch)
			{
			case 'g': //switch gps sensor debug mode
				global.debug_handler.gps_sensor_debug = !global.debug_handler.gps_sensor_debug;
				Serial.println("Command recieved switch gps sensor debug mode");
				break;
			case 't': //gps tracking debug
				global.debug_handler.gps_tracking_debug = !global.debug_handler.gps_tracking_debug;
				Serial.println("Command recieved switch gps tracking debug");
				break;
			case 'b': //bearing tracking
				global.debug_handler.bearing_tracking_debug = !global.debug_handler.bearing_tracking_debug;
				Serial.println("Command recieved switch bearing tracking debug");
				break;
			case 'd': //data logging
				global.debug_handler.data_logging_debug = !global.debug_handler.data_logging_debug;
				Serial.println("Command recieved switch data logging debug");
				break;
			case 'p': //path finding
				global.debug_handler.path_finding_debug = !global.debug_handler.path_finding_debug;
				Serial.println("Command recieved switch path finding debug");
				break;
			case 'r': //rudder and sail control
				global.debug_handler.rudder_and_sail_control_debug = !global.debug_handler.rudder_and_sail_control_debug;
				Serial.println("Command recieved switch rudder and sail control debug");
				break;
			case 'x': //xbee wireless communication debug control
				global.debug_handler.wireless_communication_debug = !global.debug_handler.wireless_communication_debug;
				Serial.println("Command recieved switch wireless communication debug");
				break;
			case 'm' : //main debug handler
				global.debug_handler.main_debug = !global.debug_handler.main_debug;
				Serial.println("Command recieved switch main debug");
				break;
			case 'c' : //toggle compass calibration
				global.toggle_compass_calibration = !global.toggle_compass_calibration;
				Serial.println("Command recieved switch compass calibration");
				break;
			case 'w' : 
				global.debug_handler.wind_direction_debug = !global.debug_handler.wind_direction_debug;
				Serial.println("Command recieved do wind direction debug");
				break;
			case 'u':
				if (SIMULATOR_MODE) {
					global.bearing_container.compass_bearing += 10;
					if (global.bearing_container.compass_bearing > 360)
					{
						global.bearing_container.compass_bearing -= 360;
					}

					global.wind_bearing -= 10;
					if (global.wind_bearing < -180)
					{
						global.wind_bearing += 360;
					}

				}
				break;
			case 'y':
				if (SIMULATOR_MODE) {
					global.bearing_container.compass_bearing -= 10;
					if (global.bearing_container.compass_bearing < 0)
					{
						global.bearing_container.compass_bearing += 360;
					}

					global.wind_bearing += 10;
					if (global.wind_bearing > 180)
					{
						global.wind_bearing -= 360;
					}
				}
				break;
			case 'h':
				if (SIMULATOR_MODE) {
					global.bearing_container.roll -= 5;
				}
				break;
			case 'j':
				if (SIMULATOR_MODE) {
					global.bearing_container.roll += 5;
				}
				break;
			case 'i':
				if (SIMULATOR_MODE) {
					global.wind_bearing -= 10;
					if (global.wind_bearing < -180)
					{
						global.wind_bearing += 360;
					}
				}
				break;
			case 'o':
				if (SIMULATOR_MODE) {
					global.wind_bearing += 10;
					if (global.wind_bearing > 180)
					{
						global.wind_bearing -= 360;
					}
				}
				break;
			case 'n':
				if (SIMULATOR_MODE) {
					global.debug_handler.show_sim_info = !global.debug_handler.show_sim_info;
				}
				break;
			default:
				Serial.println("unrecognized command");
				break;
			}
		}
		delay(50);
	}
}

#endif