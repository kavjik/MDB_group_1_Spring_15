

#ifndef computer_input_handler_h
#define computer_input_handler_h

#include "global.h"

//this thread takes inputs from the computer attached, if such a thing is attached
//at the moment its only used for switching debug messages on or off, but later its supposed to also control things like calibration.
//se list of commands in the switch case below.

void simulator_mode_move_forward(void) {

	//below variables is for calculating going forward in simulator mode
	double a1; // Latitude of the given point in radian
	double b1; // Longitude of the given point in radian
	double a2; // Latitude of the destination point in radian
	double b2; // Latitude of the destination point in radian
	double d; // Distance from the given point to the destination point in km
	double R; // The earth radius in km
	double cb; // The compass bearing from the 0 degree of north in clockwise direction (in radian) 


	a1 = global.gps_data.location.latitude * PI / 180.0;
	b1 = global.gps_data.location.longtitude * PI / 180.0;
	R = 6378.137;
	d = 0.5 / 1000.0;
	cb = global.bearing_container.compass_bearing * PI / 180.0;

	a2 = asin(sin(a1)*cos(d / R) + cos(a1)*sin(d / R)*cos(cb));
	b2 = b1 + atan2(sin(cb)*sin(d / R)*cos(a1), cos(d / R) - sin(a1)*sin(a2));

	global.gps_data.location.latitude = a2 * 180 / PI; // Latitude of destination is a2 that we convert it again to degree
	global.gps_data.location.longtitude = b2 * 180 / PI; // Longitude of destination is b2 that we convert it again to degree

}

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
			case 'z':
				global.debug_handler.remote_commands_wireless_communication_debug = !global.debug_handler.remote_commands_wireless_communication_debug;
				Serial.println("Command recieved do remote commands wireless communication_debug");
				break;
				// below this point is the implementation of simulator mode
			case 'u':
			case '9':
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
			case '7': 
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

				//global.gps_data.location.latitude = 54.910513888889;
				//global.gps_data.location.longtitude = 9.781272222222;
			case '8': //north
				if (SIMULATOR_MODE) {
					global.gps_data.location.latitude += 0.0001;
				}
				break;
			case '2': //south 
				if (SIMULATOR_MODE) {
					global.gps_data.location.latitude -= 0.0001;
				}
				break;
			case '4': //west
				if (SIMULATOR_MODE) {
					global.gps_data.location.longtitude -= 0.0001;
				}
				break;
			case '6': // east
				if (SIMULATOR_MODE) {
					global.gps_data.location.longtitude += 0.0001;
				}
				break;
			case '+': // forward
				if (SIMULATOR_MODE) {
					simulator_mode_move_forward();
				}
				break;
			default:
				Serial.println("unrecognized command");
				break;
			}
		}
		delay(100);
	}
}


#endif