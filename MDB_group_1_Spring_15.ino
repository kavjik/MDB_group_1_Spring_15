#define mySerial Serial1
#define SIMULATOR_MODE true
#define HEART_BEAT_LED 30

#include <Scheduler.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h> 



#include "Adafruit_GPS.h"
#include "bearing_tracking.h"
#include "data_logging.h"
#include "GPS_sensor.h"
#include "path_finding.h"
#include "rudder_and_sail_control.h"
#include "wireless_cummonication.h"
#include "global.h"
#include "gps_tracking.h"
#include "Computer_input_handler.h"
#include "guidance.h"
#include "complex.h"
#include "control_simulator.h"


void setup() {

	
	Serial.begin(115200);
	pinMode(53, INPUT_PULLUP);
	pinMode(51, INPUT_PULLUP);
	pinMode(13, OUTPUT);

	Location target;
	target.latitude = 54.911513888889;
	target.longtitude = 9.781272222222;
	global.waypoints.enqueue(target);


	if (SIMULATOR_MODE) {
		//Scheduler.startLoop(Data_logging);
		//Scheduler.startLoop(Bearing_tracking);
		//Scheduler.startLoop(Location_tracking);
		Scheduler.startLoop(wireless_recieve_thread);
		Scheduler.startLoop(wireless_cummonication);
		//Scheduler.startLoop(path_finding);
		//Scheduler.startLoop(rudder_and_sail_control);
		//Scheduler.startLoop(gps_tracking); 
		//Scheduler.startLoop(computer_input_handler);
		//Scheduler.startLoop(control_simulator);
		//global.gps_data.fix = true;
		//global.gps_data.location.latitude = 54.910513888889;
		//global.gps_data.location.longtitude = 9.781272222222;
	}
	else
	{
		Scheduler.startLoop(Data_logging);
		Scheduler.startLoop(Bearing_tracking);
		Scheduler.startLoop(Location_tracking);
		Scheduler.startLoop(wireless_cummonication);
		Scheduler.startLoop(wireless_recieve_thread);
		Scheduler.startLoop(path_finding);
		Scheduler.startLoop(rudder_and_sail_control);
		Scheduler.startLoop(gps_tracking);
		Scheduler.startLoop(computer_input_handler);
	}
	
}


void loop() { //stayin' alive, stayin' alive.	Ah, ha, ha, ha,		Stayin' alive.		Stayin' alive.		Ah, ha, ha, ha,		Stayin' alive. 
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(500);




	/*if (global.gps_data.fix && global.debug_handler.main_debug){
		Serial.println("got a fix at:");
		Serial.print("lattitude  ");
		Serial.println(global.gps_data.location.latitude);
		Serial.print("longtitude ");
		Serial.println(global.gps_data.location.longtitude);
		Serial.print("bearing    ");
		Serial.println(global.gps_data.gps_bearing);
		Serial.print("speed      ");
		Serial.println(global.gps_data.speed);
		

	}*/
	/*else {
		if (global.debug_handler.main_debug) Serial.println("didnt get a fix");
	}*/
	

}

