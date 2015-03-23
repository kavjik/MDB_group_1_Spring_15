#define mySerial Serial1

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


void setup() {

	Serial.begin(115200);
	pinMode(53, INPUT_PULLUP);
	pinMode(51, INPUT_PULLUP);
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


void loop() { //stayin' alive, stayin' alive.	Ah, ha, ha, ha,		Stayin' alive.		Stayin' alive.		Ah, ha, ha, ha,		Stayin' alive. 
	analogWrite(DAC0, 170);
	delay(1000);
	analogWrite(DAC0, 0);
	delay(1000);



	if (global.gps_data.fix && global.debug_handler.main_debug){
		Serial.println("got a fix at:");
		Serial.print("lattitude  ");
		Serial.println(global.gps_data.location.latitude);
		Serial.print("longtitude ");
		Serial.println(global.gps_data.location.longtitude);
		Serial.print("bearing    ");
		Serial.println(global.gps_data.gps_bearing);
		Serial.print("speed      ");
		Serial.println(global.gps_data.speed);
		

	}
	else {
		if (global.debug_handler.main_debug) Serial.println("didnt get a fix");
	}
	

}

