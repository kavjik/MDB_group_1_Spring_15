#define mySerial Serial1
#define SIMULATOR_MODE false
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

	Location target;
	target.latitude = 54.91382778; //first point in water
	target.longtitude = 9.779919444;
	global.waypoints.enqueue(target);
	target.latitude = 54.91358889; //south of the first point
	target.longtitude = 9.780316667;
	global.waypoints.enqueue(target);
	target.latitude = 54.91382778; //back to the first point
	target.longtitude = 9.779919444;
	global.waypoints.enqueue(target);


	if (SIMULATOR_MODE) {
		//Scheduler.startLoop(Data_logging);
		//Scheduler.startLoop(Bearing_tracking);
		//Scheduler.startLoop(Location_tracking);
		Scheduler.startLoop(wireless_cummonication);
		Scheduler.startLoop(wireless_recieve_thread);
		Scheduler.startLoop(path_finding);
		//Scheduler.startLoop(gps_tracking); 
		Scheduler.startLoop(computer_input_handler);
		Scheduler.startLoop(control_simulator);
		global.gps_data.fix = true;
		global.gps_data.location.latitude = 54.910513888889;
		global.gps_data.location.longtitude = 9.781272222222;
	}
	else
	{
		Scheduler.startLoop(Data_logging);
		Scheduler.startLoop(Bearing_tracking);
		Scheduler.startLoop(Location_tracking);
		Scheduler.startLoop(wireless_cummonication);
		Scheduler.startLoop(wireless_recieve_thread);
		Scheduler.startLoop(path_finding);
		Scheduler.startLoop(gps_tracking);
		Scheduler.startLoop(computer_input_handler);
		//Scheduler.startLoop(benchmark_loop);
	}

}

void benchmark_loop(void){ //the cpu is currently being used 72% of the time
	long int previous_time = millis();
	long int counter = 0;
	while (1){
		int j;
		for (int i = 0; i < 100; i++){
			j = i + 2 + j;
		}
		if ((millis() - previous_time) > 10000 && j>2){ //the last statement is to ensure the compiler dosnt throw to much away
			Serial.println(counter);
			counter = 0;
			previous_time = millis();
		}
		counter++;
		yield();
	}
}


void loop() { //stayin' alive, stayin' alive.	Ah, ha, ha, ha,		Stayin' alive.		Stayin' alive.		Ah, ha, ha, ha,		Stayin' alive. 
	digitalWrite(HEART_BEAT_LED, HIGH);
	delay(500);
	digitalWrite(HEART_BEAT_LED, LOW);
	delay(500);




	if (digitalRead(51) && global.debug_handler.main_debug){
		Serial.println("WE ARE NOT IN CONTROL");
	}



}

