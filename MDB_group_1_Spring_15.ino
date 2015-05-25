#define mySerial Serial1
#define SIMULATOR_MODE false
#define SIMULATOR_MODE_MOVE_AUTOMATICALLY false //only means something if simulator mode is on
#define HEART_BEAT_LED 30


//below are defines that change between boats
#define WIND_SENSOR_OFFSET_BOAT_DEPENDANT 0
#define COMPASS_X_MIN -17
#define COMPASS_X_MAX 175
#define COMPASS_Y_MIN -46
#define COMPASS_Y_MAX 129
#define THIS_BOAT boat4 //used if the get_ID fails

#include "Scheduler.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h> 



#include "Adafruit_GPS.h"
#include "bearing_tracking.h"
#include "data_logging.h"
#include "GPS_sensor.h"
#include "path_finding.h"
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
	//setup a bit north of alsion
	/*
	target.latitude = 54.91504444; //first point in water
	target.longtitude = 9.77412500;
	global.waypoints.enqueue(target);
	target.latitude = 54.91525833; //south of the first point
	target.longtitude = 9.77349167;
	global.waypoints.enqueue(target);
	target.latitude = 54.91504444; //back to the first point
	target.longtitude = 9.77340278;
	*/
	/*
	target.latitude = 54.91382778; //first point in water
	target.longtitude = 9.779919444;
	global.waypoints.enqueue(target);
	target.latitude = 54.91358889; //south of the first point
	target.longtitude = 9.780316667;
	global.waypoints.enqueue(target);
	target.latitude = 54.91382778; //back to the first point
	target.longtitude = 9.779919444;
	 // setup to outside alsion
	*/
	
	target.latitude = 54.896841667; //¨setup at beach
	target.longtitude = 9.799458333;
	global.waypoints.enqueue(target); //first target 
	target.latitude = 54.896663889;
	target.longtitude = 9.799566667;
	global.waypoints.enqueue(target); // second target
	target.latitude = 54.896630556;
	target.longtitude = 9.799941667;
	global.waypoints.enqueue(target); // thirds target
	target.latitude = 54.896663889;
	target.longtitude = 9.799566667;
	global.waypoints.enqueue(target); // second target
	target.latitude = 54.896841667; 
	target.longtitude = 9.799458333;
	global.waypoints.enqueue(target); //first target 
	

	if (SIMULATOR_MODE) {
		Scheduler.startLoop(Data_logging);
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
		global.gps_data.location.longtitude = 9.781272222222;// ALSION
		//global.gps_data.location.latitude = 54.896811111;
		//global.gps_data.location.longtitude = 9.799813889;

		global.other_boats[0].bearing = 180;
		global.other_boats[0].is_valid_boat = true;
		global.other_boats[0].latitude = 54.9129768;
		global.other_boats[0].longtitude = 9.7798641;
		global.other_boats[0].speed = 2;


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
		Scheduler.startLoop(wireless_maintain_if_boat_is_valid_thread);
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

