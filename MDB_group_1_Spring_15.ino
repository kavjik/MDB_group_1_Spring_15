#define mySerial Serial1
#define SIMULATOR_MODE true	
#define SIMULATOR_MODE_MOVE_AUTOMATICALLY true //only means something if simulator mode is on
#define HEART_BEAT_LED 30


//below are defines that change between boats
#define WIND_SENSOR_OFFSET_BOAT_DEPENDANT (0)
#define COMPASS_X_MIN -104
#define COMPASS_X_MAX 143
#define COMPASS_Y_MIN -231
#define COMPASS_Y_MAX -47
#define THIS_BOAT boat1 //used if the get_ID fails
//above are defines that change between boats


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

	target.latitude = 54.9135694444; //first point in water
	target.longtitude = 9.7813250000;
	global.waypoints.enqueue(target);

	target.latitude = 54.9127638889; //first point in water
	target.longtitude = 9.7815861111;
	global.waypoints.enqueue(target);

	target.latitude = 54.9122361111; //first point in water
	target.longtitude = 9.7826222222;
	global.waypoints.enqueue(target);

	target.latitude = 54.9131361111; //first point in water
	target.longtitude = 9.7825666667;
	global.waypoints.enqueue(target);

	target.latitude = 54.9124500000; //first point in water
	target.longtitude = 9.7840055556;
	global.waypoints.enqueue(target);

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
	target.latitude = 54.91368056; //first point in water
	target.longtitude = 9.77971667;
	global.waypoints.enqueue(target);
	target.latitude = 54.91359722; //south of the first point
	target.longtitude = 9.77990000;
	global.waypoints.enqueue(target);
	target.latitude = 54.91368056; //back to the first point
	target.longtitude = 9.77971667;
	global.waypoints.enqueue(target);
	target.latitude = 54.91360833; //home
	target.longtitude = 9.77959722;
	global.waypoints.enqueue(target);
	 // setup to outside alsion
	*/
	/*
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
	*/

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
		global.gps_data.location.latitude = 54.9135138889;
		global.gps_data.location.longtitude = 9.7804888889;// ALSION
		//global.gps_data.location.latitude = 54.896811111;
		//global.gps_data.location.longtitude = 9.799813889;


		

		global.other_boats[0].bearing = 270;
		global.other_boats[0].is_valid_boat = true;
		global.other_boats[0].latitude = 0;// 54.91353907;
		global.other_boats[0].longtitude = 0;//9.78084565;
		global.other_boats[0].speed = 2;

		global.other_boats[1].bearing = 200;
		global.other_boats[1].is_valid_boat = true;
		global.other_boats[1].latitude = 54.91237781;
		global.other_boats[1].longtitude = 9.78281377;
		global.other_boats[1].speed = 1;

		global.other_boats[2].bearing = 90;
		global.other_boats[2].is_valid_boat = true;
		global.other_boats[2].latitude = 54.91290631;
		global.other_boats[2].longtitude = 9.78294998;
		global.other_boats[2].speed = 3;

		global.other_boats[3].bearing = 40;
		global.other_boats[3].is_valid_boat = true;
		global.other_boats[3].latitude = 54.91244828;
		global.other_boats[3].longtitude = 9.78400978;
		global.other_boats[3].speed = 4;


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

