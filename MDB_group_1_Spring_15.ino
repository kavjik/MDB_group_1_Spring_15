#define mySerial Serial1
#define SIMULATOR_MODE false	
#define SIMULATOR_MODE_MOVE_AUTOMATICALLY false //only means something if simulator mode is on
#define HEART_BEAT_LED 30


////below are defines that change between boats //red boat
//#define WIND_SENSOR_OFFSET_BOAT_DEPENDANT (126)
//#define COMPASS_X_MIN -104
//#define COMPASS_X_MAX 143
//#define COMPASS_Y_MIN -231
//#define COMPASS_Y_MAX -47
//#define BIG_SWING_ON_FRONT_SAIL true
//#define THIS_BOAT boat1 //used if the get_ID fails
////above are defines that change between boats

////below are defines that change between boats //blue boat
//#define WIND_SENSOR_OFFSET_BOAT_DEPENDANT (-107)
//#define COMPASS_X_MIN -838
//#define COMPASS_X_MAX -698
//#define COMPASS_Y_MIN 358
//#define COMPASS_Y_MAX 521
//#define BIG_SWING_ON_FRONT_SAIL false
//#define THIS_BOAT boat2 //used if the get_ID fails
////above are defines that change between boats

//below are defines that change between boats //yelow boat
#define WIND_SENSOR_OFFSET_BOAT_DEPENDANT (-114)
#define COMPASS_X_MIN -84
#define COMPASS_X_MAX 61
#define COMPASS_Y_MIN -18
#define COMPASS_Y_MAX 87
#define BIG_SWING_ON_FRONT_SAIL false
#define THIS_BOAT boat3 //used if the get_ID fails
//above are defines that change between x


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
	//blue
	target.latitude = 54.913941667;
	target.longtitude = 9.779511111;
	global.waypoints.enqueue(target); //A
	target.latitude = 54.913694444;
	target.longtitude = 9.780158333;
	global.waypoints.enqueue(target); //B
	target.latitude = 54.913613889;
	target.longtitude = 9.779650000;
	global.waypoints.enqueue(target); //C
	target.latitude = 54.913941667;
	target.longtitude = 9.779511111;
	global.waypoints.enqueue(target); //A
	target.latitude = 54.913694444;
	target.longtitude = 9.780158333;
	global.waypoints.enqueue(target); //B
	target.latitude = 54.913613889;
	target.longtitude = 9.779650000;
	global.waypoints.enqueue(target); //C
	target.latitude = 54.913941667;
	target.longtitude = 9.779511111;
	global.waypoints.enqueue(target); //A
	target.latitude = 54.913694444;
	target.longtitude = 9.780158333;
	global.waypoints.enqueue(target); //B
	target.latitude = 54.913613889;
	target.longtitude = 9.779650000;
	global.waypoints.enqueue(target); //C
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


	

		global.other_boats[0].bearing = 40;
		global.other_boats[0].is_valid_boat = true;
		global.other_boats[0].latitude = 54.91326366;
		global.other_boats[0].longtitude = 9.78124791;
		global.other_boats[0].speed = 2;

		global.other_boats[1].bearing = 100;
		global.other_boats[1].is_valid_boat = true;
		global.other_boats[1].latitude = 54.91264944;
		global.other_boats[1].longtitude = 9.78197736;
		global.other_boats[1].speed = 1;

		global.other_boats[2].bearing = 200;
		global.other_boats[2].is_valid_boat = true;
		global.other_boats[2].latitude = 54.91259097;
		global.other_boats[2].longtitude = 9.7825118;
		global.other_boats[2].speed = 3;

		global.other_boats[3].bearing = 270;
		global.other_boats[3].is_valid_boat = true;
		global.other_boats[3].latitude = 54.91281952;
		global.other_boats[3].longtitude = 9.78327043;
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

