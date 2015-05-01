#ifndef gps_tracking_h
#define gps_tracking_h

#include "Location.h"
#include "global.h"

#define GPS_SAMPLE_SIZE 5 //sample size for generating an avarage of gps points

// test comment

//it would make more sense to have this as a functino that is called every time we have a new gps coordinate, but when i tried that, nothing worked, so i do it this way instead. 
void gps_tracking() {

	global.gps_data.fix = false; //set it false just to be sure, if had some problems with it being true when not meant to.
	int fix_counter = 0;

	while(1){
		if (!global.GPS_module.fix){ //if there is no gps fix, we wait for a little bit, maintaining the previous position,
			//before we tell the rest of the program that we dont know where we are
			
			if (fix_counter > 10) global.gps_data.fix = false; //at the moment its set to 10
			delay(100);
			fix_counter++;
			continue; //go to start of while 1 loop, ignoring everything below this point

		}
		global.gps_data.Timestampt_of_last_fix = millis(); //keep track on when the last gps fix was
		
		fix_counter = 0; //this is used for keeping track of how many times we dont have a gps fix, therefore its set to 0 here.

		global.gps_data.location.latitude = (fmod(global.GPS_module.latitude, 100) / (60 / 100)) / 100 + floor(global.GPS_module.latitude / 100);
		global.gps_data.location.longtitude = (fmod(global.GPS_module.longitude, 100) / (60 / 100)) / 100 + floor(global.GPS_module.longitude / 100);
		global.gps_data.location.bearing = global.GPS_module.angle;
		global.gps_data.location.speed = global.GPS_module.speed * 0.51444; //constant to convert from knots to m/s

		
		if (global.debug_handler.gps_tracking_debug){ //if we want to debug this thread, print the below.
			Serial.println("GPS tracking has run, is now at:");
			Serial.print("lattitude: ");
			Serial.println(global.gps_data.location.latitude);
			Serial.print("longtitude: ");
			Serial.println(global.gps_data.location.longtitude);
		}
		global.gps_data.fix = true; //we have a fix, now its time to tell everybody

		delay(100);
		yield();
		
	}
	
}

#endif