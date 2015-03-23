#ifndef gps_tracking_h
#define gps_tracking_h

#include "Location.h"
#include "global.h"

#define GPS_SAMPLE_SIZE 5 //sample size for generating an avarage of gps points


//it would make more sense to have this as a functino that is called every time we have a new gps coordinate, but when i tried that, nothing worked, so i do it this way instead. 
void gps_tracking() {
	Location location_array[GPS_SAMPLE_SIZE];
	double bearing[GPS_SAMPLE_SIZE];
	double speed[GPS_SAMPLE_SIZE];
	global.gps_data.location.latitude = 54.9516;
	global.gps_data.location.longtitude = 9.7645f;
	global.gps_data.fix = false; //set it false just to be sure, if had some problems with it being true when not meant to.

	while (!global.GPS_module.fix){ //wait untill the gps sensor tells us we have a gps fix.
		delay(100);
		yield();
	}
	for (int i = 0; i < GPS_SAMPLE_SIZE; i++){ //populating initial data for calculating avarage, this makes the infite loop simpler
		location_array[i].latitude = (fmod(global.GPS_module.latitude, 100) / (60 / 100)) / 100 + floor(global.GPS_module.latitude / 100);
		location_array[i].longtitude = (fmod(global.GPS_module.longitude, 100) / (60 / 100)) / 100 + floor(global.GPS_module.longitude / 100);
		bearing[i] = global.GPS_module.angle;
		speed[i] = global.GPS_module.speed * 0.51444; //constant to convert from knots to m/s
		delay(100);
		yield();
	}
	int gps_track_counter = 0;
	int fix_counter = 0;
	double lattitude_sum = 0; //double for not loosing accuracy when adding together and dividing
	double longitude_sum = 0;
	double bearing_sum = 0;
	double speed_sum = 0;


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


		location_array[gps_track_counter].latitude = (fmod(global.GPS_module.latitude,100) / (60))+ (int)(global.GPS_module.latitude / 100); //store the value in our array of GPS locations
		location_array[gps_track_counter].longtitude = (fmod(global.GPS_module.longitude , 100) / (60)) + (int)(global.GPS_module.longitude / 100); //gps track counter keeps track of the oldest element in the array, and makes sure its that element which is overwritten
		//the above conversion converts from ddmm.mmmmmm to dd.dddddd
		bearing[gps_track_counter] = global.GPS_module.angle; // i calculate the avarage of the latitude, longtitude, bearing and speed.
		speed[gps_track_counter] = global.GPS_module.speed * 0.51444; //constant to convert from knots to m/s

		for (int i = 0; i < GPS_SAMPLE_SIZE; i++){ //make a sum for calculating the avarage
			lattitude_sum += location_array[i].latitude;
			longitude_sum += location_array[i].longtitude;
			bearing_sum += bearing[i];
			speed_sum += speed[i];
		}

		global.gps_data.location.latitude = (lattitude_sum / GPS_SAMPLE_SIZE); //take the avarage
		global.gps_data.location.longtitude = (longitude_sum / GPS_SAMPLE_SIZE); 
		global.gps_data.gps_bearing = bearing_sum / GPS_SAMPLE_SIZE;
		global.gps_data.speed = (speed_sum / GPS_SAMPLE_SIZE);
		speed_sum = 0; //reset the sums for the next time around in the loop
		bearing_sum = 0;
		lattitude_sum = 0;
		longitude_sum = 0;
		if (global.debug_handler.gps_tracking_debug){ //if we want to debug this thread, print the below.
			Serial.println("GPS tracking has run, is now at:");
			Serial.print("lattitude: ");
			Serial.println(global.gps_data.location.latitude);
			Serial.print("longtitude: ");
			Serial.println(global.gps_data.location.longtitude);
			Serial.print("quick test: ");
			Serial.println(location_array[3].latitude);
		}
		global.gps_data.fix = true; //we have a fix, now its time to tell everybody


		gps_track_counter++; // increment the track counter, so it now again indicates the oldest element in the arrays.
		if (gps_track_counter == GPS_SAMPLE_SIZE) gps_track_counter = 0; //wrap around
		delay(100);
		yield();
		
	}
	
}

#endif