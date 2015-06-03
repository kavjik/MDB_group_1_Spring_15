#ifndef gps_tracking_h
#define gps_tracking_h

#include "Location.h"
#include "global.h"

#define GPS_SAMPLE_SIZE 5 //sample size for generating an avarage of gps points

// test comment


bool is_gps_module_longtitude_valid(void){ //this is a very ugly fix to a very ugly problem
	global.longtitude_fix_triggered = true;
	if (global.GPS_module.longitude > 1050) return false;
	if (global.GPS_module.longitude < 910) return false;
	global.longtitude_fix_triggered = false;
	return(true);

}
bool is_gps_module_latittude_valid(void){ //this is a very ugly fix to a very ugly problem
	global.lattitude_fix_triggered = true;
	if (global.GPS_module.latitude > 5750) return false;
	if (global.GPS_module.latitude < 5450) return false;
	global.lattitude_fix_triggered = false;
	return(true);
}

void return_on_sensible_gps_location(void){
	//first we need to collect samples over 10 seconds 
	//then we find the first where the distance to 5 other points is less than 100
	//then we say this is our new location
	//if not, we try again, not recursive, but by not breaking the while 1 loop
	Location samples[10];
	bool found = false;
	int count = 0;
	while (1){
		for (int i = 0; i < 10; i++){
			samples[i].latitude = floor(global.GPS_module.latitude / 100) + (global.GPS_module.latitude / 100 - floor(global.GPS_module.latitude / 100)) * 100 / 60;
			samples[i].longtitude = floor(global.GPS_module.longitude / 100) + (global.GPS_module.longitude / 100 - floor(global.GPS_module.longitude / 100)) * 100 / 60;
			delay(1000);
		}
		for (int i = 0; i < 5; i++){ //if we havnt found it in the first 10, we wont
			count = 0;
			for (int j = i + 1; j < 10; j++){ //for everyone, we check for all after that
				if (samples[i].distance_to(samples[j]) < 100){
					count++;
					if (count >= 5){
						found = true; 
						global.gps_data.location = samples[i]; //we have found a valid location
						break;
					}
				}
				if (found)
				{
					break;
				}
			}
			if (found)
			{
				break;
			}
		}
		if (global.debug_handler.gps_tracking_debug){
			if(!found) Serial.println("failed finding a valid gps position");
			else Serial.println("found a valid gps position");
		}
		if (found)
		{
			break;
		}
	}
}

//it would make more sense to have this as a functino that is called every time we have a new gps coordinate, but when i tried that, nothing worked, so i do it this way instead. 
void gps_tracking() {
	global.longtitude_fix_triggered = false;
	global.lattitude_fix_triggered = false;

	global.gps_data.fix = false; //set it false just to be sure, i had some problems with it being true when not meant to.
	int fix_counter = 0;


	delay(1000); //get a chance for it to start

	int fail_count = 0;
	while (!global.GPS_module.fix){ //wait till it says we got a fix
		delay(100);
	}

	return_on_sensible_gps_location(); //then find the current location

	while(1){
		if (!global.GPS_module.fix){ 
			if (fix_counter > 10) global.gps_data.fix = false; //at the moment its set to 10
			delay(100);
			fix_counter++;
			continue; //go to start of while 1 loop, ignoring everything below this point

		}

		global.gps_data.Timestampt_of_last_fix = millis(); //keep track on when the last gps fix was
		
		fix_counter = 0; //this is used for keeping track of how many times we dont have a gps fix, therefore its set to 0 here.

		//global.gps_data.location.latitude = (fmod(global.GPS_module.latitude, 100) / (60)) + floor(global.GPS_module.latitude / 100);

		Location potential_current_location;
		
		potential_current_location.latitude = floor(global.GPS_module.latitude / 100) + (global.GPS_module.latitude / 100 - floor(global.GPS_module.latitude / 100)) * 100 / 60;
		potential_current_location.longtitude = floor(global.GPS_module.longitude / 100) + (global.GPS_module.longitude / 100 - floor(global.GPS_module.longitude / 100)) * 100 / 60;

		if (potential_current_location.distance_to(global.gps_data.location) < 200) { //its valid
			global.gps_data.location = potential_current_location;
			fail_count = 0;
		}
		else
		{
			fail_count++;
			if (global.debug_handler.gps_tracking_debug){
				Serial.print("GPS invalid location found, count: ");
				Serial.println(fail_count);
			}
			if (fail_count > 50){
				return_on_sensible_gps_location();
				fail_count = 0;
			}
		}



		global.gps_data.location.bearing = global.GPS_module.angle;
		global.gps_data.location.speed = global.GPS_module.speed * 0.51444; //constant to convert from knots to m/s

		/*
		if (global.debug_handler.gps_tracking_debug){ //if we want to debug this thread, print the below.
			Serial.println("GPS tracking has run, is now at:");
			Serial.print("lattitude: ");
			Serial.println(global.gps_data.location.latitude);
			Serial.print("longtitude: ");
			Serial.println(global.gps_data.location.longtitude);
		}
		*/ //demed so reliable, we only want to see error messages
		global.gps_data.fix = true; //we have a fix, now its time to tell everybody

		delay(100);
		yield();
		
	}
	
}



#endif