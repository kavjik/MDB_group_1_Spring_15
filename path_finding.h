#ifndef path_finding_h
#define path_finding_h

#include "guidance.h"


void path_finding(){
	//this code is build like this, since the Navigation_guidance class is based on code from the previous group, which used a lot of global variables, i dont like global variables, so i have contained them in a class.
	Navigation_guidance guidance_object;
	guidance_object.guidance_start();
	//Location target;
	//target.latitude = 55.90738055555560;
		
	//target.longtitude = 9.79864444444445;


	while (1){
		delay(100);
		
		guidance_object.guidance();
		guidance_object.rudder_pid_controller();
		delay(500);
		//Serial.println(global.gps_data.location.distance_to(target));
		//Serial.println(global.gps_data.location.bearing_to(target));
	}
	

	yield();
	delay(500);
}

#endif