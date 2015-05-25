#ifndef path_finding_h
#define path_finding_h

#include "guidance.h"
#include "global.h"

#define FRONT_SAIL_PIN 7
#define rudder_servo_pin 8
#define sail_servo_pin 9
#define PATH_FINDING_SIMULATOR_MODE_WAIT_TIME 750
#define PATH_FINDING_DEFAULT_MODE_WAIT_TIME 100
#define PATH_FINDING_AUTOMATIC_MODE_DELAY_TIME 1000

Navigation_guidance guidance_object; //cant be in global, because of circular references

void path_finding(){
	//this code is build like this, since the Navigation_guidance class is based on code from the previous group, which used a lot of global variables, i dont like global variables, so i have contained them in a class.
	
	guidance_object.guidance_start();

	guidance_object.Rudder_Servo.attach(rudder_servo_pin); //TODO move this to guidance.h
	guidance_object.sail_servo.attach(sail_servo_pin);
	guidance_object.front_sail_servo.attach(FRONT_SAIL_PIN);
	delay(2000); //wait till everything is setup
	while (!global.gps_data.fix && SIMULATOR_MODE == false){ // while we dont have a GPS fix and simulator mode is false, wait.
		delay(100);
	} 
	while (1){
	
		guidance_object.guidance();
		guidance_object.rudder_pid_controller();
		if (SIMULATOR_MODE) {
			if (SIMULATOR_MODE_MOVE_AUTOMATICALLY) {
				delay(PATH_FINDING_AUTOMATIC_MODE_DELAY_TIME);
			}
			else {
				delay(PATH_FINDING_SIMULATOR_MODE_WAIT_TIME);
			}
		}
		else {
			delay(PATH_FINDING_DEFAULT_MODE_WAIT_TIME);
		}

	}
	yield();
}

#endif