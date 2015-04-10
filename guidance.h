#ifndef guidance_h
#define guidance_h

#include "global.h"


enum boat_states
{
	close_hauled_wind_from_left, //as close to the wind as possible
	close_hauled_wind_from_right,
	generel_direction_wind_from_right,
	generel_direction_wind_from_left,
	jibe_going_from_wind_from_left_to_right,
	jibe_going_from_wind_from_right_to_left,
	down_wind_wind_from_right,
	down_wind_wind_from_left,
};


class Navigation_guidance{ //this class contains all the previous groups work on navigating the boat, with mayor changes to suit the way i have made the program.
private:
	boat_states state = close_hauled_wind_from_left;
	boat_states next_state;
	float bearing_to_target; //compass bearing to target
	float bearing_to_target_relative_to_wind; //if 35, target is 35 degrees to the right of the wind
	float distance_to_target; //distance in m
	Location target_location;
	
	float real_part_of_complex_from_old_code = 0;
	float imaginary_part_of_complex_from_old_code = 0;
public:
	Servo Rudder_Servo;

#define INTEGRATOR_MAX 20 // [degrees], influence of the integrator
#define RATEOFTURN_MAX 36 // [degrees/second]
#define GAIN_P -1 //PID Controller Setting
#define GAIN_I 0 //PID Controller Setting
#define TACKING_ZONE 55
#define TACKING_VERTICAL_RANGE 50


	




	void guidance()
	{
		handle_target(); //checks if we are at the target, if we are, go to next
		get_target_info();//calculate distance to target beraing to target and so on
		next_state_logic();
		determine_path_bearing(); //determines which way to go, dependent on which state we are in.
	}

	void guidance_start()
	{
		//test comment
		if (global.waypoints.count() == 0) { //there is no targets
			target_location.latitude = 55;
			target_location.longtitude = 9; //we default to somewhere
			Serial.println("no waypoints avaliable, defaulted to build in guidance target");
		}
		else {
			target_location = global.waypoints.dequeue();
			Serial.println("got start target from waypoint queue");
		}

		distance_to_target = global.gps_data.location.distance_to(target_location);
		bearing_to_target = global.gps_data.location.bearing_to(target_location);
		bearing_to_target_relative_to_wind = ((int)((bearing_to_target - global.global_wind_bearing)))%360;

		if (bearing_to_target_relative_to_wind > 0){ //target is right of wind
			if (bearing_to_target_relative_to_wind > TACKING_ZONE) {
				state = generel_direction_wind_from_left;
				next_state = state;
			}
			else {
				state = close_hauled_wind_from_left;
				next_state = state;
			}
		}
		else { //target is left of wind
			if (bearing_to_target_relative_to_wind < -TACKING_ZONE){
				state = generel_direction_wind_from_right;
				next_state = state;
			}
			else {
				state = close_hauled_wind_from_right;
				next_state = state;
			}
		}
	}

	void handle_target(void){
		if (global.gps_data.location.distance_to(target_location) < 10){ //TODO define this as a constant
			if (global.waypoints.count() != 0){ //there are still waypoints to take
				target_location = global.waypoints.dequeue();
			}
			else{
				//TODO print message here
			}
		}
	}

	void get_target_info(void){

		distance_to_target = global.gps_data.location.distance_to(target_location);
		bearing_to_target = global.gps_data.location.bearing_to(target_location);
		//bearing_to_target_relative_to_wind = fmod(bearing_to_target - global.global_wind_bearing, 360) - 180; //ensure the range is from -180  to +180
		bearing_to_target_relative_to_wind = ((int)((bearing_to_target - global.global_wind_bearing))) % 360;
	
		if (bearing_to_target_relative_to_wind > 180) bearing_to_target_relative_to_wind -= 360;
		if (bearing_to_target_relative_to_wind < -180) bearing_to_target_relative_to_wind += 360;
		
		real_part_of_complex_from_old_code = sin(bearing_to_target_relative_to_wind*PI / 180)*distance_to_target;
		imaginary_part_of_complex_from_old_code = cos(bearing_to_target_relative_to_wind*PI / 180)*distance_to_target;

		
		if (global.debug_handler.path_finding_debug) {
			Serial.println("");
			Serial.print("distance_to_target: ");
			Serial.print(distance_to_target);
			Serial.println("");
			Serial.print("bearing_to_target: ");
			Serial.print(bearing_to_target);
			Serial.println("");
			Serial.print("bearing_to_target_relative_to_wind: ");
			Serial.print(bearing_to_target_relative_to_wind);
			Serial.println("");
			Serial.print("real_part_of_complex_from_old_code: ");
			Serial.print(real_part_of_complex_from_old_code);
			Serial.println("");
			Serial.print("imaginary_part_of_complex_from_old_code: ");
			Serial.print(imaginary_part_of_complex_from_old_code);
			Serial.println("");
		}

	}

	void determine_path_bearing(void){

		//determine global.path_bearing,
		switch (state)
		{
		case close_hauled_wind_from_left:
			global.path_bearing = fmod(global.global_wind_bearing + 55, 360);

			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("close_hauled_wind_from_left");
			}
			break;
		case close_hauled_wind_from_right:
			global.path_bearing = fmod(global.global_wind_bearing - 55, 360);
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("close_hauled_wind_from_right");
			}
			//do stuff
			break;
		case generel_direction_wind_from_right:
			global.path_bearing = bearing_to_target;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("generel_direction_wind_from_right");
			}
			//do stuff
			break;
		case generel_direction_wind_from_left:
			global.path_bearing = bearing_to_target;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("generel_direction_wind_from_left");
			}
			//do stuff
			break;
		case down_wind_wind_from_right:
			global.path_bearing = int(global.global_wind_bearing -135)%360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("down_wind_wind_from_right");
			}
			break;

		case down_wind_wind_from_left:
			global.path_bearing = int(global.global_wind_bearing + 135)%360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("down_wind_wind_from_left");
			}
			break;
		case jibe_going_from_wind_from_left_to_right:
			global.path_bearing = int(global.bearing_container.compass_bearing - 20)%360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("jibe_going_from_wind_from_left_to_right");
			}
			break;
		case jibe_going_from_wind_from_right_to_left:
			global.path_bearing = int(global.bearing_container.compass_bearing + 20)%360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("jibe_going_from_wind_from_right_to_left");
			}
			break;
		default:
			Serial.println("state logic failed, should not be here");
			break;
		}

	}

	void next_state_logic(void){
		/*
		close_hauled_wind_from_left, 
		close_hauled_wind_from_right,
		generel_direction_wind_from_right,
		generel_direction_wind_from_left,
		jibe_going_from_wind_from_left_to_right,
		jibe_going_from_wind_from_right_to_left,
		down_wind_wind_from_right,
		down_wind_wind_from_left,
		*/
		//then next state logic
		next_state = state; //default is staying in the current state
		switch (state)
		{
		case close_hauled_wind_from_left:
			if (bearing_to_target_relative_to_wind > TACKING_ZONE) {
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind < (-TACKING_ZONE)){
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < (-TACKING_ZONE / 2) || bearing_to_target_relative_to_wind < -5 && real_part_of_complex_from_old_code > 10){
				next_state = close_hauled_wind_from_right;
			}

			break;

		case close_hauled_wind_from_right:
			if (bearing_to_target_relative_to_wind < -TACKING_ZONE) {
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind > TACKING_ZONE){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > (TACKING_ZONE / 2) || bearing_to_target_relative_to_wind > 5 && real_part_of_complex_from_old_code < -10){
				next_state = close_hauled_wind_from_left;
			}

			break;

		case generel_direction_wind_from_right:
			if (bearing_to_target_relative_to_wind > -TACKING_ZONE) {
				next_state = close_hauled_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < -135){
				next_state = down_wind_wind_from_right;
			}
			break;

		case generel_direction_wind_from_left:
			if (bearing_to_target_relative_to_wind < TACKING_ZONE) {
				next_state = close_hauled_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > 135){
				next_state = down_wind_wind_from_left;
			}
			break;

		case jibe_going_from_wind_from_left_to_right:
			if (angle_between_two_angles(global.bearing_container.compass_bearing, int(global.global_wind_bearing - 135) % 360) < 10) {
				//we are there
				next_state = down_wind_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < 135 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > -135 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			break;

		case	jibe_going_from_wind_from_right_to_left:
			if (angle_between_two_angles(global.bearing_container.compass_bearing, int(global.global_wind_bearing + 135) % 360) < 10) {
				//we are there
				next_state = down_wind_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < 135 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > -135 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			break;

		case	down_wind_wind_from_right:
			if (bearing_to_target_relative_to_wind > -135 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < 155 && bearing_to_target_relative_to_wind > 135){
				next_state = jibe_going_from_wind_from_right_to_left;
			}
			else if (bearing_to_target_relative_to_wind < 135 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			break;

		case	down_wind_wind_from_left:
			if (bearing_to_target_relative_to_wind < 135 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > -155 && bearing_to_target_relative_to_wind < -135){
				next_state = jibe_going_from_wind_from_left_to_right;
			}
			else if (bearing_to_target_relative_to_wind > -135 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}

			break;

		default:
			Serial.println("error in next boat state logic");
			break;
		}
		//if we want to do special things when changing state, it can be done here.
		state = next_state;

	}

	float angle_between_two_angles(float angle_1, float angle_2){
		float result = angle_1 - angle_2;
		if (result > 180) result -= 360; //if this is true, we went the wrong way around the unit circle
		if (result < -180) result += 360;//if this is true, we went the wrong way around the unit circle
		if (result < 0) result *= -1; //we want the absolute angle in bewteen, we dont care about the sign, so i make sure its always positive.
		return result;
	}


	void rudder_pid_controller()
	{

		float  pValue, integralValue, temp_ang;
		int dHeading;
		dHeading = global.path_bearing - global.bearing_container.compass_bearing; // in degrees
		if (dHeading > 180){
			dHeading -= 360;
		}
		if (dHeading < -180){
			dHeading += 360;
		}
		// Singularity translation
	

		// Limit control statement
		//{
		// P controller
		pValue = GAIN_P * dHeading;

		// Integration part
		// The following checks, will keep integratorSum within -0.2 and 0.2
		/*
		if (integratorSum < -INTEGRATOR_MAX && dHeading > 0) {
			integratorSum = dHeading + integratorSum;
		}
		else if (integratorSum > INTEGRATOR_MAX && dHeading < 0) {
			integratorSum = dHeading + integratorSum;
		}
		else {
			integratorSum = integratorSum;
		}
		integralValue = GAIN_I * integratorSum;
		// result
		*/
		temp_ang = pValue;// +integralValue; // Angle in radians

	
		global.Rudder_Desired_Angle = round(temp_ang);

		if (global.Rudder_Desired_Angle > 35) global.Rudder_Desired_Angle = 35;
		if (global.Rudder_Desired_Angle < -35) global.Rudder_Desired_Angle = -35;

		//Rudder Calibration can be done here
		Rudder_Servo.write(90 - global.Rudder_Desired_Angle);
		if (global.debug_handler.rudder_and_sail_control_debug) {
			Serial.println("");
			Serial.print("d_heading on rudder: ");
			Serial.print(dHeading);
			Serial.println("");
			Serial.print("Rudder_Desired_Angle: ");
			Serial.print(global.Rudder_Desired_Angle);
			Serial.println("");
			Serial.print("global.path_bearing: ");
			Serial.print(global.path_bearing);
			Serial.println("");
			Serial.print("global.bearing_container.compass_bearing: ");
			Serial.print(global.bearing_container.compass_bearing);
			Serial.println("");
			Serial.print("global.global_wind_bearing: ");
			Serial.print(global.global_wind_bearing);
			Serial.println("");

		}

	}

};



#endif
