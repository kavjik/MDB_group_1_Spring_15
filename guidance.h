#ifndef guidance_h
#define guidance_h

#include "global.h"
#include "Computer_input_handler.h"


enum boat_states
{
	close_hauled_wind_from_left, //0
	close_hauled_wind_from_right,//1
	tacking_going_from_wind_from_left_to_right, //2
	tacking_going_from_wind_from_right_to_left, //3
	generel_direction_wind_from_right, //4
	generel_direction_wind_from_left, //5
	jibe_going_from_wind_from_left_to_right, //6
	jibe_going_from_wind_from_right_to_left, //7
	down_wind_wind_from_right, // 8
	down_wind_wind_from_left, // 9
};


class Navigation_guidance{ //this class contains all the previous groups work on navigating the boat, with mayor changes to suit the way i have made the program.
public:
	bool a = 0;
	boat_states state = close_hauled_wind_from_left;
	boat_states next_state;
	float bearing_to_target; //compass bearing to target
	float bearing_to_target_relative_to_wind; //if 35, target is 35 degrees to the right of the wind
	float distance_to_target; //distance in m
	float frozen_tacking_direction;
	bool has_boat_been_at_tacking_target;
	long int time_stamp_for_tacking;
	Location target_location;
	
	float real_part_of_complex_from_old_code = 0;
	float imaginary_part_of_complex_from_old_code = 0;
	float theta_LOS;
	float Theta_LOS_relative_to_wind;
	float bearing_to_target_relative_to_LOS;

	Servo Rudder_Servo;
	Servo sail_servo;
	Servo front_sail_servo;


	float theta_A;
	float theta_B;
	float theta_AB;
	float theta_BA;
	float x;
	bool collision_avoidance_active = false;
	bool collision_avoidance_did_evasion = false;


#define INTEGRATOR_MAX 20 // [degrees], influence of the integrator
#define RATEOFTURN_MAX 36 // [degrees/second]
#define GAIN_P -1 //PID Controller Setting
#define GAIN_I 0 //PID Controller Setting
#define TACKING_ZONE 55
#define SAIL_CONTROL_ZERO_POINT 45
#define IN_RANGE_DISTANCE 6
#define DOWN_WIND_ZONE 135
#define TACKING_TURNING_VALUE 20
#define TACKING_ZONE_DISTANCE 10
#define TACKING_ZONE_WIDE_ANGLE (TACKING_ZONE / 2)
#define TACKING_ZONE_NARROW_ANGLE 5
#define TOLERANCE_FOR_COMPLETED_TACKING 15
#define TIMEOUT_FOR_COMPLETED_TACKING 8000
#define RUDDER_LIMIT 45 
#define COLLISION_AVOIDANCE_INNER_LIMIT 10
#define COLLISION_AVOIDANCE_OUTER_LIMIT 20
#define MAX_ALLOWED_ROLL_BEFORE_CORRECTION 30
#define JIBING_TURNING_VALUE 40


	


	void guidance()
	{
		handle_target(); //checks if we are at the target, if we are, go to next
		get_target_info();//calculate distance to target beraing to target and so on
		next_state_logic();
		state = next_state; //this is here to ensure that collision avoidance can "avoid" this thing. 
		determine_path_bearing(); //determines which way to go, dependent on which state we are in.
		do_colission_avoidance(); //may overwrite the next state logic
		
		sail_control();
		
		if (SIMULATOR_MODE && SIMULATOR_MODE_MOVE_AUTOMATICALLY) {
			global.global_wind_bearing = 330;

			global.bearing_container.compass_bearing = global.desired_heading; //turn to the way we want to go
			global.wind_bearing = (int)(global.bearing_container.compass_bearing - global.global_wind_bearing) % 360;
			if (global.wind_bearing > 180) global.wind_bearing -= 360;
			if (global.wind_bearing < -180) global.wind_bearing += 360;
			simulator_mode_move_forward(); //move forward
			//Serial.println(global.wind_bearing);
		}
		rudder_pid_controller();
	}

	void do_colission_avoidance(void){ //TODO add debug messages to this
		
		//first determine if we should do collision avoidance
		int do_avoidance = -1; //this marks which boat we avoid, its implemented as only 1 boat for now.
		bool forced = false; //this determine wheter we avoid no mater what, this is the case if we are very close to the other boat.
		int minimum_distance_to_other_boat = COLLISION_AVOIDANCE_OUTER_LIMIT;
		for (int i = 0; i < NUMBER_OF_OTHER_BOATS_IN_BOAT_ARRAY; i++)
		{
			if (global.other_boats[i].is_valid_boat){
				if (global.gps_data.location.distance_to(global.other_boats[i]) < minimum_distance_to_other_boat) {
					minimum_distance_to_other_boat = global.gps_data.location.distance_to(global.other_boats[i]);
					do_avoidance = i;
				}
			}
		}

		// HARD COLLISION AVOIDANCE 
		collision_avoidance_active = false;
		collision_avoidance_did_evasion = false;
		if (do_avoidance != -1) {
			collision_avoidance_active = true;
			bool do_avoidance = false;
			theta_A = global.bearing_container.compass_bearing - global.global_wind_bearing;//Boat A heading direction refers to winding direction
			if (theta_A > 180) theta_A -= 360;
			if (theta_A < -180) theta_A += 360;
			theta_B = global.other_boats[do_avoidance].bearing - global.global_wind_bearing;//Boat B heading direction refers to winding direction 
			if (theta_B > 180) theta_B -= 360;
			if (theta_B < -180) theta_B += 360;
			theta_AB = global.gps_data.location.bearing_to(global.other_boats[do_avoidance]) - global.global_wind_bearing;//theta_AB is the angle between wind direction and line A-B(draw straight line from boat A to boat B )
			if (theta_AB > 180) theta_AB -= 360;
			if (theta_AB < -180) theta_AB += 360;
			theta_BA = theta_AB - 180;//theta _BA is the angle between wind direction and Line B-A(draw straight line from boat B to boat A )
			if (theta_BA < -180) theta_BA += 360; 
			if (theta_BA > 180) theta_BA -= 360;
			
			float v_A = global.gps_data.location.speed;//the velocity of Boat A
			float v_B = global.other_boats[do_avoidance].speed;//the velocity of Boat B
			x = global.gps_data.location.distance_to(global.other_boats[do_avoidance]);//is the distance between boat A and boat B
			float theta_rub = TACKING_ZONE;
			float theta_lub = -TACKING_ZONE;
			float theta_rdb = DOWN_WIND_ZONE;
			float theta_ldb = -DOWN_WIND_ZONE;
			float avoidance_angle = 55;

			if (x < 10)
			{
				do_avoidance = true;
			}
			else
			{
				if (theta_A > 0 && theta_B > 0){
					if (theta_AB > theta_A){
						//the other boat is to the right, we should move
						do_avoidance = true;
					}
					else { //its to the left of us, we should move
						do_avoidance = false;
					}
				}
				else if(theta_A > 0 && theta_B < 0){
					do_avoidance = true;
				}
				else if(theta_A < 0 && theta_B > 0){
					do_avoidance = false;
				}
				else if(theta_A < 0 && theta_B < 0){ //the check raelly dosnt have to be there, but its just for checking
					if (theta_AB > theta_A){
						//same as aboive, but inversed, since wind is coming from the right now
						do_avoidance = false;
					}
					else { 
						do_avoidance = false;
					}
				}
				 //special overtaking check, only applies if we 
				if (angle_between_two_angles(theta_A, theta_AB) < 20 && angle_between_two_angles(theta_B, theta_A) < 20){ //its straight ahead, and pointing the same way as us, we should move
					do_avoidance = true;
				}
				else if (angle_between_two_angles(theta_A, theta_AB) > (180 - 20) && angle_between_two_angles(theta_B, theta_A) < 20)//its right behind us and pointing the same way as us, they are overtaking and should move.
				{
					do_avoidance = false;
				}
				
			}

			if (do_avoidance){
				//now we know we should avoid, but if we are already sailing away, there is no need to activly do anything, so lets check that.
				float angle_to_other_boat = angle_between_two_angles(bearing_to_target_relative_to_wind, theta_BA);
				if (angle_to_other_boat < 50) //todo make this depend on simulator mode
				{
					avoidance_angle = 70 - angle_to_other_boat; //should depend on simulator mode

				}
				else {
					avoidance_angle = 40+(130 - (angle_to_other_boat - 50))*0.25; //TODO make this depend on simulator mode and defines
				}
				
				if (bearing_to_target_relative_to_wind > 0){ //wind from left
					if (theta_B < bearing_to_target_relative_to_wind && theta_B > bearing_to_target_relative_to_wind - 180){ //the other boat is to the left of us, turn right 
						global.desired_heading = (int)(bearing_to_target_relative_to_wind + avoidance_angle) % 360;
					}
					else {//the other boat is to the right of us, turn left 
						global.desired_heading = (int)(bearing_to_target_relative_to_wind - avoidance_angle) % 360;
					}
				}
				else { //wind from right
					if (theta_B < bearing_to_target_relative_to_wind && theta_B > bearing_to_target_relative_to_wind + 180){ //the other boat is to the left of us, turn right 
						global.desired_heading = (int)(bearing_to_target_relative_to_wind + avoidance_angle) % 360;
					}
					else {//the other boat is to the right of us, turn left 
						global.desired_heading = (int)(bearing_to_target_relative_to_wind - avoidance_angle) % 360;
					}
				}
				//now we have determined the new path, we should check if that makes us go into dead zone, if it does, change the state 
				float desired_heading_relative_to_wind = ((int)((global.desired_heading - global.global_wind_bearing))) % 360;
				if (desired_heading_relative_to_wind > 180) desired_heading_relative_to_wind -= 360;
				if (desired_heading_relative_to_wind < -180) desired_heading_relative_to_wind += 360;
				if (desired_heading_relative_to_wind < TACKING_ZONE && desired_heading_relative_to_wind > -TACKING_ZONE){
					//we should tack
					do_tack_for_collision_avoidance();
				}
				else if (desired_heading_relative_to_wind > DOWN_WIND_ZONE && desired_heading_relative_to_wind < -DOWN_WIND_ZONE){
					//we should jibe
					do_jibe_for_collision_avoidance();
				}
				
			
			}
		}
	}
	void do_tack_for_collision_avoidance(void){
		if (theta_AB > 0) { //the other boat is to the right of the wind
			next_state = tacking_going_from_wind_from_left_to_right;
		}
		else {
			next_state = tacking_going_from_wind_from_right_to_left;
		}
		determine_path_bearing();//TODO determine wheter this is a good idea
	
	}
	void do_jibe_for_collision_avoidance(void){
		if (theta_AB > 0) { //the other boat is to the right of the wind
			next_state = jibe_going_from_wind_from_left_to_right;
		}
		else {
			next_state = jibe_going_from_wind_from_right_to_left;
		}
		determine_path_bearing(); //TODO determine wheter this is a good idea
	}

	void sail_control(void){
		int sail_control_value = global.desired_heading - global.global_wind_bearing;// SAIL_CONTROL_ZERO_POINT;
		if (sail_control_value < 0) sail_control_value *= -1;
		if (sail_control_value > 180){
			sail_control_value *= -1;
			sail_control_value += 360;
		}
		
		if (sail_control_value < 55) sail_control_value = 0;
		else sail_control_value -= 55; 

		sail_control_value *= 2.25;
		sail_control_value *= -1; //inverting
		sail_control_value += 180; //inverting
		if (global.bearing_container.roll > MAX_ALLOWED_ROLL_BEFORE_CORRECTION || global.bearing_container.roll < -MAX_ALLOWED_ROLL_BEFORE_CORRECTION){
			sail_control_value = 0;
		}
		sail_servo.write(sail_control_value);

		if (global.debug_handler.path_finding_debug) {
			Serial.print("main sail: ");
			Serial.print(sail_control_value);
			Serial.println("");


		}
		if (global.wind_bearing > 0){
			front_sail_servo.write(180);
			if(global.debug_handler.path_finding_debug) Serial.println("wind from right");
		}
		else {
			front_sail_servo.write(0);
			if (global.debug_handler.path_finding_debug) Serial.println("wind from left");
		}
		
	}
	 public:
	void guidance_start()
	{
		//test comment
		if (global.waypoints.actual_size== 0) { //there is no targets
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
		if (bearing_to_target_relative_to_wind > 180) bearing_to_target_relative_to_wind -= 360;
		if (bearing_to_target_relative_to_wind < -180) bearing_to_target_relative_to_wind += 360;
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
		theta_LOS = global.gps_data.location.bearing_to(target_location);
	}

	void handle_target(void){
		if (global.gps_data.location.distance_to(target_location) < IN_RANGE_DISTANCE || global.force_load_waypoint_in_guidance){ 
			if (global.waypoints.actual_size> 0){ //there are still waypoints to take
				target_location = global.waypoints.dequeue();
				global.force_load_waypoint_in_guidance = false;
				theta_LOS = global.gps_data.location.bearing_to(target_location);
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
		Theta_LOS_relative_to_wind = theta_LOS - global.global_wind_bearing;
		if (Theta_LOS_relative_to_wind > 180) Theta_LOS_relative_to_wind -= 360;
		if (Theta_LOS_relative_to_wind < -180) Theta_LOS_relative_to_wind += 360;
		bearing_to_target_relative_to_LOS = theta_LOS - bearing_to_target; //if positive, we are left of the original trajectory
		if (bearing_to_target_relative_to_LOS > 180) bearing_to_target_relative_to_LOS -= 360;
		if (bearing_to_target_relative_to_LOS < -180) bearing_to_target_relative_to_LOS += 360;
		
	
		
		real_part_of_complex_from_old_code = sin(bearing_to_target_relative_to_LOS*PI / 180)*distance_to_target;
		imaginary_part_of_complex_from_old_code = cos(bearing_to_target_relative_to_LOS*PI / 180)*distance_to_target;

		//now in order to make the boat stay within the boundaries, we simply modify the bearing to target according to how far away from the original path we are, though to a maximum of 60
		

		bearing_to_target_relative_to_wind -= (real_part_of_complex_from_old_code > 80 ? 40 : real_part_of_complex_from_old_code/2);
		
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
			Serial.print("global.waypoints.count: ");
			Serial.print(global.waypoints.count());
			Serial.println("");
			Serial.print("x: ");
			Serial.print(x);
			Serial.println("");
		}


	}

	void determine_path_bearing(void){

		//determine global.path_bearing,
		switch (state)
		{
		case close_hauled_wind_from_left:
			global.desired_heading = int(global.global_wind_bearing + TACKING_ZONE)% 360;

			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("close_hauled_wind_from_left");
			}
			break;
		case close_hauled_wind_from_right:
			global.desired_heading = int(global.global_wind_bearing - TACKING_ZONE) % 360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("close_hauled_wind_from_right");
			}
			//do stuff
			break;

			/*	tacking_going_from_wind_from_left_to_right,
	tacking_going_from_wind_from_right_to_left,
	*/
		case tacking_going_from_wind_from_left_to_right: 
			//høns
			global.desired_heading = frozen_tacking_direction;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("tacking_going_from_wind_from_left_to_right");
			}
			//do stuff
			break;
		case tacking_going_from_wind_from_right_to_left:
			//høns
			global.desired_heading = frozen_tacking_direction;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("tacking_going_from_wind_from_right_to_left");
			}
			//do stuff
			break;
		case generel_direction_wind_from_right:
			global.desired_heading = bearing_to_target;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("generel_direction_wind_from_right");
			}
			//do stuff
			break;
		case generel_direction_wind_from_left:
			global.desired_heading = bearing_to_target;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("generel_direction_wind_from_left");
			}
			//do stuff
			break;
		case down_wind_wind_from_right:
			global.desired_heading = int(global.global_wind_bearing - DOWN_WIND_ZONE) % 360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("down_wind_wind_from_right");
			}
			break;

		case down_wind_wind_from_left:
			global.desired_heading = int(global.global_wind_bearing + DOWN_WIND_ZONE) % 360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("down_wind_wind_from_left");
			}
			break;
		case jibe_going_from_wind_from_left_to_right:
			global.desired_heading = int(global.bearing_container.compass_bearing - JIBING_TURNING_VALUE) % 360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("jibe_going_from_wind_from_left_to_right");
			}
			break;
		case jibe_going_from_wind_from_right_to_left:
			global.desired_heading = int(global.bearing_container.compass_bearing + JIBING_TURNING_VALUE) % 360;
			if (global.debug_handler.path_finding_debug){
				Serial.print("state is:");
				Serial.println("jibe_going_from_wind_from_right_to_left");
			}
			break;
		default:
			Serial.println("state logic failed, should not be here");
			break;
		}

		if (global.desired_heading > 360) global.desired_heading -= 360;
		if (global.desired_heading < 0) global.desired_heading += 360;

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
			if (bearing_to_target_relative_to_wind > TACKING_ZONE+5) {
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind < (-TACKING_ZONE-5)){
				next_state = generel_direction_wind_from_right;
			}
			/*
			else if (real_part_of_complex_from_old_code > 20){
				next_state = tacking_going_from_wind_from_left_to_right;
				has_boat_been_at_tacking_target = false;
				frozen_tacking_direction = global.global_wind_bearing - TACKING_ZONE;
				if (frozen_tacking_direction < 0) frozen_tacking_direction += 360;
			}
			else if (real_part_of_complex_from_old_code < -20){
				next_state = generel_direction_wind_from_left;
			}*/
			/*
			else if (bearing_to_target_relative_to_wind < (-TACKING_ZONE_WIDE_ANGLE) || bearing_to_target_relative_to_wind < -TACKING_ZONE_NARROW_ANGLE && real_part_of_complex_from_old_code > TACKING_ZONE_DISTANCE){
				next_state = tacking_going_from_wind_from_left_to_right;
				has_boat_been_at_tacking_target = false;
				frozen_tacking_direction = global.global_wind_bearing - TACKING_ZONE;
				if (frozen_tacking_direction < 0) frozen_tacking_direction += 360;
			}
			*/
			else if (/*bearing_to_target_relative_to_LOS < (-TACKING_ZONE_WIDE_ANGLE) || bearing_to_target_relative_to_LOS < -TACKING_ZONE_NARROW_ANGLE && */real_part_of_complex_from_old_code > TACKING_ZONE_DISTANCE){
				next_state = tacking_going_from_wind_from_left_to_right;
				has_boat_been_at_tacking_target = false;
				frozen_tacking_direction = global.global_wind_bearing - TACKING_ZONE;
				if (frozen_tacking_direction < 0) frozen_tacking_direction += 360;
			}
			else if (real_part_of_complex_from_old_code < -20){
				next_state = generel_direction_wind_from_left;
			}
			break;

		case close_hauled_wind_from_right:
			if (bearing_to_target_relative_to_wind < -TACKING_ZONE) {
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind > TACKING_ZONE){
				next_state = generel_direction_wind_from_left;
			}
			/*
			#define TACKING_ZONE_DISTANCE 20
#define TACKING_ZONE_WIDE_ANGLE TACKING_ZONE / 2
#define TACKING_ZONE_NARROW_ANGLE 5
			*/
			else if (/*bearing_to_target_relative_to_LOS > (TACKING_ZONE_WIDE_ANGLE) || bearing_to_target_relative_to_LOS > TACKING_ZONE_NARROW_ANGLE &&*/ real_part_of_complex_from_old_code < -TACKING_ZONE_DISTANCE){
				next_state = tacking_going_from_wind_from_right_to_left;
				has_boat_been_at_tacking_target = false;
				frozen_tacking_direction = global.global_wind_bearing + TACKING_ZONE;
				if (frozen_tacking_direction > 360) frozen_tacking_direction -= 360;
			}
			else if (real_part_of_complex_from_old_code > 20){
				next_state = generel_direction_wind_from_right;
			}
			/*
			else if (bearing_to_target_relative_to_wind > (TACKING_ZONE_WIDE_ANGLE) || bearing_to_target_relative_to_wind > TACKING_ZONE_NARROW_ANGLE && real_part_of_complex_from_old_code < -TACKING_ZONE_DISTANCE){
				next_state = tacking_going_from_wind_from_right_to_left;
				frozen_tacking_direction = global.global_wind_bearing + TACKING_ZONE;
				has_boat_been_at_tacking_target = false;
				if (global.global_wind_bearing > 360) global.global_wind_bearing -= 360;
			}
			*/
			break;


			/*	tacking_going_from_wind_from_left_to_right,
			tacking_going_from_wind_from_right_to_left,
			*/
		case tacking_going_from_wind_from_left_to_right :
			if (angle_between_two_angles(global.bearing_container.compass_bearing, frozen_tacking_direction) < TOLERANCE_FOR_COMPLETED_TACKING && has_boat_been_at_tacking_target == false) {
				has_boat_been_at_tacking_target = true;
				time_stamp_for_tacking = millis();
			}
			if ((millis() - time_stamp_for_tacking) > TIMEOUT_FOR_COMPLETED_TACKING && has_boat_been_at_tacking_target == true) {
				next_state = close_hauled_wind_from_right;
			}
			if (bearing_to_target_relative_to_wind < (-TACKING_ZONE-5)) {
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind > (TACKING_ZONE+5)){
				next_state = generel_direction_wind_from_left;
			}

			break;
			/*#define TOLERANCE_FOR_COMPLETED_TACKING 15
#define TIMEOUT_FOR_COMPLETED_TACKING 5000*/
		case tacking_going_from_wind_from_right_to_left :
			if (angle_between_two_angles(global.bearing_container.compass_bearing, frozen_tacking_direction) < TOLERANCE_FOR_COMPLETED_TACKING && has_boat_been_at_tacking_target == false) {
				has_boat_been_at_tacking_target = true;
				time_stamp_for_tacking = millis();
			}
			if ((millis() - time_stamp_for_tacking) > 5000 && has_boat_been_at_tacking_target == true) {
				next_state = close_hauled_wind_from_left;
			}
			if (bearing_to_target_relative_to_wind < (-TACKING_ZONE - 5)) {
				next_state = generel_direction_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind >(TACKING_ZONE + 5)){
				next_state = generel_direction_wind_from_left;
			}
			break;

			break;
		case generel_direction_wind_from_right:
			if (bearing_to_target_relative_to_wind > -TACKING_ZONE) {
				next_state = close_hauled_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < -DOWN_WIND_ZONE){
				next_state = down_wind_wind_from_right;
			}
			break;

		case generel_direction_wind_from_left:
			if (bearing_to_target_relative_to_wind < TACKING_ZONE) {
				next_state = close_hauled_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > DOWN_WIND_ZONE){
				next_state = down_wind_wind_from_left;
			}
			break;

		case jibe_going_from_wind_from_left_to_right:
			if (angle_between_two_angles(global.bearing_container.compass_bearing, int(global.global_wind_bearing - DOWN_WIND_ZONE) % 360) < 20) {
				//we are there
				next_state = down_wind_wind_from_right;
			}
			else if (bearing_to_target_relative_to_wind < DOWN_WIND_ZONE+5 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > -DOWN_WIND_ZONE-5 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			break;

		case	jibe_going_from_wind_from_right_to_left:
			if (angle_between_two_angles(global.bearing_container.compass_bearing, int(global.global_wind_bearing + DOWN_WIND_ZONE) % 360) < 20) {
				//we are there
				next_state = down_wind_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind < DOWN_WIND_ZONE+5 && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (bearing_to_target_relative_to_wind > -DOWN_WIND_ZONE-5 && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			break;

		case	down_wind_wind_from_right:
			if (bearing_to_target_relative_to_wind > -DOWN_WIND_ZONE && bearing_to_target_relative_to_wind < 0){
				next_state = generel_direction_wind_from_right;
			}
			else if (real_part_of_complex_from_old_code > 15 || (bearing_to_target_relative_to_wind < 155 && bearing_to_target_relative_to_wind > 0)){
				next_state = jibe_going_from_wind_from_right_to_left;
			}

			break;

		case	down_wind_wind_from_left:
			if (bearing_to_target_relative_to_wind < DOWN_WIND_ZONE && bearing_to_target_relative_to_wind > 0){
				next_state = generel_direction_wind_from_left;
			}
			else if (real_part_of_complex_from_old_code < -15 || (bearing_to_target_relative_to_wind > -DOWN_WIND_ZONE && bearing_to_target_relative_to_wind < 0)){
				next_state = jibe_going_from_wind_from_left_to_right;
			}


			break;

		default:
			Serial.println("error in next boat state logic");
			break;
		}
		//if we want to do special things when changing state, it can be done here.
		

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
		dHeading = global.desired_heading - global.bearing_container.compass_bearing; // in degrees
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

		if (global.Rudder_Desired_Angle > RUDDER_LIMIT) global.Rudder_Desired_Angle = RUDDER_LIMIT;
		if (global.Rudder_Desired_Angle < -RUDDER_LIMIT) global.Rudder_Desired_Angle = -RUDDER_LIMIT;

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
			Serial.print("global.desired_heading: ");
			Serial.print(global.desired_heading);
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
