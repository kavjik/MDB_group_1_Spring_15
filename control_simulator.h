
#ifndef control_simulator_h
#define control_simulator_h


void control_simulator()
{
	//global.gps_data.location.latitude = 54.912766666667;
	//global.gps_data.location.longtitude = 9.779730555556;


	while (1){
		if (global.debug_handler.show_sim_info == true)
		{
			Serial.print("Compass Bearing: ");
			Serial.print(global.bearing_container.compass_bearing);

			Serial.print(", Roll: ");
			Serial.print(global.bearing_container.roll);

			Serial.print(", Wind: ");
			Serial.print(global.wind_bearing);

			Serial.print(", gWind: ");
			Serial.print(global.global_wind_bearing);

			Serial.println("");
		}
		if (!SIMULATOR_MODE_MOVE_AUTOMATICALLY)	global.global_wind_bearing = global.bearing_container.compass_bearing + global.wind_bearing; //global wind bearing denotes the compass bearing of the wind. might be usefull at some point in time..

		if (global.global_wind_bearing > 360)
		{
			global.global_wind_bearing -= 360;
		}
		else if (global.global_wind_bearing < 0)
		{
			global.global_wind_bearing += 360;
		}

		delay(100);
	}
}

#endif