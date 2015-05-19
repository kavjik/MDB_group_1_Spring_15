

#ifndef debug_handler_h
#define debug_handler_h

//this class is just a container, (could be a struct) that contains the boolean indicators for whether we want to see debug messages or not.
//there is one per thread.
//at the moment they all default to false, i would like to keep it that way

class Debug_handler
{
public:
	Debug_handler() :gps_sensor_debug(false), gps_tracking_debug(false), bearing_tracking_debug(false),
		Computer_input_handler_debug(false), data_logging_debug(false), path_finding_debug(false),
		rudder_and_sail_control_debug(false), wireless_communication_debug(false), main_debug(false), show_sim_info(false),
		remote_commands_wireless_communication_debug(false)
		
	{}

	bool gps_sensor_debug;
	bool gps_tracking_debug;
	bool bearing_tracking_debug;
	bool Computer_input_handler_debug;
	bool data_logging_debug;
	bool path_finding_debug;
	bool rudder_and_sail_control_debug;
	bool wireless_communication_debug;
	bool remote_commands_wireless_communication_debug;
	bool main_debug;
	bool wind_direction_debug;
	bool show_sim_info;

private:

};




#endif