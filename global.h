    #ifndef global_global_h
#define global_global_h

#define GPSSerial Serial1 //this is for the gps sensor, kept this way since the libary did it this way




#include "Location.h"
#include "GPS.h"
#include "QueueArray.h"
#include "Adafruit_GPS.h"
#include "Debug_handler.h"
#include "Bearing_container.h"


union XYZBuffer { //this is used for reading the 9DOF sensor, its declared here so that others might also read the data from the 9DOF sensor directly, although i dont think its going to be used.
	struct {
		short x, y, z;
	} value;
	byte buff[6];
};

struct Data_from_navigation_to_log { //this is for creating an easy way to get a lot of data from the navigation to the datalogger
public:


	double Boat1_Data_X_b_real;
	double Boat1_Data_X_b_imag;
	double Boat1_Data_X_T_b_real;
	double Boat1_Data_X_T_b_imag;
	double global_Rudder_Desired_Angle;
	int Boat1_Data_TACKINGRANGE;
	int Boat1_Data_RADIUS_ACCEPTED;
	int inrange;
	int sig1;
	int sig2;
	int sig3;
	float theta_d_b;
	float theta_d1_b;
	float theta_LOS;
	float global_path_bearing;
};

//this class contains all the global elements, its meant as being the only thing defined in the global scope
//this makes it very clear in all the different threds when a global variable is accesed, instead of a local variable

class Global
{
public:
	Global() : wind_bearing(20), path_bearing(30), toggle_compass_calibration(false){
		GPS_module = Adafruit_GPS(&GPSSerial);
	}
	Adafruit_GPS  GPS_module = 0; //used in the gps sensor thread
	Gps gps_data; //contains the filtered gps data, usefull for all the other threads
	QueueArray <Location> waypoints; //a queue containging the waypoints we want to go to, excluding the current objective, which the path finding should maintain
	float path_bearing = 0; //bearing we want the boat to travel, this is used by the rudder control
	Location other_boats[4]; //a simple array containing the location of the other boats, due to lack of vectors, its kept as an array
	QueueArray <String> messages_to_be_logged; //special messages we want to log specifically //TODO implement.
	Debug_handler debug_handler; //contains the boolean values for wheter we want to look at debug messages in the console.
	Bearing_container bearing_container;
	bool toggle_compass_calibration;
	float wind_bearing = 0;
	float global_wind_bearing;
	float Rudder_Desired_Angle;

	Data_from_navigation_to_log data_from_navigation_to_log;
	
};

Global global; //we also make the object here, since everybody can use it directly then.

#endif
