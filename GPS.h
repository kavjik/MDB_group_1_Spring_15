#ifndef gps_h
#define gps_h

class Gps { //this contains the data one could need from the gps
public:
	Gps() : gps_bearing(25), Timestampt_of_last_fix(19), speed(11), fix(false) {} //initialize to dummy values, they are overwritten when real world data makes sense
	Location location;
	float gps_bearing = 0; //in degrees
	unsigned long Timestampt_of_last_fix = 0; //use millis();
	float speed = 0; //meters per second
	bool fix;
};

#endif
