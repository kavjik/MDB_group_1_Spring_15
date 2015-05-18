#ifndef location_h
#define location_h

class Location { // a location consist of a longtitude and a lattitude, optionaly a bearing can also be includded, that is mostly for when getting info from other boats.
	//this class can also give the distance and bearing between two points.
public:
	Location() : latitude(65.55), longtitude(11.55), is_valid_boat(false) {} //filled with dummy data
	double latitude = 0; //doubles to avoid any loss of precision
	double longtitude = 0;
	float speed;
	bool is_valid_boat = false;
	float bearing = 0; //optional bearing, used for tracking the other boats
	long int timestamp;
	float distance_to(Location target_location){
		if (target_location.latitude == this->latitude && target_location.longtitude == this->longtitude) return(0);

		double lat_a = this->latitude*0.0174532925199; //convert to radians
		double lon_a = this->longtitude*0.0174532925199;
		double lat_b = target_location.latitude*0.0174532925199;
		double lon_b = target_location.longtitude*0.0174532925199;
		double radius = 6371; //radius of the earth

		double sin_lat_squared = sin(lat_b / 2 - lat_a / 2)*sin(lat_b / 2 - lat_a / 2);
		double sin_lon_sqaured = sin(lon_b / 2 - lon_a / 2)*sin(lon_b / 2 - lon_a / 2);
		double var1 = 2 * radius * asin(sqrt(sin_lat_squared + cos((lat_a))*cos((lat_b))*sin_lon_sqaured));//I DONT WANT TO WRITE THAT AGAIN

		//this whole method could be simplified to just be within the return statement, but that would be LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONG, so i don't
		//source for above equation http://en.wikipedia.org/wiki/Haversine_formula
		// it may be a bit overkill, but this way it support cross continent distances, as well as the back yard
		//and with double there is no lack in precision, a small calculation shows that with 52bit precision, i can describe any point on earth within about 10 nano meters
		//with floats that would be in the range of meters
		return (var1 * 1000); //this way the result is in meters


	}
	int bearing_to(Location target_location){
		double lat_b = this->latitude*0.0174532925199; //convert to radians
		double lon_b = this->longtitude*0.0174532925199;
		double lat_a = target_location.latitude*0.0174532925199;
		double lon_a = target_location.longtitude*0.0174532925199;
		double d_lon = lon_b - lon_a;

		double y = sin(d_lon) * cos(lat_b);
		double x = cos(lat_a) * sin(lat_b) - sin(lat_a) * cos(lat_b) * cos(d_lon);
		return ((int)(180 + atan2(y, x)*57.2957795)%360); //return the result in degrees

		//same source as the getDistance
	
	}
};

#endif
