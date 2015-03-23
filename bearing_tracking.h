#ifndef bearing_h
#define bearing_h
#define SAMPLE_SIZE 20
#define NINE_DOF_SENSOR_POWER_PIN 52

#include "global.h"
#include "Bearing_hardware_class.h" //TODO perhaps rename this to bearing_hardware_class

class Bearing_thread_class //TOOO move this to another file,
{
public:

	Bearing_thread_class() {

		xMax = 75;
		xMin = -48; //TODO move these to a seperate file called something like calibration.h
		yMax = -34;
		yMin = -158;

		compass_y_sum = 0;
		compass_x_sum = 0;
		pitch_sum = 0;
		roll_sum = 0;
		compass_y_array[SAMPLE_SIZE] = { 0 }; //i dont take the avarage of the compass angle, since that may jump between 360 and 0
		compass_x_array[SAMPLE_SIZE] = { 0 }; //default initialized to be zero
		pitch_array[SAMPLE_SIZE] = { 0 };
		roll_array[SAMPLE_SIZE] = { 0 };
		bearing_tracking_conuter = 0;

		pinMode(NINE_DOF_SENSOR_POWER_PIN, OUTPUT); //this pin powers the sensor
		digitalWrite(NINE_DOF_SENSOR_POWER_PIN, HIGH);
		delay(100); //give it a little bit of time to power up before we contact it
		Wire.begin();        // join i2c bus (address optional for master)
		Wire.setClock(1000000UL); //set the speed to 10 times the standard, ensures quick communication
		Bearing_hardware_object.return_on_sensor_contact(); //establishes contact and returns when its succesfull
		
		//this for loop fills up the arrays that are used for calulating the avarage values, this simplifies a lot of stuff later.
		for (int i = 0; i < SAMPLE_SIZE; i++){
			Bearing_hardware_object.readAccel(ACCELADDR, &Bearing_hardware_object.accel);
			Bearing_hardware_object.readCompass(COMPASSADDR, &Bearing_hardware_object.compass);
			Bearing_hardware_object.readGyro(GYROADDR, &Bearing_hardware_object.gyro);

			if (Bearing_hardware_object.accel.value.x>256) Bearing_hardware_object.accel.value.x = 256;
			if (Bearing_hardware_object.accel.value.x<-256) Bearing_hardware_object.accel.value.x = -256;

			if (Bearing_hardware_object.accel.value.x>256) Bearing_hardware_object.accel.value.x = 256;
			if (Bearing_hardware_object.accel.value.x<-256) Bearing_hardware_object.accel.value.x = -256;

			compass_y_array[i] = Bearing_hardware_object.compass.value.z;
			compass_x_array[i] = Bearing_hardware_object.compass.value.x;

			pitch_array[i] = asin(Bearing_hardware_object.accel.value.y / 256.0f)*57.29576f;
			roll_array[i] = asin(Bearing_hardware_object.accel.value.x / 256.0f)*57.29576f;
			yield();
		}

	}

	void update_data(void){
		this->update_wind_direction_sensor(); //gives wind direction and global wind direction //TODO change those names
		this->maintain_tracking_counter();    // for calculating the avarage compass bearing and pitch we have a tracking counter, this always point in the oldest element in the array we use to take an avarage over a range of values
		this->read_9DOF_sensor();			  //reads the durect data from the 9DOF sensor, this does no calculations on its own
		this->update_pitch_and_roll();		  //based on the above data, this calculates the pitch and roll of the boat
		this->update_compass_data();		  //calculates the compass bearing of the boat
		if (global.debug_handler.bearing_tracking_debug && bearing_tracking_conuter == 0){ //we only print debug messages if the setting for doing so is on, and we have the extra precausing that we do it corrosponding to the avarage time we sample samples, since otherwise the console would be absolutely spammed with messages. 
			this->print_debug();
		}
		if (global.toggle_compass_calibration) this->do_compass_calibration();
	}

	void read_9DOF_sensor(void){
		timestamp = millis();

		//then we read the 3 sensors, accelerometer, gyro and compass
		Bearing_hardware_object.readAccel(ACCELADDR, &Bearing_hardware_object.accel);
		Bearing_hardware_object.readCompass(COMPASSADDR, &Bearing_hardware_object.compass);
		Bearing_hardware_object.readGyro(GYROADDR, &Bearing_hardware_object.gyro);

		if ((millis() - timestamp) > 50) //something went terribly wrong, the sensor is properly disconnected or had some sort of error, lets fix that.
		{
			Bearing_hardware_object.return_on_sensor_contact();
			return; //there is no data, exit the function, we will try again shortly
		}
	}

	void update_pitch_and_roll(void){
		if (Bearing_hardware_object.accel.value.x>256) Bearing_hardware_object.accel.value.x = 256;
		if (Bearing_hardware_object.accel.value.x<-256) Bearing_hardware_object.accel.value.x = -256;
		roll_array[bearing_tracking_conuter] = asin(Bearing_hardware_object.accel.value.x / 256.0f)*57.29577f;

		if (Bearing_hardware_object.accel.value.z>256) Bearing_hardware_object.accel.value.z = 256;
		if (Bearing_hardware_object.accel.value.z<-256) Bearing_hardware_object.accel.value.z = -256;
		pitch_array[bearing_tracking_conuter] = asin(Bearing_hardware_object.accel.value.y / 256.0f)*57.29577f;

		pitch_sum = 0;
		roll_sum = 0;

		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			pitch_sum += pitch_array[i];
			roll_sum += roll_array[i];

		}
		global.bearing_container.pitch = pitch_sum / SAMPLE_SIZE;
		global.bearing_container.roll = roll_sum / SAMPLE_SIZE;

	}

	void update_compass_data(void){
		compass_x_array[bearing_tracking_conuter] = (Bearing_hardware_object.compass.value.x - ((xMin + xMax) / 2.0f)) / ((xMax - xMin) / 200.0f);
		compass_y_array[bearing_tracking_conuter] = (Bearing_hardware_object.compass.value.z - ((yMin + yMax) / 2.0f)) / ((yMax - yMin) / 200.0f); //changed to z temporarely, becuase it actually works, which y dosnt.
		compass_y_sum = 0;
		compass_x_sum = 0;


		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			compass_y_sum += compass_y_array[i];
			compass_x_sum += compass_x_array[i];
		}

		//angle between the measure compass x and y, and the vector (0,100)
		// see http://www.vitutor.com/geometry/vec/angle_vectors.html

		global.bearing_container.compass_bearing = (180 + ((atan2(compass_y_sum / SAMPLE_SIZE, compass_x_sum / SAMPLE_SIZE)*57.29577f))); //this is based on the vector (x,z) and then converted from radians to degrees. 


	}

	void maintain_tracking_counter(void){
		bearing_tracking_conuter++; //we use this to keep track of where we are in the array, so we always overwrite the oldest entry
		if (bearing_tracking_conuter == SAMPLE_SIZE) bearing_tracking_conuter = 0; //wrap around
		yield();
	}


	void update_wind_direction_sensor(void){
		static bool do_wind_direction_debug = false;
		static float global_wind_debug = 10;
		//TODO take an avarage of the global wind bearing, and use that to calculate the local wind bearing, since we can better trust the global wind bearing
		//explanation: the variable wind_bearing is the direction the wind is coming from compared to the direction of the boat, the global wind bearing is the direction compared to north. 

		//first we read the wind sensor:
		//the full range of it is 2.5V, so we multiply with 360/2.5f TODO improve algorithm.

		global.wind_bearing = ((analogRead(A4) - 37*1.25)*360.0f) / (645.0f*1.25); //numbers here are based on a quick reading of the sensor, and asumes its totally linear in that range, i know its not, but its close
		global.wind_bearing += 14; //we offset the measurement based on measurement on the sensor //TODO moce to a calibration_values.h
		
		global.wind_bearing = fmod(global.wind_bearing, 360); //modulus, takes care of over and underflow, if its in range, this does nothing
		//TODO test that the values that comes from here are correct, since the algorith has changed somewhat. 
		
		//global.wind_bearing -= 180; //change the range from 0 to 360 to -180 to 180, since its the angle relative to north we are interested in.
		global.global_wind_bearing = global.bearing_container.compass_bearing + global.wind_bearing; //global wind bearing denotes the compass bearing of the wind. might be usefull at some point in time..
		//TODO implement an avaraging of this, that takes many samples to smoothe it out, the wind direction does not change that often.
		if (global.global_wind_bearing > 360) global.global_wind_bearing -= 360;
		//TODO REMOVE THIS DEBUG
		if (global.debug_handler.wind_direction_debug) {
			do_wind_direction_debug = true;
		}
		if (do_wind_direction_debug){
			if (global.debug_handler.wind_direction_debug){

			
				global_wind_debug += 10;
				global_wind_debug = fmod(global_wind_debug, 360);
				global.debug_handler.wind_direction_debug = false;
			}
			global.global_wind_bearing = global_wind_debug;
		}

		
	}

	void print_debug(void) {
		Serial.print(" compass: ");
		Serial.print(global.bearing_container.compass_bearing);

		Serial.print(" bpitch: ");
		Serial.print(global.bearing_container.pitch);

		Serial.print(" roll: ");
		Serial.print(global.bearing_container.roll);
		Serial.print(" wind bearing: ");
		Serial.print(global.wind_bearing);
		Serial.println("");
		Serial.print("this took: ");
		Serial.print((millis() - previous_time));
		previous_time = millis();
		Serial.print("for samples: ");
		Serial.print(SAMPLE_SIZE);
		Serial.println("");
	}

	void do_compass_calibration(void)
	{ //we want to calibrate the compass sensor, so now focus exclusivly on that.
		//this function only returns after the compass sensor is calibrated, we can do this, since this mode is only entered, when we dont actually need the compass data
		global.toggle_compass_calibration = false;
		xMax = Bearing_hardware_object.compass.value.x;
		xMin = Bearing_hardware_object.compass.value.x;
		yMax = Bearing_hardware_object.compass.value.z;
		yMin = Bearing_hardware_object.compass.value.z;

		while (1)
		{
			yield();
			Bearing_hardware_object.readCompass(COMPASSADDR, &Bearing_hardware_object.compass);


			if (Bearing_hardware_object.compass.value.x > xMax)xMax = Bearing_hardware_object.compass.value.x;
			if (Bearing_hardware_object.compass.value.x < xMin)xMin = Bearing_hardware_object.compass.value.x;
			if (Bearing_hardware_object.compass.value.z > yMax)yMax = Bearing_hardware_object.compass.value.z;
			if (Bearing_hardware_object.compass.value.z < yMin)yMin = Bearing_hardware_object.compass.value.z;

			if (global.debug_handler.bearing_tracking_debug) {
				Serial.print("min x: ");
				Serial.print(xMin);

				Serial.print("max x: ");
				Serial.print(xMax);

				Serial.print("min y: ");
				Serial.print(yMin);

				Serial.print("max y: ");
				Serial.print(yMax);

				Serial.println("");
			}

			if (global.toggle_compass_calibration) //we are done calibrating, exit.
			{
				global.toggle_compass_calibration = false;
				break;
			}
			delay(10);
		}
		yield();
	}
	



private:
	short xMax = 75;
	short xMin = -48; //TODO move these to a seperate file called something like calibration.h
	short yMax = -34;
	short yMin = -158;
	
	float xCali;
	float yCali;

	float compass_y_sum = 0;
	float compass_x_sum = 0;
	float pitch_sum = 0;
	float roll_sum = 0;
	float compass_y_array[SAMPLE_SIZE] = { 0 }; //i dont take the avarage of the compass angle, since that may jump between 360 and 0
	float compass_x_array[SAMPLE_SIZE] = { 0 }; //default initialized to be zero
	float pitch_array[SAMPLE_SIZE] = { 0 };
	float roll_array[SAMPLE_SIZE] = { 0 };
	int bearing_tracking_conuter = 0;
	long int timestamp;
	long int previous_time;

	Bearing_hardware_class Bearing_hardware_object;

};

void Bearing_tracking() {

	Bearing_thread_class bearing_thread_object;
	while (1) {
		bearing_thread_object.update_data();
		delay(3);
	}
}

#endif