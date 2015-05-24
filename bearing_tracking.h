#ifndef bearing_h
#define bearing_h
#define SAMPLE_SIZE 5
#define NINE_DOF_SENSOR_POWER_PIN 26
#define WIND_DIRECTION_SAMPLE_SIZE 5
#define STEPS_BETWEEN_WIND_DIRECTION_SAMPLES 20
#define COMPASS_X_MAX 25
#define COMPASS_X_MIN -203
#define COMPASS_Y_MAX 17
#define COMPASS_Y_MIN -185
#define DEBUG_LED 31


#include "global.h"
#include "Bearing_hardware_class.h" //TODO perhaps rename this to bearing_hardware_class

class Bearing_thread_class //TOOO move this to another file,
{
public:
	float initial_pitch = 0;
	float initial_roll = 0;
	Bearing_thread_class() {
		wind_direction_bearing_tracking_counter = 0;
		xMax = COMPASS_X_MAX;
		xMin = COMPASS_X_MIN; //TODO move these to a seperate file called something like calibration.h
		yMax = COMPASS_Y_MAX;
		yMin = COMPASS_Y_MIN;

		compass_y_sum = 0;
		compass_x_sum = 0;
		pitch_sum = 0;
		roll_sum = 0;
		compass_y_array[SAMPLE_SIZE] = { 0 }; //i dont take the avarage of the compass angle, since that may jump between 360 and 0
		compass_x_array[SAMPLE_SIZE] = { 0 }; //default initialized to be zero
		pitch_array[SAMPLE_SIZE] = { 0 };
		roll_array[SAMPLE_SIZE] = { 0 };
		bearing_tracking_counter = 0;

		pinMode(NINE_DOF_SENSOR_POWER_PIN, OUTPUT); //this pin powers the sensor
		digitalWrite(NINE_DOF_SENSOR_POWER_PIN, HIGH);
		delay(100); //give it a little bit of time to power up before we contact it
		Wire.begin();        // join i2c bus (address optional for master)
		Wire.setClock(1000000UL); //set the speed to 10 times the standard, ensures quick communication
		Bearing_hardware_object.return_on_sensor_contact(); //establishes contact and returns when its succesfull
		
		//this for loop fills up the arrays that are used for calulating the avarage values, this simplifies a lot of stuff later, since i can asume the array is populated with valid data at all times
		delay(100);
		for (int i = 0; i < SAMPLE_SIZE; i++){
			Bearing_hardware_object.readAccel(ACCELADDR, &Bearing_hardware_object.accel);
			Bearing_hardware_object.readCompass(COMPASSADDR, &Bearing_hardware_object.compass);
			Bearing_hardware_object.readGyro(GYROADDR, &Bearing_hardware_object.gyro);

			if (Bearing_hardware_object.accel.value.x>255.9) Bearing_hardware_object.accel.value.x = 255.9;
			if (Bearing_hardware_object.accel.value.x<-255.9) Bearing_hardware_object.accel.value.x = -255.9;

			if (Bearing_hardware_object.accel.value.z>255.9) Bearing_hardware_object.accel.value.z = 255.9;
			if (Bearing_hardware_object.accel.value.z<-255.9) Bearing_hardware_object.accel.value.z = -255.9;

			compass_y_array[i] = Bearing_hardware_object.compass.value.z;
			compass_x_array[i] = Bearing_hardware_object.compass.value.x;

			roll_array[i] = atan2((float)Bearing_hardware_object.accel.value.z, (float)Bearing_hardware_object.accel.value.x) *57.29577f; // asin(Bearing_hardware_object.accel.value.x / 256.0f)*57.29577f;
			pitch_array[i] = atan2((float)Bearing_hardware_object.accel.value.z, (float)Bearing_hardware_object.accel.value.y) *57.29577f;

			delay(5);
		}


	}

	void update_data(void){
		this->update_wind_direction_sensor(); //gives wind direction and global wind direction //TODO change those names
		this->maintain_tracking_counter();    // for calculating the avarage compass bearing and pitch we have a tracking counter, this always point in the oldest element in the array we use to take an avarage over a range of values
		this->read_9DOF_sensor();			  //reads the durect data from the 9DOF sensor, this does no calculations on its own
		this->update_pitch_and_roll();		  //based on the above data, this calculates the pitch and roll of the boat
		this->update_compass_data();		  //calculates the compass bearing of the boat
		if (global.debug_handler.bearing_tracking_debug && bearing_tracking_counter == 0){ //we only print debug messages if the setting for doing so is on, and we have the extra precausing that we do it corrosponding to the avarage time we sample samples, since otherwise the console would be absolutely spammed with messages. 
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

		if ((millis() - timestamp) > 1500) //something went terribly wrong, the sensor is properly disconnected or had some sort of error, lets fix that.
		{
			Bearing_hardware_object.return_on_sensor_contact();
			return; //there is no data, exit the function, we will try again shortly
		}
	}

	void update_pitch_and_roll(void){

		roll_array[bearing_tracking_counter] = atan2((float)Bearing_hardware_object.accel.value.z, (float)Bearing_hardware_object.accel.value.x) *57.29577f; // asin(Bearing_hardware_object.accel.value.x / 256.0f)*57.29577f;
		pitch_array[bearing_tracking_counter] = atan2((float)Bearing_hardware_object.accel.value.z, (float)Bearing_hardware_object.accel.value.y) *57.29577f;

		pitch_sum = 0;
		roll_sum = 0;

		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			pitch_sum += pitch_array[i];
			roll_sum += roll_array[i];

		}
		global.bearing_container.pitch = (pitch_sum / SAMPLE_SIZE) - initial_pitch;
		global.bearing_container.roll = (roll_sum / SAMPLE_SIZE) - initial_roll;
		if (global.bearing_container.pitch > 180) global.bearing_container.pitch -= 360;
		if (global.bearing_container.pitch < -180) global.bearing_container.pitch += 360;
		if (global.bearing_container.roll > 180) global.bearing_container.roll -= 360;
		if (global.bearing_container.roll < -180) global.bearing_container.roll += 360;

	}

	void update_compass_data(void){
		compass_x_array[bearing_tracking_counter] = (Bearing_hardware_object.compass.value.x - ((xMin + xMax) / 2.0f)) / ((xMax - xMin) / 200.0f);
		compass_y_array[bearing_tracking_counter] = (Bearing_hardware_object.compass.value.z - ((yMin + yMax) / 2.0f)) / ((yMax - yMin) / 200.0f); //changed to z temporarely, becuase it actually works, which y dosnt.
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
		bearing_tracking_counter++; //we use this to keep track of where we are in the array, so we always overwrite the oldest entry
		if (bearing_tracking_counter == SAMPLE_SIZE) bearing_tracking_counter = 0; //wrap around
		yield();
	}


	void update_wind_direction_sensor(void){

		//explanation: the variable wind_bearing is the direction the wind is coming from compared to the direction of the boat, the global wind bearing is the direction compared to north. 

		//first we read the wind sensor:
		//the full range of it is 2.5V, so we multiply with 360/2.5f TODO improve algorithm.
		//the below is the instantanious wind direction relative to the boat, the value we actually broadcast to everyone else, is based on an average based on the wind direction relative to the compass
		wind_direction_relative_to_boat = ((int((((analogRead(A4)-46)/840.0)*360.0)+210))%360)-180; //numbers here are based on a quick reading of the sensor, and asumes its totally linear in that range, i know its not, but its close

		global_wind_bearing = global.bearing_container.compass_bearing + wind_direction_relative_to_boat; //global wind bearing denotes the compass bearing of the wind. might be usefull at some point in time..

		if (wind_direction_bearing_tracking_counter % STEPS_BETWEEN_WIND_DIRECTION_SAMPLES == 0){
		
			wind_direction_cos_array[wind_direction_bearing_tracking_counter / STEPS_BETWEEN_WIND_DIRECTION_SAMPLES] = cos(global_wind_bearing*PI/180);
			wind_direction_sin_array[wind_direction_bearing_tracking_counter / STEPS_BETWEEN_WIND_DIRECTION_SAMPLES] = sin(global_wind_bearing*PI / 180);		

			if (global.debug_handler.wind_direction_debug){
				this->print_wind_direction_debug();
			}
			float sin_array_sum = 0;
			float cos_array_sum = 0;
			//global_wind_direction_sum = 0;
			for (int i = 0; i < WIND_DIRECTION_SAMPLE_SIZE; i++){
				cos_array_sum += wind_direction_cos_array[i];
				sin_array_sum += wind_direction_sin_array[i];
				//global_wind_direction_sum += global_wind_direction_array[i];
			}
			global.global_wind_bearing = atan2(sin_array_sum, cos_array_sum) * 180 / PI; //TODO CHECK IF THIS WORKS
			//global.global_wind_bearing = (int)(global_wind_direction_sum / WIND_DIRECTION_SAMPLE_SIZE) % 360;
			//global.global_wind_bearing = global_wind_bearing; //TODO implemhjvt the above averaging in an working function
			global.wind_bearing = global.global_wind_bearing - global.bearing_container.compass_bearing;

			if (global.global_wind_bearing > 360) global.global_wind_bearing -= 360;
			if (global.global_wind_bearing < 0) global.global_wind_bearing += 360;

			if (global.wind_bearing > 180) global.wind_bearing = -360;
			if (global.wind_bearing < -180) global.wind_bearing += 360;
		}

		wind_direction_bearing_tracking_counter = (++wind_direction_bearing_tracking_counter) % (WIND_DIRECTION_SAMPLE_SIZE*STEPS_BETWEEN_WIND_DIRECTION_SAMPLES);


	}

	void print_wind_direction_debug(void){
		Serial.print(" wind_direction_relative_to_boat: ");
		Serial.print(wind_direction_relative_to_boat);
		Serial.println("");
		Serial.print(" global.bearing_container.compass_bearing: ");
		Serial.print(global.bearing_container.compass_bearing);
		Serial.println("");
		Serial.print(" global_wind_bearing: ");
		Serial.print(global_wind_bearing);
		Serial.println("");
		Serial.print("global.global_wind_bearing: ");
		Serial.print((global.global_wind_bearing));
		Serial.println("");
		Serial.print("global.wind_bearing: ");
		Serial.print((global.wind_bearing));
		Serial.println("");
		Serial.print("from sensor: ");
		Serial.print(analogRead(A4));
		Serial.println("");
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
		digitalWrite(DEBUG_LED, HIGH);
		
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

				Serial.print(" max x: ");
				Serial.print(xMax);

				Serial.print(" min y: ");
				Serial.print(yMin);

				Serial.print(" max y: ");
				Serial.print(yMax);

				Serial.println("");
			}

			if (global.toggle_compass_calibration) //we are done calibrating, exit.
			{
				global.toggle_compass_calibration = false;
				digitalWrite(DEBUG_LED, LOW);
				break;
			}
			delay(10);
		}
		yield();
	}
	



private:
	float global_wind_bearing;
	int wind_direction_relative_to_boat;
	int xMax = COMPASS_X_MAX;
	int xMin = COMPASS_X_MIN; //TODO move these to a seperate file called something like calibration.h
	int yMax = COMPASS_Y_MAX;
	int yMin = COMPASS_Y_MIN;
	
	float xCali;
	float yCali;

	float compass_y_sum = 0;
	float compass_x_sum = 0;

	float pitch_sum = 0;
	float roll_sum = 0;
	//float global_wind_direction_sum = 0;
	float compass_y_array[SAMPLE_SIZE] = { 0 }; //i dont take the avarage of the compass angle, since that may jump between 360 and 0, i use the vector instead
	float compass_x_array[SAMPLE_SIZE] = { 0 }; //default initialized to be zero
	float pitch_array[SAMPLE_SIZE] = { 0 };
	float roll_array[SAMPLE_SIZE] = { 0 };
	float wind_direction_cos_array[WIND_DIRECTION_SAMPLE_SIZE] = { 0 };
	//float global_wind_direction_array[WIND_DIRECTION_SAMPLE_SIZE] = { 0 };
	float wind_direction_sin_array[WIND_DIRECTION_SAMPLE_SIZE] = { 0 };
	int wind_direction_bearing_tracking_counter = 0;
	int bearing_tracking_counter = 0;
	long int timestamp;
	long int previous_time;

	Bearing_hardware_class Bearing_hardware_object;

};

void Bearing_tracking() {

	Bearing_thread_class bearing_thread_object;
	int i = 0;
	delay(200);
	while (1) {
		bearing_thread_object.update_data();
		delay(10);
		if (i == 15) {
			bearing_thread_object.initial_pitch = global.bearing_container.pitch;
			bearing_thread_object.initial_roll = global.bearing_container.roll;
		}
		i++;
	}
}

#endif