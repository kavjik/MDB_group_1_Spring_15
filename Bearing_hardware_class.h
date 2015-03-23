#ifndef bearing_tracking_class_h
#define bearing_tracking_class_h

//#include "Wire\Wire.h"
#include "global.h"


#define GYROADDR 0x68
#define COMPASSADDR 0x1e
#define ACCELADDR 0x53

#define STATUS_LED_PIN 13

class Bearing_hardware_class //this is a class derived directly from an internet example //TODO source // which then has been contained in this class.
{

public:
	void return_on_sensor_contact(void){
		while (1) {
			if (this->setupCompass(COMPASSADDR) && this->setupAccel(ACCELADDR) && this->setupGyro(GYROADDR))
			{
				break; //everything went well, break the loop
				
			}
			else
			{ //stay here and try and turn the 9DOF sensor on off till it works.
				if (global.debug_handler.bearing_tracking_debug) Serial.println("9DOF sensor reading problems");
				digitalWrite(NINE_DOF_SENSOR_POWER_PIN, LOW);
				delay(1000);
				digitalWrite(NINE_DOF_SENSOR_POWER_PIN, HIGH);
				delay(1000);
			}
		}
	}


	// Generically useful reading into a union type
	void readXYZ(int device, union XYZBuffer *xyz) {
		Wire.requestFrom(device, 6);
		long start = millis();
		while (!Wire.available() && (millis() - start)<100);
		if (millis() - start<100) {
			for (int i = 0; i<6; i++)
				xyz->buff[i] = Wire.read();
		}
	}

	bool setupAccel(int device) {
		// Check ID to see if we are communicating
		Wire.beginTransmission(device);
		Wire.write(0x00); // One Reading
		Wire.endTransmission();
		Wire.requestFrom(device, 1);
		while (!Wire.available());
		byte ch = Wire.read();
		if(global.debug_handler.bearing_tracking_debug) Serial.print("Accel id is 0x");
		if (global.debug_handler.bearing_tracking_debug) Serial.println(ch, HEX);
		bool success = (ch != 0); // if 0 we have failed otherwise we are fine
		// Should output E5

		// https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
		// Page 16
		Wire.beginTransmission(device);
		Wire.write(0x2d);
		Wire.write(0x08);
		Wire.endTransmission();
		Wire.beginTransmission(device);
		Wire.write(0x38);
		Wire.write(0x84);
		Wire.endTransmission();
		return success;

	}
	void readAccel(int device, union XYZBuffer *xyz) {
		Wire.beginTransmission(device);
		Wire.write(0x32); // One Reading
		Wire.endTransmission();
		readXYZ(device, xyz);
	}

	bool setupCompass(int device) {
		// Check ID to see if we are communicating
		if (global.debug_handler.bearing_tracking_debug) Serial.print("Compass id is ");
		Wire.beginTransmission(device);
		Wire.write(10); // One Reading
		Wire.endTransmission();
		Wire.requestFrom(device, 2);
		while (!Wire.available());
		char ch = Wire.read();
		if (global.debug_handler.bearing_tracking_debug) Serial.print(ch);
		ch = Wire.read();
		if (global.debug_handler.bearing_tracking_debug) Serial.println(ch);
		bool success = (ch != 0); // if 0 we have failed otherwise we are fine
		// Should output H4  

		// Page 18
		// at http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
		Wire.beginTransmission(device);
		Wire.write(0x00); Wire.write(0x70);
		Wire.endTransmission();
		Wire.beginTransmission(device);
		Wire.write(0x01); Wire.write(0xA0);
		Wire.endTransmission();
		Wire.beginTransmission(device);
		Wire.write(0x02); Wire.write(0x00); //  Reading
		Wire.endTransmission();
		delay(6);
		return(success);
	}

	void changeEndian(union XYZBuffer *xyz) {
		for (int i = 0; i<6; i += 2) {
			byte t = xyz->buff[i];
			xyz->buff[i] = xyz->buff[i + 1];
			xyz->buff[i + 1] = t;
		}
	}
	void readCompass(int device, union XYZBuffer *xyz) {
		readXYZ(device, xyz);
		changeEndian(xyz);
		Wire.beginTransmission(device);
		Wire.write(0x03);
		Wire.endTransmission();
	}

	bool setupGyro(int device) {
		// Check ID to see if we are communicating
		Wire.beginTransmission(device);
		Wire.write(0x00); // One Reading
		Wire.endTransmission();
		Wire.requestFrom(device, 1);
		while (!Wire.available());
		byte ch = Wire.read();
		if (global.debug_handler.bearing_tracking_debug) Serial.print("Gyro id is 0x");
		if (global.debug_handler.bearing_tracking_debug) Serial.println(ch, HEX);
		bool success = (ch != 0); // if 0 we have failed otherwise we are fine
		return success;
		// Should output 69
	}
	void readGyro(int device, union XYZBuffer *xyz) {
		// https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
		// page 20
		Wire.beginTransmission(device);
		Wire.write(0x1d);
		Wire.endTransmission();
		readXYZ(device, xyz);
		changeEndian(xyz);
	}

	void pad(int width, int number) {
		int n = abs(number);
		int w = width;
		if (number<0) w--;
		while (n>0) {
			w--;
			n /= 10;
		}
		if (number == 0) w--;
		for (int i = 0; i<w; i++) if (global.debug_handler.bearing_tracking_debug) Serial.print(' ');
	}

	void output(union XYZBuffer xyz) {
		//  pad(6,xyz.value.x);
		if (global.debug_handler.bearing_tracking_debug) Serial.print(xyz.value.x);
		if (global.debug_handler.bearing_tracking_debug) Serial.print(',');
		//  pad(6,xyz.value.y);
		if (global.debug_handler.bearing_tracking_debug) Serial.print(xyz.value.y);
		if (global.debug_handler.bearing_tracking_debug) Serial.print(',');
		//  pad(6,xyz.value.z);
		if (global.debug_handler.bearing_tracking_debug) Serial.print(xyz.value.z);
	}
	union XYZBuffer compass, gyro, accel;

private:

};



#endif
