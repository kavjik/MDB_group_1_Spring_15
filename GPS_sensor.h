#ifndef location_tracking_h
#define location_tracking_h
#include "Adafruit_GPS.h"
#include "global.h"

void Location_tracking(){
	
//code below is taken from a Adafruit GPS module code example
	//then modified heavily for our needs

// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada



// This sketch is ONLY for the Arduino Due!
// You should make the following connections with the Due and GPS module:
// GPS power pin to Arduino Due 3.3V output.
// GPS ground pin to Arduino Due ground.
// For hardware SerialUSB 1 (recommended):
//   GPS TX to Arduino Due SerialUSB1 RX pin 19
//   GPS RX to Arduino Due SerialUSB1 TX pin 18


//Adafruit_GPS GPS(&mySerialUSB); //mmoved to global



// Set GPSECHO to 'false' to turn off echoing the GPS data to the SerialUSB console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy



	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	global.GPS_module.begin(9600); //TODO test if it works, standard 9600
	mySerial.begin(9600);

	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	global.GPS_module.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time

	// Set the update rate
	global.GPS_module.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate 
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	global.GPS_module.sendCommand(PGCMD_ANTENNA);

	// the nice thing about this code is you can have a timer0 interrupt go off
	// every 1 millisecond, and read data from the GPS for you. that makes the
	// loop code a heck of a lot easier!

#ifdef __arm__
	usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
	useInterrupt(true);
#endif

	delay(1000);
	//we wait a bit to make sure the gps is ready
	// Ask for firmware version
	mySerial.println(PMTK_Q_RELEASE);

	uint32_t timer = millis();
	while (1) //we have a break in the below while 1 loop, therefore its nested in another while(1)
	{
		while(1)                    // run over and over again
		{
			// in case you are not using the interrupt above, you'll
			// need to 'hand query' the GPS, not suggested :(
			if (!usingInterrupt) {
				// read data from the GPS in the 'main loop'
				char c = global.GPS_module.read();
				// if you want to debug, this is a good time to do it!
				if (GPSECHO)
					if (c) Serial.print(c);
			}

			// if a sentence is received, we can check the checksum, parse it...
			if (global.GPS_module.newNMEAreceived()) {
				// a tricky thing here is if we print the NMEA sentence, or data
				// we end up not listening and catching other sentences! 
				// so be very wary if using OUTPUT_ALLDATA and trytng to print out data
				//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

				if (!global.GPS_module.parse(global.GPS_module.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
					break;  // we can fail to parse a sentence in which case we should just wait for another
							//this was changed from a return to a break because of different program architecture
			}
			
			// if millis() or timer wraps around, we'll just reset it
			
			if (timer > millis())  timer = millis();

			// approximately every 10 seconds or so, print out the current stats
			if (millis() - timer > 1000 && global.debug_handler.gps_sensor_debug) {
				timer = millis(); // reset the timer

				Serial.print("\nTime: ");
				Serial.print(global.GPS_module.hour, DEC); Serial.print(':');
				Serial.print(global.GPS_module.minute, DEC); Serial.print(':');
				Serial.print(global.GPS_module.seconds, DEC); Serial.print('.');
				Serial.println(global.GPS_module.milliseconds);
				Serial.print("Date: ");
				Serial.print(global.GPS_module.day, DEC); Serial.print('/');
				Serial.print(global.GPS_module.month, DEC); Serial.print("/20");
				Serial.println(global.GPS_module.year, DEC);
				Serial.print("Fix: "); Serial.print((int)global.GPS_module.fix);
				Serial.print(" quality: "); Serial.println((int)global.GPS_module.fixquality);
				if (global.GPS_module.fix) {
					Serial.print("Location: ");
					Serial.print(global.GPS_module.latitude, 4); Serial.print(global.GPS_module.lat);
					Serial.print(", ");
					Serial.print(global.GPS_module.longitude, 4); Serial.println(global.GPS_module.lon);

					Serial.print("Speed (knots): "); Serial.println(global.GPS_module.speed);
					Serial.print("Angle: "); Serial.println(global.GPS_module.angle);
					Serial.print("Altitude: "); Serial.println(global.GPS_module.altitude);
					Serial.print("Satellites: "); Serial.println((int)global.GPS_module.satellites);
				}
			}
			delay(5);
		}
		delay(5);
	}

	delay(5);
}


#endif