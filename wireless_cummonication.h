#ifndef wireless_communication_h
#define wireless_communication_h
#include "global.h"
#include "XBee.h"
#include <stdlib.h>
class wireless_communication_class

#define COORDINATOR 0x0013a200,0x40B5BBD0
#define BOAT_1		0x0013A200,0x40B5BBF0
#define BOAT_2		0x0013A200,0x40B5BBF5
#define BOAT_3		0x0013A200,0x40B48973

{
public:
	wireless_communication_class(){
		payload[0] = 's';
		payload[1] = 'l';
		payload[2] = 'a';
		payload[3] = 'v';
		payload[4] = 'e';
		payload[5] = '1';
	}
	void init(void){
		Serial2.begin(9600);
		xbee.setSerial(Serial2);
		xbee.begin(Serial2);
	}
	typedef struct{          // size of this struct should be multiples of 4
		double lattitude;
		double longtitude;
		float  bearing;
		float  speed;
	}OUTPUT_DATA_;

	void send_info(void/*double lattitude, double longtitude, float bearing, float speed*/){
		float x = 2228.476;
		uint8_t b[sizeof(x)];
		memcpy(b, &x, sizeof(x));
		addr64 = XBeeAddress64(COORDINATOR);		//		BOAT_1		COORDINATOR
		zbTx = ZBTxRequest(addr64, b, sizeof(b));

		xbee.send(zbTx);
	}
	void get_info(void){
		
		xbee.readPacket();
		
		if (xbee.getResponse().isAvailable()) {
			// got something
			if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
				// got a zb rx packet

				// now fill our zb rx class
				xbee.getResponse().getZBRxResponse(rx);
			}

			float y = 1111.111;
			uint8_t b[sizeof(y)];
			uint8_t *bp;

			memcpy(&y, rx.getData(), sizeof(y));
			Serial.print(y);

			Serial.print('\r');
			Serial.print('\n');
		}
		else if(global.debug_handler.wireless_communication_debug) Serial.println("didnt get anything");
	}
	void add_waypoint(Location target){
		global.waypoints.enqueue(target);
	}
private:
	OUTPUT_DATA_ oData;

	void update_other_boats_location(int i /*change*/){

		global.other_boats[i].bearing = 1;
		global.other_boats[i].latitude = 1;
		global.other_boats[i].longtitude = 1;
		global.other_boats[i].speed = 1;
		global.other_boats[i].is_valid_boat = true;
	}

	XBee xbee = XBee();
	uint8_t payload[80] = {};// = { 'h', 'e', 'y' };
	XBeeAddress64 addr64 = XBeeAddress64(COORDINATOR);			// address of coordinator: 0x0013a200, 0x40B5BBD0 BOAT_1 COORDINATOR
																// address of Boat 1: 0x0013A200, 0x40B5BBF0
	ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
	XBeeResponse response = XBeeResponse();
	// create reusable response objects for responses we expect to handle 
	ZBRxResponse rx = ZBRxResponse();
	ModemStatusResponse msr = ModemStatusResponse();
	//ZBTxStatusResponse txStatus = ZBTxStatusResponse();

};

//the object corrosponding to this class is implemented in global, so the two threads both can read it.
wireless_communication_class wireless_communication_object;

void wireless_cummonication(){
	wireless_communication_object.init();
	while (1){
		wireless_communication_object.send_info(/*global.gps_data.location.latitude, global.gps_data.location.longtitude, global.bearing_container.compass_bearing, global.gps_data.speed*/);
		
		delay(1000);
	}
	yield();
}
void wireless_recieve_thread(){
	wireless_communication_object.init();
	while (1){
		wireless_communication_object.get_info();
		delay(100);
	}
}



#endif