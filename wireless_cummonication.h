#ifndef wireless_communication_h
#define wireless_communication_h
#include "global.h"
#include "XBee.h"

#define COORDINATOR_ADDR	0x0013a200,0x40B5BBD0
#define BOAT_1_ADDR			0x0013A200,0x40B5BBF0
#define BOAT_2_ADDR			0x0013A200,0x40B5BBF5
#define BOAT_3_ADDR			0x0013A200,0x40B48973

class wireless_communication_class
{
public:
	wireless_communication_class(){}
	void init(void){
		Serial2.begin(9600);
		xbee.setSerial(Serial2);
		xbee.begin(Serial2);
	}
	enum XBEE_ID {
		boat1,
		boat2,
		boat3,
		boat4,
		coordinator
	};
	//Data package to send and receive
	typedef struct{          
		XBEE_ID ID;
		double latitude;
		double longitude;
		float  bearing;
		float  speed;
	}DATA_;

	void send_info(double latitude, double longitude, float bearing, float speed){
		outData.ID = boat1;											//*********** Set ID of sender ( coordinator = 4	boat1 = 0	boat2 = 1 ... )
		outData.latitude = latitude;
		outData.longitude = longitude;
		outData.bearing = bearing;
		outData.speed = speed;

		uint8_t pay_load[sizeof(outData)];
		memcpy(pay_load, &outData, sizeof(DATA_));					//Convert DATA_ to uint8

		addr64 = XBeeAddress64(COORDINATOR_ADDR);					//*********** Set Address of receiver ( BOAT_1_ADDR	BOAT_2_ADDR	COORDINATOR_ADDR	...)
		zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
		xbee.send(zbTx);

		//addr64 = XBeeAddress64(BOAT_1_ADDR);						//*********** Set Address of receiver ( BOAT_1_ADDR	BOAT_2_ADDR	COORDINATOR_ADDR	...)
		//zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
		//xbee.send(zbTx);
	}
	void get_info(void){
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {
			// got something
			if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
				// got a zb rx packet
				xbee.getResponse().getZBRxResponse(rx);// now fill our zb rx class
			}
			inData = {};//clear inData package
			memcpy(&inData, rx.getData(), sizeof(DATA_));				//Convert uint8 to DATA_

			update_other_boats_location(inData);

			Serial.println(inData.ID);
			Serial.println(inData.latitude,7);
			Serial.println(inData.longitude, 7);
			Serial.println(inData.bearing, 7);
			Serial.println(inData.speed, 7);
			Serial.print('\r');
			Serial.print('\n');

		}
		else if(global.debug_handler.wireless_communication_debug) Serial.println("didnt get anything");
	}
	void add_waypoint(Location target){
		global.waypoints.enqueue(target);
	}
private:
	DATA_ outData;
	DATA_ inData;

	XBee xbee = XBee();
	uint8_t payload[80] = {};// = { 'h', 'e', 'y' };
	XBeeAddress64 addr64 = XBeeAddress64(0x0,0x0);			// address of coordinator: 0x0013a200, 0x40B5BBD0
																// address of Boat 1: 0x0013A200, 0x40B5BBF0
	ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
	XBeeResponse response = XBeeResponse();
	ZBRxResponse rx = ZBRxResponse();					// create reusable response objects for responses we expect to handle 
	//ModemStatusResponse msr = ModemStatusResponse();

	void update_other_boats_location(DATA_ inData){
		global.other_boats[inData.ID].bearing = inData.bearing;
		global.other_boats[inData.ID].latitude = inData.latitude;
		global.other_boats[inData.ID].longtitude = inData.longitude;
		global.other_boats[inData.ID].speed = inData.speed;
		global.other_boats[inData.ID].is_valid_boat = true;
	}

};

//the object corrosponding to this class is implemented in global, so the two threads both can read it.
wireless_communication_class wireless_communication_object;

void wireless_cummonication(){
	wireless_communication_object.init();
	while (1){
		//wireless_communication_object.send_info(/*global.gps_data.location.latitude, global.gps_data.location.longtitude, global.bearing_container.compass_bearing, global.gps_data.speed*/);
		wireless_communication_object.send_info(33.3333333, -44.4444444, 55.5, 66.6);
		
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
	yield();
}



#endif