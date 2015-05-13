#ifndef wireless_communication_h
#define wireless_communication_h
#include "global.h"
#include "XBee.h"
#include "Location.h"

#define COORDINATOR_ADDR	0x0013a200,0x40B5BBD0
#define BOAT_1_ADDR			0x0013A200,0x40B5BBF0
#define BOAT_2_ADDR			0x0013A200,0x40B5BBF5
#define BOAT_3_ADDR			0x0013A200,0x40B48973
#define BROADCAST_ADDR		0x0,ZB_BROADCAST_ADDRESS//defined in Xbee.h	

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
	enum COMMAND {
		update_other_boats_location_enum,
		add_waypoint_enum,
		force_waypoint_enum,
		turn_compass_calibration_on_enum,
		turn_compass_calibration_off_enum
	};
	//Data package to send and receive
	typedef struct{
		XBEE_ID ID;
		COMMAND	command;							//A command is set by sender and receiver acts on it. ( e.g. calls: update_other_boats_location()	add_waypoint()	force_waypoint() )
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

		addr64 = XBeeAddress64(COORDINATOR_ADDR);			//*********** Set Address of receiver ( BOAT_1_ADDR	BOAT_2_ADDR	COORDINATOR_ADDR	...)
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

			Location target;
			switch (inData.command)
			{
			case update_other_boats_location_enum:
				update_other_boats_location(inData);
				break;
			case add_waypoint_enum:
				target.latitude = inData.latitude;
				target.longtitude = inData.longitude;
				add_waypoint(target);
				break;
			case force_waypoint_enum:
				target.latitude = inData.latitude;
				target.longtitude = inData.longitude;
				force_waypoint(target);
				break;
			case turn_compass_calibration_on_enum:
				turn_compass_calibration_on();
				break;
			case turn_compass_calibration_off_enum:
				turn_compass_calibration_off();
				break;
			default:
				break;
			}
			if (global.debug_handler.wireless_communication_debug){
				Serial.println(inData.ID);
				Serial.println(inData.latitude, 7);
				Serial.println(inData.longitude, 7);
				Serial.println(inData.bearing, 7);
				Serial.println(inData.speed, 7);
				Serial.print('\r');
				Serial.print('\n');
			}

		}
		else if (global.debug_handler.wireless_communication_debug) Serial.println("didnt get anything");
	}
	void add_waypoint(Location target){
		global.waypoints.enqueue(target);
	}
	void force_waypoint(Location target){
		while (global.waypoints.actual_size > 0) {
			global.waypoints.dequeue();
		}
		global.waypoints.enqueue(target);
		global.force_load_waypoint_in_guidance = true;
	}
	void turn_compass_calibration_on(void){
		global.toggle_compass_calibration = true;
	}
	void turn_compass_calibration_off(void){
		global.toggle_compass_calibration = false;
	}

private:
	DATA_ outData;
	DATA_ inData;

	XBee xbee = XBee();
	uint8_t payload[80] = {};// = { 'h', 'e', 'y' };
	XBeeAddress64 addr64 = XBeeAddress64(0x0, 0x0);			// address of coordinator: 0x0013a200, 0x40B5BBD0
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

	//Returns an ID of current xbee according to its Serial number
	XBEE_ID get_ID(void){
		uint8_t slCmd[] = { 'S', 'L' };
		AtCommandRequest atRequest = AtCommandRequest(slCmd);
		AtCommandResponse atResponse = AtCommandResponse();


		// send the command
		xbee.send(atRequest);

		// wait up to 5 seconds for the status response
		if (xbee.readPacket(5000)) {
			// got a response!

			// should be an AT command response
			if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
				xbee.getResponse().getAtCommandResponse(atResponse);

				if (atResponse.isOk()) {
					if (atResponse.getValueLength() > 0) {
						Serial.print("Command value: ");

						for (int i = 0; i < atResponse.getValueLength(); i++) {
							Serial.print(atResponse.getValue()[i], HEX);
						}
						Serial.println("");
					}
				}
				else {
					Serial.print("Command return error code: ");
					Serial.println(atResponse.getStatus(), HEX);
				}
			}
			else {
				Serial.print("Expected AT response but got ");
				Serial.print(xbee.getResponse().getApiId(), HEX);
			}
		}
		else {
			// at command failed
			if (xbee.getResponse().isError()) {
				Serial.print("Error reading packet.  Error code: ");
				Serial.println(xbee.getResponse().getErrorCode());
			}
			else {
				Serial.print("No response from radio");
			}
		}

	}
};

//the object corrosponding to this class is implemented in global, so the two threads both can read it.
wireless_communication_class wireless_communication_object;

void wireless_cummonication(){
	wireless_communication_object.init();
	while (1)
	{
		wireless_communication_object.send_info(global.gps_data.location.latitude, global.gps_data.location.longtitude, global.bearing_container.compass_bearing, global.gps_data.location.speed);
		//wireless_communication_object.send_info(33.3333333, -44.4444444, 55.55, 66.66);
		delay(500);

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