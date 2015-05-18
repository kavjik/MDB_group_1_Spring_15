#ifndef wireless_communication_h
#define wireless_communication_h
#include "global.h"
#include "XBee.h"
#include "Location.h"

#define COORDINATOR_ADDR	0x0013a200,0x40B5BBD0		
#define BOAT_1_ADDR			0x0013A200,0x40B5BBF0
#define BOAT_2_ADDR			0x0013A200,0x40B5BBF5
#define BOAT_3_ADDR			0x0013A200,0x40B48973
#define BOAT_4_ADDR			0x0013A200,0x40BD3A3F

#define BROADCAST_ADDR		0x0,ZB_BROADCAST_ADDRESS//defined in Xbee.h	(not used because of lower performance)

class wireless_communication_class
{
public:
	wireless_communication_class(){}
	void init(void){
		Serial2.begin(9600);
		xbee.setSerial(Serial2);
		xbee.begin(Serial2);
		//Startup delay to wait for XBee radio to initialize.
		// you may need to increase this value if you are not getting a response
		delay(2000);
		get_ID();
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
		uint8_t	waypoints_count;
		uint8_t checksum;
	}DATA_;

	void send_info(double latitude, double longitude, float bearing, float speed, int count){
		outData.ID = ID_;
		//outData.ID = boat2;
		outData.latitude = latitude;
		outData.longitude = longitude;
		outData.bearing = bearing;
		outData.speed = speed;
		outData.waypoints_count = global.waypoints.count();
		outData.checksum = 0;

		uint8_t pay_load[sizeof(DATA_)];
		memcpy(pay_load, &outData, sizeof(DATA_));					//Convert DATA_ to uint8

		for (int i = 0; i < sizeof(DATA_) - 1; i++){
			outData.checksum += ~(uint8_t)(pay_load[i]);
		}
		pay_load[sizeof(DATA_) - 1] = outData.checksum;

		if (ID_ != boat1){
			//addr64 = XBeeAddress64(BOAT_1_ADDR);						//*********** Set Address of receiver
			//zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
			//xbee.send(zbTx);
		}
		if (ID_ != boat2){
			//addr64 = XBeeAddress64(BOAT_2_ADDR);					
			//zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
			//xbee.send(zbTx);
		}
		if (ID_ != boat3){
			addr64 = XBeeAddress64(BOAT_3_ADDR);
			zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
			xbee.send(zbTx);
		}
		if (ID_ != boat4){
			//addr64 = XBeeAddress64(BOAT_4_ADDR);					
			//zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
			//xbee.send(zbTx);
		}
		if (ID_ != coordinator){										//for testing purposes... coordinator(ground station) should not be sending anything
			addr64 = XBeeAddress64(COORDINATOR_ADDR);
			zbTx = ZBTxRequest(addr64, pay_load, sizeof(DATA_));
			xbee.send(zbTx);
		}
	}
	void get_info(void){
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {
			// got something
			if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
				// got a zb rx packet
				xbee.getResponse().getZBRxResponse(rx);// now fill our zb rx class
			}
			//inData = {};//clear inData package
			uint8_t check = 0;
			uint8_t pay_load2[sizeof(DATA_)];
			memcpy(pay_load2, rx.getData(), sizeof(DATA_));				//Convert uint8 to DATA_

			memcpy(&inData, pay_load2, sizeof(DATA_));

			for (int i = 0; i < sizeof(DATA_) - 1; i++){ //Hard coded for now, could be changed to sizeof(DATA_) -1
				check += ~(uint8_t)(pay_load2[i]);
			}

			if (check == pay_load2[sizeof(DATA_) - 1]) {

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
					Serial.println(inData.waypoints_count);
					Serial.println(check);
					Serial.println(pay_load2[sizeof(DATA_) - 1]);
					Serial.print('\r');
					Serial.print('\n');

				}

			}
			else {
				if (global.debug_handler.wireless_communication_debug){
					Serial.println("checksum error");
					Serial.println(inData.ID);
					Serial.println(inData.latitude, 7);
					Serial.println(inData.longitude, 7);
					Serial.println(inData.bearing, 7);
					Serial.println(inData.speed, 7);
					Serial.println(inData.waypoints_count);
					Serial.println(check);
					Serial.println(pay_load2[sizeof(DATA_) - 1]);
					Serial.print('\r');
					Serial.print('\n');
				}
			}
		}
		else if (global.debug_handler.wireless_communication_debug) Serial.println("didnt get anything");
	}
	void add_waypoint(Location target){
		global.waypoints.enqueue(target);
	}
	void force_waypoint(Location target){
		while (global.waypoints.count() > 0) {
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
	XBEE_ID ID_;

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
		global.other_boats[inData.ID].timestamp = millis();
	}

	//Returns an ID of current xbee according to its Serial number
	void get_ID(void){
		uint8_t slCmd[] = { 'S', 'L' };
		AtCommandRequest atRequest = AtCommandRequest(slCmd);
		AtCommandResponse atResponse = AtCommandResponse();
		char SL_response[10];
		char SL_boat1[] = "40b5bbf0";
		char SL_boat2[] = "40b5bbf5";
		char SL_boat3[] = "40b48973";
		char SL_boat4[] = "40bd3a3f";
		char SL_coordinator[] = "40b5bbd0";

		xbee.send(atRequest);// send the command

		// wait up to 5 seconds for the status response
		if (xbee.readPacket(5000)) {
			// got a response!
			if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
				xbee.getResponse().getAtCommandResponse(atResponse);
				if (atResponse.isOk()) {
					if (atResponse.getValueLength() > 0) {
						for (int i = 0; i < atResponse.getValueLength(); i++) {
							sprintf(&SL_response[i * 2], "%02x", atResponse.getValue()[i]);
						}

						if (strcmp(SL_response, SL_boat1) == 0){
							ID_ = boat1;
						}
						else if (strcmp(SL_response, SL_boat2) == 0){
							ID_ = boat2;
							Serial.print(SL_response);
							Serial.println("");
						}
						else if (strcmp(SL_response, SL_boat3) == 0){
							ID_ = boat3;
							Serial.print(SL_response);
							Serial.println("");
						}
						else if (strcmp(SL_response, SL_boat4) == 0){
							ID_ = boat4;
						}
						else if (strcmp(SL_response, SL_coordinator) == 0){
							ID_ = coordinator;
							Serial.print(SL_response);
							Serial.println("");
						}
						else
							Serial.println("not able to set correct ID check SL");
					}
				}
			}
		}
		else {
			Serial.print("No response from radio");// at command failed
		}
	}
};

//the object corrosponding to this class is implemented in global, so the two threads both can read it.
wireless_communication_class wireless_communication_object;

void wireless_cummonication(){
	wireless_communication_object.init();
	while (1)
	{
		//wireless_communication_object.send_info(global.gps_data.location.latitude, global.gps_data.location.longtitude, global.bearing_container.compass_bearing, global.gps_data.speed, global.waypoints.count());
		wireless_communication_object.send_info(22.3333333, -77.4444444, 33.55, 23.66, 3);
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
#define WAIT_TIME_BEFORE_OTHER_BOAT_IS_INVALID 5000
void wireless_maintain_if_boat_is_valid_thread(void){
	for (int i = 0; i < NUMBER_OF_OTHER_BOATS_IN_BOAT_ARRAY; i++){
		if (global.other_boats[i].timestamp < (millis() - WAIT_TIME_BEFORE_OTHER_BOAT_IS_INVALID)) {
			global.other_boats[i].is_valid_boat = false;
		}
	}
}



#endif