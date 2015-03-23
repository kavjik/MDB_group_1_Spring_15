#ifndef rudder_and_sail_control_h
#define rudder_and_sail_control_h

void rudder_and_sail_control(){ //TODO either make this, implement from the previous group, or delete, atm its just another staying alive loop, so we can see the program actually runs.
	analogWrite(DAC1, 170);
	delay(1237);
	analogWrite(DAC1, 0);
	delay(1237);
}

#endif