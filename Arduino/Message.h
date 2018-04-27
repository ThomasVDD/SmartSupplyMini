#ifndef Message_h
#define Message_h

#include "Arduino.h"

class Message {

public:

    Message(float voltage, float current, bool isCC, bool isOn, float maxPower) : 
    	voltage(voltage), current(current), isCC(isCC), isOn(isOn), maxPower(maxPower) {}

    String getString();

private:
	const String delimiter = ">";

	float voltage;
	float current;
	bool isCC;
	bool isOn;
	float maxPower;

};

#endif


/*//Message(String operation, float value) : operation(operation), value(value) {}*/
//Message(String operation, float value) : operation(operation), value(value) {}