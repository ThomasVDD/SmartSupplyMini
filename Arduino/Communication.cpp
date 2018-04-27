#include "Communication.h"

void Communication::setup() {
	if (serial == "serial1") {
		Serial1.begin(baudrate);
	} else {
		Serial.begin(baudrate);
	}
};

String Communication::read() {
	String readString = "";
	if (serial == "serial1") {
		while (Serial1.available()) {
    		char c = Serial1.read();         //gets one byte from serial buffer
    		readString += c;                //makes the string readString
    		delay(2);                       //slow looping to allow buffer to fill with next character
  		}
		//Serial1.println(message);
	} else {
		while (Serial.available()) {
    		char c = Serial.read();         //gets one byte from serial buffer
    		readString += c;                //makes the string readString
    		delay(2);                       //slow looping to allow buffer to fill with next character
  		}
		//Serial.println(message);
	}
	return readString;
};

void Communication::send(Message message) {
	if (serial == "serial1") {
		Serial1.println(message.getString());
	} else {
		Serial.println(message.getString());
	}
};

/*void Communication::sendVoltage(float voltage) {
    send(Message("Voltage", voltage));
};*/


