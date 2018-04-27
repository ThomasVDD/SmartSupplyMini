#include "Message.h"

String Message::getString() {
	String result = "";
	result += delimiter;
	result += voltage + delimiter;
	result += current + delimiter;
	result += (isCC ? "CC" : "CV") + delimiter;
	result += (isOn ? "ON" : "OFF") + delimiter;
	result += maxPower + delimiter;
    return result;
};