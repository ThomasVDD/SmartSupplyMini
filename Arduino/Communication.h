#ifndef Communication_h
#define Communication_h

#include "Arduino.h"
#include "Message.h"

class Communication {

public:

    Communication(String serialName, int baud) : serial(serialName), baudrate(baud) {}

    void setup();
    String read();    
    void send(Message message);

private:
	String serial;
	int baudrate;

};

#endif