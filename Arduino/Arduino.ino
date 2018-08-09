/* //==\\ ||\\  //||     //\\     ||===\\   ========  //==\\  ||    ||  ||===\\   ||===\\   ||        \\    //
 * ||     || \\// ||    //  \\    ||    ||     ||     ||      ||    ||  ||    ||  ||    ||  ||         \\  //
 * \\==\\ ||  \/  ||   //    \\   ||===//      ||     \\==\\  ||    ||  ||===//   ||===//   ||          \\//    ||\\//|| || ||\ || ||
 *     || ||      ||  //======\\  ||   \\      ||         ||  ||    ||  ||        ||        ||           //     || \/ || || || \\| ||
 * \\==// ||      || //        \\ ||    \\     ||     \\==//  \\====//  ||        ||        ||======    //      ||    || || ||  || ||
 * 
 * Jeroen De Geeter
 * Thomas Van den Dries
 * github.com/ThomasVDD
 * ThomasVDD on Instructables 
 * 
 * Board: arduino micro
 * Programmer: AVR ISP / AVRISP MKII
 */

#include "communication.h"
#include "HardwareInterface.h"
#include "Message.h"

Communication mobile("serial1", 9600);
Communication pc("serial", 9600);
HardwareInterface hardwareInterface;

int presetPosition = 0;
float presetVoltages[4] = {3.3, 5, 12, 20};
float presetCurrents[4] = {500, 500, 1000, 500};

void setup() {
  Serial.begin(9600);
  Serial.println("SmartSupply Mini");
	//attachInterrupt(digitalPinToInterrupt(ISR_PIN), USB_ISR, RISING);
  hardwareInterface.setup();
	mobile.setup();
	pc.setup();  
}

void loop() {
	Message message(hardwareInterface.getMeasuredVoltage(), 
              		hardwareInterface.getMeasuredCurrent(),
              		hardwareInterface.checkCC(),
              		hardwareInterface.getOutputOn(),
              		hardwareInterface.getMaxPower());

	mobile.send(message);
	String messageR = mobile.read();

	if (messageR == "ON") {
		hardwareInterface.setOutputOn();
	}
	if (messageR == "OFF") {
  		hardwareInterface.setOutputOff();			
	}
	if (messageR.startsWith("V")) {
  		hardwareInterface.setVoltage(messageR.substring(1).toFloat());			
	}
	if (messageR.startsWith("I")) {
		hardwareInterface.setCurrent(messageR.substring(1).toFloat());
	}

	hardwareInterface.measureVoltage();
	hardwareInterface.measureCurrent();
	if(hardwareInterface.readButton()){
		presetPosition = (++presetPosition) % 4;
		hardwareInterface.setCurrent(min(presetCurrents[presetPosition],1000*hardwareInterface.getMaxPower()/presetVoltages[presetPosition]));
		hardwareInterface.setVoltage(presetVoltages[presetPosition]);
		delay(1000);	
	}
	delay(200);
}

void USB_ISR(){
	hardwareInterface.overcurrentProtectionUSB();
}
