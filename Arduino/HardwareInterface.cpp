#include "HardwareInterface.h"

#include <INA219.h>
#include <SPI.h>

INA219 ina219;

void HardwareInterface::setup() {
	SPI.begin();
	ina219.begin();
	ina219.setCalibration_320();
	analogReference(EXTERNAL);

	pinMode(CC1_PIN, INPUT);
	pinMode(CC2_PIN, INPUT);
	pinMode(GLED_PIN, OUTPUT);
	pinMode(RLED_PIN, OUTPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(ADC5V_PIN, INPUT);
	pinMode(BTN_PIN, INPUT_PULLUP);
	pinMode(TEMP_PIN, INPUT);
	pinMode(PGOOD_PIN, INPUT);
	pinMode(DPLUS_PIN, INPUT);
	pinMode(DMIN_PIN, INPUT);
	pinMode(PWMVOUT_PIN, OUTPUT);
	pinMode(PWMIOUT_PIN, OUTPUT);
	pinMode(ADCVOUT_PIN, INPUT);
	pinMode(ADCIOUT_PIN, INPUT);

	analogWrite(PWM_PIN, 128);
	calibratePowerSupply();
	setBoostConverter(0);
	checkUSB();
	lightLED();
}

float HardwareInterface::getMaxPower() {
	return maxPower;
}

float HardwareInterface::getMeasuredVoltage() {
	return voltageReading;
}

float HardwareInterface::getMeasuredCurrent() {
	return currentReading;
}

float HardwareInterface::getVoltage() {
	return voltageSetting;
}

float HardwareInterface::getCurrent() {
	return currentSetting;
}

void HardwareInterface::lightLED(){
	analogWrite(GLED_PIN, 255 - 10*getVoltage());
	analogWrite(RLED_PIN, 10*getVoltage());
}

bool HardwareInterface::readButton(){
	if(digitalRead(BTN_PIN) == LOW){
		delay(200); // debounce
		// short press
		if(digitalRead(BTN_PIN) == HIGH){
			outputOn = !outputOn;
			setVoltage(getVoltage());
			setCurrent(getCurrent());
			Serial.println("short press");
		}
		// long press
		else{
			Serial.println("preset");
			return true;
		}
	}
	return false;	
}

void HardwareInterface::setVoltage(float voltage) {
	Serial.print("Set voltage ");
	Serial.println(voltage);
	voltageSetting = voltage;
	int setting = (int) (voltageSetting / supplyVoltage / VOLTAGE_AMPLIFICATION * MAX_REGISTER_VALUE);
	Serial.println(setting);
	if (outputOn) {
		analogWrite16(PWMVOUT_PIN, setting);
		setBoostConverter(voltage);
	}
	else{
		analogWrite16(PWMVOUT_PIN, 0);
		setBoostConverter(0);
	}
	lightLED();
}

void HardwareInterface::setCurrent(float current) {
	Serial.print("Set current ");
	Serial.println(current);
	currentSetting = current;
	int setting = (int) (0.001 * currentSetting / supplyVoltage / VOLTAGE_TO_CURRENT * MAX_REGISTER_VALUE);
	Serial.println(setting);
	if (outputOn){
		analogWrite16(PWMIOUT_PIN, (uint16_t)setting);
	}
	else{
		analogWrite16(PWMIOUT_PIN, 0);
	}
}

bool HardwareInterface::getOutputOn() {
	return outputOn;
}

void HardwareInterface::setOutputOn() {
	Serial.println("Power on");
	outputOn = true;
	setVoltage(getVoltage());
	setCurrent(getCurrent());
}

void HardwareInterface::setOutputOff() {
	Serial.println("Power off");
	outputOn = false;
	setVoltage(getVoltage());
  //setCurrent(getCurrent());
}

bool HardwareInterface::checkCC() {
	return currentReading >= currentSetting;
}

void HardwareInterface::calibratePowerSupply() {
	supplyVoltage = 0;
	// Taking average of 100 measuremnts
	for (int i=0; i<100; i++) {
		supplyVoltage += analogRead(ADC5V_PIN);
		delay(10);
	}
	supplyVoltage /= 100;
	// Convert to actual voltage
	supplyVoltage = supplyVoltage * VOLTAGE_REFERENCE * VOLTAGE_DIVIDER_SUPPLY / MAX_REGISTER_VALUE;
	// Setup PWM with procentual error
	setupPWM16(supplyVoltage / 5); 
	// Configure pin as output for digital potmeter (only input while calibrating)
	pinMode(ADC5V_PIN, OUTPUT);
	digitalWrite(ADC5V_PIN, HIGH);
}

/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void HardwareInterface::setupPWM16(float error) {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);              /* mode 14: fast PWM, TOP=ICR1 */
	TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS10);               /* no prescaling */
  ICR1 = error * MAX_REGISTER_VALUE;  /* TOP counter value = 1000; +/- 10 bit resolution @ 15 kHz*/
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void HardwareInterface::analogWrite16(uint8_t pin, uint16_t val)
{
	switch (pin) {
		case PWMVOUT_PIN: 
		OCR1A = val; break;
		case PWMIOUT_PIN: 
		OCR1B = val; break;
	}
}

/* Measure the voltage & average for lower noise */
void HardwareInterface::measureVoltage() {
	totalV = totalV - readingsV[readIndexV];      //subtract the last reading
	readingsV[readIndexV] = analogRead(ADCVOUT_PIN);  //read from the sensor
	totalV = totalV + readingsV[readIndexV];      //add the reading to the total
	readIndexV = readIndexV + 1;                  //advance to the next position in the array
	if (readIndexV >= numReadingsV) {             //if we're at the end of the array...
	  readIndexV = 0;                             //...wrap around to the beginning
}
	averageV = totalV / numReadingsV;             //calculate the average
	voltageReading = averageV * VOLTAGE_REFERENCE * VOLTAGE_DIVIDER_VOLTAGE / MAX_REGISTER_VALUE;     //map 1-2.048V range to corresponding value

	Serial.println(voltageReading);
}

/* Measure the current & average for lower noise*/
void HardwareInterface::measureCurrent() {
	totalA = totalA - readingsA[readIndexA];
	readingsA[readIndexA] = analogRead(ADCIOUT_PIN);
	totalA = totalA + readingsA[readIndexA];
	readIndexA = readIndexA + 1;
	if (readIndexA >= numReadingsA) {
		readIndexA = 0;
	}
	averageA = totalA / numReadingsA;
	realAverageA = averageA * VOLTAGE_REFERENCE * VOLTAGE_DIVIDER_CURRENT / MAX_REGISTER_VALUE;  ;

	// Measure the current with the INA219
	if (realAverageA < 300) {                      // current < 320 mA can be measured with INA219
		if (realAverageA < 20 && currentRange != 40) {
			currentRange = 40;
			ina219.setCalibration_40();
		}
		else if (realAverageA >= 20 && realAverageA < 50 && currentRange != 80) {
			currentRange = 80;
			ina219.setCalibration_80();
		}
		else if (realAverageA >= 50 && realAverageA < 150 && currentRange != 160) {
			currentRange = 160;
			ina219.setCalibration_160();
		}
		else if (realAverageA >= 150 && realAverageA < 320 && currentRange != 320) {
			currentRange = 320;
			ina219.setCalibration_320();
		}
		currentReading = ina219.getCurrent_mA() - CALIBRATION_CURRENT;
		if (currentReading < 0) currentReading = 0;
	} 
	else {
		currentReading = realAverageA - CALIBRATION_CURRENT;
		if (currentReading < 0) currentReading = 0;
	}
}

void HardwareInterface::setBoostConverter(float voltage) {
  //voltage += 2.5;
  int boostConverter = 0; // 0 - 255
  int potentiometer = 0;  // 0 - 10000
  // 7 - 25V
  if (voltage < 7){ 
  	potentiometer = 10000;
  	boostConverter = 0;
  }
  else{
  	potentiometer = (int)(2200/(0.176*voltage - 1)) - 620;
  	boostConverter = 255 - (potentiometer / 40);
  }
  
	SPI.setClockDivider(SPI_CLOCK_DIV8);   //ensure correct clockdivider
	digitalWrite(ADC5V_PIN, LOW);          //pull pin low to connect
	SPI.transfer(B00010011);               //let MCP41010 know we want to set the potentiometer
	SPI.transfer(boostConverter);          //send actual value
	digitalWrite(ADC5V_PIN, HIGH);         //pull pin high to disconnect
}

void HardwareInterface::checkUSB(){
  // measure pins
  float CC1 = analogRead(CC1_PIN) * VOLTAGE_REFERENCE / MAX_REGISTER_VALUE;
  float CC2 = analogRead(CC2_PIN) * VOLTAGE_REFERENCE / MAX_REGISTER_VALUE;
  float Dplus = analogRead(DPLUS_PIN) * VOLTAGE_REFERENCE / MAX_REGISTER_VALUE;
  float Dminus = analogRead(DMIN_PIN) * VOLTAGE_REFERENCE / MAX_REGISTER_VALUE;
  //check proprietary chargers
  if (checkVoltage(Dplus, 2) && checkVoltage(Dminus, 2)){  
  	// low power: 500 mA
  	maxPower = 2;
  }
  else if (checkVoltage(Dplus, 2) && checkVoltage(Dminus, 2.7)){ 
  	// medium power: 1000 mA
  	maxPower = 4;
  }
  else if (checkVoltage(Dplus, 2.7) && checkVoltage(Dminus, 2)){ 
  	// high power: 2100 mA
  	maxPower = 8;
  }
  else{
    maxPower = 5;
  }
  // Maximum USB C power
  if (CC1 >= 1.5 || CC2 >= 1.5){
    maxPower = 12; 
  }
  Serial.print("Maximum Power: ");
  Serial.println(maxPower);
}

bool HardwareInterface::checkVoltage(int pin, int value){
	return (pin < (value + 0.2) && pin > (value - 0.2));
}

void HardwareInterface::overcurrentProtectionUSB(){
	// check set power
	float powerset = getVoltage() * getCurrent() * 1000;
	// adjust max power to be 1W less than the set power
	maxPower = (int)powerset - 1;
	// adjust set current to obey the maxPower
	setCurrent(1000*maxPower/getVoltage());
	Serial.println("==================================================");
}
