#ifndef HardwareInterface_h
#define HardwareInterface_h

#include "Arduino.h"

#define CC1_PIN A11
#define CC2_PIN A7
#define GLED_PIN 5
#define RLED_PIN 13
#define PWM_PIN 11
#define ADC5V_PIN A8
#define BTN_PIN 4
#define TEMP_PIN A5
#define PGOOD_PIN A4
#define DPLUS_PIN A1
#define DMIN_PIN A0
#define PWMVOUT_PIN 9
#define PWMIOUT_PIN 10
#define ADCVOUT_PIN A3
#define ADCIOUT_PIN A2
#define ISR_PIN 7

#define VOLTAGE_REFERENCE 2.56
#define VOLTAGE_DIVIDER_VOLTAGE 10
#define VOLTAGE_AMPLIFICATION 5
#define VOLTAGE_DIVIDER_CURRENT 0.4
#define VOLTAGE_TO_CURRENT 0.2 //5V = 1A ==> *0.2
#define VOLTAGE_DIVIDER_SUPPLY 5

#define MAX_REGISTER_VALUE 1023

#define CALIBRATION_CURRENT 2

class HardwareInterface {

public:

	HardwareInterface() {}

	void setup();

	float getMaxPower();
	float getVoltage();
	float getCurrent();
	float getMeasuredCurrent();
	float getMeasuredVoltage();
	bool getOutputOn();

	void setVoltage(float);
	void setCurrent(float);
	void setOutputOn();
	void setOutputOff();

	bool checkCC();
	bool readButton();

	void measureVoltage();
	void measureCurrent();
	void overcurrentProtectionUSB();

private:
	float voltageSetting = 0;
	float currentSetting = 500;
	float voltageReading = 0;
	float currentReading = 0;

	float supplyVoltage = 0;

	bool outputOn = false;
	int maxPower = 12;

	void calibratePowerSupply();
	void setupPWM16(float);
	void analogWrite16(uint8_t, uint16_t);

	void setBoostConverter(float);
	void checkUSB();
	bool checkVoltage(int, int);
	void lightLED();

	/* averages for voltage and current */
	const int numReadingsV = 5;        // number of readings to be averaged
	int readingsV[20] = {0};        	// the readings from the analog input
	int readIndexV = 0;                 // the index of the current reading
	int totalV = 0;                     // the running total
	float averageV = 0;                 // the average
	float realAverageV = 0;             // the remapped average
	
	const int numReadingsA = 5;
	int readingsA[20] = {0};
	int readIndexA = 0;
	int totalA = 0;
	float averageA = 0;
	float realAverageA = 0;
	
	float finalA = 0;                   // holds the most accurate current measurement to be displayed
	int currentRange = 320;             // the range of the current measurement, initialized at 320 mA
};

#endif
