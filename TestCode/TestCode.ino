#include<SPI.h>
#include<Wire.h>
#include<INA219.h>
INA219 ina219;

float CC1;
float CC2;
float ADC5V;
float T;
float Dplus;
float Dminus;
float Vout;
float Iout;
int state = 0;

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Default communication rate of the Bluetooth module
  SPI.begin();
  ina219.begin();
  ina219.setCalibration_320();
  analogReference(EXTERNAL);

  //PWM
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);              /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS10);               /* no prescaling */
  ICR1 = 1000;

  //cc1
  pinMode(A11, INPUT);
  //cc2
  pinMode(A7, INPUT);
  //green led
  pinMode(5, OUTPUT);
  //red led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //pwm
  pinMode(11, OUTPUT);
  analogWrite(11, 128);
  //adc_5v
  pinMode(A8, INPUT);
  //switch
  pinMode(4, INPUT_PULLUP);
  //temperature
  pinMode(A5, INPUT);
  //PGOOD (A4 should be 18, but 18 is not working)
  pinMode(A4, INPUT);
  //D+
  pinMode(A1, INPUT);
  //D-
  pinMode(A0, INPUT);
  //PWM_Vout
  pinMode(9, OUTPUT);
  analogWrite16(9, 1000);
  //analogWrite(9, 75);
  //PWM_Iout
  pinMode(10, OUTPUT);
  analogWrite16(10, 400);
  //analogWrite(10, 255);
  //SS pin
  pinMode(A8, OUTPUT);
  digitalWrite(A8, HIGH);
  //ADC_Vout
  pinMode(A3, INPUT);
  //ADC_Iout
  pinMode(A2, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  Serial.println(ina219.getCurrent_mA());

  Serial.println("start");
  SPI.setClockDivider(SPI_CLOCK_DIV8);          //ensure correct clockdivider (lcd library also uses a clockdivider)
  digitalWrite(A8, LOW);          //pull pin low to connect
  SPI.transfer(B00010011);                      //let MCP41010 know we want to set the potentiometer
  SPI.transfer(255);                 //send actual value
  digitalWrite(A8, HIGH);         //pull pin high to disconnect


  Serial.println("test");
  CC1 = analogRead(A11) * 2.56 * 0.001;
  CC2 = analogRead(A7) * 2.56 * 0.001;
  Serial.print("CC1 = ");
  Serial.println(CC1);
  Serial.print("CC2 = ");
  Serial.println(CC2);

  //  ADC5V = analogRead(A8)*2.56*0.001;
  //  Serial.print("ADC5V = ");
  //  Serial.println(ADC5V);

  Vout = analogRead(A3) * 2.56 * 0.001 * 10;
  Serial.print("Vout = ");
  Serial.println(Vout);

  Iout = analogRead(A2) * 2.56 * 0.001 * 0.4;
  Serial.print("Iout = ");
  Serial.println(Iout);

  Dplus = analogRead(A1) * 2.56 * 0.001;
  Serial.print("D+ = ");
  Serial.println(Dplus);

  Dminus = analogRead(A0) * 2.56 * 0.001;
  Serial.print("D- = ");
  Serial.println(Dminus);

  if ( (Dplus < 0.9 && Dplus > 0.6) && (Dminus < 1 && Dminus > 0.6)) {
  }

  if ( (Dminus < 0.95 && Dminus > 0.6) && (Dplus < 1 && Dplus > 0.6)) {
  }

  //Vtemp = 0.75V + (T°C ✕ 10mV/°C) for the MAX6610
  //(Vtemp - 0.75V)/0.010V = T
  T = analogRead(A5) * 2.56 * 0.0009766;
  T = (T - 0.75) * 100;
  Serial.print("temperature = ");
  Serial.println(T);

  if (digitalRead(4) == LOW) {
    Serial.println("button pressed");
    Serial1.println("OUTPUT");
  }

  if (digitalRead(A4) == HIGH) {
    Serial.println("PGOOD");
  }

  analogWrite(5, 1);
  delay(100);              // wait for a second
  digitalWrite(5, HIGH);
  delay(100);              // wait for a second

  //bluetooth
  String readString = "";
  if (Serial1.available() > 0) {
    while (Serial1.available()) {
      char c = Serial1.read();         //gets one byte from serial buffer
      readString += c;                //makes the string readString
      delay(2);                       //slow looping to allow buffer to fill with next character
    }
    Serial.print("RECEIVED DATA: ");
    Serial.println(readString);
  }

//  if (Serial1.available() > 0) { // Checks whether data is comming from the serial port
//    state = Serial1.read(); // Reads the data from the serial port
//    Serial.print("RECEIVED DATA: ");
//    Serial.println(state);
//  }
//  if (state == 0) {
//    digitalWrite(13, LOW); // Turn LED OFF
//    Serial1.println("CC"); // Send back, to the phone, the String "LED: ON"
//    state = 2;
//  }
//  else if (state == 1) {
//    digitalWrite(13, HIGH);
//    Serial1.println("CV");
//    state = 2;
//  }
  Serial1.println("12>14>CC>ON>2.5");
  delay(2000);
}
