#include <avr/wdt.h>

const int pin_B1IN = A0;
const int pin_B2IN = A1;
const int pin_B3IN = A2;

const int pin_B1OUT = 3;
const int pin_B2OUT = A5;
const int pin_B3OUT = 2;

const int pin_EIN = 11;
const int pin_BSIG = 7;

const int pin_StOUT = A4;
const int pin_ROUT = 13;
const int pin_LOUT = 12;

const int GESTURE_DELAY = 250;
const int RESET_TIME = 10*1000;
const int END_DELAY = 5*1000;
const int B_DELAY = 1000;

unsigned long time_StOUT;
unsigned long time_ROUT;
unsigned long time_LOUT;
unsigned long time_EIN;
unsigned long time_RESET;

unsigned long time_B1IN;
unsigned long time_B2IN;
unsigned long time_B3IN;

void setup() {
  Serial.begin(9600);
  pinMode(pin_B1IN, INPUT);
  pinMode(pin_B2IN, INPUT);
  pinMode(pin_B3IN, INPUT);
  
  pinMode(pin_B1OUT, OUTPUT);
  pinMode(pin_B2OUT, OUTPUT);
  pinMode(pin_B3OUT, OUTPUT);

  pinMode(pin_EIN, INPUT);
  pinMode(pin_BSIG, OUTPUT);

  pinMode(pin_StOUT, OUTPUT);
  pinMode(pin_ROUT, OUTPUT);
  pinMode(pin_LOUT, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  time_StOUT = 0;
  time_ROUT = 0;
  time_LOUT = 0;
  time_EIN = 0;
  time_B1IN = 0;
  time_B2IN = 0;
  time_B3IN = 0;
  time_RESET = millis();
}

void loop() {
  int B1IN = digitalRead(pin_B1IN);
  int B2IN = digitalRead(pin_B2IN);
  int B3IN = digitalRead(pin_B3IN);

  int EIN = digitalRead(pin_EIN);

  if (B1IN == HIGH && millis()-time_B1IN>=B_DELAY) {
    digitalWrite(pin_B1OUT, LOW);
    Serial.write('b');
    time_B1IN = millis();
  } else {
    digitalWrite(pin_B1OUT, HIGH);
  }

  if (B2IN == HIGH && millis()-time_B2IN>=B_DELAY) {
    digitalWrite(pin_B2OUT, LOW);
    Serial.write('b');
    time_B2IN = millis();
  } else {
    digitalWrite(pin_B2OUT, HIGH);
  }

  if (B3IN == HIGH && millis()-time_B3IN>=B_DELAY) {
    digitalWrite(pin_B3OUT, LOW);
    Serial.write('b');
    time_B3IN = millis();
  } else {
    digitalWrite(pin_B3OUT, HIGH);
  }

  if (EIN == LOW && millis()-time_EIN>=END_DELAY) {
    Serial.write('e');
    time_EIN = millis();
  }

  //if (B1IN + B2IN + B3IN >= 1) {
    //Serial.write('b');
  //}

  if (Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    char *input = (char *)malloc(10*sizeof(char));
    input = Serial.read();
    //sinput.trim();
    //char input = sinput[0];
    //Serial.print(input);

    if (input == 's') {
      time_StOUT = millis();
      //digitalWrite(pin_StOUT, LOW);
    } else if (input == 'r') {
      time_ROUT = millis();
      //digitalWrite(pin_ROUT, LOW);
    } else if (input == 'l') {
      time_LOUT = millis();
      //digitalWrite(pin_LOUT, LOW);
    }
  } else {
      digitalWrite(LED_BUILTIN, LOW);
  }

  if (millis() - time_StOUT <= GESTURE_DELAY) {
    digitalWrite(pin_StOUT, LOW);
  } else {
    digitalWrite(pin_StOUT, HIGH);
  }

  if (millis() - time_ROUT <= GESTURE_DELAY) {
    digitalWrite(pin_ROUT, LOW);
  } else {
    digitalWrite(pin_ROUT, HIGH);
  }

  if (millis() - time_LOUT <= GESTURE_DELAY) {
    digitalWrite(pin_LOUT, LOW);
  } else {
    digitalWrite(pin_LOUT, HIGH);
  }

  //if (millis() - time_RESET > RESET_TIME) {
    //wdt_enable(WDTO_1S);
    //while (1) {

    //}
  //}
  delay(50);
}