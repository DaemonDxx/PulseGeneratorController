#include <EEPROM.h>

#include "arduino-timer.h"

#define LED_PIN 13
#define BUZZER_PIN 5
#define BUTTON_PIN 2
#define SIGNAL_PIN A0
#define OUT_PIN 6
#define TIMEOUT_SWITCH_STATE 1000
#define MIN_VOLTAGE 200
#define ERROR 0.8

#define TIME_ON_LED 20
#define FREQUENCY_BUZZER 1822

bool state = false;
unsigned int analogIn = 0;
unsigned long afterTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
Timer<10> timer;
bool up = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);

  waitStateSwitch();
  attachInterrupt(0, pushButton, CHANGE);
}

void pushButton() {
  delay(1);
   if (digitalRead(BUTTON_PIN)) {
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, FREQUENCY_BUZZER);
      if (state) {
      catchSignal();
      } else {
        digitalWrite(OUT_PIN, HIGH);
      }
    } else {
      digitalWrite(LED_PIN, LOW); 
      noTone(BUZZER_PIN);
      if (!state) {
        digitalWrite(OUT_PIN, LOW);
      }
    }
}

void loop() {
  analogIn = analogRead(SIGNAL_PIN);
  if (analogIn > MIN_VOLTAGE && !up) {
    up = true;
    if (state) {
      digitalWrite(LED_PIN, HIGH);
      timer.in(100, offLed);
      catchSignal();
    } else {
      transferSignal();
    }
  } else if (analogIn < (MIN_VOLTAGE-50)) {
    up = false;
  }
  timer.tick();
}

void transferSignal() {
  digitalWrite(OUT_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(OUT_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void catchSignal() {
  currentTime = millis();
  if (afterTime == 0) {
    afterTime = currentTime;
    digitalWrite(OUT_PIN, HIGH);
    timer.in(100, offSignal);
  } else {
    deltaTime = currentTime - afterTime;
    afterTime = 0;
    unsigned long delayError = calculateTimeDelay(calculatePower(deltaTime));
    Serial.println(deltaTime);
    timer.in(delayError, setSignal);
  }
  delay(1);
  
}

float calculatePower(unsigned long deltaTime) {
  return (3600000ul/deltaTime)/600;
}

float calculateDelta(float power) {
  return 3600/(power*600);
}

unsigned long calculateTimeDelay(float power) {
  float errorPower = power*ERROR;
  float deltaError = calculateDelta(errorPower);
  return deltaError*1000-deltaTime;
}


bool offLed(void *argument) {
  digitalWrite(LED_PIN, LOW);
  return false;
}

bool setSignal(void *argument) {
  digitalWrite(OUT_PIN, HIGH);
  timer.in(33, offSignal);
  return false;
}

bool offSignal(void *argument) {
  digitalWrite(OUT_PIN, LOW);
  return false;
}

void waitStateSwitch() {
  state = readState();
  if (digitalRead(BUTTON_PIN)) {
    delay(1500);
    if (digitalRead(BUTTON_PIN)) {
       Serial.println("Switch!");
       state = !state;
       saveState(state);
       tone(BUZZER_PIN, FREQUENCY_BUZZER);
       delay(50);
       noTone(BUZZER_PIN);
    }
  }
  Serial.println(state);
}

void saveState(bool state) {
  EEPROM.update(0, state);
}

bool readState() {
  return EEPROM.read(0);
}
