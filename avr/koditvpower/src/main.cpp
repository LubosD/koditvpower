#define DEBUG_OUTPUT
#include <Arduino.h>

#ifdef DEBUG_OUTPUT
#include <SoftwareSerial.h>
#endif

#ifdef ATTINY85
// ATtiny85
const uint8_t PIN_OUTPUT_POWER = 2; // PB2 / HW pin 7
const uint8_t PIN_INPUT_POWEROFF = 3; // PB3 / HW pin 2
const uint8_t PIN_OUTPUT_SHUTDOWN = 4; // PB4 / HW pin 3
const uint8_t PIN_INPUT_TV_POWER = 1; // PB1 / HW pin 6
#endif

#ifdef ATTINY412
// ATtiny412
const uint8_t PIN_OUTPUT_POWER = 4; // PA3 / HW pin 7
const uint8_t PIN_INPUT_POWEROFF = 0; // PA6 / HW pin 2
const uint8_t PIN_OUTPUT_SHUTDOWN = 1; // PA7 / HW pin 3
const uint8_t PIN_INPUT_TV_POWER = 3; // PA2 / HW pin 5
#endif

enum class State
{
  Off,
  On,
  ShuttingDown,
};

State gState;

#ifdef DEBUG_OUTPUT
SoftwareSerial mySerial(PIN_INPUT_POWEROFF, 2);
//#define mySerial Serial
#endif

void setup()
{
  pinMode(PIN_OUTPUT_POWER, OUTPUT);
  digitalWrite(PIN_OUTPUT_POWER, LOW);

  pinMode(PIN_OUTPUT_SHUTDOWN, OUTPUT);
  digitalWrite(PIN_OUTPUT_SHUTDOWN, HIGH);

  pinMode(PIN_INPUT_TV_POWER, INPUT_PULLUP);
  pinMode(PIN_INPUT_POWEROFF, INPUT);

  gState = State::Off;

#ifdef DEBUG_OUTPUT
  // debug
  pinMode(2, OUTPUT);
  //mySerial.swap(1);
  mySerial.begin(9600);
#endif
}

void loop()
{
#ifdef DEBUG_OUTPUT
  int pow = digitalRead(PIN_INPUT_TV_POWER);
  mySerial.printf("pow %d\n", pow);
#endif

  const bool tvPowerOn = pow == LOW;

  if (digitalRead(PIN_INPUT_POWEROFF) == HIGH)
  {
#ifdef DEBUG_OUTPUT
    mySerial.println("poweroff is high");
#endif

    if (gState != State::Off)
    {
      // If the RPi signals poweroff, turn the power off
      digitalWrite(PIN_OUTPUT_POWER, LOW);
      gState = State::Off;
    }
  }
  else if (tvPowerOn && gState == State::Off)
  {
#ifdef DEBUG_OUTPUT
    mySerial.println("tvPowerOn && gState == State::Off");
#endif

    // If we're off and the TV is on, turn on the RPi
    digitalWrite(PIN_OUTPUT_POWER, HIGH);
    gState = State::On;
  }
  else if (!tvPowerOn && gState == State::On)
  {
#ifdef DEBUG_OUTPUT
    mySerial.println("!tvPowerOn && gState == State::On");
#endif

    // If the TV turns off, signal shutdown to the RPi
    digitalWrite(PIN_OUTPUT_SHUTDOWN, LOW);
    delay(500);
    digitalWrite(PIN_OUTPUT_SHUTDOWN, HIGH);

    gState = State::ShuttingDown;
  }
  else
  {
#ifdef DEBUG_OUTPUT
    mySerial.println(tvPowerOn ? "nothing:power is on" : "nothing:power is off");
#endif
  }

  delay(1000);
}
