/*
  AdcWakeup

  This sketch demonstrates the usage of the ADC to wakeup a chip in sleep mode.
  Sleep modes allow a significant drop in the power usage of a board while it does nothing waiting for an event to happen. Battery powered application can take advantage of these modes to enhance battery life significantly.

  In this sketch, changing the voltage on pin A0 will wake up the board. You can test this by connecting a potentiometer between VCC, A0, and GND.
  Please note that, if the processor is sleeping, a new sketch can't be uploaded. To overcome this, manually reset the board (usually with a single or double tap to the RESET button)

  This example code is in the public domain.
*/

#include "ArduinoLowPower.h"

// Blink sequence number
// Declare it volatile since it's incremented inside an interrupt
volatile int repetitions = 1;

// Pin used to trigger a wakeup
const int pin = A0;
// How sensitive to be to changes in voltage
const int margin = 10;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin, INPUT);
}

void loop() {
  for (int i = 0; i < repetitions; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  // Read the voltage at the ADC pin
  int value = analogRead(pin);

  // Define a window around that value
  uint16_t lo = max(value - margin, 0);
  uint16_t hi = min(value + margin, UINT16_MAX); 

  // Attach an ADC interrupt on pin A0, calling repetitionsIncrease when the voltage is outside the given range.
  // This should be called immediately before LowPower.sleep() because it reconfigures the ADC internally.
  LowPower.attachAdcInterrupt(pin, repetitionsIncrease, ADC_INT_OUTSIDE, lo, hi);

  // Triggers an infinite sleep (the device will be woken up only by the registered wakeup sources)
  // The power consumption of the chip will drop consistently
  LowPower.sleep();

  // Detach the ADC interrupt. This should be called immediately after LowPower.sleep() because it restores the ADC configuration after waking up.
  LowPower.detachAdcInterrupt();
}

void repetitionsIncrease() {
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
  repetitions ++;
}