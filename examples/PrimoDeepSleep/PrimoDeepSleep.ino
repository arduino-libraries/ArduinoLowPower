/*
  PrimoDeepSleep.ino
  
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  This example for the Arduino Primo board shows how to use
  low power library to enter in power off mode and save power.
  This mode ensure the deepest power saving mode. If you need
  a faster response from the board use standby function instead.
  
  Please note that once exited from the deepest sleep mode the
  board will reset (so setup will be run again).
  
  The functions enableWakeupFrom set the peripheral that will wake up
  the board. By calling it more than once you can choose more than
  a wakeup source.
  The board will be reset when it wakes up from power off.
  You can use wakeUpCause() function to find out what signals woke up
  the board if you use more than one wakeUpBy.. function.
  
  This example code is in the public domain.
*/

#include "ArduinoLowPower.h"


// Pin used to wakeup the board
const int digitalPin = 10;

// Pin used in Compatarot module to wake up the board
const int analogPin = A0;


void StmEspPM(bool sleep){
  // enable USER1_BUTTON to turn STM32 off and on when pressed.
  // note that when STM32 is off you cannot load any new sketch.
  pinMode(USER1_BUTTON, STM32_IT);

  // turn ESP8266 off or on
  digitalWrite(GPIO_ESP_PW, sleep ? LOW: HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  //look for what peripheral woke up the board
  //reason is 0 at the first execution
  wakeup_reason reason=LowPower.wakeupReason();
  if(reason==GPIO_WAKEUP) //GPIO caused the wake up
    doMyStuff();
  else
    if(reason==NFC_WAKEUP) //NFC caused the wake up
      doMyStuffWithNFC();
  else
    if(reason==ANALOG_COMPARATOR_WAKEUP) //Comparator caused the wake up
      doOtherStuff();

  Serial.println("Hi all, I return to sleep");

  LowPower.companionLowPowerCallback(StmEspPM);
  // Send sleep command to ESP and enable USER1_BUTTON to turn STM off
  LowPower.companionSleep();

  //set digital pin 10 to wake up the board when LOW level is detected
  LowPower.enableWakeupFrom(GPIO_WAKEUP, digitalPin, LOW);
  //let the board be woken up by any NFC field
  LowPower.enableWakeupFrom(NFC_WAKEUP);
  //wake up the board when the voltage on pin A0 goes below the voltage on pin AREF
  LowPower.enableWakeupFrom(ANALOG_COMPARATOR_WAKEUP, analogPin, AREF, UP);
  //go in low power mode. Note that the board will reset once it is woken up
  LowPower.deepSleep();
}


void loop() {}

void doMyStuff(){
  //insert your code here
}

void doMyStuffWithNFC(){
  //insert your code here
}

void doOtherStuff(){
  //insert your code here
}
