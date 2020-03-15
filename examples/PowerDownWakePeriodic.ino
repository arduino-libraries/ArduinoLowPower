//This is a code to enter power down state for a fixed period of time.

#include "LowPower.h"


void setup()
{
    // No setup is required for this library
}


void loop() 
{
    // Enter power down state for 8 s with ADC and BOD module disabled.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    //You may change the power down time by changing the first parameter.
    
    // Do something here
    // Example: Read sensor, data logging, data transmission.
}
