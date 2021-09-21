# Arduino Low Power Library

This library allows you to use the low power features of the SAMD21 MCU, which is used for all [MKR family boards](https://store.arduino.cc/collections/mkr-family) and the [Nano 33 IoT board](https://store.arduino.cc/products/arduino-nano-33-iot).

In these pages, the term companion chip is used. This term refers to a board co-processor like the MIPS processor on the [Arduino Tian](https://docs.arduino.cc/retired/boards/arduino-tian).

To use this library:

```
#include "<ArduinoLowPower.h>"
```

Examples:

- [ExternalWakeup](https://github.com/arduino-libraries/ArduinoLowPower/blob/master/examples/ExternalWakeup/ExternalWakeup.ino) : Demonstrates how to wake your board from an external source like a button.
- [TianStandby](https://github.com/arduino-libraries/ArduinoLowPower/blob/master/examples/TianStandby/TianStandby.ino) : Demonstrates how to put a Tian in standby
- [TimedWakeup](https://github.com/arduino-libraries/ArduinoLowPower/blob/master/examples/TimedWakeup/TimedWakeup.ino) : Demonstrates how to put in sleep your board for a certain amount of time