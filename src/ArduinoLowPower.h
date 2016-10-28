#ifndef _ARDUINO_LOW_POWER_H_
#define _ARDUINO_LOW_POWER_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_SAMD
#include "RTCZero.h"
#endif

//typedef void (*voidFuncPtr)( void ) ;

class ArduinoLowPowerClass {
	public:
		void idle(void);
		void idle(uint32_t millis);

		void sleep(void);
		void sleep(uint32_t millis);

		void deepSleep(void);
		void deepSleep(uint32_t millis);

		void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode);

	private:
		#ifdef ARDUINO_ARCH_SAMD
		void setAlarmIn(uint32_t millis);
		RTCZero rtc;
		#define RTC_ALARM_WAKEUP	0xFF
		#endif
};

extern ArduinoLowPowerClass LowPower;

#endif
