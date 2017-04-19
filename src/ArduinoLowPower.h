#ifndef _ARDUINO_LOW_POWER_H_
#define _ARDUINO_LOW_POWER_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#error The library is not compatible with AVR boards
#endif

#ifdef ARDUINO_ARCH_SAMD
#include "RTCZero.h"
#endif

#if defined(ARDUINO_SAMD_TIAN) || defined(ARDUINO_NRF52_PRIMO)
// add here any board with companion chip which can be woken up
#define BOARD_HAS_COMPANION_CHIP
#endif

#define RTC_ALARM_WAKEUP	0xFF

//typedef void (*voidFuncPtr)( void ) ;
typedef void (*onOffFuncPtr)( bool ) ;

typedef enum{
	OTHER_WAKEUP = 0,
	GPIO_WAKEUP = 1,
	NFC_WAKEUP = 2,
	ANALOG_COMPARATOR_WAKEUP = 3
} wakeup_reason;


class ArduinoLowPowerClass {
	public:
		void idle(void);
		void idle(uint32_t millis);
		void idle(int millis) {
			idle((uint32_t)millis);
		}

		void sleep(void);
		void sleep(uint32_t millis);
		void sleep(int millis) {
			sleep((uint32_t)millis);
		}

		void deepSleep(void);
		void deepSleep(uint32_t millis);
		void deepSleep(int millis) {
			deepSleep((uint32_t)millis);
		}

		void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode);

		#ifdef BOARD_HAS_COMPANION_CHIP
		void companionLowPowerCallback(onOffFuncPtr callback) {
			companionSleepCB = callback;
		}
		void companionSleep() {
			companionSleepCB(true);
		}
		void companionWakeup() {
			companionSleepCB(false);
		}
		#endif

		#ifdef ARDUINO_ARCH_NRF52
		void enableWakeupFrom(wakeup_reason peripheral, uint32_t pin = 0xFF, uint32_t event = 0xFF, uint32_t option = 0xFF);
		wakeup_reason wakeupReason();
		#endif

	private:
		void setAlarmIn(uint32_t millis);
		#ifdef ARDUINO_ARCH_SAMD
		RTCZero rtc;
		#endif
		#ifdef BOARD_HAS_COMPANION_CHIP
		void (*companionSleepCB)(bool);
		#endif
};

extern ArduinoLowPowerClass LowPower;

#endif
