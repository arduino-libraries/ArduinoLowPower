#ifndef _ARDUINO_LOW_POWER_H_
#define _ARDUINO_LOW_POWER_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#error The library is not compatible with AVR boards
#endif

#ifdef ARDUINO_ARCH_ARC32
#include "include/arc32/defines.h"
#endif

#ifdef ARDUINO_ARCH_SAMD
#include "RTCZero.h"
#endif

//typedef void (*voidFuncPtr)( void ) ;

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

		#ifdef ARDUINO_ARCH_ARC32
        void wakeFromSleepCallback(void);
        #endif

	private:
		#ifdef ARDUINO_ARCH_SAMD
		void setAlarmIn(uint32_t millis);
		RTCZero rtc;
		#define RTC_ALARM_WAKEUP	0xFF
		#endif

		#ifdef ARDUINO_ARCH_ARC32
        void wakeFromDoze();
        void switchToHybridOscillator();
        void switchToCrystalOscillator();
        void attachWakeInterruptRTC(void (*userCallBack)());
		void turnOffUSB();
        void turnOnUSB();
        void setRTCCMR(int milliseconds);
        uint32_t readRTC_CCVR();
        bool isSleeping = false;
        uint32_t millisToRTCTicks(int milliseconds);
        void enableRTCInterrupt();
        void x86_C2Request();
        void (*pmCB)();
        #define RTC_ALARM_WAKEUP	0xFF
		#define RESET_BUTTON_WAKEUP	0xFE
		#endif

};

extern ArduinoLowPowerClass LowPower;

#endif
