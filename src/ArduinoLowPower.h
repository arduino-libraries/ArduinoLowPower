#ifndef _ARDUINO_LOW_POWER_H_
#define _ARDUINO_LOW_POWER_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#error The library is not compatible with AVR boards
#endif

#ifdef __ARDUINO_ARC__
#include "include/arc32/defines.h"
#endif

#ifdef ARDUINO_ARCH_SAMD
#include "RTCZero.h"
#endif

#if defined(ARDUINO_SAMD_TIAN)
// add here any board with companion chip which can be woken up
#define BOARD_HAS_COMPANION_CHIP
#endif

//typedef void (*voidFuncPtr)( void ) ;
typedef void (*onOffFuncPtr)( bool ) ;

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

		#ifdef __ARDUINO_ARC__
		void wakeFromSleepCallback(void);
		void wakeFromDoze(void);
		void detachInterruptWakeup(uint32_t pin);
		uint32_t arc_restore_addr;
		#endif

	private:
		#ifdef ARDUINO_ARCH_SAMD
		void setAlarmIn(uint32_t millis);
		RTCZero rtc;
		#define RTC_ALARM_WAKEUP	0xFF
		#endif

		#ifdef __ARDUINO_ARC__
		void turnOffUSB();
		void turnOnUSB();
		void switchToHybridOscillator();
		void switchToCrystalOscillator();
		void setRTCCMR(int seconds);
		uint32_t readRTC_CCVR();
		bool isSleeping = false;
		volatile bool dozing = false;
		uint32_t millisToRTCTicks(int milliseconds);
		void enableRTCInterrupt(int seconds);
		void enableAONGPIOInterrupt(int aon_gpio, int mode);
		void enableAONPTimerInterrrupt(int millis);
		static void resetAONPTimer();
		static void wakeFromRTC();
		void x86_C2Request();
		void x86_C2LPRequest();
		void (*pmCB)();
		#define RTC_ALARM_WAKEUP	0xFF
		#define RESET_BUTTON_WAKEUP	0xFE
		#endif
		void (*companionSleepCB)(bool);
};

extern ArduinoLowPowerClass LowPower;

#endif
