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

#ifdef ARDUINO_API_VERSION
using irq_mode = PinStatus;
#else
using irq_mode = uint32_t;
#endif

//typedef void (*voidFuncPtr)( void ) ;
typedef void (*onOffFuncPtr)( bool ) ;

typedef enum{
	OTHER_WAKEUP = 0,
	GPIO_WAKEUP = 1,
	NFC_WAKEUP = 2,
	ANALOG_COMPARATOR_WAKEUP = 3
} wakeup_reason;

#ifdef ARDUINO_ARCH_SAMD
enum adc_interrupt
{
	ADC_INT_BETWEEN,
	ADC_INT_OUTSIDE,
	ADC_INT_ABOVE_MIN,
	ADC_INT_BELOW_MAX,
};
#endif


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

		void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, irq_mode mode);

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

		#ifdef ARDUINO_ARCH_SAMD
		void attachAdcInterrupt(uint32_t pin, voidFuncPtr callback, adc_interrupt mode, uint16_t lo, uint16_t hi);
		void detachAdcInterrupt();
		#endif

	private:
		void setAlarmIn(uint32_t millis);
		#ifdef ARDUINO_ARCH_SAMD
		RTCZero rtc;
		voidFuncPtr adc_cb;
		friend void ADC_Handler();
		#endif
		#ifdef BOARD_HAS_COMPANION_CHIP
		void (*companionSleepCB)(bool);
		#endif
};

extern ArduinoLowPowerClass LowPower;

#endif
