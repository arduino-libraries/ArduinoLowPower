#ifndef _ARDUINO_LOW_POWER_H_
#define _ARDUINO_LOW_POWER_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#error The library is not compatible with AVR boards
#endif

#ifdef ARDUINO_ARCH_ARC32
#include "include/arc32/power_states.h"
#include "include/arc32/ss_power_states.h"
#include "include/arc32/qm_sensor_regs.h"
#include "include/arc32/qm_soc_regs.h"
#endif

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

		#ifdef ARDUINO_ARCH_ARC32
		void ss_power_soc_lpss_enable();
		void ss_power_soc_lpss_disable();
		void ss_power_cpu_ss1(const ss_power_cpu_ss1_mode_t mode);
		void ss_power_cpu_ss2();
		void ss_power_soc_sleep_restore();
		void ss_power_soc_deep_sleep_restore();
		void ss_power_sleep_wait();
		void power_soc_set_ss_restore_flag();
		void power_soc_sleep();
		void power_soc_deep_sleep();
		void setAlarmIn(uint32_t millis);
		#define RTC_ALARM_WAKEUP	0xFF
		#define RESET_BUTTON_WAKEUP	0xFE
		#endif

};

extern ArduinoLowPowerClass LowPower;

#endif
