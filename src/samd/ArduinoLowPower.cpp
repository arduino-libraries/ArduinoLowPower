#if defined(ARDUINO_ARCH_SAMD)

#include "ArduinoLowPower.h"
#include "WInterrupts.h"

void ArduinoLowPowerClass::idle() {
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	PM->SLEEP.reg = 2;
	__DSB();
	__WFI();
}

void ArduinoLowPowerClass::idle(uint32_t millis) {
	setAlarmIn(millis);
	idle();
}

void ArduinoLowPowerClass::sleep() {
	bool restoreUSBDevice = false;
	if (SERIAL_PORT_USBVIRTUAL) {
		USBDevice.standby();
	} else {
		USBDevice.detach();
		restoreUSBDevice = true;
	}
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	if (restoreUSBDevice) {
		USBDevice.attach();
	}
}

void ArduinoLowPowerClass::sleep(uint32_t millis) {
	setAlarmIn(millis);
	sleep();
}

void ArduinoLowPowerClass::deepSleep() {
	sleep();
}

void ArduinoLowPowerClass::deepSleep(uint32_t millis) {
	sleep(millis);
}

void ArduinoLowPowerClass::setAlarmIn(uint32_t millis) {

	if (!rtc.isConfigured()) {
		attachInterruptWakeup(RTC_ALARM_WAKEUP, NULL, 0);
	}

	uint32_t now = rtc.getEpoch();
	rtc.setAlarmEpoch(now + millis/1000);
	rtc.enableAlarm(rtc.MATCH_HHMMSS);
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode) {

	if (pin > PINS_COUNT) {
		// check for external wakeup sources
		// RTC library should call this API to enable the alarm subsystem
		switch (pin) {
			case RTC_ALARM_WAKEUP:
				rtc.begin(false);
				rtc.attachInterrupt(callback);
			/*case UART_WAKEUP:*/
		}
		return;
	}

	EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
	if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
    		return;

	//pinMode(pin, INPUT_PULLUP);
	attachInterrupt(pin, callback, mode);

	// enable EIC clock
	GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);

	/* Errata: Make sure that the Flash does not power all the way down
     	* when in sleep mode. */

	NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
}

ArduinoLowPowerClass LowPower;

#endif // ARDUINO_ARCH_SAMD
