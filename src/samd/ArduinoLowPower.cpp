#if defined(ARDUINO_ARCH_SAMD)

#include "ArduinoLowPower.h"

static void configGCLK6()
{
	// enable EIC clock
	GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	/* Errata: Make sure that the Flash does not power all the way down
     	* when in sleep mode. */

	NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
}

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
	// Disable systick interrupt:  See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	// Enable systick interrupt
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;	
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
		attachInterruptWakeup(RTC_ALARM_WAKEUP, NULL, (irq_mode)0);
	}

	uint32_t now = rtc.getEpoch();
	rtc.setAlarmEpoch(now + millis/1000);
	rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, irq_mode mode) {

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

	configGCLK6();

	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);
}

void ArduinoLowPowerClass::attachAdcInterrupt(uint32_t pin, voidFuncPtr callback, adc_interrupt mode, uint16_t lo, uint16_t hi)
{
	uint8_t winmode = 0;

	switch (mode) {
		case ADC_INT_BETWEEN:   winmode = ADC_WINCTRL_WINMODE_MODE3; break;
		case ADC_INT_OUTSIDE:   winmode = ADC_WINCTRL_WINMODE_MODE4; break;
		case ADC_INT_ABOVE_MIN: winmode = ADC_WINCTRL_WINMODE_MODE1; break;
		case ADC_INT_BELOW_MAX: winmode = ADC_WINCTRL_WINMODE_MODE2; break;
		default: return;
	}

	adc_cb = callback;

	configGCLK6();

	// Configure ADC to use GCLK6 (OSCULP32K)
	while (GCLK->STATUS.bit.SYNCBUSY) {}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
						| GCLK_CLKCTRL_GEN_GCLK6
						| GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {}

	// Set ADC prescaler as low as possible
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Configure window mode
	ADC->WINLT.reg = lo;
	ADC->WINUT.reg = hi;
	ADC->WINCTRL.reg = winmode;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable window interrupt
	ADC->INTENSET.bit.WINMON = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable ADC in standby mode
	ADC->CTRLA.bit.RUNSTDBY = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable continuous conversions
	ADC->CTRLB.bit.FREERUN = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Configure input mux
	ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable the ADC
	ADC->CTRLA.bit.ENABLE = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Start continuous conversions
	ADC->SWTRIG.bit.START = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Enable the ADC interrupt
	NVIC_EnableIRQ(ADC_IRQn);
}

void ArduinoLowPowerClass::detachAdcInterrupt()
{
	// Disable the ADC interrupt
	NVIC_DisableIRQ(ADC_IRQn);

	// Disable the ADC
	ADC->CTRLA.bit.ENABLE = 0;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable continuous conversions
	ADC->CTRLB.bit.FREERUN = 0;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable ADC in standby mode
	ADC->CTRLA.bit.RUNSTDBY = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable window interrupt
	ADC->INTENCLR.bit.WINMON = 1;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Disable window mode
	ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Restore ADC prescaler
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
	while (ADC->STATUS.bit.SYNCBUSY) {}

	// Restore ADC clock
	while (GCLK->STATUS.bit.SYNCBUSY) {}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
						| GCLK_CLKCTRL_GEN_GCLK0
						| GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY) {}

	adc_cb = nullptr;
}

void ADC_Handler()
{
	// Clear the interrupt flag
	ADC->INTFLAG.bit.WINMON = 1;
	LowPower.adc_cb();
}

ArduinoLowPowerClass LowPower;

#endif // ARDUINO_ARCH_SAMD
