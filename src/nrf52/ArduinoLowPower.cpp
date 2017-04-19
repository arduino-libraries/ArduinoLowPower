/*
  ArduinoLowPower class for nRF52.
  
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  Copyright (c) 2017 Arduino AG. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if defined(ARDUINO_ARCH_NRF52)

#include "ArduinoLowPower.h"
#include "WInterrupts.h"
#include "nrf_rtc.h"

volatile bool event = false;
void (*functionPointer)(void);
nrf_lpcomp_input_t aPin[]={NRF_LPCOMP_INPUT_1, NRF_LPCOMP_INPUT_2, NRF_LPCOMP_INPUT_4, NRF_LPCOMP_INPUT_5, NRF_LPCOMP_INPUT_6, NRF_LPCOMP_INPUT_7};

void wakeUpGpio(){
	event = true;
	if(functionPointer)
		functionPointer();
}

void ArduinoLowPowerClass::idle() {
	// nRF52 has just two low power modes. Call sleep if idle is called.
	sleep();
}

void ArduinoLowPowerClass::idle(uint32_t millis) {
	setAlarmIn(millis);
	idle();
}

void ArduinoLowPowerClass::sleep() {
	sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
	event=false;		
	while(!event){
		sd_app_evt_wait();
	}				
}

void ArduinoLowPowerClass::sleep(uint32_t millis) {
	setAlarmIn(millis);
	sleep();
}

void ArduinoLowPowerClass::deepSleep() {
	//Enter in systemOff mode only when no EasyDMA transfer is active
	//this is achieved by disabling all peripheral that use it
	NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;								//disable UART
	NRF_SAADC ->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);	//disable ADC
	NRF_PWM0  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);		//disable all pwm instance
	NRF_PWM1  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	NRF_PWM2  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	NRF_TWIM1 ->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);	//disable TWI Master
	NRF_TWIS1 ->ENABLE = (TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos);	//disable TWI Slave
	
	//Enter in System OFF mode
	sd_power_system_off();
		
	/*Only for debugging purpose, will not be reached without connected debugger*/
    while(1);
}

void ArduinoLowPowerClass::setAlarmIn(uint32_t millis) {
	nrf_rtc_prescaler_set(NRF_RTC1, 32);
	//enable interrupt
	NVIC_SetPriority(RTC1_IRQn, 2); //high priority
	NVIC_ClearPendingIRQ(RTC1_IRQn);
	NVIC_EnableIRQ(RTC1_IRQn);
	nrf_rtc_event_clear(NRF_RTC1, NRF_RTC_EVENT_COMPARE_0);
	nrf_rtc_int_enable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
	//Tick every 1 ms 
	nrf_rtc_cc_set(NRF_RTC1, 0, millis);
	
	//start RTC
	nrf_rtc_task_trigger(NRF_RTC1, NRF_RTC_TASK_START);
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode) {
	functionPointer = callback;

	if(pin == RTC_ALARM_WAKEUP)
		return;

	pinMode(pin, INPUT_PULLUP);
	attachInterrupt(pin, wakeUpGpio, mode);
}

void ArduinoLowPowerClass::enableWakeupFrom(wakeup_reason peripheral, uint32_t pin, uint32_t event, uint32_t option){
	if(peripheral == NFC_WAKEUP){
		NRF_NFCT->TASKS_SENSE=1;
		return;
	}
	if(peripheral == ANALOG_COMPARATOR_WAKEUP){
		detect_mode mode;
		if(option == DOWN)
			mode = DOWN;
		else if(option == UP)
			mode = UP;
		else
			mode = CROSS;
		nrf_lpcomp_config_t config={(nrf_lpcomp_ref_t)event, (nrf_lpcomp_detect_t)mode};
		nrf_lpcomp_configure(&config);
		if(pin<14 && pin>19)
			return;	//no analog pin is choosen
		nrf_lpcomp_input_select(aPin[pin-14]);
		nrf_lpcomp_enable();
		nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_START);
		while(!nrf_lpcomp_event_check(NRF_LPCOMP_EVENT_READY));
		return;
	}
	if(peripheral == GPIO_WAKEUP){
		if(pin > 20)// allow wake up only from digital and analog pins
			return;
		if(event==LOW)
			nrf_gpio_cfg_sense_input(g_APinDescription[pin].ulPin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
		else
			nrf_gpio_cfg_sense_input(g_APinDescription[pin].ulPin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
	}
}

wakeup_reason ArduinoLowPowerClass::wakeupReason(){
	uint32_t guilty;
	sd_power_reset_reason_get(&guilty);
	if(guilty & 0x10000){ // GPIO
		//RESETREAS is a cumulative register. We need to clear it by writing 1 in the relative field
		sd_power_reset_reason_clr(1 << 16);
		return GPIO_WAKEUP;
	}
	if(guilty & 0x80000){ //NFC
		sd_power_reset_reason_clr(1 << 19);
		return NFC_WAKEUP;
	}
	if(guilty & 0x20000){ //COMP	
		sd_power_reset_reason_clr(1 << 17);
		return ANALOG_COMPARATOR_WAKEUP;
	}
	return OTHER_WAKEUP;
}


ArduinoLowPowerClass LowPower;

#ifdef __cplusplus
extern "C"{
#endif	

void RTC1_IRQHandler(void)
{
	event=true;
	nrf_rtc_event_clear(NRF_RTC1, NRF_RTC_EVENT_COMPARE_0);
	nrf_rtc_task_trigger(NRF_RTC1, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(NRF_RTC1, NRF_RTC_TASK_STOP);
	if(functionPointer)
		functionPointer();		
}

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_ARCH_NRF52