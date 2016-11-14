/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * code ported from https://github.com/bigdinotech/Arduino101Power/
 *
 */


#include "ArduinoLowPower.h"

#ifdef ARDUINO_ARCH_ARC32

#include "WInterrupts.h"

static void PM_InterruptHandler(void)
{
    *(uint32_t*)RTC_CCR |= 0xFFFFFFFE;
    uint32_t rtc_eoi = *(uint32_t*)RTC_EOI; //clear match interrupt
    LowPower.wakeFromSleepCallback();
}

void ArduinoLowPowerClass::idle()
{
  turnOffUSB();
  
  //switch from external crystal oscillator to internal hybrid oscilator
  switchToHybridOscillator();
  
  //Set system clock to the RTC Crystal Oscillator
  uint32_t current_val = *(uint32_t*)CCU_SYS_CLK_CTL;
  *(uint32_t*)CCU_SYS_CLK_CTL = current_val & 0xFFFFFFFE;

  //Powerdown hybrid oscillator
  current_val = *(uint32_t*)OSC0_CFG1;
  *(uint32_t*)OSC0_CFG1 = current_val | 0x00000004; 
}

void ArduinoLowPowerClass::idle(uint32_t duration)
{
    idle();
    delayTicks(millisToRTCTicks(duration));
    wakeFromDoze();
}

void ArduinoLowPowerClass::wakeFromDoze()
{
  //Powerup hybrid oscillator
  uint32_t current_val = *(uint32_t*)OSC0_CFG1;
  *(uint32_t*)OSC0_CFG1 = current_val & 0xFFFFFFFB;
   
  //Set system clock to the Hybrid Oscillator
  current_val = *(uint32_t*)CCU_SYS_CLK_CTL;
  *(uint32_t*)CCU_SYS_CLK_CTL = current_val | 0x00000001;

  //switch back to the external crystal oiscillator
  void switchToCrystalOscillator();
  
  turnOnUSB();
}

void ArduinoLowPowerClass::sleep()
{
   turnOffUSB();
   //*(uint32_t*)SLP_CFG &= 0xFFFFFEFF;
   
   x86_C2Request();
   isSleeping = true;
   //*(uint32_t*)CCU_LP_CLK_CTL = (*(uint32_t*)CCU_LP_CLK_CTL) | 0x00000002;
   //uint32_t c2 = *(uint32_t*)P_LVL2;
   *(uint32_t*)PM1C |= 0x00002000;
}

void ArduinoLowPowerClass::sleep(uint32_t duration)
{
    setRTCCMR(duration);
    enableRTCInterrupt();
    sleep();
}

void ArduinoLowPowerClass::deepSleep()
{
   turnOffUSB();
   
   x86_C2Request();
   isSleeping = true;
   *(uint32_t*)CCU_LP_CLK_CTL  |= 0x00000001;
   //uint32_t c2 = *(uint32_t*)P_LVL2;
   *(uint32_t*)PM1C |= 0x00002000;
}

void ArduinoLowPowerClass::deepSleep(uint32_t duration)
{
    setRTCCMR(duration);
    enableRTCInterrupt();
    deepSleep();
}

void ArduinoLowPowerClass::switchToHybridOscillator()
{
    //read trim value from OTP
    uint32_t trimMask = *(uint16_t*)OSCTRIM_ADDR << 20;
    *(uint32_t*)OSC0_CFG1 = 0x00000002 | trimMask;  //switch to internal oscillator using trim value from OTP
}

void ArduinoLowPowerClass::switchToCrystalOscillator()
{
    *(uint32_t*)OSC0_CFG1 = 0x00070009;
}

inline void ArduinoLowPowerClass::wakeFromSleepCallback(void)
{
    if(pmCB != NULL)
        pmCB();
}

void ArduinoLowPowerClass::attachWakeInterruptRTC(void (*userCallBack)())
{
    pmCB = userCallBack;
}

//Privates

void ArduinoLowPowerClass::turnOffUSB()
{
    *(uint32_t*)USB_PHY_CFG0 |= 0x00000001; 
}

void ArduinoLowPowerClass::turnOnUSB()
{
    *(uint32_t*)USB_PHY_CFG0 &= 0xFFFFFFFE;
}

void ArduinoLowPowerClass::setRTCCMR(int milliseconds)
{
    *(uint32_t*)RTC_CMR = readRTC_CCVR() + millisToRTCTicks(milliseconds);
    //*(uint32_t*)RTC_CMR = readRTC_CCVR() + milliseconds;
}

uint32_t ArduinoLowPowerClass::readRTC_CCVR()
{
    return *(uint32_t*)RTC_CCVR;
}

uint32_t ArduinoLowPowerClass::millisToRTCTicks(int milliseconds)
{
    return (uint32_t)((double)milliseconds*32.768);
}

void ArduinoLowPowerClass::enableRTCInterrupt()
{
    *(uint32_t*)RTC_MASK_INT &= 0xFFFFFFFF;
    *(uint32_t*)RTC_CCR |= 0x00000001;
    *(uint32_t*)RTC_CCR &= 0xFFFFFFFD;
    interrupt_disable(IRQ_RTC_INTR);
    interrupt_connect(IRQ_RTC_INTR , &PM_InterruptHandler);
    interrupt_enable(IRQ_RTC_INTR);
}

void ArduinoLowPowerClass::x86_C2Request()
{
    switchToHybridOscillator();
    //set the CCU_C2_LP_EN bit
    *(uint32_t*)CCU_LP_CLK_CTL = (*(uint32_t*)CCU_LP_CLK_CTL) | 0x00000002;
    //request for the x86 core go into C2 sleep
    volatile uint32_t c2 = *(volatile uint32_t*)P_LVL2;
 
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode) {

	extern uint32_t sizeof_g_APinDescription;

	if (pin > sizeof_g_APinDescription) {
		// check for external wakeup sources
		// RTC library should call this API to enable the alarm subsystem
		switch (pin) {
			case RTC_ALARM_WAKEUP:
				attachWakeInterruptRTC(callback);
				break;
			case RESET_BUTTON_WAKEUP:
				gpio_cfg_data_t pin_cfg = {
					.gpio_type = GPIO_INTERRUPT,
					.int_type = LEVEL,
					.int_polarity = ACTIVE_LOW,
					.int_debounce = DEBOUNCE_OFF,
					.int_ls_sync = LS_SYNC_OFF,
					.gpio_cb = callback
				};
				soc_gpio_deconfig(SOC_GPIO_AON, 0);
				/* set RESET GPIO button to be an input */
				soc_gpio_set_config(SOC_GPIO_AON, 0, &pin_cfg);
				SET_PIN_PULLUP(32*3 + 8, 1);
				break;
		}
		return;
	}

	

	//pinMode(pin, INPUT_PULLUP);
	//attachInterrupt(pin, callback, mode);
}

ArduinoLowPowerClass LowPower;

#endif