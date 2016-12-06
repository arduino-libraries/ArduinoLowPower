#define OSC0_STAT       0xB0800004
#define OSC0_CFG1       0xB0800008
#define CCU_SS_PERIPH_CLK_GATE_CTL  0xB0800028
#define CCU_SYS_CLK_CTL 0xB0800038
#define CCU_LP_CLK_CTL  0xB080002C
#define P_LVL2          0xB0800504
#define PM1C            0xB0800518
#define SLP_CFG         0xB0800550
#define SS_STS          0xB0800604

#define AONC_CNT        0xB0800700
#define AONC_CFG        0xB0800704
#define AONPT_CNT       0xB0800708
#define AONPT_STAT      0xB080070C
#define AONPT_CTRL      0xB0800710
#define AONPT_CFG       0xB0800714

#define USB_PLL_CFG0    0xB0800014
#define USB_PHY_CFG0    0xB0800800

#define RTC_CCVR        (volatile int*)0xB0000400 // Current Counter Value Register
#define RTC_CMR         0xB0000404
#define RTC_CCR         0xB000040C
#define RTC_EOI         0xB0000418
#define RTC_MASK_INT    0xB0800478

#define AON_TIMER_MASK_INT      0xB08004C8
#define AON_GPIO_MASK_INT       0xB08004D4

#define AON_GPIO_SWPORTA_DR         0xB0800B00
#define AON_GPIO_SWPORTA_DDR        0xB0800B04
#define AON_GPIO_SWPORTA_CTL        0xB0800B08
#define AON_GPIO_INTEN              0xB0800B30
#define AON_GPIO_INTMASK            0xB0800B34
#define AON_GPIO_INTTYPE_LEVEL      0xB0800B38
#define AON_GPIO_INT_POL            0xB0800B3C
#define AON_GPIO_DEBOUNCE           0xB0888B48
#define AON_GPIO_PORTA_EOI           0xB0800B4C

#define OSCTRIM_ADDR    0xffffe1f8

#define QM_SS_SLEEP_MODE_CORE_OFF (0x0)
#define QM_SS_SLEEP_MODE_CORE_OFF_TIMER_OFF (0x20)
#define QM_SS_SLEEP_MODE_CORE_TIMERS_RTC_OFF (0x60)

#define SLEEP_HOST_C0       0
#define SLEEP_HOST_C1       1
#define SLEEP_HOST_C2       2
#define SLEEP_HOST_C2_LP    3
#define SLEEP_SS_SS0        4
#define SLEEP_SS_SS1        5
#define SLEEP_SS_SS2        6
#define SLEEP_LPSS          7

enum wakeSource{
    AON_GPIO0 = 100,
    AON_GPIO1 = 101,
    AON_GPIO2 = 102,
    AON_GPIO3 = 103,
    AON_GPIO4 = 104,
    AON_GPIO5 = 105,
    INT_BMI160 = 104,
    INT_BLE = 105,
    INT_RTC = 106,
    AON_TIMER = 107,
    INT_COMP = 108,
};

/* Pin Muxing */
#define PULLUP_BASE    QRK_PMUX_PULLUP_0
/* Read current pull-up reg, Zero pin bit, OR new mode into these bits, write reg - thereby preserving other pin mode settings */
#define SET_PULLUP_REG( mux_reg, enable, pin ) MMIO_REG_VAL(mux_reg) = ( MMIO_REG_VAL(mux_reg) & ~( 1 << (pin) ) ) | ( (enable) << (pin) )
/* Calculate mux register address from pin number and calculate pin number within that register - call SET_MUX_REG */
#define SET_PIN_PULLUP( pin_no, enable) SET_PULLUP_REG( ((((pin_no)/32)*4 )+ PULLUP_BASE), (enable), (pin_no) % 32)


#include <Arduino.h>
#include <stdint.h>
#include <interrupt.h>
#include <board.h>
#include "qm_sensor_regs.h"
#include "ss_power_states.h"
