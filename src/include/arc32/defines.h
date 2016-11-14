#define OSC0_CFG1       0xB0800008
#define CCU_SS_PERIPH_CLK_GATE_CTL  0xB0800028
#define CCU_SYS_CLK_CTL 0xB0800038
#define CCU_LP_CLK_CTL  0xB080002C
#define P_LVL2          0xB0800504
#define PM1C            0xB0800518
#define SLP_CFG         0xB0800550

#define USB_PLL_CFG0    0xB0800014
#define USB_PHY_CFG0    0xB0800800

#define RTC_CCVR        0xB0000400
#define RTC_CMR         0xB0000404
#define RTC_CCR         0xB000040C
#define RTC_EOI         0xB0000418
#define RTC_MASK_INT    0xB0800478

#define OSCTRIM_ADDR    0xffffe1f8

#define SLEEP_HOST_C0       0
#define SLEEP_HOST_C1       1
#define SLEEP_HOST_C2       2
#define SLEEP_HOST_C2_LP    3
#define SLEEP_SS_SS0        4
#define SLEEP_SS_SS1        5
#define SLEEP_SS_SS2        6
#define SLEEP_LPSS          7

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