# Ardunio Low Power

## Methods

### `LowPower.idle()`

#### Description

Puts the MCU in IDLE mode. This mode allows power optimization with the fastest wake-up time. The CPU is stopped. To further reduce power consumption, the user can manually disable the clocking of modules and clock sources.


#### Syntax

```
LowPower.idle();
LowPower.idle(milliseconds);
```

#### Parameters

milliseconds: the number of milliseconds to put the board in idle mode. If void the idle mode is used till a wakeup event.

### `LowPower.sleep()`

#### Description

Puts the MCU in sleep mode. The sleep mode allows power optimization with a slower wakeup time. Only the chosen peripherals are on.


#### Syntax

```
LowPower.sleep();
LowPower.sleep(milliseconds);
```

#### Parameters

milliseconds: the number of milliseconds to put the board in sleep mode. If void the sleep mode is used till a wakeup event.

### `LowPower.deepSleep()`

#### Description

Puts the MCU in deep sleep mode. The deep sleep mode allows power optimization with the slowest wake-up time. All but the RTC peripherals are stopped. The CPU can be wakeup only using RTC or wakeup on interrupt capable pins.


#### Syntax

```
LowPower.deepSleep();
LowPower.deepSleep(milliseconds);
```

#### Parameters

milliseconds: the number of milliseconds to put the board in deep sleep mode. If void the deep sleep mode is used till a wakeup event.

### `LowPower.attachInterruptWakeup()`

#### Description

Indicates the function to call and the conditions for a wakeup event.


#### Syntax

```
LowPower.attachInterruptWakeup(pin, callback, mode);
```

#### Parameters

pin: the pin used as external wakeup

callback: the function to call on wakeup

mode: the transitions to sense on the indicated pin. Can be one between:

- FALLING
- RISING
- CHANGE

### `LowPower.CompanionLowPowerCallback()`

#### Description

Indicates the function that the on-boad co-processor (Tian only) has to call just before going to sleep.


#### Syntax

LowPower.CompanionLowPowerCallback(callback);

#### Parameters

callback: the function to call before going to sleep

### `LowPower.companionSleep()`

#### Description

Puts the on-board co-processor (Tian only) in sleep mode


#### Syntax

LowPower.companionSleep();

#### Parameters

None

### `LowPower.companionWakeup()`

#### Description

Forces the on board co-processor (Tian only) wakeup from sleep mode.


#### Syntax

```
LowPower.companionWakeup();
```

#### Parameters

None