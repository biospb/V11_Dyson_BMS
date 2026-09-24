# V11_Dyson_BMS

Aftermarket firmware for Dyson V11/V15 Battery Management Systems.

Based on https://github.com/davidmpye/V10_Dyson_BMS

By using this project, you acknowledge and agree to the following:

The author and contributors are NOT responsible for any damage, injury,
loss, or legal consequences resulting from the use or misuse of this project.

This project is provided for educational, experimental, and research purposes only.

Improper battery management can result in thermal runaway, fire, toxic fumes,
or explosion.

IF YOU DO NOT FULLY UNDERSTAND THE RISKS OF LITHIUM BATTERIES, DO NOT USE THIS PROJECT.

## Supported Hardware

### MCU

| Parameter | Value |
|-----------|-------|
| Part | Microchip ATSAMD20E15 |
| Core | ARM Cortex-M0+ |
| Flash | 32 KB (1 KB reserved for EEPROM emulation) |
| RAM | 4 KB |

### BMS Frontend

| Parameter | Value |
|-----------|-------|
| Part | Texas Instruments BQ7693003 |
| Interface | I2C (address 0x08) |
| Cells | 7S |
| Features | Cell voltage monitoring, coulomb counter, charge/discharge FET control, OV/UV/OCD/SCD protection |

### Supported Vacuum Models

- Dyson V11 (click-in and screw-type motor head)
- Dyson V15
- Dyson V12 (set `TRIGGER_TOGGLE_MODE` to 1 in `config.h` — see below)

The firmware implements the Dyson serial protocol with TLV-based communication.

### Compile-time Options (`config.h`)

| Define | Default | Purpose |
|--------|---------|---------|
| `TRIGGER_TOGGLE_MODE` | `2` | Trigger behaviour. `0` = momentary (hold to run, the V11/V15 behaviour). `1` = toggle (each press flips run/stop; hold past `TRIGGER_HOLD_MS` to force stop). `2` = hybrid, see [Trigger](#trigger). Set to `1` for the Dyson V12, whose trigger is a click-to-latch button rather than a held switch. |

## Build Toolchain

### Requirements

- `arm-none-eabi-gcc` (GCC ARM Embedded toolchain)
- `cmake` >= 3.20
- `make`
- `openocd` with J-Link support (for flashing)

### Building

```bash
cd V11_BMS

# Configure and build (Debug)
make all

# Or Release build
make all BUILD_TYPE=Release

# Flash via OpenOCD + J-Link SWD
make flash

# Unlock a firmware-protected Dyson pack (full chip erase)
make unlock

# Clean
make clean
```

Alternatively, open `V11_BMS.atsln` in Microchip/Atmel Studio 7.

### Flashing

Requires either : 

- J-Link debug probe connected via SWD. OpenOCD configuration is in `openocd_samd20.cfg`.
- Atmel ICE programmer via SWD.  OpenOCD configuration is in `openocd_samd20_ice.cfg`.

(Update the CMakeLists.txt to point to the correct configuration flie for your programmer)

## Initial Battery Calibration

After flashing the firmware, the EEPROM is initialized with default values:

| Parameter | Default Value |
|-----------|---------------|
| Total pack capacity | 120% of `PACK_MAX_CAPACITY_MAH` (config.h) |
| Current charge level | 50% of nominal capacity |

The SOC displayed on the vacuum will be inaccurate until the firmware learns the true pack capacity.

### Recommended First-Use Calibration

1. **Full discharge** — use the vacuum until the battery cuts off (undervoltage fault). This anchors the charge counter to zero and sets the `full_discharge_seen` flag.
2. **Full charge** — plug in the charger and let it charge to completion (3 pause-retry cycles). Because a full discharge was seen, the firmware directly learns the measured capacity from the complete 0-to-100% cycle.

After this single full cycle, SOC and runtime estimates will be accurate.

### Automatic Capacity Learning

On every subsequent charge completion:
- If a full discharge was previously seen, the measured charge is adopted as the new capacity (hard learning).
- Otherwise, the estimated capacity decays slowly toward the measured charge (1/8 filter per cycle).

Capacity is clamped to 120% of `PACK_MAX_CAPACITY_MAH` to reject outliers.

### Factory Reset (EEPROM Defaults)

Two ways, and they reach different situations.

**By trigger.** While the battery is actively charging, press the trigger
**20 times, with no more than 2 seconds between presses** - the timeout is
re-armed on every press, so this is a comfortable tapping pace, not 20 presses
inside a 2-second window. The left error LED blinks 10 times to confirm. This
restores the default capacity and charge level.

**By reflashing.** The stored page carries `EEPROM_MAGIC`
(`eeprom_handler.h`); `eeprom_init()` keeps stored data only if it matches and
writes defaults otherwise. Bump the low byte whenever a change makes
previously stored values wrong - the struct layout changed, or a field kept its
type but changed meaning - and the first boot after flashing resets the gauge
on its own. `BMS:EEPROM_RESET_TO_DEFAULTS` appears in the debug log when it
fires.

This is the only route that reaches a bad charge level already sitting in
flash, because **programming the MCU does not erase the emulated EEPROM**: it
lives in NVM rows reserved by the fuse (`eep` in the linker script), which the
programmer does not touch. A fault that zeroed the gauge on its way down
therefore survives a reflash, and a capacity learned from that zero would be
wrong until the next full discharge-to-charge cycle.

## Charging Indication

| LEDs | Meaning |
|------|---------|
| Both fading smoothly in and out | Charging, and current is actually flowing |
| One LED blinking on/off, twice a second | Charge is commanded, but no current is flowing |
| Both on together for 1 second, then off | Charger connected, pack already full |

The one-LED blink means the firmware has done everything it can - the safety
checks passed, `ENABLE_CHARGE_PIN` is asserted, `CHG_ON` was written to the
BQ7693 - and the coulomb counter still reads below `CHARGE_CURRENT_MIN_MA`
after `CHARGE_CURRENT_GRACE_MS`. Look outside the firmware: the charger, the
dock contacts, the charge FET and its gate drive, or a board variant whose
charge-enable is not on the pin in `config.h`.

It is deliberately **not** a fault. The pack stays in the charging state and
keeps trying, because a charger that starts late must still be allowed to
start; the indication clears by itself the moment current appears.
`BMS:CHARGING current ABSENT` / `... flowing` mark the transitions in the debug
log.

## Trigger

`TRIGGER_TOGGLE_MODE` in `config.h` selects how the trigger behaves:

| Mode | Behaviour |
|------|-----------|
| 0 | Stock - the motor runs for exactly as long as the trigger is held |
| 1 | Latch - every press toggles; a press held past `TRIGGER_HOLD_MS` clears it |
| **2** | **Default** - a short press toggles, a long press behaves exactly like mode 0 |

Mode 2 keeps both gestures rather than replacing one with the other. A tap
leaves the motor running so it need not be held through a long job; a press
and hold runs it only while held, which is what the tool shipped with. Because
a long press always ends with the latch clear, it is also the way out of a
latch set by accident - the same gesture whether or not you remember the
state.

Which kind of press it was can only be known on release, so the motor responds
to the press itself either way and only the ending differs.

A latch (modes 1 and 2) is dropped, stopping the motor, when:

- the cleaner stops talking to the pack;
- a charger appears - the cleaner is being docked, and charging starts;
- any fault occurs, so the motor does not restart by itself once the fault
  clears;
- it has held the motor on for `TRIGGER_LATCH_MAX_MS` (30 minutes). A trigger
  physically held at that moment keeps the motor running.

## Charging Temperature

A charge starts, or resumes after a fault, only while the hotter of the two
thermistors is below `MAX_PACK_CHARGE_START_TEMP` (40 C). Once running it is
stopped at `MAX_PACK_CHARGE_TEMP` (45 C). The 5 C gap stops a warm pack
toggling between charging and faulting. Charging below `MIN_PACK_CHARGE_TEMP`
(0 C) is refused.

## Storage Mode

A docked pack whose motor has not run for `STORAGE_IDLE_WAKES` standby periods
(two days each, so about two weeks) is treated as stored: it tops up only
after drifting below `CELL_STORAGE_RELEASE_VOLTAGE` and then only to
`CELL_STORAGE_CHARGE_VOLTAGE`, instead of the full-charge pair. Nothing is
discharged to get there. This also applies to a cleaner left on its wall dock,
because the cleaner's sleep request ends the session the count looks at. The
first run of the motor clears it, and the next charge goes to full.

## Fault Codes

When the BMS faults, both LEDs blink a pattern, pause, and repeat. **The
length of the flashes tells you which kind of fault it is; the number of
flashes tells you which one.** Every pattern uses one flash length only, so
there is only ever one thing to count.

### Short flashes - clears itself

The BMS retries every 5 seconds and resumes on its own once the condition goes
away. Nothing to do but wait.

| Pattern | Meaning |
|---------|---------|
| `-` | Pack too cold to charge or discharge |
| `- -` | Pack too hot to charge or discharge |
| `- - -` | Overcurrent trip |
| `- - - -` | Short circuit trip |

### Long flashes - needs attention

| Pattern | Meaning |
|---------|---------|
| `___` | Pack flat - a cell below the discharge floor |
| `___ ___` | Undervoltage trip detected by the BQ7693 |
| `___ ___ ___` | A cell is too flat to charge safely |
| `___ ___ ___ ___` | Overvoltage trip |
| `___ ___ ___ ___ ___` | BQ7693 unreachable, bad CRC, internal AFE fault, or an external protector fired |
| `___ ___ ___ ___ ___ ___` | Watchdog fired - firmware stalled |

`___` = long flash (700 ms), `-` = short flash (150 ms).

A watchdog fault only appears if the firmware stalled for between 0.5 and 1.0
seconds and then recovered. A stall past 1.0 seconds resets the MCU outright,
so a genuine hang shows up as the pack restarting, not as a blink pattern.

The blinking stops after `FAULT_DISPLAY_TIME` (5 minutes) - the LEDs run from
the cells - but the re-checking carries on. Pulling the trigger leaves the
fault state immediately and shows the code again if it is still there.

On the charger the pack re-checks every `FAULT_RETRY_MS` (5 seconds) whether
it is safe to charge, and starts charging as soon as it is. That is how a flat
pack recovers, and it is why a pack that is too hot to charge shows the "too
hot" pattern on the dock until it has cooled.

When the pack gives up and shuts down (SHIP mode - press the button or re-dock
to wake it):

| Fault | Off the charger | On the charger |
|-------|-----------------|----------------|
| Short flashes (clears itself) | `FAULT_RECOVER_GIVEUP_MS` (2 h) | `FAULT_RECOVER_GIVEUP_MS` (2 h) |
| Long flashes (needs attention) | `FAULT_DISPLAY_TIME` (5 min) | `FAULT_CHARGER_GIVEUP_MS` (30 min) |

The long flashes shut down sooner because a flat pack must not sit awake
draining itself further, and a charge that has not started in half an hour
is not going to.

## License

GNU GPL v3 or later
