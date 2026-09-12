/*
 * config.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

// Pin definitions
#define LED_ERR_RIGHT                       PIN_PA00
#define LED_ERR_LEFT                        PIN_PA19

//Battery charge/discharge indicators

//- seems to go to Q3 on the charger inlet side of things, push it high to accept a charge
#define ENABLE_CHARGE_PIN                   PIN_PA01 // For V15 & V11 charging confirmed

//PA28 appears to be ALERT pin from the BQ7693
#define BQ7693_ALERT_PIN                    PIN_PA28

 //NB Goes high when trigger pulled, but don't have it pulled up or down..
#define TRIGGER_PRESSED_PIN                 PIN_PA04
//Goes high when charger plugged in.
#define CHARGER_CONNECTED_PIN               PIN_PA06
#define MODE_BUTTON_PIN                     PIN_PA09
#define MODE_BUTTON_PULLUP_ENABLE_PIN       PIN_PA18
#define PRECHARGE_PIN                       PIN_PA24

// PA25 has no known function. The original firmware drove it high, and the
// port left that commented out in pins_init() - so on a board whose charge
// path needs it, charging is commanded, the AFE reports CHG_ON set and no
// fault, and still not a milliamp moves.
//
// Set to 1 to drive it high and find out. Off by default because it is a
// guess: if something external drives PA25 low, turning it into a high output
// puts the two in contention. Measure the pin first if you can - a pin that
// already sits at a defined level is being driven by something, and driving it
// from here is then not a free experiment.
#define UNKNOWN_PA25_DRIVE_HIGH             0

// Point the BQ7693 TS1/TS2 inputs at the external thermistors (TEMP_SEL=1)
// instead of the die temperature sensor.
//
// Off, because it was tried and answered: on this board both TS pins sit at
// the pull-up rail - hundreds of kilohms, i.e. open - so there are no
// thermistors on the AFE at all. The pack's sensors are on MCU ADC pins (PA07
// and PA08); see bms_adc_ch_t. With this at 0 the AFE reports its own die
// temperature, which is at least a real measurement, and nothing in the
// firmware reads either way.
#define BQ_EXT_THERMISTOR_ENABLE            0

// Nominal cell capacity, in mAh. The pack is 7S1P so this is the capacity of a
// single cell, not the sum. Set it to what is actually fitted: it bounds what
// the coulomb counter is allowed to learn, seeds the defaults, clamps the
// runtime estimate, and is reported to the cleaner as the full-charge capacity.
// Original Dyson V11 cells are 3600mAh; repacked cells are often larger.
#define PACK_MAX_CAPACITY_MAH               4500
#define CELL_LOWEST_DISCHARGE_VOLTAGE       2500  //mV - wont allow pack to discharge if any cells lower than this
#define CELL_LOWEST_CHARGE_VOLTAGE          2000    //mV - won't try to charge the pack if any cells lower than this
// Below this a cell reading is not a flat cell, it is a broken measurement -
// a severed sense wire, a blown cell tap, or an AFE channel that never
// converted. A real 18650 sitting in a pack does not reach it. Reported as
// BMS_ERR_CELL_FAIL rather than BMS_ERR_PACK_DISCHARGED, which matters
// because the discharged path zeroes the fuel gauge.
#define CELL_IMPLAUSIBLE_VOLTAGE            500     //mV
#define CELL_FULL_CHARGE_VOLTAGE            4170    //mV - fully charged cell voltage. Original BMS serial log shows cells charging to 4.17V.
#define CELL_FULL_CHARGE_RELEASE_VOLTAGE    4100    //mV - resume charging below this (70mV hysteresis)

#define CELL_OVERVOLTAGE_TRIP               4250    //BMS will trip out at this voltage - NB DO NOT set outside of 3150mV - 4700mV or it wont' work! 
#define CELL_UNDERVOLTAGE_TRIP              2450    //BMS will trip out at this voltage - NB DO NOT set outside of 1700mv - 3000mV or it wont' work!

//18650 cell temperature limits from Molicell datasheet.
#define MAX_PACK_TEMPERATURE                60       //'C - if pack temperature greater than this, no charge/discharge allowed.
#define MAX_PACK_CHARGE_TEMP                40       //'C - if pack temperature greater than this, no charge allowed.
#define MIN_PACK_CHARGE_TEMP                0        //'C - if less than this, no charge.
#define MIN_PACK_DISCHARGE_TEMP             -10      //'C - if less than this, no discharge
// Limits disabled, as V15 & V11 have 2xRTDs, now unknown where are assigned, therefore even max temp doesnt work
// Do not charge battery when hot and not supervised! This is for Debug only for V15, battery pack outputs 24V

#define IDLE_TIME                           60 * 30 // Idle time in seconds. Pack will go into SHIP/deep sleep mode if nothing happens in this duration
// Idle timeout when no cleaner is talking to us - a pack sitting on the bench,
// or one just taken off the dock. Much shorter than IDLE_TIME because there is
// nothing to wait for.
//
// Worth knowing while working on a pack: when this expires the BQ7693 goes to
// SHIP, REGOUT drops and the MCU loses power, and the ONLY way back is the
// BOOT pin - the I2C bus is dead along with the regulator, so no amount of
// reflashing or attaching a debugger will reach it. On a board whose wake path
// is damaged that is indistinguishable from a dead pack. Raise this (or set it
// to IDLE_TIME) to keep a pack awake on the bench.
#define IDLE_NO_VACUUM_TIME                 20      // seconds

// Set to 0 to stop the pack ever entering SHIP mode.
//
// SHIP is a one-way trip from the firmware's point of view: it removes REGOUT,
// the MCU loses power, the I2C bus dies with it, and the ONLY way back is the
// BQ7693 BOOT pin. Nothing this firmware does can bring the pack back, and
// neither can a programmer - SWD needs a powered core. On a pack whose wake
// path is damaged, or one being worked on where the button is held down and
// therefore never changes, that is indistinguishable from a dead pack.
//
// With this at 0 bms_handle_sleep() commits the charge level and returns to
// idle instead. The pack then never powers down and will flatten itself given
// enough time - that is the whole reason SHIP exists - so this is a bench
// setting, not a shipping one.
#define SHIP_MODE_ENABLE                    1

// How long BMS_FAULT keeps blinking its error code before giving up and going
// to SHIP mode. Without this the fault state loops forever - which for
// BMS_ERR_PACK_DISCHARGED means an already-empty pack sits there running the
// LEDs and discharging itself further. Long enough to read the blink count.
#define FAULT_DISPLAY_TIME                  60 * 5  // seconds

// How often BMS_FAULT re-evaluates the pack: the discharge check for a
// self-recovering fault off the charger, the charge check whenever the
// charger is attached (charging is the recovery for a flat pack).
#define FAULT_RETRY_MS                      5000

// Fault codes are blinked as a uniform run of pulses, with the pulse LENGTH
// carrying the class and the COUNT carrying the code within that class:
//   short pulses -> self-recovering fault, counts 1..4
//   long pulses  -> needs attention,       counts 1..6
// Keeping every pattern uniform means there is only ever one thing to count,
// and the longest run is six rather than ten.
#define FAULT_BLINK_LONG_MS                 700
#define FAULT_BLINK_SHORT_MS                150
#define FAULT_BLINK_GAP_MS                  350   // between pulses
#define FAULT_BLINK_REPEAT_MS               2500  // before the pattern repeats

// Charge-current supervision, used only to drive the indication - it never
// faults the pack, because a charger that is simply slow to come up is not an
// error and the state machine keeps retrying regardless.
//
// The LSB of the coulomb counter is BQ7693_CC_LSB_MA (8.44mA) and the filtered
// reading dithers by about one LSB with nothing flowing, so the floor has to
// sit well clear of that. A V11 charges at roughly 1A, so this is deliberately
// low: the question being answered is "anything at all?", not "as much as
// expected?".
#define CHARGE_CURRENT_MIN_MA               50
// Grace period after the charge FET is enabled before the current is believed.
// Covers the 500ms time constant of the current filter plus however long the
// charger takes to start delivering. Re-armed on every enable.
#define CHARGE_CURRENT_GRACE_MS             3000
// Half-period of the "commanded but not flowing" blink.
#define CHARGE_NO_CURRENT_LED_MS            500

// How often to log the cell voltages while charging or running the motor.
// Both states also log them on entry and on exit, so this only has to cover
// what happens in between - drift under a long charge, sag under a long run.
#define CELL_LOG_PERIOD_MS                  30000

// Once a cell reaches max charge volts, stop charging, let the pack settle,
// then retry - this many times before declaring the pack full. The pause is
// also when the balancer gets to work on the charge path, and it only starts
// after CELL_BALANCE_RELAX_MS of the pause has elapsed, so a pause shorter
// than the relax time does no balancing at all.
#define FULL_CHARGE_PAUSE_COUNT             4
#define FULL_CHARGE_PAUSE_MS                60000ul

// While sitting on the dock with the pack "full", re-check on this interval
// whether it has dropped back below CELL_FULL_CHARGE_RELEASE_VOLTAGE and needs
// a top-up. Covers balancing bleeding the top cell down, and plain
// self-discharge over days.
#define FULL_CHARGE_RECHECK_MS              (5 * 60 * 1000ul)

// How much the charge level may drift before sleep bothers to rewrite the
// emulated EEPROM. Anything below this is discarded rather than deferred -
// SHIP mode loses RAM, so the stored value is what the next boot starts from.
// At a ~20A draw 10mAh is about 1.8s of running, so a trigger pull shorter
// than that, followed by a sleep, will not be recorded. Set to 0 to store
// every change exactly.
#define EEPROM_CHARGE_TOLERANCE_UAH         (10 * 1000ul)   // 10mAh


#define SERIAL_DEBUG                        1 //Serial debug via the spare USART on the programming pins header
#define PROT_DEBUG_PRINT                    1

// Use the small built-in integer formatter (tiny_printf.c) for the debug log
// instead of the C library snprintf. newlib's snprintf costs ~2.6KB of flash
// and ~312B of RAM, and pulls in the whole heap (_malloc_r/_free_r/_realloc_r/
// _sbrk_r) purely as a side effect. Only the conversions the firmware actually
// uses are supported: %d %i %u %x %X %c %s %%, with an optional '0' flag,
// width and 'l' modifier. Set to 0 to go back to the C library.
#define TINY_PRINTF_ENABLE                  1

// Trigger behaviour: 0 = momentary (hold to run), 1 = toggle (press to run/stop).
// Trigger behaviour:
//   0  stock - the motor runs while the trigger is physically held
//   1  latch - a press toggles; holding past TRIGGER_HOLD_MS clears the latch
//   2  hybrid - a short press toggles, a long press behaves exactly like 0
//
// Mode 2 is the default because it is a superset rather than a replacement: a
// quick tap leaves the motor running, while pressing and holding runs it only
// for as long as it is held, which is the behaviour the tool shipped with and
// the one the hand already knows. Neither gesture has to be learned at the
// expense of the other, and a long press always ends with the latch clear, so
// it doubles as the way out of one set by accident.
#define TRIGGER_TOGGLE_MODE                 2
// How long a press has to last to count as "held" rather than "tapped".
#define TRIGGER_HOLD_MS                     400

//-----------------------------------------------------------------------------
// Passive cell balancing (BQ7693 internal balance FETs)
//-----------------------------------------------------------------------------
// The BQ7693003 (bq76930) internal balance drivers are limited to I_CB = 5mA per
// cell, and the bleed path runs through both external cell-input resistors
// (Rc, spec'd 500ohm min / 1k typ for this part), so the real current is
//     I = Vcell / (2*Rc + Rds_on)  ->  ~2mA @ Rc=1k, ~4mA @ Rc=500R
// times the ~70% balancing duty cycle. Budget ~2-3mA average.
// Near top of charge a cell moves ~1mV per 2.5mAh, so shifting a 10mV spread is
// an ~8 hour job. Balancing is therefore a maintenance task that runs across
// many dock sessions - it is NOT expected to converge within one charge.
#define CELL_BALANCE_ENABLE                 1

// Start bleeding a cell once it is this far above the lowest cell...
#define CELL_BALANCE_START_MV               30      //mV
// ...and stop that cell once it is back within this of the lowest (hysteresis).
#define CELL_BALANCE_STOP_MV                10      //mV
// Whole pack is considered balanced once max-min drops below this.
#define CELL_BALANCE_TARGET_SPREAD_MV       15      //mV

// Only balance near the top of charge, where cell voltage actually tracks SOC.
// Gated on the HIGHEST cell so a weak cell cannot block balancing forever.
#define CELL_BALANCE_MIN_CELL_MV            3900    //mV

// A spread larger than this is a failing cell, not an imbalance. Bleeding the
// healthy cells down to meet it would dump most of the pack as heat, so refuse.
#define CELL_BALANCE_MAX_SPREAD_MV          300     //mV

// Hard ceiling. If any cell reaches this, no more charge goes in, and that
// cell alone is bled until it is back below. Must stay comfortably below
// CELL_OVERVOLTAGE_TRIP (4250mV), which is the AFE's own trip.
#define CELL_BALANCE_OV_GUARD_MV            4200    //mV

// Balancing burns the imbalance off as heat inside the pack and runs unattended
// for hours on the dock, so it honours the charge temperature ceiling as well.
// Tracks MAX_PACK_CHARGE_TEMP by default; lower it if you want balancing to
// back off earlier than charging does.
#define CELL_BALANCE_MAX_TEMP               MAX_PACK_CHARGE_TEMP   //'C

// How many AFE-reported internal faults (DEVICE_XREADY) or external ALERT
// overrides (OVRD_ALERT) to absorb before faulting the pack.
//
// The datasheet says DEVICE_XREADY "may be set due to excessive system
// transients" and recommends the host simply clear it, and it warns that the
// ALERT pin has no internal debounce and needs protecting from noise. On a
// pack where PA28 picks up switching noise from the motor, treating the first
// event as a hard fault would strand the pack for a glitch. Each event is
// still logged, and the AFE has already dropped both FETs by itself, so
// absorbing one costs nothing in safety terms - a real internal fault
// re-latches immediately and trips this on the next check.
#define BQ_AFE_FAULT_TOLERANCE              3

// ...and forget them again after this long without one. Without a decay the
// allowance is spent once per power cycle: after the third transient every
// later one faults immediately, so an AFE that glitches once an hour would
// still strand the pack after three hours. It also stops a fault/recovery
// cycle inheriting an already-exhausted counter.
#define BQ_AFE_FAULT_DECAY_MS               (10 * 60 * 1000ul)

// How often the balancing decision is re-evaluated.
#define CELL_BALANCE_PERIOD_MS              2000

// Let cells relax this long after the charge FET opens before trusting the
// measured spread - under charge, IR drop and surface charge dominate it.
#define CELL_BALANCE_RELAX_MS               20000

// LED indication while balancing. The two LEDs alternate left<->right, which is
// the one gesture no other state uses - every other pattern in this firmware
// (boot fade, charging breathe, fault blink-codes, standby blinks) drives both
// LEDs in unison. Time each side is lit, and brightness in percent.
#define CELL_BALANCE_LED_ALT_MS             500
#define CELL_BALANCE_LED_DUTY               40

#endif /* CONFIG_H_ */
