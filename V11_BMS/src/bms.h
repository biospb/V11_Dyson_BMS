/*
 * bms.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 

#ifndef BMS_H_
#define BMS_H_

/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "board.h"

#include "bq7693.h"
#include "serial.h"
#include "leds.h"
#include "eeprom_handler.h"
#include "serial_debug.h"
#include "config.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/
enum BMS_STATE
{
  BMS_INIT,
  BMS_IDLE,
  BMS_CHARGER_CONNECTED,
  BMS_CHARGING,
  BMS_CHARGER_CONNECTED_NOT_CHARGING,
  BMS_CHARGER_UNPLUGGED,
  BMS_VACUUM_RUNNING,
  BMS_FAULT, //error code should be logged to explain why!
  BMS_SLEEP,
};

/*
 * Fault codes. Purely internal - nothing here goes out over the Dyson
 * protocol - so the numbering is ours to choose, and two properties are
 * deliberate:
 *
 *   1. The self-recovering faults are 1..BMS_ERR_SHORTCIRCUIT. That split is
 *      what bms_blink_error_code() signals: those are blinked as short pulses
 *      counting 1..4, everything above as long pulses counting 1..8. Pulse
 *      length gives the class, pulse count gives the code within it, and no
 *      pattern ever mixes the two.
 *
 *   2. The order is still ascending by severity, because bms_set_error() only
 *      ever raises. When several conditions are true at once the highest wins,
 *      and BMS_ERR_I2C_FAIL sits near the top because a dead bus invalidates
 *      every other reading.
 */
enum BMS_ERROR_CODE
{
  BMS_ERR_NONE,            //  0  All good!

  /* --- self-recovering: retried every 5s, shown as short pulses only --- */
  BMS_ERR_PACK_UNDERTEMP,  //  1  Below MIN_PACK_DISCHARGE_TEMP / MIN_PACK_CHARGE_TEMP
  BMS_ERR_PACK_OVERTEMP,   //  2  Above MAX_PACK_TEMPERATURE / MAX_PACK_CHARGE_TEMP
  BMS_ERR_OVERCURRENT,     //  3  BMS IC overcurrent trip
  BMS_ERR_SHORTCIRCUIT,    //  4  BMS IC short circuit trip

  /* --- needs attention: at least one long pulse --- */
  BMS_ERR_PACK_DISCHARGED, //  5  A cell below CELL_LOWEST_DISCHARGE_VOLTAGE
  BMS_ERR_UNDERVOLTAGE,    //  6  BMS IC undervoltage trip - flat pack, detected by BQ
  BMS_ERR_CELL_FAIL,       //  7  A cell below CELL_LOWEST_CHARGE_VOLTAGE - too flat to charge
  BMS_ERR_OVERVOLTAGE,     //  8  BMS IC overvoltage trip
  BMS_ERR_I2C_FAIL,        //  9  BQ7693 unreachable/corrupt, or the AFE itself
                           //     reported an internal fault (DEVICE_XREADY) or
                           //     an external protector pulled ALERT (OVRD_ALERT)
  BMS_ERR_WDT,            // 10  Watchdog early warning fired - main loop stalled!
  BMS_ERR_SENSOR_FAIL,    // 11  ADC conversion or initialization failed
  BMS_ERR_EEPROM_FAIL,    // 12  Persistence failed
};

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
 extern void bms_init(void);
 extern void bms_mainloop(void);
 extern void bms_force_fault(enum BMS_ERROR_CODE code);
 extern uint16_t bms_get_soc_x100(void);
 extern uint32_t bms_get_runtime_seconds(void);
 extern uint32_t bms_get_full_charge_capacity_001mah(void);
 extern void bms_wakeup_interrupt_callback(void);
 extern void bms_interrupt_callback(void) ;
 extern void bms_interrupt_process(void);


/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* BMS_H_ */
