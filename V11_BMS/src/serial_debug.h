/*
 * serial_debug.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 
#ifndef SERIAL_DEBUG_H_
#define SERIAL_DEBUG_H_

/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "tiny_printf.h"
#include "config.h"
#include "bq7693.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
/*
 * Must hold the longest debug line in the firmware. The BMS:VACUUM_RUNNING and
 * BMS:CHARGING status lines reach 83 characters at full-scale values
 * (276553mA, 5400000uAh twice, -40C, 29400mV) and were being silently
 * truncated. GCC used to warn about this via -Wformat-truncation, but that
 * analysis only applies to the builtin snprintf, so it went quiet when the
 * debug path moved to tiny_printf.
 */
#define DEBUG_MSG_BUFFER_SIZE  128

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
extern void serial_debug_init(void);
extern void serial_debug_send_message(const char *msg);
extern void serial_debug_process(void);
extern void serial_debug_send_cell_voltages(void);
extern void serial_debug_send_pack_capacity(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* SERIAL_DEBUG_H_ */
