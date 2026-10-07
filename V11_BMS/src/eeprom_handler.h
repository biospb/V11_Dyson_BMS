/*
 * eeprom_handler.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <ctype.h>
#include <inttypes.h>
#include <string.h> //for memcpy
 
#include "eeprom.h"
#include "nvm.h"

#include "config.h"
#include "leds.h"
#include "serial_debug.h"
#include "crc.h"

/*
 * Layout and compatibility marker for the stored page.
 *
 * eeprom_init() keeps stored data only if this value matches; anything else is
 * replaced with defaults on the spot. Bump the low byte whenever a firmware
 * change makes previously stored values wrong - the struct layout changed, or
 * a field kept its type but changed meaning - and the first boot after
 * flashing resets the gauge by itself.
 *
 * This is the reset that needs no trigger gesture, and it is the only one that
 * reaches a bogus charge level already sitting in flash: programming the MCU
 * does NOT erase the emulated EEPROM. It lives in NVM rows reserved by the
 * fuse, which the programmer does not touch, so stored data survives a
 * reflash - including a charge level a fault zeroed on the way down.
 */
#define EEPROM_MAGIC  0x424D5302ul   /* 'B','M','S', layout revision 2 */

/*
 * A struct to represent the stored eeprom data.
 *
 * The CRC covers everything before the crc32 field as a flat byte range, so
 * the layout must not contain padding: a uint8_t flag here left three
 * uninitialised bytes inside the checksummed region. They happened to be zero
 * (the struct lives in .bss and the padding round-trips through flash), but
 * the integrity check was partly covering bytes with no defined value. Every
 * field is four bytes wide, so the checksummed region is exactly the four
 * fields with no padding anywhere - keep it that way when adding one.
 *
 * magic is first and sits inside the checksummed region, so a page written by
 * firmware with a different layout is rejected on the marker even in the
 * unlikely event that its CRC happens to validate against the new one.
 */
struct eeprom_data
{
  uint32_t magic;                 //EEPROM_MAGIC - see above
  int32_t  total_pack_capacity;   //micro-amp-hours
  int32_t  current_charge_level;  //micro-amp-hours
  uint32_t full_discharge_seen;   //capacity calibration flag
  uint32_t idle_wakes;            //standby wakes with no use - see STORAGE_IDLE_WAKES
  uint32_t crc32;
} ;

extern int eeprom_init(void);
extern int eeprom_read(void);
enum eeprom_write_result {
  EEPROM_WRITE_FAILED = -1,
  EEPROM_UNCHANGED = 0,
  EEPROM_WRITTEN = 1
};
extern enum eeprom_write_result eeprom_write(void);
extern bool eeprom_healthy(void);
extern int eeprom_fuses_set(void);
extern enum eeprom_write_result eeprom_write_defaults(void);
extern bool eeprom_was_reset(void);  /* true if eeprom_init() fell back to defaults */

#endif /* EEPROM_H_ */
