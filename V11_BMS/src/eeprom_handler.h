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
 * A struct to represent the stored eeprom data.
 *
 * The CRC covers everything before the crc32 field as a flat byte range, so
 * the layout must not contain padding: a uint8_t flag here left three
 * uninitialised bytes inside the checksummed region. They happened to be zero
 * (the struct lives in .bss and the padding round-trips through flash), but
 * the integrity check was partly covering bytes with no defined value.
 * full_discharge_seen is uint32_t so the checksummed region is exactly the
 * three fields, with no padding anywhere.
 *
 * The byte range is unchanged - on this little-endian target the old uint8_t
 * at offset 8 is the low byte of the new uint32_t at offset 8 - so existing
 * stored data still validates. If it does not, eeprom_init() falls back to
 * defaults, which is safe.
 */
struct eeprom_data
{
  int32_t  total_pack_capacity;   //micro-amp-hours
  int32_t  current_charge_level;  //micro-amp-hours
  uint32_t full_discharge_seen;   //capacity calibration flag
  uint32_t crc32;
} ;

extern int eeprom_init(void);
extern int eeprom_read(void);
extern bool eeprom_write(void);  /* true = page written, false = stored values already current */
extern int eeprom_fuses_set(void);
extern void eeprom_write_defaults(void);

#endif /* EEPROM_H_ */
