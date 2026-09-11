/*
 * bms_adc.h
 *
 * Created: 21-Jan-26 10:10:14
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */ 


#ifndef BMS_ADC_H_
#define BMS_ADC_H_
/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/
/*
 * Three ADC pins carry something; only TC1 was ever read by this firmware.
 *
 * TC1 and TC2 are thermistors and track each other. TC1 is the one every
 * safety interlock uses; TC2 is monitored only, until the two have been seen
 * to agree across a real temperature range.
 *
 * CHG_FB is NOT a thermistor, despite reading like a plausible room
 * temperature at rest. It steps by about 100mV the instant CHG_ON is written
 * to the BQ7693 - not when ENABLE_CHARGE_PIN goes high, not when the discharge
 * FET is dropped - and it also follows the top-of-stack voltage. It is
 * analogue feedback from the charge path, and the step is direct evidence that
 * the AFE charge-FET driver responds to the command.
 */
typedef enum
{
  BMS_ADC_CH_TC1,       /* PA07 - thermistor, used by the interlocks */
  BMS_ADC_CH_TC2,       /* PA08 - thermistor, monitored only */
  BMS_ADC_CH_CHG_FB,    /* PA05 - charge-path feedback, not a temperature */
  BMS_ADC_CH_NUM
}bms_adc_ch_t;

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
extern void bms_adc_init(void);
extern uint16_t adc_convert_channel(bms_adc_ch_t ch);
extern void adc_convert_channels(void);
extern uint16_t bms_adc_read_ch(bms_adc_ch_t ch);
extern void bms_adc_debug_sweep(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/

#endif /* BMS_ADC_H_ */