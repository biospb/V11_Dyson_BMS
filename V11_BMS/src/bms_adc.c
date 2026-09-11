/*
 * bms_adc.c
 *
 * Created: 21-Jan-26 10:09:58
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "bms_adc.h"
#include "ntc.h"
#include "serial_debug.h"

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static uint16_t adc_result[BMS_ADC_CH_NUM] = {0};
static struct adc_module adc_instance;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const enum adc_positive_input adc_ch_map_cfg[BMS_ADC_CH_NUM] =
{
  [BMS_ADC_CH_TC1]      = ADC_POSITIVE_INPUT_PIN7,   /* PA07 */
  [BMS_ADC_CH_TC2]      = ADC_POSITIVE_INPUT_PIN16,  /* PA08 */
  [BMS_ADC_CH_CHG_FB]   = ADC_POSITIVE_INPUT_PIN5,   /* PA05 */
};

/* Peripheral function for each of the above. adc_init() only muxes the pin it
   is configured with, so the other two have to be pointed at the ADC by hand
   or they convert nothing. */
static const uint32_t adc_ch_pinmux_cfg[BMS_ADC_CH_NUM] =
{
  [BMS_ADC_CH_TC1]      = PINMUX_PA07B_ADC_AIN7,
  [BMS_ADC_CH_TC2]      = PINMUX_PA08B_ADC_AIN16,
  [BMS_ADC_CH_CHG_FB]   = PINMUX_PA05B_ADC_AIN5,
};

/*-----------------------------------------------------------------------------
    Diagnostic sweep of the unclaimed ADC pins
-----------------------------------------------------------------------------*/
/*
 * PA02 is ADC-capable and appears nowhere else in this
 * firmware - nothing configures it, reads it or drives it. PA05 and PA08 used
 * to be listed here too; the sweep is what identified them as thermistors, and
 * they are proper channels now. PA02 is not one: it read 168mV, which on the
 * NTC curve is 77C and is not a room-temperature sensor.
 *
 * Left in place because what PA02 actually carries is still unknown, and the
 * cheapest way to find out is to watch whether it tracks anything. Reading a
 * pin is passive - the ADC input is high impedance - so this cannot fight
 * whatever may be driving it from outside.
 */
static const struct
{
  uint32_t                pinmux;
  enum adc_positive_input ain;
  const char             *name;
} adc_sweep_cfg[] =
{
  { PINMUX_PA02B_ADC_AIN0,  ADC_POSITIVE_INPUT_PIN0,  "PA02" },
};

/** @brief Point a pin at its ADC peripheral function. */
static void adc_pin_set_peripheral(uint32_t pinmux)
{
  uint8_t port = (uint8_t)((pinmux >> 16) / 32);
  uint8_t pin  = (uint8_t)((pinmux >> 16) - (port * 32u));

  PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
  PORT->Group[port].PMUX[pin / 2u].reg &= ~(0xF << (4u * (pin & 0x01u)));
  PORT->Group[port].PMUX[pin / 2u].reg |=  (uint8_t)((pinmux & 0x0000FFFFu) << (4u * (pin & 0x01u)));
}

/**
 * @brief Convert and report every ADC pin this firmware does not otherwise use.
 *
 * Output: "ADC <pin> raw=<n> mV=<n> T=<n>" per pin. The temperature column runs
 * the reading through the same NTC curve as the real sensor, so a pin carrying
 * a thermistor reports a believable room temperature and everything else does
 * not. mV is against the ADC's INTVCC0 reference, VCC/1.48, so full scale is
 * about 2230mV rather than 3300.
 *
 * Reading near 0 or near full scale with a nonsense temperature means nothing
 * is connected. A mid-scale value that tracks the PA07 reference means a second
 * sensor is fitted there.
 *
 * The swept pins are left muxed to the ADC afterwards. They are unused, so that
 * costs nothing; adc_convert_channel() re-selects its own input every call and
 * is unaffected.
 */
void bms_adc_debug_sweep(void)
{
#ifdef SERIAL_DEBUG
  char tmp[56];

  for (uint8_t i = 0; i < (uint8_t)(sizeof(adc_sweep_cfg) / sizeof(adc_sweep_cfg[0])); i++)
  {
    enum status_code status;
    uint16_t         raw = 0xFFFF;

    adc_pin_set_peripheral(adc_sweep_cfg[i].pinmux);
    adc_set_positive_input(&adc_instance, adc_sweep_cfg[i].ain);
    adc_start_conversion(&adc_instance);

    do
    {
      status = adc_read(&adc_instance, &raw);
    } while (status == STATUS_BUSY);

    if (status != STATUS_OK)
    {
      raw = 0xFFFF;
    }

    DEBUG_SNPRINTF(tmp, sizeof(tmp), "ADC %s raw=%u mV=%ld T=%d\r\n",
                   adc_sweep_cfg[i].name, raw,
                   ((long)raw * 2230L) / 4095L,
                   (int)(NTC_ADC2Temperature(raw) / 10));
    serial_debug_send_message(tmp);
  }
#endif
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief Initialise the ADC peripheral for thermistor readings.
 *
 * Configures the SAMD20 ADC in 12-bit single-shot mode with
 * internal VCC/1.48 reference and disables all ADC interrupts.
 */
void bms_adc_init(void)
{
  struct adc_config config_adc;

  adc_get_config_defaults(&config_adc);

  config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
  config_adc.reference       = ADC_REFERENCE_INTVCC0; /** 1/1.48V<SUB>CC</SUB> reference */
  config_adc.resolution      = ADC_RESOLUTION_12BIT;
  config_adc.freerunning     = false;
  config_adc.negative_input  = ADC_NEGATIVE_INPUT_GND;

  /* Initial channel does not matter */
  config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN7;

  adc_init(&adc_instance, ADC, &config_adc);

  /* adc_init() muxed only config_adc.positive_input. Point every channel at
     its pin, or the ones it missed convert whatever the mux happens to see. */
  for (uint8_t i = 0; i < (uint8_t)BMS_ADC_CH_NUM; i++)
  {
    adc_pin_set_peripheral(adc_ch_pinmux_cfg[i]);
  }

  /* HARD disable ADC interrupts */
  ADC->INTENCLR.reg = ADC_INTENCLR_MASK;
  ADC->INTFLAG.reg  = ADC_INTFLAG_MASK;

  adc_enable(&adc_instance);
}

/**
 * @brief Convert a single ADC channel (blocking).
 *
 * Selects the channel mux, triggers a one-shot conversion, and
 * busy-waits until the result is ready.
 *
 * @param ch  ADC channel to convert.
 * @return    12-bit ADC result, or 0xFFFF on error.
 */
uint16_t adc_convert_channel(bms_adc_ch_t ch)
{
  enum status_code status;
  uint16_t result = 0xFFFF;

  if(ch < BMS_ADC_CH_NUM)
  {
    enum adc_positive_input ch_mux = adc_ch_map_cfg[ch];

    /* Select ADC channel */
    adc_set_positive_input(&adc_instance, ch_mux);

    /* Start one-shot conversion */
    adc_start_conversion(&adc_instance);

    /* Poll until conversion is complete */
    do
    {
      status = adc_read(&adc_instance, &result);
    } while (status == STATUS_BUSY);

    if(status != STATUS_OK)
    {
      result = 0xFFFF;
    }

    adc_result[ch] = result;
  }

  return result;
}

/**
 * @brief Convert all configured ADC channels sequentially.
 *
 * Results are cached in a local array and can be read back with
 * bms_adc_read_ch().
 */
void adc_convert_channels(void)
{
  enum status_code status;
  uint16_t result;

  for(uint16_t i = 0; i < (uint16_t)BMS_ADC_CH_NUM; i++)
  {
    enum adc_positive_input ch_mux = adc_ch_map_cfg[i];

    /* Select ADC channel */
    adc_set_positive_input(&adc_instance, ch_mux);

    /* Start one-shot conversion */
    adc_start_conversion(&adc_instance);

    /* Poll until conversion is complete */
    do
    {
      status = adc_read(&adc_instance, &result);
    } while (status == STATUS_BUSY);

    if(status == STATUS_OK)
    {
      adc_result[i] = result;
    }
    else
    {
      adc_result[i] = 0xFFFF;
    }
  }
}

/**
 * @brief Read the last cached ADC result for a channel.
 *
 * @param ch  ADC channel to read.
 * @return    Cached 12-bit ADC value, or 0xFFFF if the channel is invalid.
 */
uint16_t bms_adc_read_ch(bms_adc_ch_t ch)
{
  uint16_t adc_ch_value = 0xFFFF;

  if(ch < BMS_ADC_CH_NUM)
  {
    adc_ch_value = adc_result[ch];
  }

  return adc_ch_value;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
