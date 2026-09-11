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
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void adc_pin_set_peripheral(uint32_t pinmux);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

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
