/*
 * bq7693.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */


#include "bq7693.h"

static bool bq7693_i2c_init(void);

//"internal" function primitives
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t data);
static bool bq7693_read_attempt(uint8_t addr, size_t len, uint8_t *buf);
static bool bq7693_write_attempt(uint8_t addr, uint8_t value);

uint16_t bq7693_cell_voltages[7];

/* Cleared by bq7693_comm_clear_error(), latched false by any failed transfer. */
static volatile bool bq7693_comm_ok = true;
static bool bq7693_i2c_ready = false;
static bool bq7693_configured = false;

volatile int bq7693_adc_gain = 0;   // in uV/LSB
volatile int8_t bq7693_adc_offset = 0; //in mV

// maps for settings in chip protection registers

const int SCD_delay_setting [4] =
{ 70, 100, 200, 400 };

const int SCD_threshold_setting [8] =
{ 44, 67, 89, 111, 133, 155, 178, 200 }; // mV

const int OCD_delay_setting [8] =
{ 8, 20, 40, 80, 160, 320, 640, 1280 }; // ms
const int OCD_threshold_setting [16] =
{ 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100 };  // mV

const uint8_t UV_delay_setting [4] = { 1, 4, 8, 16 }; // s
const uint8_t OV_delay_setting [4] = { 1, 2, 4, 8 }; // s

struct i2c_master_module i2c_master_instance;

/**
 * @brief Set peripheral function for a pin via direct register access.
 *
 * @param pinmux  Pin multiplexer configuration value.
 */
static inline void pin_set_peripheral_function(uint32_t pinmux)
{
  uint8_t port = (uint8_t)((pinmux >> 16)/32);
  PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >>16) & 0x01u)));
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux & 0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}

/** @brief Initialize I2C master on SERCOM1 for BQ7693 communication. */
static bool bq7693_i2c_init(void)
 {
  //I2C peripheral init
  struct i2c_master_config config_i2c_master;

  i2c_master_get_config_defaults(&config_i2c_master);
  config_i2c_master.buffer_timeout            = BQ7693_I2C_BUFFER_TIMEOUT;
  config_i2c_master.unknown_bus_state_timeout = BQ7693_I2C_BUFFER_TIMEOUT;
  /* This is an enum of CTRLA.INACTOUT values, not a count. The old value of
     100 OR'd stray bits into CTRLA and left the timeout disabled anyway;
     say so explicitly rather than change the behaviour. */
  config_i2c_master.inactive_timeout          = I2C_MASTER_INACTIVE_TIMEOUT_DISABLED;
  config_i2c_master.pinmux_pad0               = PINMUX_PA16C_SERCOM1_PAD0;
  config_i2c_master.pinmux_pad1               = PINMUX_PA17C_SERCOM1_PAD1;
  config_i2c_master.scl_low_timeout           = true;

  if (i2c_master_init(&i2c_master_instance, SERCOM1, &config_i2c_master) != STATUS_OK)
    return false;
  i2c_master_enable(&i2c_master_instance);
  bq7693_i2c_ready = true;
  return true;
}

/** @brief Initialize BQ7693 protector IC: read ADC cal, set protection thresholds, enable CC. */
bool bq7693_init(void)
{
  uint8_t offset_raw, gain1_raw, gain2_raw, trip, sys_stat;
  bq7693_configured = false;
  if (!bq7693_i2c_init() || !bq7693_write_register(SYS_CTRL2, 0x00))
    return false;
  if (!bq7693_read_register(ADCOFFSET, 1, &offset_raw) ||
      !bq7693_read_register(ADCGAIN1, 1, &gain1_raw) ||
      !bq7693_read_register(ADCGAIN2, 1, &gain2_raw))
    return false;
  bq7693_adc_offset = (int8_t)offset_raw;
  bq7693_adc_gain = 365 + (((gain1_raw & 0x0C) << 1) | ((gain2_raw & 0xE0) >> 5));
  if (!bq7693_write_register(PROTECT1, 0x82) ||
      !bq7693_write_register(PROTECT2, 0x04) ||
      !bq7693_write_register(PROTECT3, 0x00))
    return false;
  trip = (((((long)CELL_OVERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;
  if (!bq7693_write_register(OV_TRIP, trip))
    return false;
  trip = (((((long)CELL_UNDERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;
  if (!bq7693_write_register(UV_TRIP, trip) ||
      !bq7693_write_register(CELLBAL1, 0x00) ||
      !bq7693_write_register(CELLBAL2, 0x00) ||
      !bq7693_write_register(CC_CFG, 0x19) ||
      !bq7693_write_register(SYS_CTRL2, 0x40) ||
      !bq7693_write_register(SYS_CTRL1, SYS_CTRL1_RUN) ||
      !bq7693_read_register(SYS_STAT, 1, &sys_stat) ||
      !bq7693_write_register(SYS_STAT, sys_stat))
    return false;
  bq7693_configured = true;
  return true;
}

/**
 * @brief One attempt at a register read: address, data+CRC, verify, copy out.
 *
 * @param addr  Register address to read from.
 * @param len   Number of DATA bytes to read.
 * @param buf   Buffer to store the read data.
 * @return      true if the transfer completed and every CRC verified.
 */
static bool bq7693_read_attempt(uint8_t addr, size_t len, uint8_t *buf)
{
  uint16_t timeout = 0;
  bool result = true;
  /* The BQ7693003 has CRC enabled, so it sends a CRC byte after every data
     byte. Read the pair and check it - len counts DATA bytes only. */
  uint8_t raw[BQ7693_MAX_READ_LEN * 2u];
  size_t  i;

  if ((len == 0u) || (len > BQ7693_MAX_READ_LEN))
  {
    return false;
  }

  //Initial write to set register address.
  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 1,
    .data = &addr
  };

  //Tx address
  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    /* Increment timeout counter and check if timed out. */
    if (timeout++ >= BQ7693_PACKET_RETRIES)
    {
      /* The register address never landed, so whatever the read phase
         returns is meaningless - this used to fall through as success. */
      result = false;
      break;
    }
  }

  //Rx data+CRC pairs
  if (result)
  {
    packet.data_length = (uint16_t)(len * 2u);
    packet.data = raw;
    timeout = 0;

    while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
    {
      /* Increment timeout counter and check if timed out. */
      if (timeout++ >= BQ7693_PACKET_RETRIES)
      {
        result = false;
        break;
      }
    }
  }

  /* Verify every CRC before handing any of it to the caller. Datasheet
     SLUSBK2I 8.3.1.1.2: the first data byte's CRC covers the slave address
     with the R/W bit set plus the data byte; subsequent bytes are covered on
     their own. */
  if (result)
  {
    for (i = 0u; i < len; i++)
    {
      uint8_t crc;

      if (i == 0u)
      {
        crc = bq7693_calc_checksum(0x00, (uint8_t)((BQ7693_ADDR << 1) | 0x01u));
        crc = bq7693_calc_checksum(crc, raw[0]);
      }
      else
      {
        crc = bq7693_calc_checksum(0x00, raw[i * 2u]);
      }

      if (crc != raw[(i * 2u) + 1u])
      {
        result = false;
        break;
      }
    }
  }

  if (result)
  {
    for (i = 0u; i < len; i++)
    {
      buf[i] = raw[i * 2u];
    }
  }

  return result;
}

/**
 * @brief Read one or more register bytes, retrying a corrupted transfer.
 *
 * @param addr  Register address to read from.
 * @param len   Number of DATA bytes to read (CRC bytes are handled inside).
 * @param buf   Buffer for the data bytes.
 * @return      true if an attempt succeeded with a valid CRC.
 */
bool bq7693_read_register(uint8_t addr, size_t len, uint8_t *buf)
{
  if (!bq7693_i2c_ready)
  {
    bq7693_comm_ok = false;
    return false;
  }
  bool result = false;
  uint8_t attempt;

  //Disable interrupts from the EIC - we don't want to end up trying to read the
  //charge counter half way through an existing i2c op. Re-enable at the end.
  bool eic_was_enabled = system_interrupt_is_enabled(SYSTEM_INTERRUPT_MODULE_EIC);
  if (eic_was_enabled)
    system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  for (attempt = 0u; (attempt < BQ7693_READ_ATTEMPTS) && !result; attempt++)
  {
    result = bq7693_read_attempt(addr, len, buf);

    if (!result)
    {
      /*
       * Only on the slow path. A safety check is ~9 registers and nothing
       * inside it pumps the services; with every packet phase timing out at
       * BQ7693_I2C_BUFFER_TIMEOUT the worst case is ~0.9s, past the 0.5s
       * early warning, and the fault would be shown as BMS_ERR_WDT rather
       * than I2C_FAIL. The retry budget is bounded, so kicking here cannot
       * hide a genuinely stuck loop.
       */
      wdt_reset_count();
    }
  }

  /* Only a transfer that failed every attempt counts as a comm error. A single
     corrupted read on a marginal bus would otherwise fault the pack outright. */
  if (!result)
  {
    bq7693_comm_ok = false;
  }

  if (eic_was_enabled)
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
  return result;
}

/**
 * @brief One attempt at writing a register, with its CRC appended.
 *
 * @param addr   Register address to write to.
 * @param value  Byte value to write.
 * @return       true if the device accepted the transfer.
 */
static bool bq7693_write_attempt(uint8_t addr, uint8_t value)
{
  uint16_t timeout = 0;
  bool result = true;

  uint8_t buf[3];
  buf[0] = addr;
  buf[1] = value;

  //Calculate CRC (over slave address + rw bit, reg address + data.
  uint8_t crc = bq7693_calc_checksum(0x00, (BQ7693_ADDR <<1) | 0);
  crc = bq7693_calc_checksum(crc, buf[0]);
  crc = bq7693_calc_checksum(crc, buf[1]);
  buf[2] = crc;

  //Initial write to set register address.
  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 3,
    .data = buf
  };

  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    /* Increment timeout counter and check if timed out. */
    if (timeout++ >= BQ7693_PACKET_RETRIES)
    {
      result = false;
      break;
    }
  }

  return result;
}

/**
 * @brief Write a register, retrying a transfer the device rejected.
 *
 * @param addr   Register address.
 * @param value  Byte to write.
 * @return       true if an attempt was accepted.
 */
bool bq7693_write_register(uint8_t addr, uint8_t value)
{
  if (!bq7693_i2c_ready)
  {
    bq7693_comm_ok = false;
    return false;
  }
  bool result = false;
  uint8_t attempt;

  bool eic_was_enabled = system_interrupt_is_enabled(SYSTEM_INTERRUPT_MODULE_EIC);
  if (eic_was_enabled)
    system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  for (attempt = 0u; (attempt < BQ7693_WRITE_ATTEMPTS) && !result; attempt++)
  {
    result = bq7693_write_attempt(addr, value);

    if (!result)
    {
      wdt_reset_count();   /* see bq7693_read_register() */
    }
  }

  if (!result)
  {
    bq7693_comm_ok = false;
  }

  if (eic_was_enabled)
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
  return result;
}

/** @brief Clear the sticky I2C error latch before a batch of transfers. */
void bq7693_comm_clear_error(void)
{
  bq7693_comm_ok = true;
}

/** @brief False if any transfer since the last clear failed. */
bool bq7693_comm_healthy(void)
{
  return bq7693_configured && bq7693_comm_ok;
}

/**
 * @brief Compute BQ7693 I2C CRC (poly 0x07).
 *
 * @param inCrc   Current CRC accumulator value.
 * @param inData  Next data byte to fold in.
 * @return        Updated CRC byte.
 */
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t inData)
{
  // CRC is calculated over the slave address (including R/W bit), register address, and data.
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;
  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else data <<= 1;
  }
  return data;
}

// SYS_CTRL2 bits
#define SYS_CTRL2_CC_EN   0x40
#define SYS_CTRL2_DSG_ON  0x02
#define SYS_CTRL2_CHG_ON  0x01

/**
 * @brief Enable the charge FET. Caller must clear SYS_STAT faults first.
 * @return false if SYS_CTRL2 could not be read, so the FET was left alone.
 */
bool bq7693_enable_charge(void)
{
  if (!bq7693_configured)
    return false;
  uint8_t ctrl2;

  /*
   * SYS_STAT is deliberately NOT touched here. The datasheet requires the
   * fault bit to be cleared before the FET will re-enable, but this function
   * used to do that by writing back every set bit - which clears them without
   * routing them through bms_sys_stat_faults, so a fault arriving between the
   * safety check and this call was silently discarded. The caller clears them
   * through the latch instead; see bms_sys_stat_service().
   */
  /* Refuse to turn a FET on when the current register state is unknown - the
     read-modify-write below would otherwise be modifying stack garbage. */
  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
  {
    return false;
  }

  return bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_CHG_ON);
}

/** @brief Disable the charge FET, preserving DSG state. */
bool bq7693_disable_charge(void)
{
  uint8_t ctrl2;
  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
  {
    // Best effort: both FETs off, CC on; still report the failed read.
    (void)bq7693_write_register(SYS_CTRL2, SYS_CTRL2_CC_EN);
    return false;
  }
  return bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_CHG_ON);
}

/**
 * @brief Configure protection, clear errors, and enable the discharge FET.
 * @return false if the FET could not be driven.
 */
bool bq7693_enable_discharge(void)
{
  uint8_t ctrl2;
  bool ok = bq7693_configured &&
            bq7693_write_register(SYS_CTRL1, SYS_CTRL1_RUN) &&
            bq7693_write_register(PROTECT1, 0x9F) &&
            bq7693_write_register(PROTECT2, 0x04) &&
            bq7693_read_register(SYS_CTRL2, 1, &ctrl2) &&
            bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_DSG_ON);
  // Restore both protection registers even if an earlier step failed.
  bool protect2_ok = bq7693_write_register(PROTECT2, 0x04);
  bool protect1_ok = bq7693_write_register(PROTECT1, 0x82);
  return ok && protect2_ok && protect1_ok;
}

/** @brief Disable the discharge FET, preserving CHG state. */
bool bq7693_disable_discharge(void)
{
  uint8_t ctrl2;
  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
  {
    // Best effort: both FETs off, CC on; still report the failed read.
    (void)bq7693_write_register(SYS_CTRL2, SYS_CTRL2_CC_EN);
    return false;
  }
  return bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_DSG_ON);
}

/**
 * @brief Convert a raw 14-bit VC reading to mV, floored at 0.
 *
 * ADCOFFSET is signed. With a negative offset a raw reading of zero - a
 * severed sense wire, a shorted channel - came out as a small negative number,
 * which the uint16_t result wrapped to ~65530mV: past the implausible-cell
 * check, and read as a full or overvolted cell by everything downstream.
 */
static uint16_t bq7693_vc_to_mv(uint16_t raw)
{
  int32_t mv = ((int32_t)raw * bq7693_adc_gain) / 1000 + bq7693_adc_offset;

  return (mv > 0) ? (uint16_t)mv : 0u;
}

/**
 * @brief Read all 7 cell voltages from BQ7693 and apply ADC calibration.
 *
 * @return  Pointer to static array of 7 cell voltages in mV.
 */
uint16_t *bq7693_get_cell_voltages(void)
{
  uint8_t scratch[2];
  uint16_t tempval;
  //Voltages for each cell
  //The cells are connected as below on these packs...
  int cellsToRead[] = { 0,1,2,3,5,6,9};

  for (int i=0; i< 7; ++i)
  {
    //Because CRC is enabled, we need to read 3 bytes (VCx_HI, the CRC byte (ignore), then VCx_Lo)
    if (!bq7693_read_register((VC1_HI_BYTE + 2*cellsToRead[i]), 2, scratch))
    {
      /* scratch[] is a stack local - using it here would report whatever
         happened to be on the stack as a cell voltage. Report 0mV instead,
         which fails every safety check, and leave the comm error latched. */
      bq7693_cell_voltages[i] = 0;
      continue;
    }
    tempval = ((scratch[0] & 0x3F) <<8) | scratch[1];
    bq7693_cell_voltages[i] = bq7693_vc_to_mv(tempval);
  }

  return bq7693_cell_voltages;
}

/**
 * @brief Read every VC channel, ignoring the cell-to-channel map. Diagnostic.
 *
 * bq7693_get_cell_voltages() returns only the seven channels this pack is
 * believed to use - cellsToRead[] = {0,1,2,3,5,6,9}, i.e. VC1..VC4, VC6, VC7,
 * VC10 - and that map is reverse-engineered from one board. This returns all
 * ten, so a board wired differently gives itself away: a real cell voltage on
 * a channel the map skips (VC5, VC8, VC9), or nothing on one it uses.
 *
 * On a correctly-mapped 7S pack the three skipped channels read near zero,
 * because unused inputs are shorted.
 *
 * @param voltages_out  Array of 10, filled with VC1..VC10 in mV. A channel
 *                      whose read failed is reported as 0, and the comm error
 *                      is left latched for the caller to notice.
 */
void bq7693_get_all_vc(uint16_t *voltages_out)
{
  uint8_t scratch[2];
  uint16_t tempval;

  for (int i = 0; i < 10; ++i)
  {
    if (!bq7693_read_register((uint8_t)(VC1_HI_BYTE + (2 * i)), 2, scratch))
    {
      voltages_out[i] = 0;
      continue;
    }
    tempval = ((scratch[0] & 0x3F) << 8) | scratch[1];
    voltages_out[i] = bq7693_vc_to_mv(tempval);
  }
}

/**
 * @brief Read total pack voltage from BQ7693 BAT register.
 *
 * @return  Pack voltage in mV.
 */
int bq7693_get_pack_voltage(void)
{
  uint8_t scratch[2];
  uint16_t tempval;
  if (!bq7693_read_register(BAT_HI_BYTE, 2, scratch))
  {
    return 0;
  }
  tempval = scratch[0] <<8 | scratch[1];
  int bq7693_pack_voltage = 4 * bq7693_adc_gain * tempval / 1000 + ( 7 * bq7693_adc_offset);
  return bq7693_pack_voltage;
}

/**
 * @brief Put BQ7693 into SHIP mode (deep sleep).
 *
 * SLUSBK2I 8.3.1.3: the sequence is [SHUT_A=0,SHUT_B=0], [0,1], [1,0], and
 * SYS_CTRL1 carries SHUT_A in bit 1 and SHUT_B in bit 0.
 *
 * @return false if any of the three writes was not accepted. A true return
 *         does NOT mean the device shut down - it means it was asked
 *         correctly. The AFE declines while its BOOT pin is held high, and
 *         says nothing about having declined; the caller has to notice that
 *         it is still running. Note also that the sequence clears ADC_EN,
 *         so a caller that survives has to put SYS_CTRL1 back.
 */
bool bq7693_enter_sleep_mode(void)
{
  bool ok = true;

  ok &= bq7693_write_register(SYS_CTRL1, 0x00);
  ok &= bq7693_write_register(SYS_CTRL1, 0x01);
  ok &= bq7693_write_register(SYS_CTRL1, 0x02);

  return ok;
}

//-----------------------------------------------------------------------------
//  Passive cell balancing
//-----------------------------------------------------------------------------
#if CELL_BALANCE_ENABLE
// Cells on this pack are wired to VC1..VC4, VC6, VC7 and VC10 (see the
// cellsToRead[] map in bq7693_get_cell_voltages), so the balance channels are
// CB1..CB4 in CELLBAL1 and CB6, CB7, CB10 in CELLBAL2.
//
// Datasheet constraint (SLUSBK2I 8.3.1.3.3): "The host controller must ensure
// that no two adjacent cells are balanced simultaneously within each set of:
// VC1-VC5, VC6-VC10, VC11-VC15." Doing so can push cell pins past their
// absolute maximum ratings.
//
// cb_reg/cb_bit give the CELLBALn bit for each measured cell; cb_conflict is
// the set of cells that must not bleed at the same time. Two pairs are not
// formally covered by the datasheet rule because they span the strapped,
// unused inputs (SLUSBK2I Table 9-3, 7-cell column): VC5 is strapped to VC4,
// so CB4 and CB6 both bleed into the cell-4 top node, and VC8/VC9 are strapped
// to VC7, so CB7 and CB10 both bleed into the cell-6 top node. Neither pair
// shares a pin, so this is conservative - but it is the same situation twice,
// and it is treated the same way twice.

static const uint8_t cb_reg[BQ7693_NUM_CELLS] =
{
  CELLBAL1, CELLBAL1, CELLBAL1, CELLBAL1,   // CB1  CB2  CB3  CB4
  CELLBAL2, CELLBAL2, CELLBAL2              // CB6  CB7  CB10
};

static const uint8_t cb_bit[BQ7693_NUM_CELLS] =
{
  0, 1, 2, 3,     // CB1..CB4  -> CELLBAL1 bits 0..3
  0, 1, 4         // CB6, CB7, CB10 -> CELLBAL2 bits 0, 1, 4
};

static const uint8_t cb_conflict[BQ7693_NUM_CELLS] =
{
  /* 0  CB1  */ (1u << 1),
  /* 1  CB2  */ (1u << 0) | (1u << 2),
  /* 2  CB3  */ (1u << 1) | (1u << 3),
  /* 3  CB4  */ (1u << 2) | (1u << 4),
  /* 4  CB6  */ (1u << 3) | (1u << 5),
  /* 5  CB7  */ (1u << 4) | (1u << 6),
  /* 6  CB10 */ (1u << 5)
};

// Cells that were bleeding on the previous pass, for start/stop hysteresis.
static uint8_t bq7693_balance_latch = 0;

/**
 * @brief Write both balance registers directly.
 *
 * @param cellbal1  CB5..CB1 in bits 4..0.
 * @param cellbal2  CB10..CB6 in bits 4..0.
 */
void bq7693_set_balancing(uint8_t cellbal1, uint8_t cellbal2)
{
  bq7693_write_register(CELLBAL1, cellbal1 & 0x1F);
  bq7693_write_register(CELLBAL2, cellbal2 & 0x1F);
}

/** @brief Turn every balance channel off and drop the hysteresis latch. */
void bq7693_disable_balancing(void)
{
  bq7693_balance_latch = 0;
  bq7693_set_balancing(0x00, 0x00);
}

/**
 * @brief Re-evaluate which cells should be bleeding and program CELLBAL1/2.
 *
 * Reads the cell voltages, then bleeds every cell that sits above the pack
 * minimum by more than CELL_BALANCE_START_MV, highest cell first, skipping any
 * cell that conflicts with one already selected. The registers are rewritten on
 * every call, which also re-arms balancing after the BQ7693 auto-clears
 * CELLBAL1/2/3 (it does so whenever DEVICE_XREADY is set, and on entry to
 * NORMAL mode from SHIP).
 *
 * Call no faster than the ADC update rate (250ms); CELL_BALANCE_PERIOD_MS is
 * the intended cadence. Only meaningful with the charge FET off and the cells
 * relaxed - under charge the measured spread is mostly IR drop.
 *
 * @param status  Filled in with the resulting decision. Must not be NULL.
 */
void bq7693_balance_update(bq7693_balance_status_t *status)
{
  uint16_t *v;

  /*
   * Judge the pack on this tick's own readings. A failed cell read reports
   * 0mV, which looks like a 4V spread and would be announced as a failing
   * cell on a perfectly good pack - so read first, then check that every
   * transfer landed, and sit the tick out if not.
   *
   * The latch is cleared before the read because nothing else clears it in
   * BMS_CHARGER_CONNECTED_NOT_CHARGING, where no safety check runs: an error
   * latched there by anything at all - a session-timeout FET disable, one
   * corrupted coulomb-counter read - parked balancing for the rest of the
   * dock session. Nothing is lost by clearing it here; the safety checks
   * clear it before their own reads anyway.
   */
  bq7693_comm_clear_error();
  v = bq7693_get_cell_voltages();

  if (!bq7693_comm_healthy())
  {
    bq7693_disable_balancing();
    status->cell_mask = 0;
    status->num_cells = 0;
    return;
  }

  uint16_t  v_min = 0xFFFF;
  uint16_t  v_max = 0;
  uint8_t   max_cell = 0;
  uint8_t   selected = 0;
  uint8_t   reg1 = 0;
  uint8_t   reg2 = 0;
  uint8_t   n = 0;
  int i;

  for (i = 0; i < BQ7693_NUM_CELLS; ++i)
  {
    if (v[i] < v_min)
      v_min = v[i];
    if (v[i] > v_max)
    {
      v_max    = v[i];
      max_cell = (uint8_t)i;
    }
  }

  status->v_min_mv  = v_min;
  status->v_max_mv  = v_max;
  status->spread_mv = (uint16_t)(v_max - v_min);
  status->max_cell  = max_cell;
  status->cell_mask = 0;
  status->num_cells = 0;

  // --- guards, most severe first -------------------------------------------

  // A cell has reached the guard. The caller must not put charge in - but
  // this is exactly the cell that most needs bleeding, so bleed it, alone.
  // Refusing used to park the cell above the guard until self-discharge
  // brought it down, days on a good cell, with the charge path faulting on
  // it the whole time. One cell on its own cannot break the adjacency rule,
  // and the AFE's own OV trip at CELL_OVERVOLTAGE_TRIP still sits above this.
  if (v_max >= CELL_BALANCE_OV_GUARD_MV)
  {
    uint8_t bit = (uint8_t)(1u << cb_bit[max_cell]);

    bq7693_balance_latch = (uint8_t)(1u << max_cell);
    if (cb_reg[max_cell] == CELLBAL1)
      bq7693_set_balancing(bit, 0x00);
    else
      bq7693_set_balancing(0x00, bit);

    status->cell_mask = (uint8_t)(1u << max_cell);
    status->num_cells = 1;
    status->state     = BQ_BALANCE_OV;
    return;
  }

  // Not near the top of charge yet - cell voltage does not track SOC well
  // enough down here to balance on. Gated on the highest cell so that a single
  // weak cell cannot lock balancing out forever.
  if (v_max < CELL_BALANCE_MIN_CELL_MV)
  {
    bq7693_disable_balancing();
    status->state = BQ_BALANCE_TOO_LOW;
    return;
  }

  // One cell will not charge up to meet the others. This is a failing cell, not
  // an imbalance: bleeding the other six down to reach it would dump most of the
  // pack's energy as heat and achieve nothing. Refuse and report it.
  if (status->spread_mv > CELL_BALANCE_MAX_SPREAD_MV)
  {
    bq7693_disable_balancing();
    status->state = BQ_BALANCE_CELL_FAIL;
    return;
  }

  // Close enough - done.
  if (status->spread_mv <= CELL_BALANCE_TARGET_SPREAD_MV)
  {
    bq7693_disable_balancing();
    status->state = BQ_BALANCE_IDLE;
    return;
  }

  // --- selection: highest cell first, then next highest, honouring conflicts -
  for (;;)
  {
    int      best   = -1;
    uint16_t best_v = 0;

    for (i = 0; i < BQ7693_NUM_CELLS; ++i)
    {
      uint16_t threshold;

      if (selected & (1u << i))
        continue;                       // already picked this pass

      // Hysteresis: a cell already bleeding keeps going down to the tighter
      // stop threshold, so cells do not chatter around the start threshold.
      threshold = (bq7693_balance_latch & (1u << i)) ? CELL_BALANCE_STOP_MV
                                                     : CELL_BALANCE_START_MV;

      if ((uint16_t)(v[i] - v_min) <= threshold)
        continue;                       // low enough, leave it alone

      if (selected & cb_conflict[i])
        continue;                       // adjacent to a cell already bleeding

      if (best < 0 || v[i] > best_v)
      {
        best   = i;
        best_v = v[i];
      }
    }

    if (best < 0)
      break;

    selected |= (uint8_t)(1u << best);
    ++n;

    if (cb_reg[best] == CELLBAL1)
      reg1 |= (uint8_t)(1u << cb_bit[best]);
    else
      reg2 |= (uint8_t)(1u << cb_bit[best]);
  }

  bq7693_balance_latch = selected;
  bq7693_set_balancing(reg1, reg2);

  status->cell_mask = selected;
  status->num_cells = n;
  status->state     = (n > 0) ? BQ_BALANCE_ACTIVE : BQ_BALANCE_IDLE;
}

#endif /* CELL_BALANCE_ENABLE */

/**
 * @brief Read raw coulomb counter value from BQ7693.
 *
 * @param cc  Receives the signed 16-bit CC value on success.
 * @return    false if the read failed - this used to return 0, which the
 *            caller integrated as a genuine 0mA window.
 */
bool bq7693_read_cc(int16_t *cc)
{
  uint8_t scratch[2];

  if (!bq7693_read_register(CC_HI_BYTE, 2, scratch))
  {
    return false;
  }

  *cc = (int16_t)(((uint16_t)scratch[0] << 8) | scratch[1]);
  return true;
}
