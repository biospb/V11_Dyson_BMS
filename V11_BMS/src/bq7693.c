/*
 * bq7693.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */


#include "bq7693.h"

void bq7693_i2c_init(void);

//"internal" function primitives
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t data);

uint16_t bq7693_cell_voltages[7];

/* Cleared by bq7693_comm_clear_error(), latched false by any failed transfer. */
static volatile bool bq7693_comm_ok = true;

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
void bq7693_i2c_init()
 {
  //I2C peripheral init
  struct i2c_master_config config_i2c_master;

  i2c_master_get_config_defaults(&config_i2c_master);
  config_i2c_master.buffer_timeout            = BQ7693_TIMEOUT;
  config_i2c_master.unknown_bus_state_timeout = BQ7693_TIMEOUT;
  config_i2c_master.inactive_timeout          = BQ7693_TIMEOUT;
  config_i2c_master.pinmux_pad0               = PINMUX_PA16C_SERCOM1_PAD0;
  config_i2c_master.pinmux_pad1               = PINMUX_PA17C_SERCOM1_PAD1;
  config_i2c_master.scl_low_timeout           = true;

  i2c_master_init(&i2c_master_instance, SERCOM1, &config_i2c_master);
  i2c_master_enable(&i2c_master_instance);
}

/** @brief Initialize BQ7693 protector IC: read ADC cal, set protection thresholds, enable CC. */
void bq7693_init()
{
  bq7693_i2c_init();
  bq7693_write_register(SYS_CTRL2, 0x00); //Ensure that charge/discharge FETs are off so pack is safe.

  //Read the ADC offset and gain values and store
  uint8_t scratch1, scratch2;
  bq7693_read_register(ADCOFFSET, 1, &scratch1);  // convert from 2's complement
  bq7693_adc_offset = (int8_t)scratch1;
  bq7693_read_register(ADCGAIN1, 1, &scratch1);
  bq7693_read_register(ADCGAIN2, 1, &scratch2);
  bq7693_adc_gain = 365 + ((( scratch1 & 0x0C) << 1) | (( scratch2 & 0xE0) >> 5)); // uV/LSB

  bq7693_write_register(PROTECT1, 0x82);
  bq7693_write_register(PROTECT2, 0x04);

  //This sets overvolt and undervolt delays to 1 second.
  bq7693_write_register(PROTECT3, 0x00);

  //Calculate OV and UV trip voltages.
  scratch1 = (((((long)CELL_OVERVOLTAGE_TRIP - bq7693_adc_offset)*1000)/ bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(OV_TRIP, scratch1);

  scratch1 = (((((long)CELL_UNDERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(UV_TRIP, scratch1);

  bq7693_write_register(CELLBAL1, 0x00); //Disable cell balancing 1
  bq7693_write_register(CELLBAL2, 0x00); //Disable cell balancing 2

  bq7693_write_register(CC_CFG, 0x19); //'magic' value as per datasheet.
  bq7693_write_register(SYS_CTRL2, 0x40); //CC_EN - enable continuous operation of coulomb counter

  bq7693_write_register(SYS_CTRL1, 0x10); //ADC_EN

  bq7693_read_register(SYS_STAT, 1, &scratch1);
  bq7693_write_register(SYS_STAT, scratch1); //Explicitly clear any set bits in the SYS_STAT register by writing them back.
}

/**
 * @brief Read one or more bytes from a BQ7693 register via I2C.
 *
 * @param addr  Register address to read from.
 * @param len   Number of bytes to read.
 * @param buf   Buffer to store the read data.
 * @return      true on success.
 */
bool bq7693_read_register(uint8_t addr, size_t len, uint8_t *buf)
 {
  //Disable interrupts from the EIC - we don't want to end up trying to read the
  //charge counter half way through an existing i2c op. Re-enable at the end.
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint16_t timeout = 0;
  bool result = true;
  /* The BQ7693003 has CRC enabled, so it sends a CRC byte after every data
     byte. Read the pair and check it - len counts DATA bytes only. */
  uint8_t raw[BQ7693_MAX_READ_LEN * 2u];
  size_t  i;

  if ((len == 0u) || (len > BQ7693_MAX_READ_LEN))
  {
    bq7693_comm_ok = false;
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
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
    if (timeout++ >= BQ7693_TIMEOUT)
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
      if (timeout++ >= BQ7693_TIMEOUT)
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

  if (!result)
  {
    bq7693_comm_ok = false;
  }

  system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
  return result;
}

/**
 * @brief Write a single byte to a BQ7693 register with CRC.
 *
 * @param addr   Register address to write to.
 * @param value  Byte value to write.
 * @return       true on success.
 */
bool bq7693_write_register(uint8_t addr, uint8_t value)
{
  //Disable interrupts from the EIC - we don't want to end up trying to read the
  //charge counter half way through an existing i2c op. Re-enable at the end.
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

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
    if (timeout++ == BQ7693_TIMEOUT)
    {
      result = false;
      break;
    }
  }

  if (!result)
  {
    bq7693_comm_ok = false;
  }

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
  return bq7693_comm_ok;
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

/** @brief Clear SYS_STAT errors and enable the charge FET. */
void bq7693_enable_charge(void)
{
  uint8_t scratch;
  //Clear any bits in the SYS_STAT error register
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch); //Explicitly clear any set bits in the SYS_STAT register by writing them back.

  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_CHG_ON);
}

/** @brief Disable the charge FET, preserving DSG state. */
void bq7693_disable_charge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_CHG_ON);
}

/** @brief Configure protection, clear errors, and enable the discharge FET. */
void bq7693_enable_discharge(void)
{
  bq7693_write_register(SYS_CTRL1, 0x10);  //ADC_EN=1

  bq7693_write_register(PROTECT1, 0x9F);
  bq7693_write_register(PROTECT2, 0x04);

  uint8_t scratch;
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch); //Explicitly clear any set bits in the SYS_STAT register by writing them back.

  //DSG_ON turns the discharge FET on. Preserve CHG_ON so charging is not affected.
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_DSG_ON);

  bq7693_write_register(PROTECT2, 0x04);
  bq7693_write_register(PROTECT1, 0x82);
}

/** @brief Disable the discharge FET, preserving CHG state. */
void bq7693_disable_discharge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_DSG_ON);
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
    bq7693_cell_voltages[i] = tempval * bq7693_adc_gain/1000 + bq7693_adc_offset;
  }

  return bq7693_cell_voltages;
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

/** @brief Put BQ7693 into SHIP mode (deep sleep). */
void bq7693_enter_sleep_mode(void)
{
  bq7693_write_register(SYS_CTRL1, 0x00);
  bq7693_write_register(SYS_CTRL1, 0x01);
  bq7693_write_register(SYS_CTRL1, 0x02);
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
// the set of cells that must not bleed at the same time. Note cells 3 (CB4) and
// 4 (CB6) sit in different registers - the datasheet rule does not formally
// cover that pair - but VC5 is strapped to VC4 on this pack, so both bleed
// currents would meet at one physical node. Treated as conflicting.

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
  /* 5  CB7  */ (1u << 4),
  /* 6  CB10 */ 0
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
  uint16_t *v = bq7693_get_cell_voltages();
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

  // A cell has run away. Stop bleeding everything and let the caller cut charge.
  if (v_max >= CELL_BALANCE_OV_GUARD_MV)
  {
    bq7693_disable_balancing();
    status->state = BQ_BALANCE_OV;
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
 * @return  Signed 16-bit CC value.
 */
int16_t bq7693_read_cc(void)
{
  int16_t tempCC;

  uint8_t scratch[2];
  if (!bq7693_read_register(CC_HI_BYTE, 2, scratch))
  {
    return 0;
  }
  tempCC =  ((scratch[0])<<8);
  tempCC |= scratch[1];

  return tempCC;
}
