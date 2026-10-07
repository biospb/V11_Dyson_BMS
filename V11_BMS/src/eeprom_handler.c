/*
 *  eeprom_handler.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

#include "eeprom_handler.h"
volatile struct eeprom_data eeprom_data;

/*
 * Copy of what is actually stored in the emulated EEPROM, so eeprom_write()
 * can skip the flash erase when nothing has changed. Populated by a
 * successful eeprom_read() and by every real write; until then it is invalid
 * and a write always goes through.
 */
static struct eeprom_data eeprom_shadow;
static bool eeprom_shadow_valid = false;
static bool eeprom_ok = false;
static bool eeprom_data_valid = false;

bool eeprom_healthy(void)
{
  return eeprom_ok;
}

/*
 * Set when eeprom_init() rejected the stored page and replaced it with
 * defaults. Reported by the main loop rather than from here: eeprom_init()
 * runs before serial_debug_init(), which resets the transmit queue, so
 * anything queued this early is discarded before it reaches the wire.
 */
static bool eeprom_reset_on_init = false;

/**
 * @brief Write factory default values to EEPROM (capacity reset).
 */
enum eeprom_write_result eeprom_write_defaults(void)
{
  /*
   * Start at nominal, not 120% of it. The 120% seed only made sense while the
   * learning filter was decay-only and had to converge downward; now that a
   * full discharge-to-charge cycle can correct the estimate upward too, an
   * optimistic seed just means the gauge over-reads until the first full
   * cycle completes.
   */
  eeprom_data.magic                    = EEPROM_MAGIC;
  eeprom_data.total_pack_capacity      = (PACK_MAX_CAPACITY_MAH       * 1000ul);  //in micro-amp-hours
  eeprom_data.current_charge_level     = ((PACK_MAX_CAPACITY_MAH / 2) * 1000ul);
  eeprom_data.full_discharge_seen      = 0;
  eeprom_data.idle_wakes               = 0;
  eeprom_data_valid = true;
  return eeprom_write();
}

/**
 * @brief Initialize EEPROM emulator, program fuses if needed, verify stored data.
 *
 * If EEPROM fuses are not set, programs them and resets the MCU.
 * On first use, CRC mismatch, or a stored page whose EEPROM_MAGIC does not
 * match this firmware, writes factory defaults.
 *
 * @return ASF status code from eeprom_emulator_init().
 */
int eeprom_init(void)
{
  enum status_code error_code = eeprom_emulator_init();

  if (error_code == STATUS_ERR_NO_MEMORY)
  {
    //We are here because the fuses are set to 0x07, meaning eeprom is not enabled.
    //Show a few slow flashes to make it clear we're up to something, then reprogram fuses and reset
    //the mcu.
    for (int i=0; i<4; ++i)
    {
      leds_blink_leds(2000);
    }
    //This will update the fuses then reset the MCU
    eeprom_fuses_set();
  }
  else if (error_code != STATUS_OK)
  {
    //Init/format the eeprom
    eeprom_emulator_erase_memory();
    error_code = eeprom_emulator_init();
    if (error_code != STATUS_OK || eeprom_write_defaults() == EEPROM_WRITE_FAILED)
      return -1;
    eeprom_reset_on_init = true;
  }
  else
  {
    //EEPROM emulator OK - read data and verify CRC
    int read_status = eeprom_read();
    if (read_status == -2)
      return -1; // An unread page must not be replaced with defaults.
    if (read_status != 0)
    {
      //Corrupt, or written by firmware with a different EEPROM_MAGIC.
      //Either way the stored values cannot be trusted - start clean.
      if (eeprom_write_defaults() == EEPROM_WRITE_FAILED)
        return -1;
      eeprom_reset_on_init = true;
    }
  }

  return error_code;
}

/**
 * @brief Read EEPROM page, verify CRC32 integrity and the layout marker.
 *
 * @return 0 on success, -1 for invalid contents, -2 for a failed page read
 */
int eeprom_read(void)
{
  uint8_t buffer[EEPROM_PAGE_SIZE];
  eeprom_ok = false;
  eeprom_shadow_valid = false;
  eeprom_data_valid = false;
  if (eeprom_emulator_read_page(0, buffer) != STATUS_OK)
    return -2;
  memcpy((void*)&eeprom_data, buffer, sizeof(eeprom_data));

  //Verify CRC over data fields (everything before the crc32 field)
  uint32_t calc = calc_crc32((const uint8_t *)&eeprom_data,
      sizeof(eeprom_data) - sizeof(eeprom_data.crc32));
  if (calc != eeprom_data.crc32) {
    eeprom_shadow_valid = false;
    return -1;  //CRC mismatch - data corrupted
  }

  /*
   * Intact, but not necessarily ours. Firmware whose layout or field meaning
   * has changed bumps EEPROM_MAGIC, and its first boot lands here holding a
   * page that checksums perfectly and means something else.
   */
  if (eeprom_data.magic != EEPROM_MAGIC) {
    eeprom_shadow_valid = false;
    return -1;
  }

  if (eeprom_data.total_pack_capacity <= 0 ||
      eeprom_data.total_pack_capacity > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH ||
      eeprom_data.current_charge_level < 0 ||
      eeprom_data.current_charge_level > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH ||
      eeprom_data.full_discharge_seen > 1 ||
      eeprom_data.idle_wakes > STORAGE_IDLE_WAKES)
    return -1;

  //What we just read is, by definition, what is stored.
  eeprom_shadow.total_pack_capacity  = eeprom_data.total_pack_capacity;
  eeprom_shadow.current_charge_level = eeprom_data.current_charge_level;
  eeprom_shadow.full_discharge_seen  = eeprom_data.full_discharge_seen;
  eeprom_shadow.idle_wakes           = eeprom_data.idle_wakes;
  eeprom_shadow_valid = true;
  eeprom_ok = true;
  eeprom_data_valid = true;
  return 0;
}

/**
 * @brief Compute CRC32 and write the EEPROM page, unless it is already current.
 *
 * @return EEPROM_WRITTEN, EEPROM_UNCHANGED, or EEPROM_WRITE_FAILED
 */
enum eeprom_write_result eeprom_write(void)
{
  if (!eeprom_data_valid)
    return EEPROM_WRITE_FAILED;

  /*
   * Stamped on every write, so no path can store a page carrying a stale
   * marker. The skip below stays safe: a valid shadow can only have come from
   * a page that already matched, or from a write that passed through here.
   */
  eeprom_data.magic = EEPROM_MAGIC;

  /*
   * Skip the write when nothing worth storing has changed.
   *
   * The charge level gets a tolerance (EEPROM_CHARGE_TOLERANCE_UAH); the
   * capacity and the full-discharge marker do not, because they change rarely
   * and losing one costs a whole learning cycle.
   *
   * Note what the tolerance actually costs. SHIP mode loses RAM, so the shadow
   * is re-established from flash on every boot: a skipped delta is discarded
   * permanently, not deferred to the next write. Discharge only goes one way,
   * so repeated sessions that each land under the threshold accumulate error
   * without bound - the gauge drifts high by up to the tolerance per
   * sleep cycle. At 10mAh and a ~20A draw that is roughly 1.8s of running per
   * cycle, which is why the threshold wants to stay small. Set it to 0 for an
   * exact compare.
   *
   * Comparing fields rather than memcmp() also sidesteps the three padding
   * bytes the struct carries before crc32.
   */
  if (eeprom_ok && eeprom_shadow_valid
      && (eeprom_data.total_pack_capacity == eeprom_shadow.total_pack_capacity)
      && (eeprom_data.full_discharge_seen == eeprom_shadow.full_discharge_seen)
      && (eeprom_data.idle_wakes          == eeprom_shadow.idle_wakes))
  {
    int32_t drift = eeprom_data.current_charge_level - eeprom_shadow.current_charge_level;

    if (drift < 0)
    {
      drift = -drift;
    }

    if (drift <= (int32_t)EEPROM_CHARGE_TOLERANCE_UAH)
    {
      return EEPROM_UNCHANGED;
    }
  }

  //Compute CRC over data fields (everything before the crc32 field)
  eeprom_data.crc32 = calc_crc32((const uint8_t *)&eeprom_data,
      sizeof(eeprom_data) - sizeof(eeprom_data.crc32));

  uint8_t buffer[EEPROM_PAGE_SIZE] = {0};
  memcpy(buffer, (const void*)&eeprom_data, sizeof(eeprom_data));
  if (eeprom_emulator_write_page(0, buffer) != STATUS_OK ||
      eeprom_emulator_commit_page_buffer() != STATUS_OK)
  {
    eeprom_ok = false;
    eeprom_shadow_valid = false;
    return EEPROM_WRITE_FAILED;
  }
  eeprom_ok = true;

  eeprom_shadow.total_pack_capacity  = eeprom_data.total_pack_capacity;
  eeprom_shadow.current_charge_level = eeprom_data.current_charge_level;
  eeprom_shadow.full_discharge_seen  = eeprom_data.full_discharge_seen;
  eeprom_shadow.idle_wakes           = eeprom_data.idle_wakes;
  eeprom_shadow_valid = true;
  return EEPROM_WRITTEN;
}

/**
 * @brief True if eeprom_init() rejected the stored page and wrote defaults.
 *
 * Latched for the life of the power cycle, so the main loop can report it once
 * the debug UART is actually up.
 */
bool eeprom_was_reset(void)
{
  return eeprom_reset_on_init;
}

/**
 * @brief Program NVM fuses to enable 1024-byte EEPROM, then reset MCU.
 *
 * @return Never returns (triggers NVIC_SystemReset).
 */
int eeprom_fuses_set(void)
{
  //Set the the NVM
  struct nvm_config config_nvm;
  nvm_get_config_defaults(&config_nvm);
  nvm_set_config(&config_nvm);

  uint32_t temp;
  uint32_t data[2];

  /* Wait for NVM command to complete */
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  /* Read the fuse settings in the user row, 64 bit */
  data[0] = *((uint32_t *)NVMCTRL_AUX0_ADDRESS);
  data[1] = *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1);

  //Configure the fuse bits to enable EEPROM 1024bytes - minimal size for the ASF eeprom library to use.
  //Bits 4-6 specify eeprom size.
  //Clear bits 4-6.
  data[0] &= ~0x00000070;
  //Eeprom to 1024 bytes / 4 rows (EEPROM bits 0x04)
  data[0] |=  0x00000040;

  //Writeback sequence from https://microchip.my.site.com/s/article/SAMD20-SAMD21-Programming-the-fuses-from-application-code
  /* Disable Cache */
  temp = NVMCTRL->CTRLB.reg;
  NVMCTRL->CTRLB.reg = temp | NVMCTRL_CTRLB_CACHEDIS;

  /* Clear error flags */
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  /* Set address, command will be issued elsewhere */
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  /* Erase the user page */
  NVMCTRL->CTRLA.reg = NVM_COMMAND_ERASE_AUX_ROW | NVMCTRL_CTRLA_CMDEX_KEY;

  /* Wait for NVM command to complete */
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  /* Clear error flags */
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  /* Set address, command will be issued elsewhere */
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  /* Erase the page buffer before buffering new data */
  NVMCTRL->CTRLA.reg = NVM_COMMAND_PAGE_BUFFER_CLEAR | NVMCTRL_CTRLA_CMDEX_KEY;

  /* Wait for NVM command to complete */
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  /* Clear error flags */
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  /* Set address, command will be issued elsewhere */
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  // Write back the updated fuse bits.
  *((uint32_t *)NVMCTRL_AUX0_ADDRESS) = data[0];
  *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1) = data[1];

  /* Write the user page */
  NVMCTRL->CTRLA.reg = NVM_COMMAND_WRITE_AUX_ROW | NVMCTRL_CTRLA_CMDEX_KEY;

  /* Restore the settings */
  NVMCTRL->CTRLB.reg = temp;

  //Reset the MCU
  NVIC_SystemReset();
}
