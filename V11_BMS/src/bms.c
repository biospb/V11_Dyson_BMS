/*
 * bms.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "bms.h"
#include "bms_adc.h"
#include "ntc.h"
#include "crc.h"
#include "sw_timer.h"
#include "dsn_protocol.h"
#include "dio.h"
#include "bms_wdt.h"

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
static void bms_set_error(enum BMS_ERROR_CODE code);

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
/* Set by CMake from `git rev-parse --short HEAD`, with a trailing '+' when the
   working tree differs from it. Falls back for builds made outside CMake. */
/*
 * Temperatures are carried in 0.1C. Printing them as "%d.%d" needs the sign
 * handled apart from the digits: at -0.5C the whole part is 0, so the minus
 * would vanish and a below-zero pack would read as above it.
 */
#define T_SIGN(v)   ((((v) < 0) && ((v) > -10)) ? "-" : "")
#define T_WHOLE(v)  ((int)((v) / 10))
#define T_FRAC(v)   ((int)abs((v) % 10))

#ifndef FW_GIT_REV
#define FW_GIT_REV "unknown"
#endif

#ifdef SERIAL_DEBUG
#define BMS_PRINT(...) \
{ \
  char _dbg_tmp[DEBUG_MSG_BUFFER_SIZE]; \
  DEBUG_SNPRINTF(_dbg_tmp, sizeof(_dbg_tmp), __VA_ARGS__); \
  serial_debug_send_message(_dbg_tmp);  \
}
#else
#define BMS_PRINT(...)
#endif

#define PACK_CAPACITY_UPPER_BOUND_UAH       (PACK_MAX_CAPACITY_MAH * 1200ul)  // 120% of nominal, in uAh
#define PACK_CAPACITY_LOWER_BOUND_UAH       (PACK_MAX_CAPACITY_MAH *  300ul)  //  30% of nominal, in uAh

// RTC standby wake timer: GCLK2 = ULP32K/32 (1024 Hz), RTC prescaler = DIV1024 → 1 Hz
// N days = N * 86400 seconds × 1 tick/sec
#define RTC_STANDBY_WAKE_TICKS  ((uint32_t)2 * (24UL * 60UL * 60UL))

/*
 * ALERT is the OR of every SYS_STAT bit and EXTINT 8 is configured rising-edge
 * only, so any latched bit holds the line high and no further edge can ever
 * arrive. Poll on this interval as well so a latched fault cannot wedge the
 * interrupt permanently.
 *
 * This MUST be shorter than the coulomb counter's 250ms conversion window.
 * CC_READY latches and the CC register holds only the most recent window -
 * the datasheet is explicit that the bit stays latched if it is not cleared
 * between two adjacent readings, and the older reading is simply gone.
 * Polling slower than the window therefore integrates one window in every N
 * and undercounts the charge by that factor, silently, for as long as ALERT
 * is wedged. Polling at exactly 250ms is not enough either: the timer fires
 * at 251ms and the two clocks drift, so one window in every ~250 was still
 * dropped. Polling faster costs only the occasional read that finds CC_READY
 * clear.
 *
 * Costs nothing when ALERT is healthy: every service restarts this timer, so
 * the poll only fires if no ALERT has arrived for this long.
 */
#define SYS_STAT_POLL_MS        (200ul)

/*
 * How long bms_handle_idle() waits before re-trying a discharge arm that
 * failed its safety check or its FET enable. The retry is what makes a
 * transient (a momentary cell dip, one bad transfer) recoverable without
 * re-plugging the cleaner, but retrying on every 50ms pass ran the whole
 * check - seven cell reads and a log line per low cell - twenty times a
 * second, which floods the debug queue and the bus for as long as the
 * condition lasts.
 */
#define IDLE_DSG_RETRY_MS       (2000ul)

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
//We start off idle.
//volatile: bms_force_fault() writes both of these from the watchdog
//early-warning interrupt, and the state machine reads them in the main loop.
static volatile enum BMS_STATE bms_state      = BMS_INIT;
//If a fault occurs, it'll be lodged here.
static volatile enum BMS_ERROR_CODE bms_error = BMS_ERR_NONE;

static int32_t current_mA = 0;
static int32_t current_filt_sum_mA = 0;
static int32_t current_filt_mA = 0;

static uint16_t charge_pause_counter = 0;
static sw_timer bms_timer = 0;
static int16_t  pack_temperature = 0;    /* TC1 - the cold check */
static int16_t  pack_temperature_2 = 0;  /* TC2 - against the cells */
//volatile: set by the BQ7693 ALERT interrupt, cleared in the main loop.
static volatile bool process_bms_interrupt = false;
/*
 * SYS_STAT has exactly one owner: bms_sys_stat_service(). Any fault bit it
 * finds is accumulated here and consumed by the safety checks, so clearing the
 * bit on the chip (which is what lets ALERT de-assert and re-arm) cannot lose
 * the event.
 */
static volatile uint8_t bms_sys_stat_faults = 0;
/* Absorbed DEVICE_XREADY / OVRD_ALERT events - see BQ_AFE_FAULT_TOLERANCE.
   Decays after BQ_AFE_FAULT_DECAY_MS without one. */
static uint8_t  bms_afe_fault_count = 0;
static sw_timer bms_afe_fault_timer = 0;
/*
 * Set when an AFE event is absorbed. The AFE has already dropped BOTH FET
 * drivers for it (SLUSBK2I Table 8-1) and never re-enables one on its own, so
 * the state that owns a FET has to turn it back on - otherwise "absorbed"
 * means charging stops silently, or the motor drops out with the trigger
 * still reported as pulled. Consumed by bms_afe_fet_dropped().
 */
static bool     bms_afe_fet_dropped_flag = false;
static sw_timer bms_sys_stat_timer = 0;
static volatile bool rtc_wakeup_flag = false;
//volatile: set by the trigger/charger/mode-button EIC callbacks while the
//wake sources are armed, tested under a critical section before WFI.
static volatile bool bms_wake_event = false;
static struct rtc_module rtc_instance;

extern volatile struct eeprom_data eeprom_data;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
#ifdef SERIAL_DEBUG
const char *bms_state_names[] =
{
  "INIT",
  "IDLE",
  "CHARGER_CONNECTED",
  "CHARGING",
  "CHARGER_CONNECTED_NOT_CHARGING",
  "CHARGER_UNPLUGGED",
  "VACUUM_RUNNING",
  "FAULT",
  "SLEEP"
};
#endif

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void    pins_init(void);
static void    pins_deinit(void);
static void    interrupts_init(void);
static int16_t bms_read_temperature(void);
static void    bms_sample_temperatures(void);
static int16_t bms_temperature_hottest(void);
static bool    bms_trigger_active(void);
static bool    bms_is_safe_to_discharge(void);
static bool    bms_is_safe_to_charge(void);
static bool    bms_is_pack_full(void);
static bool    bms_in_storage_mode(void);
static void    bms_handle_idle(void);
static void    bms_handle_sleep(void);
static void    bms_handle_vacuum_running(void);
static void    bms_handle_fault(void);
static void    bms_handle_charger_connected(void);
static void    bms_handle_charger_connected_not_charging(void);
static void    bms_handle_charging(void);
static void    bms_handle_charger_unplugged(void);
static bool    bms_fault_pending(void);
static bool    bms_afe_fault_is_real(uint8_t sys_stat, const char *who);
static bool    bms_afe_fet_dropped(void);
static void    bms_blink_error_code(enum BMS_ERROR_CODE code);
static void    bms_sys_stat_service(void);
static uint8_t bms_sys_stat_take(void);
static bool    bms_charge_fet_on(void);
static bool    bms_discharge_fet_on(void);
static void    bms_enter_standby(void);
static void    bms_debug_dump_pins(const char *who);
static void    bms_debug_dump_temps(const char *who);
static void    bms_leave_standby(void);
static void    rtc_standby_timer_init(void);
static void    rtc_standby_timer_start(void);
static void    rtc_standby_timer_stop(void);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
/** @brief Initialize all BMS hardware: clocks, timers, pins, ADC, BQ7693, LEDs, EEPROM, UART. */
void bms_init(void)
{
  //sets up clocks/IRQ handlers etc.
  system_init();
  //Initialise the sw_timer
  delay_init();
  sw_timer_init();
  dsn_prot_init();

  //Set up the pins
  pins_init();
  dio_init();

  bms_adc_init();
  //BQ7693 init
  bq7693_init();

  //Init the LEDs
  leds_init();
  //Init eeprom emulator
  /* eeprom_init() already reads the page, verifies its CRC and falls back to
     defaults if it does not check out, so eeprom_data is populated either
     way - a second read here was redundant. */
  eeprom_init();

  //Initialise the USART we need to talk to the vacuum cleaner
  serial_init();

  //Enable interrupts
  interrupts_init();

  //Initialise RTC for standby wakeup (one-time config)
  rtc_standby_timer_init();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  serial_debug_init();
#endif
}

/** @brief External interrupt callback for the standby wake sources. */
void bms_wakeup_interrupt_callback(void)
{
  /* The edge itself is what wakes the core. This flag exists for the window
     between arming the wake sources and executing WFI, where the ISR would
     otherwise consume the edge and the core would then sleep through it. */
  bms_wake_event = true;
}

/** @brief BQ7693 ALERT pin interrupt callback, sets processing flag. */
void bms_interrupt_callback(void)
{
  process_bms_interrupt = true;
}

/** @brief Process pending BQ7693 interrupt: read coulomb counter and update charge level. */
void bms_interrupt_process(void)
{
  uint8_t sys_stat;

  /* Serviced on the ALERT edge, and periodically in case the edge can no
     longer arrive because a fault bit is holding ALERT high. */
  if ((false == process_bms_interrupt) &&
      (false == sw_timer_is_elapsed(&bms_sys_stat_timer, SYS_STAT_POLL_MS)))
  {
    return;
  }

  sw_timer_start(&bms_sys_stat_timer);
  process_bms_interrupt = false;

  if (!bq7693_read_register(SYS_STAT, 1, &sys_stat))
  {
    return;
  }

  /* Latch and clear every fault bit. Clearing is what allows ALERT to fall
     and the next edge to be seen; the bits are handed to the safety checks
     through bms_sys_stat_take() so nothing is lost by clearing them here. */
#ifdef SERIAL_DEBUG
  /*
   * Report the fault bits only when the set of them changes. A latched
   * condition re-arms this every poll, and at SYS_STAT_POLL_MS that flood
   * buried everything else in the log. Nothing is lost: each safety check
   * still prints its own "SYS_STAT faults=" line with the value it acted on,
   * and clearing back to zero is tracked here too, so a condition that goes
   * away and returns is reported again.
   */
  {
    static uint8_t last_reported = 0u;
    uint8_t flags = (uint8_t)(sys_stat & STAT_FLAGS);

    if (flags != last_reported)
    {
      last_reported = flags;

      if (flags != 0u)
      {
        BMS_PRINT("BMS:SYS_STAT=0x%02X\r\n", sys_stat);
      }
    }
  }
#endif

  if (sys_stat & STAT_FLAGS)
  {
    bms_sys_stat_faults |= (uint8_t)(sys_stat & STAT_FLAGS);
    bq7693_write_register(SYS_STAT, (uint8_t)(sys_stat & STAT_FLAGS));
  }

  if (sys_stat & STAT_CC_READY)
  {
    int16_t cc_raw;
    int32_t ccVal;

    /*
     * If the CC read itself fails, leave CC_READY set and come back. The
     * window is still in the register, ALERT stays high, and the next poll
     * (SYS_STAT_POLL_MS, shorter than the 250ms window) reads it before the
     * following conversion overwrites it. Clearing the bit regardless, as
     * this used to, integrated the window as a silent 0mA.
     */
    if (!bq7693_read_cc(&cc_raw))
    {
      return;
    }
    ccVal = cc_raw;

    //This needs better handling....
    current_mA = (ccVal * (uint16_t)(BQ7693_CC_LSB_MA * 4096.0f)) / 4096;

    /*
     * First-order IIR:  sum += alpha * (x - sum>>FRAC),  filtered = sum>>FRAC
     *
     * This used a Q16 accumulator, which can only represent a filtered
     * current of +-32767mA before sum overflows int32 - and it truncated
     * both the input and the feedback term to int16_t on the way in. At
     * BQ7693_CC_LSB_MA per CC LSB the coulomb counter reaches +-276A, and a vacuum
     * motor inrush passes 32.7A easily, at which point the difference wraps
     * and the filter runs away. Q8 leaves 1/256 mA of resolution on the
     * output - far more than the 8.44mA input step - while keeping the
     * accumulator and the alpha*delta product inside int32 across the
     * counter's whole range.
     */
    #define FILT_MS         (500ul)
    #define PERIOD_MS       (250ul)
    #define FILT_FRAC_BITS  (8)
    #define FILT_ALPHA      ((int32_t)(((1L << FILT_FRAC_BITS) * (long)PERIOD_MS) / (long)FILT_MS))

    current_filt_sum_mA += FILT_ALPHA * (current_mA - (current_filt_sum_mA >> FILT_FRAC_BITS));
    current_filt_mA = current_filt_sum_mA >> FILT_FRAC_BITS;

    //Ignore tiny values.
    //if ( (ccVal > 0 && ccVal > 2)  || (ccVal < 0 && ccVal < -2) )
    {
      int32_t cc_uah;
      //i = V/R
      //sense resistor = 1mOhm
      //microV / milliOhms gives current in mA.
      //so ccVal has current in mA.
      //Dividing by 14400 would give mAH. (number of 250mS periods in 1 hr.
      //Dividing by 14.4 will give microAH (what we want)
      // 14.4 = ((3600 * 1000) / 250ms) / 1000mAh
      cc_uah = ccVal * (int16_t)(((BQ7693_CC_LSB_MA * BQ7693_CC_PERIOD_MS * 32768.0f) / (3600.0f)));
      cc_uah /= 32768;
      eeprom_data.current_charge_level += cc_uah;

      /*
       * Clamp to the physical upper bound, NOT to the learned capacity.
       * Clamping to total_pack_capacity meant the level could never exceed
       * the stored figure, so the learning step below - which assigns the
       * level to the capacity - could only ever lower it. One bad cycle
       * ratcheted the gauge down permanently with no path back.
       * bms_get_soc_x100() already clamps the reported percentage at 100%,
       * so a level briefly above the learned capacity mid-learn is harmless.
       */
      if (eeprom_data.current_charge_level > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH)
        eeprom_data.current_charge_level = (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH;
      if (eeprom_data.current_charge_level < 0)
        eeprom_data.current_charge_level = 0;
    }
    //Update the CC bit so it'll refire in another 250mS as per datasheet.
    bq7693_write_register(SYS_STAT, STAT_CC_READY);//Clear CC bit.
  }
}

/**
 * @brief Read SYS_STAT, latch any fault bits and clear them on the chip.
 *
 * Called by the safety checks so that a fault which arrived since the last
 * service is picked up immediately rather than waiting for the next poll.
 */
static void bms_sys_stat_service(void)
{
  uint8_t sys_stat;

  if (!bq7693_read_register(SYS_STAT, 1, &sys_stat))
  {
    return;
  }

  if (sys_stat & STAT_FLAGS)
  {
    bms_sys_stat_faults |= (uint8_t)(sys_stat & STAT_FLAGS);
    bq7693_write_register(SYS_STAT, (uint8_t)(sys_stat & STAT_FLAGS));
  }
}

/** @brief Return every fault bit seen since the last call, and clear the latch. */
static uint8_t bms_sys_stat_take(void)
{
  uint8_t faults;

  system_interrupt_enter_critical_section();
  faults = bms_sys_stat_faults;
  bms_sys_stat_faults = 0u;
  system_interrupt_leave_critical_section();

  return faults;
}

/**
 * @brief Clear latched faults through the owner, then drive the charge FET.
 *
 * The one place a charge enable goes through, so the result cannot be
 * dropped. A failed enable latches the comm error, but the next safety check
 * clears that latch before its own reads - so if the bus recovers in between,
 * a caller that ignored this return would sit in BMS_CHARGING with the FET
 * off and nothing would ever notice.
 *
 * @return false if the FET could not be driven.
 */
static bool bms_charge_fet_on(void)
{
  /* about to (re)assert it anyway, so a pending drop is dealt with */
  bms_afe_fet_dropped_flag = false;
  bms_sys_stat_service();

  if (!bq7693_enable_charge())
  {
    BMS_PRINT("BMS:CHG_ENABLE_FAILED\r\n");
    return false;
  }

#ifdef SERIAL_DEBUG
  /*
   * bq7693_enable_charge() reports whether the WRITE was accepted, not whether
   * the FET came on: the AFE can ACK the transfer and leave CHG_ON clear, and
   * it drops the bit itself for a latched fault. Read both registers back so
   * the log says which of the two happened. Expect CTRL2=0x41 (CC_EN|CHG_ON)
   * and STAT=0x00.
   */
  {
    uint8_t ctrl2 = 0u;
    uint8_t stat  = 0u;

    (void)bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
    (void)bq7693_read_register(SYS_STAT,  1, &stat);
    BMS_PRINT("BMS:CHG_FET readback CTRL2=0x%02X STAT=0x%02X\r\n",
              (unsigned)ctrl2, (unsigned)stat);
  }
#endif

  return true;
}

/** @brief As bms_charge_fet_on(), for the discharge FET. */
static bool bms_discharge_fet_on(void)
{
  bms_afe_fet_dropped_flag = false;
  bms_sys_stat_service();

  if (!bq7693_enable_discharge())
  {
    BMS_PRINT("BMS:DSG_ENABLE_FAILED\r\n");
    return false;
  }

  return true;
}

/** @brief True once if an absorbed AFE event has dropped the FETs since the last enable. */
static bool bms_afe_fet_dropped(void)
{
  bool dropped = bms_afe_fet_dropped_flag;

  bms_afe_fet_dropped_flag = false;
  return dropped;
}

/**
 * @brief Get state of charge as percent * 100 for vacuum protocol.
 * @return SOC in 0.01% units (100-10000), minimum 1% to avoid critical battery screen.
 */
uint16_t bms_get_soc_x100(void)
{
  uint16_t soc = 100;
  int32_t current_charge_level = eeprom_data.current_charge_level;
  /* int32_t, not int16_t: the scaled capacity is 4218 for this pack but
     narrowing here silently wraps negative for anything above ~33Ah, which
     would make the whole SOC calculation fall through to the 1% floor. */
  int32_t total_pack_capacity  = eeprom_data.total_pack_capacity  >> 10;

  if(total_pack_capacity > 0 && current_charge_level > 0)
  {
    /*
     * soc_x100 = level * 10000 / capacity, with capacity pre-scaled by >>10.
     * The exact factor is 10000/1024 = 9.765625; ROUND() produced
     * (uint16_t)(9.7656 + 0.5) = 10, a flat +2.4% over-read at full charge.
     * 10000/1024 == 625/64 exactly, so shift the level down by 6 first and
     * the whole thing stays integer and inside int32:
     *   4.32e6 >> 6 = 67500, * 625 = 4.2e7, well under 2.1e9.
     * The >>6 discards at most 63uAh of a 3.6Ah pack - 0.0015%.
     */
    soc = (uint16_t)(((current_charge_level >> 6) * 625) / total_pack_capacity);
    soc = (soc > 10000) ? 10000 : ((soc == 0) ? 100 : soc);
  }

  return soc;
}

/**
 * @brief Estimate remaining runtime based on filtered current.
 * @return seconds, 0 when idle, minimum 60 during discharge.
 */
uint32_t bms_get_runtime_seconds(void)
{
  int32_t current_filt_mA_abs = abs(current_filt_mA);
  int32_t current_charge_level;
  int32_t runtime = 0;

  if(    bms_state == BMS_VACUUM_RUNNING // estimate only while the motor is actually running
      && current_filt_mA_abs > 1000)     // and current is > 1A, to keep the result bounded
  {
    /* clamp to [0, PACK_CAPACITY_UPPER_BOUND_UAH] - the same bound the coulomb
       counter is allowed to reach. Clamping at the nominal figure instead cut
       the runtime estimate short for any pack that learned above nominal. */
    current_charge_level = eeprom_data.current_charge_level < 0 ? 0
                         : eeprom_data.current_charge_level > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH ? (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH
                         : eeprom_data.current_charge_level;

    runtime = ((current_charge_level / current_filt_mA_abs) * (uint16_t)((3600.0f / 1000.0f) * 1024.0f)) >> 10;

    // always limit runtime to 1 minute
    runtime = runtime < 60 ? 60 : runtime;
  }

  return (uint32_t)runtime;
}

/** @brief Main BMS state machine loop (never returns). */
void bms_mainloop(void)
{
  bms_wdt_init();
  //Handle the state machinery.
  while (1)
  {
    BMS_PRINT("BMS_STATE: %s\r\n", bms_state_names[bms_state]);

    switch (bms_state)
    {
    //-----------------------------------------------------------------------
      case BMS_INIT:
        bms_state = BMS_IDLE;
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_message("Dyson V11/V15 BMS After market firmware " FW_GIT_REV "\r\n");
        /*
         * A physically attached debugger keeps SWDIO driven, which
         * forward-biases that pin's ESD clamp into the VDD rail once REGOUT
         * goes and parks it near 600mV - not enough to run the MCU, enough to
         * stop it powering down cleanly. The pack then looks dead: silent,
         * still reachable over SWD, unwakeable by button or charger. It cost
         * two separate rounds of chasing a firmware bug that was a probe lead,
         * so it is worth a line in the log.
         *
         * Read what DBGPRES actually is, though. It latches when the debug
         * interface is powered up and stays latched until the part loses power
         * - a reset, including the one a programmer issues after flashing,
         * does not clear it. So it reports "a debugger has been attached since
         * this pack was last powered up", NOT "a debugger is attached now",
         * and the wording has to match or it sends people looking for a cable
         * they already unplugged.
         *
         * That is also why nothing is gated on it. Refusing SHIP here looked
         * sensible for about ten minutes and then stopped a pack from ever
         * sleeping again after a single flash.
         */
        if (DSU->STATUSB.bit.DBGPRES)
        {
          serial_debug_send_message("BMS:DEBUGGER_SEEN since power-up - if still plugged in, sleep will misbehave\r\n");
        }
        /* eeprom_init() runs before serial_debug_init(), which clears the
           queue, so it cannot report this itself - see eeprom_was_reset(). */
        if (eeprom_was_reset())
        {
          serial_debug_send_message("BMS:EEPROM_RESET_TO_DEFAULTS\r\n");
        }
#endif
        leds_sequence();
        wdt_reset_count();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_cell_voltages();
        bms_debug_dump_temps("boot");
        serial_debug_send_pack_capacity();
#endif
        wdt_reset_count();
      break;
      //-----------------------------------------------------------------------
      case BMS_IDLE:
        bms_handle_idle();
      break;
      //-----------------------------------------------------------------------
      case BMS_SLEEP:
        bms_handle_sleep();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_CONNECTED:
        bms_handle_charger_connected();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGING:
        bms_handle_charging();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_CONNECTED_NOT_CHARGING:
        bms_handle_charger_connected_not_charging();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_UNPLUGGED:
        bms_handle_charger_unplugged();
      break;
      //-----------------------------------------------------------------------
      case BMS_VACUUM_RUNNING:
        bms_handle_vacuum_running();
      break;
      //-----------------------------------------------------------------------
      case BMS_FAULT:
        bms_handle_fault();
      break;
      //-----------------------------------------------------------------------
      default:
      break;
    }


    dio_mainloop();
    dsn_prot_mainloop();
    bms_interrupt_process();
    bms_wdt_mainloop();
    /* Drains one byte per call. Without it here the queue only moved inside
       sw_timer_delay_ms(), so a state handler that returns promptly - INIT,
       CHARGER_CONNECTED, CHARGER_UNPLUGGED - could leave its own log lines
       sitting in the buffer until something else happened to block. */
    serial_debug_process();
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
    Passive cell balancing
-----------------------------------------------------------------------------*/
#if CELL_BALANCE_ENABLE

static sw_timer                bms_balance_timer = 0;
static sw_timer                bms_balance_led_timer = 0;
static bq7693_balance_status_t bms_balance_status;
static uint8_t                 bms_balance_last_mask  = 0;
static bq7693_balance_state_t  bms_balance_last_state = BQ_BALANCE_IDLE;

/** @brief Turn balancing off and re-arm the tick so the next call evaluates immediately. */
static void bms_balance_stop(void)
{
  bq7693_disable_balancing();
  bms_balance_status.state = BQ_BALANCE_IDLE;
  bms_balance_status.cell_mask = 0;
  bms_balance_last_mask    = 0;
  bms_balance_last_state   = BQ_BALANCE_IDLE;
  sw_timer_stop(&bms_balance_timer);

  //Park the LEDs too - callers either redraw immediately (charging breathe,
  //fault blink code) or leave them off, and this stops a half-finished
  //alternation being left lit on the way out to BMS_IDLE.
  sw_timer_stop(&bms_balance_led_timer);
  leds_off();
}

/**
 * @brief Periodic balancing tick.
 *
 * Only call with the charge FET OFF and the cells given CELL_BALANCE_RELAX_MS to
 * settle - while charging, the measured spread is mostly IR drop across the cell
 * interconnects and the balancer would chase noise.
 *
 * @return true while one or more cells are actively bleeding.
 */
static bool bms_balance_tick(void)
{
  if (!sw_timer_is_elapsed(&bms_balance_timer, CELL_BALANCE_PERIOD_MS))
    return (bms_balance_status.state == BQ_BALANCE_ACTIVE);

  sw_timer_start(&bms_balance_timer);

  //Balancing dumps the imbalance as heat inside the pack, unattended, for hours
  //at a time - so it respects the charge temperature ceiling too. Checked here
  //rather than in bq7693_balance_update() because the NTC is not the AFE's.
  bms_sample_temperatures();

  if ((bms_temperature_hottest() / 10) >= CELL_BALANCE_MAX_TEMP)
  {
    bq7693_disable_balancing();
    bms_balance_status.state     = BQ_BALANCE_TOO_HOT;
    bms_balance_status.cell_mask = 0;
    bms_balance_status.num_cells = 0;
  }
  else
  {
    bq7693_balance_update(&bms_balance_status);
  }

  // Only log on a change, otherwise a multi-hour balance floods the debug queue.
  if (   bms_balance_status.state     != bms_balance_last_state
      || bms_balance_status.cell_mask != bms_balance_last_mask)
  {
    switch (bms_balance_status.state)
    {
      case BQ_BALANCE_ACTIVE:
        BMS_PRINT("BMS:BAL n=%u m=0x%02X d=%umV lo=%u hi=%u\r\n",
                  bms_balance_status.num_cells, bms_balance_status.cell_mask,
                  bms_balance_status.spread_mv,
                  bms_balance_status.v_min_mv,  bms_balance_status.v_max_mv);
        break;

      case BQ_BALANCE_IDLE:
        BMS_PRINT("BMS:BAL_DONE d=%umV\r\n", bms_balance_status.spread_mv);
        break;

      case BQ_BALANCE_CELL_FAIL:
        // A cell will not charge up to meet the rest. Bleeding the healthy cells
        // down to it would waste most of the pack, so we refuse and say so.
        BMS_PRINT("BMS:BAL_CELL_FAIL d=%umV lo=%umV hi=%umV\r\n",
                  bms_balance_status.spread_mv,
                  bms_balance_status.v_min_mv, bms_balance_status.v_max_mv);
        break;

      case BQ_BALANCE_OV:
        BMS_PRINT("BMS:BAL_OV c%u=%umV\r\n",
                  bms_balance_status.max_cell, bms_balance_status.v_max_mv);
        break;

      case BQ_BALANCE_TOO_HOT:
        BMS_PRINT("BMS:BAL_HOT t=%dC max=%dC\r\n",
                  (int)(bms_temperature_hottest() / 10), (int)CELL_BALANCE_MAX_TEMP);
        break;

      case BQ_BALANCE_TOO_LOW:
      default:
        break;
    }

    bms_balance_last_state = bms_balance_status.state;
    bms_balance_last_mask  = bms_balance_status.cell_mask;
  }

  return (bms_balance_status.state == BQ_BALANCE_ACTIVE);
}

/**
 * @brief Non-blocking LED indication for the balancing states.
 *
 * Must not use leds_blink_*() - those all spin in sw_timer_delay_ms() and would
 * stall the balancing supervision. The pattern position is derived from elapsed
 * time rather than a step counter, so it looks the same whatever rate the
 * caller happens to poll at.
 *
 * ACTIVE    - LEDs alternate left<->right. Every other pattern in this firmware
 *             drives both LEDs together, so this cannot be mistaken for one.
 * CELL_FAIL - three fast blinks then a pause: a cell will not come up to meet
 *             the others and the pack needs looking at.
 */
static void bms_balance_leds(void)
{
  const uint32_t alt = CELL_BALANCE_LED_ALT_MS;
  uint32_t t;
  bool     left;

  if (!sw_timer_is_started(&bms_balance_led_timer))
    sw_timer_start(&bms_balance_led_timer);

  t = (uint32_t)sw_timer_get_elapsed_time(&bms_balance_led_timer);

  switch (bms_balance_status.state)
  {
    case BQ_BALANCE_OV:      //one cell being bled down from the guard: same gesture
    case BQ_BALANCE_ACTIVE:
      //Steady left<->right sweep at reduced brightness.
      left = (((t / alt) & 1ul) == 0ul);
      leds_set_led_duty(LEDS_LED_ERR_LEFT,  left ? CELL_BALANCE_LED_DUTY : 0);
      leds_set_led_duty(LEDS_LED_ERR_RIGHT, left ? 0 : CELL_BALANCE_LED_DUTY);
      break;

    case BQ_BALANCE_CELL_FAIL:
    {
      //Same sweep so it still reads as "balancing", but at full brightness and
      //broken by a blackout every three sweeps - "a cell will not come up".
      //Deliberately not a blink-count pattern: that would be read as a
      //BMS_FAULT error code.
      uint32_t ph = t % (alt * 8ul);

      if (ph < (alt * 6ul))
      {
        left = (((ph / alt) & 1ul) == 0ul);
        leds_set_led_duty(LEDS_LED_ERR_LEFT,  left ? 100 : 0);
        leds_set_led_duty(LEDS_LED_ERR_RIGHT, left ? 0 : 100);
      }
      else
      {
        leds_off();
      }
      break;
    }

    default:
      leds_off();
      break;
  }
}

/** @brief True once a cell has passed CELL_BALANCE_OV_GUARD_MV - charging must stop. */
static bool bms_balance_overvoltage(void)
{
  return (bms_balance_status.state == BQ_BALANCE_OV);
}

#else /* !CELL_BALANCE_ENABLE */
#define bms_balance_stop()         do { } while (0)
#define bms_balance_leds()         do { } while (0)
#define bms_balance_tick()         (false)
#define bms_balance_overvoltage()  (false)
#endif

/** @brief Set bms_error to code only if code is more severe than the current error. */
static void bms_set_error(enum BMS_ERROR_CODE code)
{
  if (bms_error < code)
    bms_error = code;
}

/**  @brief Trigger state */
static bool bms_trigger_active(void)
{
#if (TRIGGER_TOGGLE_MODE == 1)
  static bool     latched    = false;
  static uint8_t  prev_level = 0;
  static sw_timer held_timer = 0;

  uint8_t level = dio_read(DIO_TRIGGER_PRESSED);

  /* See mode 2 below - a latch outlives the session that justified it. */
  if (!dsn_prot_get_vacuum_connected())
  {
    latched = false;
  }

  if (level && !prev_level) // rising edge
  {
    latched = !latched;
    sw_timer_start(&held_timer);
  }
  else if (level && prev_level)
  {
    if (latched && sw_timer_is_elapsed(&held_timer, TRIGGER_HOLD_MS))
      latched = false;
  }
  prev_level = level;

  return latched;

#elif (TRIGGER_TOGGLE_MODE == 2)
  /*
   * Short press toggles, long press is the stock trigger.
   *
   * The decision is taken on RELEASE, not on the press, because until the
   * trigger comes back up there is no telling which of the two it was. What
   * cannot wait is the motor: the return below is latched OR physically held,
   * so a press acts immediately either way and only its ending differs.
   *
   *   tap        -> the latch flips. Running stays running after release,
   *                 stopped starts and keeps going.
   *   press-hold -> runs while held and stops on release, exactly like mode 0.
   *                 It also always leaves the latch clear, which makes a long
   *                 press the way out of a latch that was set by accident -
   *                 the same gesture whether or not you remember the state.
   */
  static bool     latched    = false;
  static bool     was_long   = false;
  static uint8_t  prev_level = 0;
  static sw_timer held_timer = 0;

  uint8_t level = dio_read(DIO_TRIGGER_PRESSED);

  /*
   * A latch only means anything while there is a cleaner listening. Sessions
   * do not always end with the trigger being released - PROT:MS_WDT ends one
   * when the cleaner simply stops talking - and a latch left set through that
   * is reported as a held trigger for ever after. The pack then bounces
   * between idle and running every time the cleaner announces itself, never
   * notices a charger being docked because the running state does not look
   * for one, and starts the motor unasked the moment a cleaner reconnects.
   */
  if (!dsn_prot_get_vacuum_connected())
  {
    latched = false;
  }

  if (level && !prev_level)          /* pressed */
  {
    sw_timer_start(&held_timer);
    was_long = false;
  }
  else if (!level && prev_level)     /* released */
  {
    latched = was_long ? false : !latched;
  }
  else if (level)                    /* still held */
  {
    if (sw_timer_is_elapsed(&held_timer, TRIGGER_HOLD_MS))
    {
      was_long = true;
    }
  }
  prev_level = level;

  return (latched || (level != 0u));

#else
  return dio_read(DIO_TRIGGER_PRESSED);
#endif
}

/**
 * @brief Force the state machine into BMS_FAULT with the given error code.
 *
 * Called from interrupt context (the watchdog early warning). The two writes
 * are separate, so the ORDER matters: bms_error first, bms_state second. A
 * reader that observes BMS_FAULT must already be able to see the code that
 * goes with it, otherwise bms_handle_fault() blinks a stale or empty one.
 * Both are volatile, so the compiler may not reorder them relative to each
 * other, and this is a single-core M0+ with one reader (the main loop), so no
 * barrier is needed - but do not swap these two lines.
 */
void bms_force_fault(enum BMS_ERROR_CODE code)
{
  bms_error = code;
  bms_state = BMS_FAULT;
}

/**
 * @brief Decide whether an AFE-reported fault should fault the pack.
 *
 * DEVICE_XREADY is an internal chip fault and OVRD_ALERT means something
 * external drove the ALERT pin; the AFE turns both FET drivers off for either.
 * But the datasheet expects transients on both - XREADY "may be set due to
 * excessive system transients", and ALERT has no internal debounce - so the
 * first few are absorbed and only a persistent condition is escalated.
 *
 * @return true if the pack should fault.
 */
static bool bms_afe_fault_is_real(uint8_t sys_stat, const char *who)
{
  if ((sys_stat & (STAT_DEVICE_XREADY | STAT_OVRD_ALERT)) == 0u)
  {
    return false;
  }

  /*
   * Forget earlier events once the AFE has been quiet for a while. Without
   * this the allowance is spent once for the life of the power cycle: after
   * the third transient every later one faults at once, so a chip that
   * glitches occasionally would still strand the pack eventually, and a
   * fault/recovery cycle would inherit an exhausted counter and re-fault
   * immediately.
   */
  if (sw_timer_is_elapsed(&bms_afe_fault_timer, BQ_AFE_FAULT_DECAY_MS))
  {
    bms_afe_fault_count = 0u;
  }
  sw_timer_start(&bms_afe_fault_timer);

  if (bms_afe_fault_count < BQ_AFE_FAULT_TOLERANCE)
  {
    bms_afe_fault_count++;
    /* the AFE turned both FETs off for this - whoever owns one must
       re-assert it, see bms_afe_fet_dropped() */
    bms_afe_fet_dropped_flag = true;
    BMS_PRINT("%s: AFE fault 0x%02X absorbed %u/%u\r\n", who,
              (unsigned)(sys_stat & (STAT_DEVICE_XREADY | STAT_OVRD_ALERT)),
              bms_afe_fault_count, BQ_AFE_FAULT_TOLERANCE);
    return false;
  }

  BMS_PRINT("%s: AFE fault 0x%02X persistent\r\n", who,
            (unsigned)(sys_stat & (STAT_DEVICE_XREADY | STAT_OVRD_ALERT)));
  return true;
}

/**
 * @brief Blink a fault code as a uniform run of pulses.
 *
 * Pulse LENGTH carries the class, pulse COUNT carries the code within it:
 *
 *   self-recovering (1..BMS_ERR_SHORTCIRCUIT)  short pulses, count 1..4
 *   needs attention (above that)               long pulses,  count 1..6
 *
 * Every pattern is uniform, so there is only ever one thing to count, and the
 * longest run is six. Ten identical blinks - the previous scheme - cannot be
 * counted by eye at all.
 */
static void bms_blink_error_code(enum BMS_ERROR_CODE code)
{
  const bool     recoverable = (code <= BMS_ERR_SHORTCIRCUIT);
  const uint32_t on_ms       = recoverable ? FAULT_BLINK_SHORT_MS : FAULT_BLINK_LONG_MS;
  const uint8_t  count       = recoverable ? (uint8_t)code
                                           : (uint8_t)(code - BMS_ERR_SHORTCIRCUIT);
  uint8_t i;

  for (i = 0u; i < count; i++)
  {
    leds_on();
    sw_timer_delay_ms(on_ms);
    leds_off();
    sw_timer_delay_ms(FAULT_BLINK_GAP_MS);
    wdt_reset_count();
  }

  leds_off();
}

/**
 * @brief True if something outside the current handler has demanded a fault.
 *
 * bms_force_fault() is called from the watchdog early-warning interrupt, but
 * every state handler runs its own while(1) and none of them re-read
 * bms_state, so the demand used to be overwritten and lost when the handler
 * eventually returned. The long-running loops poll this and bail out.
 */
static bool bms_fault_pending(void)
{
  return (bms_state == BMS_FAULT);
}

/** @brief Configure GPIO pins for charge control, sense inputs, and precharge. */
static void pins_init(void)
{
  //Set up the output charge pin

  struct port_config charge_pin_config;
  port_get_config_defaults(&charge_pin_config);
  /* WTH_READBACK leaves the input buffer enabled, so PORT.IN reports what the
     pin is actually AT rather than what we are driving it to. That difference
     is the whole point of bms_debug_dump_pins(): a pin driven high that reads
     back low is being held down by something external. Costs a little leakage
     and nothing else. */
  charge_pin_config.direction = PORT_PIN_DIR_OUTPUT_WTH_READBACK;
  port_pin_set_config(ENABLE_CHARGE_PIN, &charge_pin_config);
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  //Two input pins, CHARGER and TRIGGERs
  struct port_config sense_pin_config;
  port_get_config_defaults(&sense_pin_config);
  sense_pin_config.direction = PORT_PIN_DIR_INPUT;
  sense_pin_config.input_pull = PORT_PIN_PULL_NONE;
  port_pin_set_config(CHARGER_CONNECTED_PIN, &sense_pin_config);
  port_pin_set_config(TRIGGER_PRESSED_PIN, &sense_pin_config);


  struct port_config io_pin_config;
  port_get_config_defaults(&io_pin_config);
  io_pin_config.direction = PORT_PIN_DIR_OUTPUT_WTH_READBACK;   //see above

  // pack voltage feedback
  port_pin_set_config(PIN_PA03, &io_pin_config);
  port_pin_set_output_level(PIN_PA03, true);

  // mode pin pullup voltage
  port_pin_set_config(MODE_BUTTON_PULLUP_ENABLE_PIN, &io_pin_config);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, true);

  // precharge
  port_pin_set_config(PRECHARGE_PIN, &io_pin_config);
  port_pin_set_output_level(PRECHARGE_PIN, false);

  // unknown functionality pin - see UNKNOWN_PA25_DRIVE_HIGH
#if UNKNOWN_PA25_DRIVE_HIGH
  port_pin_set_config(PIN_PA25, &io_pin_config);
  port_pin_set_output_level(PIN_PA25, true);
#else
  /* Left as an input, but configured as one: a pin that has never been through
     port_pin_set_config() has its input buffer off and reads 0 in PORT.IN no
     matter what it is sitting at, so bms_debug_dump_pins() reported PA25 low
     unconditionally. It in fact sits high in every state, through a high
     impedance, and does not follow ENABLE_CHARGE_PIN - so whatever it carries,
     it is not the charge-path control this pin was suspected of being. */
  port_pin_set_config(PIN_PA25, &sense_pin_config);
#endif

  // mode button
  port_pin_set_config(MODE_BUTTON_PIN, &sense_pin_config);
}

/** @brief De-initialize GPIO outputs before entering sleep. */
static void pins_deinit(void)
{
#if UNKNOWN_PA25_DRIVE_HIGH
  port_pin_set_output_level(PIN_PA25, false);
#endif
  port_pin_set_output_level(PIN_PA03, false);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
  port_pin_set_output_level(PRECHARGE_PIN, false);
}

/**
 * @brief Report both thermistors, plus the raw charge-path feedback.
 *
 * Only TC1 (PA07) feeds the over/under-temperature interlocks. TC2 (PA08) sits
 * against the cells and is read for comparison only - until the two have been
 * watched across a real temperature range there is no basis for letting it
 * gate anything.
 *
 * Temperatures print in 0.1C rather than whole degrees: the question being
 * answered is whether the two agree, and a one-degree print cannot show a
 * half-degree disagreement.
 *
 * CHG_FB prints as a raw conversion, deliberately - running it through the NTC
 * curve produced a number that looked like a room temperature and was not one.
 *
 * @param who  Short tag identifying the call site.
 */
static void bms_debug_dump_temps(const char *who)
{
#ifdef SERIAL_DEBUG
  int16_t t1 = NTC_ADC2Temperature(adc_convert_channel(BMS_ADC_CH_TC1));
  int16_t t2 = NTC_ADC2Temperature(adc_convert_channel(BMS_ADC_CH_TC2));
  uint16_t fb = adc_convert_channel(BMS_ADC_CH_CHG_FB);

  BMS_PRINT("TEMP(%s) PA07=%s%d.%d PA08=%s%d.%d 'C  CHG_FB=%u\r\n", who,
            T_SIGN(t1), T_WHOLE(t1), T_FRAC(t1),
            T_SIGN(t2), T_WHOLE(t2), T_FRAC(t2), fb);
#else
  (void)who;
#endif
}

/**
 * @brief Dump the state of every PORT A pin to the debug UART.
 *
 * Two lines. The first carries everything: DIR says which pins are outputs,
 * OUT what we are driving them to, IN what they are actually sitting at. An
 * output whose OUT and IN bits disagree is being fought by something external
 * - that is the case this exists to catch.
 *
 * The second decodes the handful that matter for the charge path, so the
 * common case needs no bit counting.
 *
 * Deliberately short: the debug queue is DEBUG_QUEUE_SIZE (256) bytes and
 * drains one byte per call, so a per-pin listing would overflow it and lose
 * its own tail.
 *
 * @param who  Short tag identifying the call site.
 */
static void bms_debug_dump_pins(const char *who)
{
#ifdef SERIAL_DEBUG
  const uint32_t dir = PORT->Group[0].DIR.reg;
  const uint32_t out = PORT->Group[0].OUT.reg;
  const uint32_t in  = PORT->Group[0].IN.reg;

  #define PIN_DRV(p)  ((unsigned)((out >> (p)) & 1ul))
  #define PIN_LVL(p)  ((unsigned)((in  >> (p)) & 1ul))

  BMS_PRINT("PINS(%s) DIR=%08lX OUT=%08lX IN=%08lX\r\n", who,
            (unsigned long)dir, (unsigned long)out, (unsigned long)in);
  BMS_PRINT("PINS CHG_EN=%u/%u CHARGER=%u TRIG=%u PRECHG=%u PA25=%u/%u ALERT=%u\r\n",
            PIN_DRV(1), PIN_LVL(1),      /* PA01 ENABLE_CHARGE - drive/actual */
            PIN_LVL(6),                  /* PA06 CHARGER_CONNECTED */
            PIN_LVL(4),                  /* PA04 TRIGGER_PRESSED */
            PIN_LVL(24),                 /* PA24 PRECHARGE */
            PIN_DRV(25), PIN_LVL(25),    /* PA25 unknown */
            PIN_LVL(28));                /* PA28 BQ7693 ALERT */

  #undef PIN_DRV
  #undef PIN_LVL
#else
  (void)who;
#endif
}

/** @brief Configure EIC for BQ7693 ALERT, mode button, trigger, and charger pins. */
static void interrupts_init(void)
{
  //A single interrupt, focused on the BQ7693's alert line (PA28), which
  //is on EXTINT 8.
  struct extint_chan_conf config_alert_pin;
  extint_chan_get_config_defaults(&config_alert_pin);
  config_alert_pin.wake_if_sleeping = false;
  config_alert_pin.gpio_pin     = PIN_PA28A_EIC_EXTINT8;
  config_alert_pin.gpio_pin_mux = MUX_PA28A_EIC_EXTINT8;
  //This line is designed to be possible for either device to pull it up or down to indicate a fault condition, so
  //no pullups.
  config_alert_pin.gpio_pin_pull      = EXTINT_PULL_NONE;
  config_alert_pin.detection_criteria = EXTINT_DETECT_RISING;

  extint_chan_set_config(8, &config_alert_pin);
  extint_register_callback(bms_interrupt_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // mode pin interrupt generation, used to exit from standby mode - rising edge
  //is on EXTINT 9 - PA09
  struct extint_chan_conf config_mode_pin;
  extint_chan_get_config_defaults(&config_mode_pin);

  config_mode_pin.wake_if_sleeping    = true;
  config_mode_pin.filter_input_signal = true;
  config_mode_pin.gpio_pin            = PIN_PA09A_EIC_EXTINT9;
  config_mode_pin.gpio_pin_mux        = MUX_PA09A_EIC_EXTINT9;
  config_mode_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_mode_pin.detection_criteria  = EXTINT_DETECT_RISING;

  extint_chan_set_config(9, &config_mode_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 9, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // trigger pin, can also wakeup the system, rising edge
  // is on EXTINT 4 - PA04
  struct extint_chan_conf config_trigger_pin;
  extint_chan_get_config_defaults(&config_trigger_pin);

  config_trigger_pin.wake_if_sleeping    = true;
  config_trigger_pin.filter_input_signal = true;
  config_trigger_pin.gpio_pin            = PIN_PA04A_EIC_EXTINT4;
  config_trigger_pin.gpio_pin_mux        = MUX_PA04A_EIC_EXTINT4;
  config_trigger_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_trigger_pin.detection_criteria  = EXTINT_DETECT_RISING;

  extint_chan_set_config(4, &config_trigger_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 4, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // charger connected pin, can also wakeup the system, both edges will wakeup the system
  // is on EXTINT 6 - PA06
  struct extint_chan_conf config_charger_pin;
  extint_chan_get_config_defaults(&config_charger_pin);

  config_charger_pin.wake_if_sleeping    = true;
  config_charger_pin.filter_input_signal = true;
  config_charger_pin.gpio_pin            = PIN_PA06A_EIC_EXTINT6;
  config_charger_pin.gpio_pin_mux        = MUX_PA06A_EIC_EXTINT6;
  config_charger_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_charger_pin.detection_criteria  = EXTINT_DETECT_BOTH;

  extint_chan_set_config(6, &config_charger_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 6, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);

  //Enable interrupts.
  system_interrupt_enable_global();
}

/**
 * @brief Read pack temperature from NTC thermistor
 * @return temperature in 0.1 degC, 2560 on sensor disagreement.
 */
static int16_t bms_read_temperature(void)
{
  int16_t tc1_temp;
  uint16_t adc_value;

  // get tc1
  adc_value = adc_convert_channel(BMS_ADC_CH_TC1);
  tc1_temp  = NTC_ADC2Temperature(adc_value);

  return tc1_temp;
}

/**
 * @brief Sample both pack thermistors into pack_temperature and the hot one.
 *
 * The pack carries two, in different places: TC1 (PA07) and TC2 (PA08), the
 * latter sitting against the cells. They were measured across 24C to 31C and
 * from 3A to 21A of load, and held a constant 1.3-1.5C apart throughout, so
 * both are live and both are believable.
 *
 * The two are used asymmetrically, on purpose.
 *
 * Every HOT check takes the higher of them. That can only make a limit trip
 * sooner, never later, so the second sensor can add protection and cannot take
 * any away - and until now a pack getting hot where TC2 sits was simply not
 * seen at all.
 *
 * The COLD check stays on TC1 alone. The thermistor is the lower leg of its
 * divider, so an open sensor reads as extreme cold, not extreme heat: taking
 * the lower of the two would let a broken TC2 strand the pack on a permanent
 * false undertemp, while it cannot fake an overtemp no matter how it fails.
 */
static void bms_sample_temperatures(void)
{
  pack_temperature   = bms_read_temperature();
  pack_temperature_2 = NTC_ADC2Temperature(adc_convert_channel(BMS_ADC_CH_TC2));

}

/** @brief The higher of the two thermistors, in 0.1C. */
static int16_t bms_temperature_hottest(void)
{
  return (pack_temperature_2 > pack_temperature) ? pack_temperature_2 : pack_temperature;
}

/**
 * @brief Check if pack conditions allow discharge.
 * @return true if safe.
 */
static bool bms_is_safe_to_discharge(void)
{
  /*
   * Snapshot a fault that is already pending - bms_force_fault() sets both
   * bms_error and bms_state from the watchdog ISR - so that evaluating fresh
   * conditions here cannot erase its code. Without this the fault reached
   * bms_handle_fault() as BMS_ERR_NONE and blinked zero times: honoured, but
   * completely silent and misattributed.
   */
  const enum BMS_ERROR_CODE pending = (bms_state == BMS_FAULT) ? bms_error : BMS_ERR_NONE;
  bool safe;

  //Clear error status.
  bms_error = BMS_ERR_NONE;
  bq7693_comm_clear_error();

  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  //Check any cells undervolt.
  for (int i=0; i<7;++i)
  {
    if (cell_voltages[i] < CELL_IMPLAUSIBLE_VOLTAGE)
    {
      /*
       * Not a flat cell - a cell cannot sit here and still be part of a pack
       * the BMS is being asked to discharge. A severed sense wire, a blown
       * cell tap or an AFE channel that never converted all land here.
       * Deliberately NOT BMS_ERR_PACK_DISCHARGED: that path zeroes the fuel
       * gauge, and throwing away a good charge level because of a broken
       * measurement is the wrong trade.
       */
      bms_set_error(BMS_ERR_CELL_FAIL);
      BMS_PRINT("BMS:CELL_IMPLAUSIBLE c=%d v=%dmV\r\n", i, cell_voltages[i]);
    }
    else if (cell_voltages[i] < CELL_LOWEST_DISCHARGE_VOLTAGE)
    {
      bms_set_error(BMS_ERR_PACK_DISCHARGED);
      BMS_PRINT("BMS:CELL_LOW c=%d v=%dmV\r\n", i, cell_voltages[i]);
    }
  }
  //Check pack temperature remains in acceptable range - hot on the hotter of
  //the two sensors, cold on TC1 alone. See bms_sample_temperatures().
  bms_sample_temperatures();
  int temp     = pack_temperature / 10;
  int temp_hot = bms_temperature_hottest() / 10;

  if (temp_hot > MAX_PACK_TEMPERATURE)
  {
    bms_set_error(BMS_ERR_PACK_OVERTEMP);
    BMS_PRINT("%s : Pack overtemp %d 'C, max %d\r\n",__FUNCTION__ ,  temp_hot, MAX_PACK_TEMPERATURE);
  }
  else if (temp < MIN_PACK_DISCHARGE_TEMP)
  {
    bms_set_error(BMS_ERR_PACK_UNDERTEMP);
    BMS_PRINT("%s: Pack undertemp %d 'C, min %d\r\n", __FUNCTION__ , temp, MIN_PACK_DISCHARGE_TEMP);
  }

  //Pick up anything that arrived since the last service, then evaluate every
  //fault bit latched since the previous safety check.
  bms_sys_stat_service();
  uint8_t sys_stat = bms_sys_stat_take();

  if (sys_stat != 0u)
  {
    BMS_PRINT("%s: SYS_STAT faults=0x%02X\r\n", __FUNCTION__, sys_stat);
  }

  if (sys_stat & STAT_OCD)
  {
    bms_set_error(BMS_ERR_OVERCURRENT);
    BMS_PRINT("%s: BMS IC Overcurrent Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_SCD)
  {
    bms_set_error(BMS_ERR_SHORTCIRCUIT);
    BMS_PRINT("%s: BMS IC Short Circuit Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_UV)
  {
    bms_set_error(BMS_ERR_UNDERVOLTAGE);
    BMS_PRINT("%s: BMS IC Undervoltage Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_OV)
  {
    bms_set_error(BMS_ERR_OVERVOLTAGE);
    BMS_PRINT("%s: BMS IC Overvoltage Trip\r\n", __FUNCTION__);
  }
  if (bms_afe_fault_is_real(sys_stat, __FUNCTION__))
  {
    bms_set_error(BMS_ERR_I2C_FAIL);
  }

  /* Checked last: if the bus failed, everything decided above was decided on
     data we never actually received. BMS_ERR_I2C_FAIL outranks the rest. */
  if (!bq7693_comm_healthy())
  {
    bms_set_error(BMS_ERR_I2C_FAIL);
    BMS_PRINT("%s: BQ7693 I2C failure\r\n", __FUNCTION__);
  }

  /* The verdict is the fresh evaluation only, so the fault handler's retry
     sees a true recovery. bms_set_error() only raises, so restoring the
     pending code cannot mask a worse finding made just now. */
  safe = (bms_error == BMS_ERR_NONE);
  bms_set_error(pending);

  return safe;
}

/**
 * @brief Check if pack conditions allow charging.
 * @return true if safe.
 */
static bool bms_is_safe_to_charge(void)
{
  /* See bms_is_safe_to_discharge() - a pending fault must survive a fresh
     evaluation. */
  const enum BMS_ERROR_CODE pending = (bms_state == BMS_FAULT) ? bms_error : BMS_ERR_NONE;
  bool safe;

  //Clear error status.
  bms_error = BMS_ERR_NONE;
  bq7693_comm_clear_error();

  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  //Check no cells are so flat they cannot be charged.
  for (int i=0; i<7;++i)
  {
    if ( cell_voltages[i] < CELL_LOWEST_CHARGE_VOLTAGE )
    {
      bms_set_error(BMS_ERR_CELL_FAIL);
      BMS_PRINT("%s: Cell %d below min charge voltage %d, min %d\r\n", __FUNCTION__, i, cell_voltages[i], CELL_LOWEST_CHARGE_VOLTAGE);
    }
  }

  //Check pack temperature acceptable (<=60'C) - see bms_sample_temperatures()
  //for why hot and cold do not use the same reading.
  bms_sample_temperatures();
  int temp     = pack_temperature / 10;
  int temp_hot = bms_temperature_hottest() / 10;

  if (temp_hot >= MAX_PACK_CHARGE_TEMP || temp_hot >= MAX_PACK_TEMPERATURE)
  {
    bms_set_error(BMS_ERR_PACK_OVERTEMP);
    BMS_PRINT("%s: Pack overtemp %d 'C, max %d\r\n", __FUNCTION__, temp_hot, MAX_PACK_CHARGE_TEMP);
  }
  else if (temp < MIN_PACK_CHARGE_TEMP)
  {
    bms_set_error(BMS_ERR_PACK_UNDERTEMP);
  }

  //Pick up anything that arrived since the last service, then evaluate every
  //fault bit latched since the previous safety check.
  bms_sys_stat_service();
  uint8_t sys_stat = bms_sys_stat_take();

  if (sys_stat != 0u)
  {
    BMS_PRINT("%s: SYS_STAT faults=0x%02X\r\n", __FUNCTION__, sys_stat);
  }

  if (sys_stat & STAT_OCD)
  {
    bms_set_error(BMS_ERR_OVERCURRENT);
  }
  if (sys_stat & STAT_SCD)
  {
    /* was not checked on the charge path at all */
    bms_set_error(BMS_ERR_SHORTCIRCUIT);
  }
  /*
   * STAT_UV is deliberately NOT a reason to refuse a charge. UV only drops
   * DSG (SLUSBK2I Table 8-1), and the bit re-latches for as long as a cell sits
   * under UV_TRIP - so refusing on it stranded exactly the pack that most needs
   * charging: one that self-discharged below CELL_UNDERVOLTAGE_TRIP in storage.
   * CELL_LOWEST_CHARGE_VOLTAGE above is the floor for charging.
   */
  if (sys_stat & STAT_OV)
  {
    bms_set_error(BMS_ERR_OVERVOLTAGE);
  }
  if (bms_afe_fault_is_real(sys_stat, __FUNCTION__))
  {
    bms_set_error(BMS_ERR_I2C_FAIL);
  }

  /* Checked last: if the bus failed, everything decided above was decided on
     data we never actually received. BMS_ERR_I2C_FAIL outranks the rest. */
  if (!bq7693_comm_healthy())
  {
    bms_set_error(BMS_ERR_I2C_FAIL);
    BMS_PRINT("%s: BQ7693 I2C failure\r\n", __FUNCTION__);
  }

  /* The verdict is the fresh evaluation only, so the fault handler's retry
     sees a true recovery. bms_set_error() only raises, so restoring the
     pending code cannot mask a worse finding made just now. */
  safe = (bms_error == BMS_ERR_NONE);
  bms_set_error(pending);

  return safe;
}

/**
 * @brief True when this pack has been sitting unused long enough to store.
 *
 * A pack in service is emptied and refilled every few days; one that has gone
 * STORAGE_IDLE_WAKES standby periods without the motor running is not waiting
 * to be picked up, it is being kept. Holding lithium near full is the single
 * cheapest way to age it for nothing, so such a pack is allowed to sit lower.
 *
 * Counted in EEPROM rather than RAM because the count only means anything over
 * weeks, and the pack may lose power in between - it is incremented on each
 * RTC wake and cleared the moment the motor runs.
 */
static bool bms_in_storage_mode(void)
{
  return (eeprom_data.idle_wakes >= STORAGE_IDLE_WAKES);
}

/**
 * @brief Check if any cell reached full charge voltage (with hysteresis).
 * @return true if any cell is at/above threshold.
 */
static bool bms_is_pack_full(void)
{
  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  const bool storage = bms_in_storage_mode();

  // Use hysteresis: apply the lower release threshold when pack was already
  // full. A stored pack uses the same pair shifted down, so the whole
  // pause/top-up machinery above works unchanged against a lower target.
  uint16_t threshold;

  if (bms_state == BMS_CHARGING)
  {
    threshold = storage ? CELL_STORAGE_CHARGE_VOLTAGE : CELL_FULL_CHARGE_VOLTAGE;
  }
  else
  {
    threshold = storage ? CELL_STORAGE_RELEASE_VOLTAGE : CELL_FULL_CHARGE_RELEASE_VOLTAGE;
  }

  for (int i=0; i<7; ++i)
  {
    if (cell_voltages[i] >= threshold)
    {
      return true;
    }
  }

  return false;
}

/** @brief Idle state: wait for trigger, charger, or sleep timeout. */
static void bms_handle_idle(void)
{
  uint32_t sleep_time;
  bool vacuum_was_connected = false;
  bool trigger_was_pressed  = false;
  bool debug_grace_used     = false;
  sw_timer dsg_retry_timer  = 0;    // never started, so the first pass is immediate

  /*
   * Outer loop so the idle period can be granted a second time. The check has
   * to live out here: inside the body it could never fire, because the 50ms
   * delay at the bottom is where the timeout almost always expires, and the
   * while() condition below sees that before the next pass of the body does.
   */
  for (;;)
  {
  sw_timer_start(&bms_timer);

  do
  {
    bool vacuum_connected = dsn_prot_get_vacuum_connected();
    bool trigger_pressed  = bms_trigger_active();

    /*
     * Discharge is armed, and an AFE event has been latched since. The AFE
     * dropped DSG_ON for it (SLUSBK2I Table 8-1), and no safety check runs
     * in this state to absorb the event and re-arm - the cleaner just lost
     * its supply and would only get it back through a session timeout and
     * reconnect. Forget the arm instead, so the edge logic below re-runs
     * the check, which absorbs (or faults) the event and re-enables the
     * FET. Peeked rather than taken: the check is what consumes it.
     */
    if (vacuum_was_connected
        && ((bms_sys_stat_faults & (STAT_DEVICE_XREADY | STAT_OVRD_ALERT)) != 0u))
    {
      BMS_PRINT("BMS:IDLE AFE event, re-arming discharge\r\n");
      vacuum_was_connected = false;
    }

    if (vacuum_connected && !vacuum_was_connected)
    {
      if (sw_timer_is_elapsed(&dsg_retry_timer, IDLE_DSG_RETRY_MS))
      {
        if (bms_is_safe_to_discharge())
        {
          sw_timer_delay_ms(300);
          /* the 300ms above pumps dsn_prot_mainloop(), which can time the
             session out underneath us - do not arm the FET for a vacuum that
             has since gone away */
          if (dsn_prot_get_vacuum_connected())
          {
            /* only consume the edge once the FET is actually on; if either
               the safety check or the enable itself failed we leave
               vacuum_was_connected clear so a later pass retries, rather
               than never arming discharge again for this idle stay */
            vacuum_was_connected = bms_discharge_fet_on();
          }
        }

        if (!vacuum_was_connected)
        {
          /* pace the retry - see IDLE_DSG_RETRY_MS */
          sw_timer_start(&dsg_retry_timer);
        }
      }
    }
    else
    {
      vacuum_was_connected = vacuum_connected;
    }

    if(true == vacuum_connected)
      sleep_time = (IDLE_TIME * 1000ul);
    else
      sleep_time = (IDLE_NO_VACUUM_TIME * 1000ul);

    if (bms_fault_pending())
    {
      /* raised from interrupt context - honour it instead of overwriting it */
      return;
    }

    if (dio_read(DIO_CHARGER_CONNECTED) == true)
    {
      bms_state = BMS_CHARGER_CONNECTED;
      return;
    }
    else if (trigger_pressed)
    {
      if (!trigger_was_pressed)
      {
        leds_blink_leds(10);
        sw_timer_start(&bms_timer);
      }

      if (vacuum_connected)
      {
        bms_state = BMS_VACUUM_RUNNING;
        return;
      }
    }
    else if(dsn_prot_get_sleep_flag() == true)
      sw_timer_stop(&bms_timer); // go to sleep requested by cleaner

    trigger_was_pressed = trigger_pressed;

    sw_timer_delay_ms(50);
    wdt_reset_count();

  } while (false == sw_timer_is_elapsed(&bms_timer, sleep_time));

  /*
   * One extra idle period before sleeping, once, if a debugger has been seen
   * since power-up.
   *
   * Not a refusal: gating sleep on DBGPRES stopped a pack from ever sleeping
   * again after a single flash, because the bit outlives the programmer's
   * reset. A grace period costs one idle period if the adapter really is gone,
   * and buys a warning with time to act on it if it is not.
   */
  if (debug_grace_used || (DSU->STATUSB.bit.DBGPRES == 0u))
  {
    break;
  }

  debug_grace_used = true;
  serial_debug_send_message("BMS:DEBUGGER_SEEN - unplug the programmer, sleeping after one more idle period\r\n");
  }

  //Reached the end of our wait loop, with nobody pulling the trigger, or plugging in charger.
  //Transit to sleep state
  bms_state = BMS_SLEEP;
}

/** @brief Sleep: save EEPROM, disable FETs, enter BQ7693 SHIP mode. */
static void bms_handle_sleep(void)
{
#if !SHIP_MODE_ENABLE
  /*
   * Bench mode - see SHIP_MODE_ENABLE. Commit what we would have committed on
   * the way down, then carry on idling rather than taking the one-way trip.
   */
  serial_debug_send_message("BMS:SHIP_DISABLED, staying awake\r\n");
  (void)eeprom_write();
  bms_state = BMS_IDLE;
  return;
#endif

  bms_wdt_deinit();
  serial_debug_send_message("BMS:GOING_TO_SLEEP\r\n");
  //CELLBAL must be cleared before SHIP mode - the BQ7693 re-enters NORMAL with
  //balancing off, but leaving bits set here would bleed cells on the way down.
  bms_balance_stop();
  bq7693_disable_charge();
  bq7693_disable_discharge();

  leds_sequence();

  /*
   * The fade ends at zero duty and that is all the parking the LEDs need.
   *
   * If they appear faintly lit after the pack has powered down, check what is
   * still plugged into the programming header before suspecting this code: a
   * connected debugger keeps SWDIO driven, which forward-biases that pin's ESD
   * clamp into the dead VDD rail and holds it around 600mV. Enough to bias the
   * LED drivers, nowhere near enough to run the MCU. Unplug the adapter and it
   * goes away.
   *
   * Nothing here can affect it either way - once REGOUT is gone every pin is
   * high impedance regardless of what was last written to it.
   */

  pins_deinit();

  delay_ms(1000);

  /*
   * The single automatic commit point. SHIP mode turns off the BQ7693's
   * REGOUT, so the MCU loses power here and RAM with it - everything the
   * coulomb counter has accumulated has to land in the emulated EEPROM
   * before bq7693_enter_sleep_mode() below. Every state that can shut the
   * pack down routes through here.
   */
  if (eeprom_write())
  {
    serial_debug_send_message("BMS:EEPROM_WRITTEN\r\n");
  }
  else
  {
    serial_debug_send_message("BMS:EEPROM_UNCHANGED\r\n");
  }

  /*
   * Last thing in the log before the lights go out, and the only record of
   * what the wake sources were doing at the moment it mattered. A pack that
   * does not come back is otherwise silent about why: BOOT is driven from the
   * board, not from here, so if CHARGER or the trigger were already asserted
   * when this ran, whatever edge would have woken the AFE has been and gone.
   */
  bms_debug_dump_pins("ship");

  if (!bq7693_enter_sleep_mode())
  {
    serial_debug_send_message("BMS:SHIP_WRITE_FAILED\r\n");
  }

  /*
   * If SHIP took, REGOUT is gone and nothing below this line ever runs.
   *
   * If we are still executing a second later it did not take, and sitting here
   * forever is the wrong answer: the original code relied on re-arming the
   * watchdog and then not kicking it, which is the one path in the firmware
   * that stops feeding the dog and also the one that never checked whether
   * wdt_set_config() had worked. A pack that reached here could sit silent,
   * powered and executing nothing until somebody reflashed it.
   *
   * NVIC_SystemReset() needs nothing to have been configured correctly
   * beforehand and cannot quietly not happen.
   *
   * Deliberately no I2C in here. The AFE has just been told to shut down and
   * its supply is collapsing; probing the bus through that window tells us
   * little and is exactly the sort of thing not to do to a device mid
   * power-down.
   */
  delay_ms(1000);

  NVIC_SystemReset();
}

/** @brief Vacuum running: monitor safety while trigger held and vacuum connected. */
static void bms_handle_vacuum_running(void)
{
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif
  sw_timer cell_log_timer = 0;   // never started, so the first pass logs

  //Never bleed cells while the motor is drawing current.
  bms_balance_stop();

  if (!bms_is_safe_to_discharge())
  {
    dsn_prot_set_trigger(false);
    bms_state = BMS_FAULT;
    return;
  }

  /*
   * Arm discharge unconditionally before telling the cleaner it may run.
   *
   * This used to happen only after an absorbed AFE event. The normal arming is
   * done in bms_handle_idle() when the cleaner connects, and if that single
   * attempt failed it retries every IDLE_DSG_RETRY_MS - but a trigger pulled
   * inside that window came straight here with DSG still off. The cleaner was
   * told to run, the motor had no supply, and nothing re-armed it until the
   * session ended. bq7693_enable_discharge() is idempotent, so doing it on
   * every entry costs one register round trip and closes the window.
   */
  if (!bms_discharge_fet_on())
  {
    dsn_prot_set_trigger(false);
    bms_set_error(BMS_ERR_I2C_FAIL);
    bms_state = BMS_FAULT;
    return;
  }
  /*
   * The pack is in service. Cleared in RAM only: the next commit carries it to
   * flash, and a flash erase is not something to start while the motor is
   * about to pull twenty amps. Losing the clear to a power cut costs nothing -
   * the count is weeks long and this happens every time the trigger is used.
   */
  if (eeprom_data.idle_wakes != 0u)
  {
    eeprom_data.idle_wakes = 0;
    serial_debug_send_message("BMS:IN_SERVICE, storage idle count cleared\r\n");
  }

  dsn_prot_set_trigger(true);

#ifdef SERIAL_DEBUG
  /* What the cells looked like before the motor touched them - the reference
     the exit dump gets compared against. */
  serial_debug_send_cell_voltages();
#endif
  /* Started here, not left at zero: an unstarted timer reads as elapsed, so
     the paced dump below would fire on the first pass and repeat the line
     just printed. */
  sw_timer_start(&cell_log_timer);

  while (1)
  {
    if (!bms_is_safe_to_discharge())
    {
      dsn_prot_set_trigger(false);
      bms_state = BMS_FAULT;
      return;
    }

    if (bms_afe_fet_dropped() && !bms_discharge_fet_on())
    {
      dsn_prot_set_trigger(false);
      bms_set_error(BMS_ERR_I2C_FAIL);
      bms_state = BMS_FAULT;
      return;
    }

#ifdef SERIAL_DEBUG
    /* Under 21A the cells sag, and how evenly they sag is the interesting
       part. Paced rather than per-pass: this loop runs every 60ms and nothing
       meaningful moves that fast. */
    if (sw_timer_is_elapsed(&cell_log_timer, CELL_LOG_PERIOD_MS))
    {
      sw_timer_start(&cell_log_timer);
      serial_debug_send_cell_voltages();
    }
#endif

    if (bms_fault_pending())
    {
      /* raised from interrupt context - honour it instead of overwriting it */
      dsn_prot_set_trigger(false);
      bq7693_disable_discharge();
      leds_off();
      return;
    }

    if (!bms_trigger_active() || !dsn_prot_get_vacuum_connected())
    {
      dsn_prot_set_trigger(false);
      leds_off();
#ifdef SERIAL_DEBUG
      /* What the motor left behind: the cells have just been through the
         heaviest load they ever see, and this is where sag and imbalance show
         up while they are still warm. */
      bms_debug_dump_temps("vac_end");
      serial_debug_send_cell_voltages();
#endif
      bms_state = BMS_IDLE;
      return;
    }

#ifdef SERIAL_DEBUG
    if(++debug_print_cnt > 5)
    {
      BMS_PRINT("BMS:VACUUM_RUNNING I:%d mA @ %ld mAH, C:%ld mAH, T:%s%d.%d/%s%d.%d 'C, P:%d mV\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), T_SIGN(pack_temperature), T_WHOLE(pack_temperature), T_FRAC(pack_temperature), T_SIGN(pack_temperature_2), T_WHOLE(pack_temperature_2), T_FRAC(pack_temperature_2), bq7693_get_pack_voltage());
      debug_print_cnt = 0;
    }
#endif


    sw_timer_delay_ms(60);
  }
}

/** @brief Fault: display error, retry safety for transient faults, keep protocol alive. */
static void bms_handle_fault(void)
{
  const enum BMS_ERROR_CODE original_error = bms_error;
  /*
   * The self-recovering codes are exactly 1..BMS_ERR_SHORTCIRCUIT - see the
   * enum in bms.h. Deriving it from the ordering rather than listing them
   * again keeps that property true: it is the same ordering the blink pattern
   * relies on to show recoverable faults without a long pulse.
   */
  const bool auto_recover = ((original_error != BMS_ERR_NONE)
                          && (original_error <= BMS_ERR_SHORTCIRCUIT));
  sw_timer retry_timer = 0;
  sw_timer fault_timer = 0;
  sw_timer giveup_timer = 0;

  BMS_PRINT("BMS:FAULT err=%d auto_recover=%d\r\n", original_error, auto_recover);
#ifdef SERIAL_DEBUG
  /* What the pack looked like at the moment it faulted. The safety check names
     only the cell it tripped on. VCALL and the BQ7693 thermistors used to be
     dumped here too - both questions are settled (the cell map is correct,
     TS1/TS2 have nothing connected), so they are boot-only now. */
  serial_debug_send_cell_voltages();
  /* All ten channels, including the three the cell map does not use: a fault
     is where a wrong map or a disturbed measurement group would show. */
  serial_debug_send_all_vc();
  bms_debug_dump_temps("fault");
#endif

  sw_timer_start(&fault_timer);
  sw_timer_start(&giveup_timer);

  leds_off();
  dsn_prot_set_trigger(false);
  bq7693_disable_discharge();
  bq7693_disable_charge();
  bms_balance_stop();
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  /* Paces both recovery routes below; the charger route applies to every
     code, so it is started unconditionally. */
  sw_timer_start(&retry_timer);

  if (bms_error == BMS_ERR_PACK_DISCHARGED || bms_error == BMS_ERR_UNDERVOLTAGE)
  {
    eeprom_data.current_charge_level = 0;
    eeprom_data.full_discharge_seen = 1;
  }

  /*
   * Commit here, after the discharged/undervoltage handling above has settled
   * the values. Sleep is still the normal commit point and this state reaches
   * it via the display timeout, but a fault is exactly when the pack is most
   * likely to lose power before getting there - and it is also when the
   * full_discharge_seen marker the capacity learning depends on gets set.
   *
   * Costs nothing when there is nothing to save: eeprom_write() compares
   * against what is stored and returns without touching flash if the values
   * match, so a fault that repeats without the charge level moving does not
   * write at all. That comparison is what makes committing here safe - the
   * original firmware wrote unconditionally on every entry to this state.
   */
  (void)eeprom_write();

  bool trigger_state = bms_trigger_active();

  while (1)
  {
    /* Sticky once true: sw_timer_is_elapsed() stops the timer when it fires
       and a stopped timer reads as elapsed from then on. */
    const bool display_over = sw_timer_is_elapsed(&fault_timer, (uint32_t)FAULT_DISPLAY_TIME * 1000ul);

    if (!display_over)
    {
      // Blink the error code, then pause so the user can read the pattern.
      bms_blink_error_code(bms_error);
    }
    sw_timer_delay_ms(FAULT_BLINK_REPEAT_MS);
    wdt_reset_count();

    const bool charger   = dio_read(DIO_CHARGER_CONNECTED);
    const bool retry_due = sw_timer_is_elapsed(&retry_timer, FAULT_RETRY_MS);

    if (charger)
    {
      /*
       * Hand over to charging only once the pack is actually chargeable.
       * Going straight to BMS_CHARGER_CONNECTED bounced back here on its
       * safety check for anything the charge path rejects - a pack between
       * MAX_PACK_CHARGE_TEMP and MAX_PACK_TEMPERATURE, a dead bus, a broken
       * cell tap - once per blink cycle, each time re-entering this state
       * with the FET disables and the EEPROM compare.
       *
       * The same check is what recovers a flat pack: charging IS the fix
       * for PACK_DISCHARGED and UNDERVOLTAGE, and bms_is_safe_to_charge()
       * passes both. So this applies to every code, not just the
       * self-recovering ones.
       *
       * A code the charge path can never clear - a broken cell tap, a dead
       * bus - keeps the pack here for as long as it is docked. That is
       * deliberate: SHIP mode with the charger attached may not remove
       * power. But the LEDs do go quiet after FAULT_DISPLAY_TIME, because
       * with the charge FET off they run from the cells, and blinking a
       * flat pack for days is exactly what the timeout exists to prevent.
       * Pulling the trigger shows the code again.
       */
      /*
       * Docked and still faulting after all this time. The re-check below is
       * the cure for a flat pack, but only if charging can actually happen -
       * and if it cannot, running the MCU and the AFE from the cells is what
       * empties them. See FAULT_CHARGER_GIVEUP_MS.
       */
      if (sw_timer_is_elapsed(&giveup_timer, FAULT_CHARGER_GIVEUP_MS))
      {
        BMS_PRINT("BMS:FAULT giving up on charger after %lu min, sleeping\r\n",
                  (unsigned long)(FAULT_CHARGER_GIVEUP_MS / 60000ul));
        leds_off();
        bms_state = BMS_SLEEP;
        return;
      }

      if (retry_due)
      {
        sw_timer_start(&retry_timer);
#ifdef SERIAL_DEBUG
        /* Repeated every FAULT_RETRY_MS: an intermittent sense connection is
           only visible as movement between samples, so one snapshot at entry
           is not enough to tell it from a genuinely flat cell. */
        serial_debug_send_cell_voltages();
        bms_debug_dump_temps("fault");
#endif

        if (bms_is_safe_to_charge())
        {
          BMS_PRINT("BMS:FAULT_RECOVERED err=%d, charger\r\n", original_error);
          bms_error = BMS_ERR_NONE;
          leds_off();
          bms_state = BMS_CHARGER_CONNECTED;
          return;
        }
        /* not safe: bms_error now carries the fresh verdict, which is at
           least the original code - a worse finding is shown, not hidden */
      }
    }
    else if (display_over)
    {
      /* The user has had long enough to read the code. Shut the pack down
         rather than blinking forever - which on a flat pack means draining
         it further - and let bms_handle_sleep() commit the charge level.
         Off the charger only: on the dock the display stops but the
         re-check above carries on, see there. */
      BMS_PRINT("BMS:FAULT display timeout, sleeping\r\n");
      bms_state = BMS_SLEEP;
      return;
    }

    bool trigger_now = bms_trigger_active();

    if (trigger_now && !trigger_state)
    {
      /* Every other exit clears the code; this one left it set on the way to
         idle. Harmless today - the next safety check starts from NONE - but
         bms_set_error() only raises, so a stale high code is a trap for any
         later path that sets a lower one before that check runs. */
      bms_error = BMS_ERR_NONE;
      leds_off();
      bms_state = BMS_IDLE;
      return;
    }
    trigger_state = trigger_now;

    if (auto_recover && !charger && retry_due)
    {
      sw_timer_start(&retry_timer);

      if (bms_is_safe_to_discharge())
      {
        BMS_PRINT("BMS:FAULT_RECOVERED err=%d\r\n", original_error);
        bms_error = BMS_ERR_NONE;
        leds_off();
        bms_state = BMS_IDLE;
        return;
      }
      /*
       * Deliberately no "bms_error = original_error" here. The check
       * snapshots the pending code and restores it through bms_set_error(),
       * which only raises, so bms_error is now max(fresh, original). Forcing
       * it back to the original kept blinking "overtemp" while the bus had
       * died underneath, and kept retrying as if that were recoverable.
       */
    }
  }
}

/** @brief Charger connected: evaluate pack and begin charging or report full. */
static void bms_handle_charger_connected(void)
{
  bms_debug_dump_temps("chg_in");
#ifdef SERIAL_DEBUG
  serial_debug_send_cell_voltages();
#endif

  if (bms_is_pack_full())
  {
    bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;
  }
  else if (bms_is_safe_to_charge())
  {
    // force trigger state
    dsn_prot_set_trigger(false);
    bms_state = BMS_CHARGING;
  }
  else
  {
    bms_state = BMS_FAULT;
  }
}

/** @brief Not charging: manage standby sleep while charger is connected. */
static void bms_handle_charger_connected_not_charging(void)
{
  sw_timer relax_timer    = 0;
  sw_timer recheck_timer  = 0;
  sw_timer dock_idle_timer = 0;
  sw_timer cell_log_timer  = 0;
  bool     balancing      = false;
  //Do not drop into standby before balancing has had a chance to look at the
  //pack - the vacuum usually asks to sleep within a second or two of docking,
  //and the next RTC wake is days away. Always false when balancing is compiled
  //out, so standby behaviour is unchanged in that build.
  bool     balance_evaluated = !CELL_BALANCE_ENABLE;

  leds_blink_leds(2000);

  //The charge FET is already off on entry to this state; give the cells time to
  //settle before the first balancing decision is taken.
  sw_timer_start(&relax_timer);
  sw_timer_start(&recheck_timer);
  sw_timer_start(&dock_idle_timer);
  sw_timer_start(&cell_log_timer);

  while(1)
  {
#ifdef SERIAL_DEBUG
    /*
     * This state printed nothing at all, and a pack spends most of its life
     * here - balancing, waiting out the top-up recheck, counting down to
     * standby. Silence is indistinguishable from a hang, and there is no way
     * to see balancing working or the cells drifting between top-ups.
     *
     * It stops on its own: once standby is entered the loop blocks inside
     * system_sleep(), so this costs a handful of lines per dock and nothing
     * thereafter.
     */
    if (sw_timer_is_elapsed(&cell_log_timer, CELL_LOG_PERIOD_MS))
    {
      sw_timer_start(&cell_log_timer);
      serial_debug_send_cell_voltages();
      bms_debug_dump_temps("dock");

      /*
       * Every input the standby decision below is made from, because when it
       * does not happen the log gives no way to tell which of them is holding
       * it. Four booleans and a countdown answer it outright instead of
       * leaving the state looking hung.
       *
       * vac  - a connected cleaner re-arms the idle timer every pass
       * slp  - the cleaner has asked to sleep, which bypasses the timer
       * bal  - cells are bleeding, which holds standby off deliberately
       * eval - balancing has not had its first look yet (20s after entry)
       * idle - seconds counted towards DOCK_STANDBY_IDLE_MS; "done" once the
       *        timer has fired, since sw_timer_is_elapsed() stops it
       */
      BMS_PRINT("DOCK vac=%u slp=%u bal=%u eval=%u idle=%lu/%lus\r\n",
                dsn_prot_get_vacuum_connected() ? 1u : 0u,
                dsn_prot_get_sleep_flag()       ? 1u : 0u,
                balancing                       ? 1u : 0u,
                balance_evaluated               ? 1u : 0u,
                (unsigned long)(sw_timer_get_elapsed_time(&dock_idle_timer) / 1000ul),
                (unsigned long)(DOCK_STANDBY_IDLE_MS / 1000ul));
    }
#endif

    if (bms_fault_pending())
    {
      /* raised from interrupt context - honour it instead of overwriting it */
      bms_balance_stop();
      leds_off();
      return;
    }

    if (!dio_read(DIO_CHARGER_CONNECTED))
    {
      bms_balance_stop();
      bms_state = BMS_IDLE;
      return;
    }

    //This is where the pack sits for hours on the dock, so this is where the
    //balancing actually gets done. NB sw_timer_is_elapsed() stops the timer when
    //it fires and a stopped timer keeps reading as elapsed, so this latches on
    //after the relax period; bms_balance_tick() rate-limits itself from there.
    if (sw_timer_is_elapsed(&relax_timer, CELL_BALANCE_RELAX_MS))
    {
      balancing         = bms_balance_tick();
      balance_evaluated = true;
    }

    bms_balance_leds();

    //A cell at the OV guard is bled on its own by the balancer, and nothing
    //feeds it here: the charge FET is off in this state, and the top-up
    //recheck below cannot start a charge while any cell is above
    //CELL_FULL_CHARGE_RELEASE_VOLTAGE. This used to fault on it instead,
    //which stopped the bleed and bounced FAULT <-> here every half minute
    //until the cell self-discharged below the guard.

    /*
     * Re-check for a top-up periodically, not only after a standby wake.
     *
     * Balancing bleeds the highest cell down, and dropping back below
     * CELL_FULL_CHARGE_RELEASE_VOLTAGE is exactly the event that should put
     * charge back in - that is what makes top-balancing worth doing. But the
     * only bms_is_pack_full() call used to live inside the standby branch
     * below, which the "still balancing" continue skips entirely, so the pack
     * could balance itself down and never resume charging until balancing
     * stopped AND the cleaner happened to ask to sleep.
     *
     * This also covers plain self-discharge over days on the dock.
     */
    if (sw_timer_is_elapsed(&recheck_timer, FULL_CHARGE_RECHECK_MS))
    {
      sw_timer_start(&recheck_timer);

      if (!bms_is_pack_full())
      {
        BMS_PRINT("BMS:TOP_UP needed\r\n");
        bms_balance_stop();
        bms_state = BMS_CHARGER_CONNECTED;
        return;
      }
    }

    /*
     * Two ways to end up asleep on a dock.
     *
     * A pack in a cleaner is told: the cleaner asks within a second or two of
     * docking, and that is the path this state was written for.
     *
     * A pack sitting in a charger by itself is never asked, and used to stay
     * awake for as long as it was left there - months, on a spare - burning
     * the cells through the MCU and the AFE and topping itself back up every
     * five minutes to cover it. So time it out instead. The cleaner being
     * connected re-arms the timer, so nothing changes for a pack that has one.
     */
    if (dsn_prot_get_vacuum_connected())
    {
      sw_timer_start(&dock_idle_timer);
    }

    if (   (dsn_prot_get_sleep_flag() == true)
        || sw_timer_is_elapsed(&dock_idle_timer, DOCK_STANDBY_IDLE_MS))
    {
      if (balancing || !balance_evaluated)
      {
        //Standby stops the 1ms tick, and the balancing supervision with it.
        //Stay awake while cells are bleeding: a few mA of MCU is noise next to
        //the balance current, and the alternative is balancing never running.
        sw_timer_delay_ms(250);
        continue;
      }

      rtc_standby_timer_start();
      bms_enter_standby();
      serial_debug_send_message("BMS_STANDBY\r\n");
      leds_blink_leds_num(LEDS_NUM, 4, 100);

      /*
       * NB TC0 is clocked from GCLK1, which does not run in standby, so the
       * sw_timer tick stops here and every running sw_timer under-counts by
       * however long standby lasted. Nothing currently depends on that - the
       * timers that matter are restarted below or read as already elapsed,
       * which is the safe direction - but any new timer used across this
       * point needs restarting after the wake.
       */
      system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);

      /*
       * Test-then-sleep has to be atomic. The wake sources were armed in
       * bms_enter_standby() and the LED blink above took 400ms: an edge in
       * that window ran the ISR, which cleared it, and WFI then slept
       * through it - until the next edge, or the RTC two days later. A
       * cleaner lifted off the dock in that window was not noticed until
       * the trigger was pulled, and that pull was spent on waking up.
       *
       * With interrupts masked the ISR cannot run between the test and the
       * WFI. WFI still returns on a pending interrupt, which is then taken
       * as soon as the critical section is left.
       */
      system_interrupt_enter_critical_section();
      if (!bms_wake_event && !rtc_wakeup_flag)
      {
        system_sleep(); // WFI
      }
      system_interrupt_leave_critical_section();

      bms_leave_standby();
      rtc_standby_timer_stop();

      if (rtc_wakeup_flag)
      {
        rtc_wakeup_flag = false;
        serial_debug_send_message("BMS:RTC_WAKE\r\n");

        /*
         * Another standby period gone by on a dock with nobody using the pack.
         * Committed here because this is a rare event with the FETs off, and
         * because the count has to survive the pack losing power - which is
         * the whole difference between "docked for a fortnight" and "docked
         * since the last time anyone looked".
         */
        if (   !dsn_prot_get_vacuum_connected()
            && (eeprom_data.idle_wakes < STORAGE_IDLE_WAKES))
        {
          eeprom_data.idle_wakes++;
          (void)eeprom_write();
          BMS_PRINT("BMS:IDLE_WAKES=%lu of %lu%s\r\n",
                    (unsigned long)eeprom_data.idle_wakes,
                    (unsigned long)STORAGE_IDLE_WAKES,
                    bms_in_storage_mode() ? " - storage mode" : "");
        }
      }
      else
      {
        serial_debug_send_message("BMS:EIC_WAKE\r\n");
        dsn_prot_reset();
        leds_blink_leds_num(LEDS_NUM, 2, 100);
        // add some time for vacuum to connect
        sw_timer_delay_ms(250);
      }

      //Cells rested through standby, but re-arm the relax timer anyway so the
      //post-wake protocol traffic settles before the next balancing decision.
      sw_timer_start(&relax_timer);
      balance_evaluated = !CELL_BALANCE_ENABLE;

      // check if pack needs top-up charging on any wakeup
      if (!bms_is_pack_full())
      {
        bms_balance_stop();
        bms_state = BMS_CHARGER_CONNECTED;
        return;
      }
    }

    sw_timer_delay_ms(250);
  }
}

/** @brief Charging: manage charge cycle with pause/retry and capacity learning. */
static void bms_handle_charging(void)
{
  uint8_t charging_leds_duty = 0;
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  // First-cycle reset: 20 trigger pushes while charging resets learned capacity
  uint8_t trigger_push_count = 0;
  bool    trigger_was_pressed = false;
  sw_timer trigger_timeout_timer = 0;

  // Charge-current supervision - drives the indication only, see below.
  sw_timer cell_log_timer         = 0;   // never started, so the first pass logs
  sw_timer charge_flow_timer      = 0;
  // Separate and much longer: this one does act, see CHARGE_NO_CURRENT_TIMEOUT_MS.
  sw_timer noflow_timeout         = 0;
  sw_timer noflow_led_timer       = 0;
  bool     noflow_led_on          = false;
  bool     current_flowing_prev   = true;

  //Sanity check...
  if (!bms_is_safe_to_charge())
  {
    bms_state = BMS_FAULT;
    return;
  }

  // Disable the discharge FET while charging; precharge pin stays asserted
  // by dsn_protocol so the vacuum keeps logic power.
  bq7693_disable_discharge();

  //Enable charging.
  port_pin_set_output_level(ENABLE_CHARGE_PIN, true);

  //Enable the charge FET in the BQ7693.
  if (!bms_charge_fet_on())
  {
    port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
    bq7693_disable_charge();
    bms_set_error(BMS_ERR_I2C_FAIL);
    bms_state = BMS_FAULT;
    return;
  }

  sw_timer_start(&charge_flow_timer);
  sw_timer_start(&noflow_timeout);
  /* See bms_handle_vacuum_running() - bms_handle_charger_connected() has just
     printed the cells, so do not have the paced dump repeat them. */
  sw_timer_start(&cell_log_timer);

  charge_pause_counter = 0;

  while (1)
  {
     #define DUTY_MAX    100

     /*
      * Does the current match the command? The FET is enabled and every safety
      * check passes, but neither of those says a single milliamp is moving. A
      * dead charger, dirty dock contacts, a charge path wired to a pin this
      * firmware does not drive, or an AFE that accepts the CHG_ON write while
      * the FET never conducts all look identical from in here. Show that
      * difference rather than breathing a charge that is not happening.
      *
      * Believed only after CHARGE_CURRENT_GRACE_MS - the current filter has a
      * 500ms time constant and a charger takes a moment to come up - and the
      * grace is re-armed every time the FET is enabled, including after a
      * full-charge pause.
      *
      * NB indication only. Nothing here faults the pack or stops the retrying:
      * a charger that starts late must still be allowed to start.
      */
     bool current_flowing = true;

     if (sw_timer_is_elapsed(&charge_flow_timer, CHARGE_CURRENT_GRACE_MS))
     {
       /* Signed, not abs(): charge current is positive (it is what raises
          current_charge_level). A docked cleaner drawing from the pack while
          the charger delivers nothing reads negative, and abs() counted that
          drain as a working charge - hiding the very failure the no-current
          shutdown below exists for. */
       current_flowing = (current_filt_mA >= CHARGE_CURRENT_MIN_MA);
     }

#ifdef SERIAL_DEBUG
     if (sw_timer_is_elapsed(&cell_log_timer, CELL_LOG_PERIOD_MS))
     {
       sw_timer_start(&cell_log_timer);
       serial_debug_send_cell_voltages();
     }
#endif

     /*
      * Current flowing resets the clock; the absence of it runs it down. A
      * pack that cannot charge must not sit on a dead dock emptying itself -
      * see CHARGE_NO_CURRENT_TIMEOUT_MS for what that cost the first time.
      */
     if (current_flowing)
     {
       sw_timer_start(&noflow_timeout);
     }
     else if (sw_timer_is_elapsed(&noflow_timeout, CHARGE_NO_CURRENT_TIMEOUT_MS))
     {
       /*
        * Not every stalled current is a fault. A charger in its constant
        * voltage phase tapers to nothing by design, and on a pack that is
        * nearly full that taper can outlast the timeout - shutting down there
        * would abandon the last few percent of a charge that is working
        * perfectly.
        *
        * The hazard this guards against is a pack that cannot fill, sitting on
        * a dock burning what it has left. A pack already at the release
        * threshold has nothing left to lose, so above it, keep waiting.
        *
        * Checked only once the timeout has expired, so the cell read costs
        * nothing in the normal case.
        */
       uint16_t *v_now   = bq7693_get_cell_voltages();
       uint16_t  v_max   = 0;

       for (int i = 0; i < 7; ++i)
       {
         if (v_now[i] > v_max)
         {
           v_max = v_now[i];
         }
       }

       /*
        * The release threshold in force, not the full-charge one: a stored pack
        * finishes at CELL_STORAGE_CHARGE_VOLTAGE, far below the full release
        * level, so comparing against that would never recognise its taper.
        */
       const uint16_t v_release = bms_in_storage_mode() ? CELL_STORAGE_RELEASE_VOLTAGE
                                                        : CELL_FULL_CHARGE_RELEASE_VOLTAGE;

       if (v_max >= v_release)
       {
         /* Nearly there - keep waiting. No continue: that skipped the safety
            checks and the loop delay below for the pass. */
         sw_timer_start(&noflow_timeout);
       }
       else
       {
         port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
         bq7693_disable_charge();
         bms_balance_stop();
         leds_off();

         serial_debug_send_message("BMS:CHARGE_NO_CURRENT timeout, shutting down\r\n");
#ifdef SERIAL_DEBUG
         serial_debug_send_cell_voltages();
         bms_debug_dump_pins("no_current");
#endif
         bms_state = BMS_SLEEP;
         return;
       }
     }

     if (current_flowing != current_flowing_prev)
     {
       BMS_PRINT("BMS:CHARGING current %s\r\n", current_flowing ? "flowing" : "ABSENT");
       /* The interesting moment: charge is commanded, so every output on the
          charge path should be asserted. Whatever is not, is the lead. */
       bms_debug_dump_pins(current_flowing ? "chg_ok" : "chg_absent");
       bms_debug_dump_temps(current_flowing ? "chg_ok" : "chg_absent");
       current_flowing_prev = current_flowing;
     }

     if (current_flowing)
     {
       uint8_t duty_loc = charging_leds_duty;

       if(charging_leds_duty > DUTY_MAX)
       {
         duty_loc = ((DUTY_MAX * 2) - charging_leds_duty);
       }

       (duty_loc < 10) ? duty_loc = 0 : (duty_loc);

       leds_set_led_duty(LEDS_LED_ERR_RIGHT, duty_loc);
       leds_set_led_duty(LEDS_LED_ERR_LEFT,  duty_loc);
       charging_leds_duty = (charging_leds_duty + ((charging_leds_duty > 20) ? 10 : 1)) % ((DUTY_MAX * 2) + 1);
     }
     else
     {
       /*
        * Commanded, but nothing is flowing. One LED, hard on/off, so it cannot
        * be read as the smooth two-LED breathe. Driven from a timer rather
        * than leds_blink_*(), which spin inside sw_timer_delay_ms() and would
        * stall every check below for the length of the blink.
        */
       charging_leds_duty = 0;
       leds_set_led_duty(LEDS_LED_ERR_RIGHT, 0);

       if (sw_timer_is_elapsed(&noflow_led_timer, CHARGE_NO_CURRENT_LED_MS))
       {
         sw_timer_start(&noflow_led_timer);
         noflow_led_on = !noflow_led_on;
         leds_set_led_duty(LEDS_LED_ERR_LEFT, noflow_led_on ? 100 : 0);
       }
     }

    // Detect trigger pushes for eeprom reset (20 pushes = reset)
    {
      bool trigger_now = dio_read(DIO_TRIGGER_PRESSED);
      if (trigger_now && !trigger_was_pressed)
      {
        // Rising edge detected
        trigger_push_count++;
        sw_timer_start(&trigger_timeout_timer);

        if (trigger_push_count >= 20)
        {
          eeprom_write_defaults();
          trigger_push_count = 0;
          leds_off();
          leds_blink_leds_num(LEDS_LED_ERR_LEFT, 10, 100);
        }
      }
      trigger_was_pressed = trigger_now;

      // If trigger not pressed for >2 seconds, reset counter
      if (trigger_push_count > 0 && sw_timer_is_elapsed(&trigger_timeout_timer, 2000))
      {
        trigger_push_count = 0;
      }
    }

    if (bms_fault_pending())
    {
      /* raised from interrupt context - honour it instead of overwriting it */
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();
      bms_balance_stop();
      leds_off();
      return;
    }

    if (!bms_is_safe_to_charge())
    {
      //Safety error.
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      bms_balance_stop();
      leds_off();
      bms_state = BMS_FAULT;
      return;
    }

    //An absorbed AFE transient has taken CHG_ON away. Nothing else in this
    //loop re-enables it, so without this the charge just stops.
    if (bms_afe_fet_dropped() && !bms_charge_fet_on())
    {
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();
      bms_balance_stop();
      leds_off();
      bms_set_error(BMS_ERR_I2C_FAIL);
      bms_state = BMS_FAULT;
      return;
    }

    if ( !dio_read(DIO_CHARGER_CONNECTED))
    {
      //Charger unplugged.
      //Turn off charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();
      bms_balance_stop();

      // Re-enable discharge FET only if a vacuum is currently connected;
      // otherwise leave it to the idle loop's vacuum-connect edge. A failure
      // here is not fatal for the same reason: the idle loop re-arms it.
      if (dsn_prot_get_vacuum_connected() && bms_is_safe_to_discharge())
      {
        (void)bms_discharge_fet_on();
      }

      leds_off();
      bms_state = BMS_CHARGER_UNPLUGGED;
      return;
    }

    if (bms_is_pack_full())
    {
      charging_leds_duty = 0;
      leds_off();
#ifdef SERIAL_DEBUG
      BMS_PRINT("BMS:CHARGING Paused - full, attempt %d of %d\r\n", charge_pause_counter, FULL_CHARGE_PAUSE_COUNT);
      serial_debug_send_cell_voltages();
      debug_print_cnt = 0;
#endif
      //Pause the charging.
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      //Pause, then go and try again. Stepped at 250ms rather
      //than 1s so the balancing LED alternation is not aliased, and so an
      //unplugged charger is noticed within a quarter second.
      {
        sw_timer pause_timer = 0;
        sw_timer_start(&pause_timer);

        while (!sw_timer_is_elapsed(&pause_timer, FULL_CHARGE_PAUSE_MS))
        {
          sw_timer_delay_ms(250);
          wdt_reset_count();
          //If it has, abandon the charge process and return to main loop
          if (!dio_read(DIO_CHARGER_CONNECTED))
          {
            //Charger's been unplugged.
            bms_balance_stop();
            if (dsn_prot_get_vacuum_connected() && bms_is_safe_to_discharge())
            {
              (void)bms_discharge_fet_on();
            }
            leds_off();
            bms_state = BMS_CHARGER_UNPLUGGED;
            return;
          }

          //Once the cells have relaxed, start bleeding the high ones. The bulk
          //of the balancing happens later in BMS_CHARGER_CONNECTED_NOT_CHARGING
          //- at ~2-3mA there is nothing meaningful to gain in 30 seconds - but
          //starting here means a pack that is already close only needs the pause.
          if (sw_timer_get_elapsed_time(&pause_timer) >= CELL_BALANCE_RELAX_MS)
          {
            (void)bms_balance_tick();
          }

          bms_balance_leds();
        }
      }
      if (bms_balance_overvoltage())
      {
        //A cell is at the OV guard: no more charge goes in. Count this as
        //the last pause so the full-charge path below runs, rather than
        //resuming and pausing again at CELL_FULL_CHARGE_VOLTAGE 50ms later.
        BMS_PRINT("BMS:CHARGING OV guard, treating as full\r\n");
        charge_pause_counter = FULL_CHARGE_PAUSE_COUNT;
      }
      else
      {
        charge_pause_counter++;
      }

      //Balancing decisions are only valid on relaxed cells, so stop bleeding
      //before the charge current comes back. (Read the OV verdict above
      //first - this resets it.)
      bms_balance_stop();

      if (charge_pause_counter < FULL_CHARGE_PAUSE_COUNT)
      {
        //Restart charging
        port_pin_set_output_level(ENABLE_CHARGE_PIN, true);
        //The pause left the current at zero by design - do not let either
        //supervision above read that as a dead charger.
        sw_timer_start(&charge_flow_timer);
        sw_timer_start(&noflow_timeout);
        /* And the cell dump, which otherwise fires on the first pass after the
           pause - while the current is still ramping. The AFE samples the
           seven channels in sequence, so each one catches a different point of
           the ramp and the readings come out scrambled: cells that have held
           the same order for hours appear to swap places. Nothing is wrong;
           the snapshot is just taken across a moving target. */
        sw_timer_start(&cell_log_timer);
        if (!bms_charge_fet_on())
        {
          port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
          bq7693_disable_charge();
          leds_off();
          bms_set_error(BMS_ERR_I2C_FAIL);
          bms_state = BMS_FAULT;
          return;
        }
      }
    }
    else
    {
#ifdef SERIAL_DEBUG
      /*
       * Slower when nothing is flowing, and carrying the cell voltages: the
       * current is known to be zero and says nothing more, whereas whether the
       * cells hold steady while charge is commanded is exactly the question.
       * 20 passes of the 50ms loop is about a second, against ~300ms when
       * there is a real charge to follow.
       */
      if(++debug_print_cnt > (current_flowing ? 5u : 20u))
      {
        BMS_PRINT("BMS:CHARGING I:%d mA @ %ld mAH, C:%ld mAH, T:%s%d.%d/%s%d.%d 'C, P:%d mV\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), T_SIGN(pack_temperature), T_WHOLE(pack_temperature), T_FRAC(pack_temperature), T_SIGN(pack_temperature_2), T_WHOLE(pack_temperature_2), T_FRAC(pack_temperature_2), bq7693_get_pack_voltage());
        debug_print_cnt = 0;
      }
#endif
    }

    if (charge_pause_counter >= FULL_CHARGE_PAUSE_COUNT)
    {
      //After FULL_CHARGE_PAUSE_COUNT pauses, we are full.
      //Disable the charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      leds_off();

      bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;

      /*
       * None of the learning below applies to a charge that stopped at the
       * storage level. The pack is not full, so calling the coulombs counted
       * "the capacity" would shrink the learned figure by a third, and
       * declaring the level equal to it would put the gauge at 100% on a pack
       * holding about sixty percent. The coulomb counter carries on tracking
       * the real level either way - that part needs no help.
       */
      if (bms_in_storage_mode())
      {
        BMS_PRINT("BMS:CHARGING Stopped at storage level\r\n");
        (void)eeprom_write();
        return;
      }

      if (eeprom_data.full_discharge_seen)
      {
        /*
         * A full discharge-to-charge cycle was observed, so the coulomb count
         * accumulated since empty IS the pack capacity - in either direction.
         * This is the only path that may raise the figure, which is what stops
         * a single bad cycle from being permanent.
         */
        eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;
        eeprom_data.full_discharge_seen = 0;
      }
      else
      {
        /*
         * No full cycle to calibrate against, so only let the estimate decay
         * towards the measured value, never jump up on partial-cycle noise.
         */
        int32_t gap = eeprom_data.total_pack_capacity - eeprom_data.current_charge_level;
        if (gap > 0)
          eeprom_data.total_pack_capacity -= gap >> 3;
      }

      /* Never learn an implausibly small pack either - a mid-charge fault or a
         charger unplugged early would otherwise be taken as the real capacity. */
      if (eeprom_data.total_pack_capacity < (int32_t)PACK_CAPACITY_LOWER_BOUND_UAH)
        eeprom_data.total_pack_capacity = (int32_t)PACK_CAPACITY_LOWER_BOUND_UAH;

      // Clamp to upper bound
      if (eeprom_data.total_pack_capacity > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH)
        eeprom_data.total_pack_capacity = (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH;

      // we are full
      eeprom_data.current_charge_level = eeprom_data.total_pack_capacity;

      /*
       * Commit here, where the learning happened.
       *
       * The commit points were sleep and fault, on the theory that sleep is
       * where a pack always ends up. A docked pack does not sleep - it goes to
       * standby, which keeps RAM but writes nothing - so a capacity just
       * learned could sit uncommitted for weeks and be lost to any power cut,
       * or to a reflash. It was: a charge that finished at 4249mAh came back
       * as the 2250 seed.
       *
       * A rare event with both FETs off, and eeprom_write() skips the erase
       * when nothing has changed, so it costs nothing to put the commit where
       * the value is actually produced.
       */
      (void)eeprom_write();

      BMS_PRINT("BMS:CHARGING Stopped\r\n");
#ifdef SERIAL_DEBUG
      serial_debug_send_pack_capacity();
#endif
      return;
    }


    sw_timer_delay_ms(50);
  }
}

/** @brief Charger unplugged: show cell balance via LED blinks. */
static void bms_handle_charger_unplugged(void)
{
  bms_balance_stop();
  bms_debug_dump_temps("chg_out");

  //Do a little flash to show how out of sync the pack is, then go to idle.
  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  uint8_t highest_cell = 0;
  uint8_t lowest_cell = 0;

  for (int i=0; i < 7; ++i)
  {
    if (cell_voltages[i] > cell_voltages[highest_cell])
    {
      highest_cell = i;
    }
    if (cell_voltages[i] < cell_voltages[lowest_cell])
    {
      lowest_cell = i;
    }
  }

  uint16_t spread = cell_voltages[highest_cell] - cell_voltages[lowest_cell];

  //Flash the error led for 100ms for each 50mV the pack is out of balance
  for (int i = 0; i < round(spread/50); ++i)
  {
    leds_blink_leds(100);
  }

#ifdef SERIAL_DEBUG
  BMS_PRINT("Charger unplugged\r\n");
  serial_debug_send_cell_voltages();
#endif

  bms_state = BMS_IDLE;
}

/** @brief RTC callback - sets wakeup flag and generates interrupt that wakes from standby. */
static void rtc_wakeup_callback(void)
{
  rtc_wakeup_flag = true;
}

/** @brief One-time RTC init: configure peripheral and register callback. */
static void rtc_standby_timer_init(void)
{
  struct rtc_count_config config;

  rtc_count_get_config_defaults(&config);
  config.prescaler         = RTC_COUNT_PRESCALER_DIV_1024;
  config.mode              = RTC_COUNT_MODE_32BIT;
  config.clear_on_match    = true;
  config.compare_values[0] = RTC_STANDBY_WAKE_TICKS;

  rtc_count_init(&rtc_instance, RTC, &config);

  rtc_count_register_callback(&rtc_instance, rtc_wakeup_callback,
                              RTC_COUNT_CALLBACK_COMPARE_0);
}

/** @brief Start the RTC standby wakeup timer (reset count and enable). */
static void rtc_standby_timer_start(void)
{
  rtc_count_set_count(&rtc_instance, 0);
  rtc_wakeup_flag = false;

  // Clear any stale compare-match flag and NVIC pending bit.
  // rtc_count_disable() does NOT clear the NVIC pending bit, so a leftover
  // pending RTC IRQ would fire the moment system_interrupt_enable() runs
  // inside rtc_count_enable(), setting rtc_wakeup_flag before standby.
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_MASK;
  NVIC_ClearPendingIRQ(RTC_IRQn);

  rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_enable(&rtc_instance);
  // wait for ENABLE write to sync across clock domains before entering standby
  while (RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY);
}

/** @brief Stop and disable the RTC standby wakeup timer. */
static void rtc_standby_timer_stop(void)
{
  rtc_count_disable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_disable(&rtc_instance);
}

/** @brief Switch EIC to low-power oscillator and enable wakeup interrupts for standby. */
static void bms_enter_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  bms_wdt_deinit();

  /* 1) Stop EIC while changing its clock */
  _extint_disable();
  /* 2) Disable the generic clock channel feeding EIC */
  system_gclk_chan_disable(EIC_GCLK_ID);
  /* 3) Route a new generator to EIC */
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_3; // low power 32khz oscillator
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  /* 4) Enable it again */
  system_gclk_chan_enable(EIC_GCLK_ID);
  /* 5) Start EIC again */
  _extint_enable();

  // Edge detection runs on these channels all the time, only the interrupt
  // was masked - so an edge from hours ago is still flagged, and enabling the
  // callback would fire on it at once and count as a wake. Drop the stale
  // flags first; anything that happens from here on is a real wake.
  bms_wake_event = false;
  extint_chan_clear_detected(9);
  extint_chan_clear_detected(4);
  extint_chan_clear_detected(6);

  // enable callbacks, need to wakeup the mcu
  extint_chan_enable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);  // MODE_BUTTON            EXTINT 9 - PA09
  extint_chan_enable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);  // TRIGGER_PRESSED_PIN    EXTINT 4 - PA04
  extint_chan_enable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);  // CHARGER_CONNECTED_PIN  EXTINT 6 - PA06
  // disable alert callback
  extint_chan_disable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);
}

/** @brief Restore EIC to main oscillator and disable wakeup interrupts. */
static void bms_leave_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  /* 1) Stop EIC while changing its clock */
  _extint_disable();
  /* 2) Disable the generic clock channel feeding EIC */
  system_gclk_chan_disable(EIC_GCLK_ID);
  /* 3) Route a new generator to EIC */
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  /* 4) Enable it again */
  system_gclk_chan_enable(EIC_GCLK_ID);
  /* 5) Start EIC again */
  _extint_enable();

  // return from sleep, disable external interrupts
  extint_chan_disable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);  // MODE_BUTTON            EXTINT 9 - PA09
  extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);  // TRIGGER_PRESSED_PIN    EXTINT 4 - PA04
  extint_chan_disable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);  // CHARGER_CONNECTED_PIN  EXTINT 6 - PA06
  // enable alert callback
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  bms_wdt_init();
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
