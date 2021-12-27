/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    templates/i2c_lld.h
 * @brief   I2C Driver subsystem low level driver header template.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef _I2C_LLD_H_
#define _I2C_LLD_H_

#if HAL_USE_I2C || defined(__DOXYGEN__)

#include "bcm2835.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/


/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct I2CDriver I2CDriver;

/**
 * @brief   I2C status type
 */
typedef uint32_t i2cstatus_t;

/**
 * @brief   I2C flags type
 */
typedef uint32_t i2cflags_t;

/**
 * @brief   I2C address type
 * @details I2C address type. May support 10 bit addressing in the future.
 */
typedef uint16_t i2caddr_t;

/**
 * @brief   I2C completion callback type.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] sts       operation status
 */
typedef void (*i2ccallback_t)(I2CDriver *i2cp, i2cstatus_t sts);

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /** @brief I2C bus bit rate.*/
  uint32_t                  ic_speed;

  // TODO: Would adding this field break existing code? (Currently unimplemented.)
  /** @brief   Which GPIO pin alt-mode to assume when initializing.
   *
   *  @details This effectively decides which GPIO pins are used to service
   *           the I2C controller that's being configured.
   *
   *           The valid values are GPFN_ALT0 through GPFN_ALT5. If some other
   *           value is placed here, GPFN_ALT0 will be assumed. If the given
   *           alt-mode does not define any GPIO pins for the controller
   *           being configured, then no GPIO pins will be set into I2C mode
   *           for that control (e.g. when `i2cStart` is called), which
   *           renders that controller inaccessible. (This behavior prevents
   *           different pins from different alt-modes from being accidentally
   *           set into I2C mode.)
   *
   *           As an example: configuring controller #1 to use alt-mode 0
   *           (GPFN_ALT0) will cause GPIO pins 2 and 3 to be set as SDA1 and SCL1,
   *           respectively. Likewise, configuring controller #1 to use the
   *           alt-mode 2 (GPFN_ALT2) will cause GPIO pins 44 and 45 to be
   *           set as SDA1 and SCL1.
   */
  //uint8_t                   alt_mode;
  /* End of the mandatory fields.*/
} I2CConfig;

/**
 * @brief   Structure representing an I2C driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct I2CDriver {
  /** @brief Driver state.*/
  i2cstate_t                state;
  /** @brief Current configuration data.*/
  const I2CConfig           *config;
  /** @brief Error flags.*/
  i2cflags_t                errors;
  /** @brief BSC device registers.*/
  bscdevice_t               *device;
#if I2C_USE_MUTUAL_EXCLUSION
#if CH_USE_MUTEXES
  Mutex                     mutex;
#endif /* CH_USE_MUTEXES */
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Thread waiting for I/O completion.
   */
  Thread                    *thread;
  /**
   * @brief   Address of slave device.
   */
  i2caddr_t                 addr;
  /**
   * @brief   Pointer to the buffer with data to send.
   */
  const uint8_t             *txbuf;
  /**
   * @brief   Number of bytes of data to send.
   */
  size_t                    txbytes;
  /**
   * @brief   Current index in buffer when sending data.
   */
  size_t                    txidx;
  /**
   * @brief   Pointer to the buffer to put received data.
   */
  uint8_t                   *rxbuf;
  /**
   * @brief   Number of bytes of data to receive.
   */
  size_t                    rxbytes;
  /**
   * @brief   Current index in buffer when receiving data.
   */
  size_t                    rxidx;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define i2c_lld_master_start(i2cp, header)

#define i2c_lld_master_stop(i2cp)

#define i2c_lld_master_restart(i2cp)

#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

#if ( BCM2835_I2C_USE_I2C0 \
  /*|| (defined(BOARD_...) || ...)*/ )
  extern I2CDriver I2CD0;
  #define BCM2835_I2C_BSC0_COUNT_ 1
#else
  #define BCM2835_I2C_BSC0_COUNT_ 0
#endif

#if ( BCM2835_I2C_USE_I2C1 \
  || (defined(BOARD_RP_ZERO) /* || ... */) )
  extern I2CDriver I2CD1;
  #define BCM2835_I2C_BSC1_COUNT_ 1
#else
  #define BCM2835_I2C_BSC1_COUNT_ 0
#endif

#if BCM2835_I2C_USE_I2C2
  extern I2CDriver I2CD2;
  #define BCM2835_I2C_BSC2_COUNT_ 1
#else
  #define BCM2835_I2C_BSC2_COUNT_ 0
#endif

#define BCM2835_I2C_CONTROLLER_INITIAL_COUNT_ \
  ( BCM2835_I2C_BSC0_COUNT_ \
  + BCM2835_I2C_BSC1_COUNT_ \
  + BCM2835_I2C_BSC2_COUNT_ )

// Backwards compatibility:
//
// If the caller didn't specify which I2C controller to use, and we don't have
// a board definition telling us which one makes sense, then fall back to using
// the zero'th controller.
//
// If not for backwards compability, this would be undesirable, as it can lead
// to bugs (e.g. mysteriously non-working I2C busses). The caller should be
// explicit about what controller they want to use, or provide enough contextual
// information (ex: which board) that we can "figure out" which I2C makes sense.
//
#if BCM2835_I2C_CONTROLLER_INITIAL_COUNT_ == 0
  extern I2CDriver I2CD0;
  #define BCM2835_I2C_BSC0_ENABLED_ 1
  #define BCM2835_I2C_BSC0_COMPAT_  1
  #define BCM2835_I2C_CONTROLLER_COUNT_ (1)
#else
  #define BCM2835_I2C_CONTROLLER_COUNT_ BCM2835_I2C_CONTROLLER_INITIAL_COUNT_
#endif

#ifndef BCM2835_I2C_BSC0_ENABLED_
  #define BCM2835_I2C_BSC0_ENABLED_  BCM2835_I2C_BSC0_COUNT_
#endif

#ifndef BCM2835_I2C_BSC1_ENABLED_
  #define BCM2835_I2C_BSC1_ENABLED_  BCM2835_I2C_BSC1_COUNT_
#endif

#ifndef BCM2835_I2C_BSC2_ENABLED_
  #define BCM2835_I2C_BSC2_ENABLED_  BCM2835_I2C_BSC2_COUNT_
#endif

#endif /* !defined(__DOXYGEN__) */


#if (BCM2835_I2C_BSC0_ENABLED_ && !defined(BOARD_NAME)) || defined(__DOXYGEN__)
/**
 * @brief   Original symbol used to refer to the BCM2835's 0th I2C controller.
 *
 * @details Replaced by I2CD0, as this naming is more consistent with other
 *          multiple-I2C-controller chips in the ChibiOS project (ex: STM32).
 *
 * @deprecated
 */
#define I2C0 I2CD0
#endif

#ifdef BOARD_NAME

  #if defined(BOARD_RP_ZERO) || defined(__DOXYGEN__)
/**
 * @brief   The default I2C controller for the board that the BCM2835 is installed on.
 *
 * @details Of the I2C controllers with GPIO pins exposed on the selected
 *          board configuration (ex: `$(CHIBIOS)boards/RP_ZERO`) and that are
 *          also not reserved by Broadcom or the board producer/manufacturer
 *          (ex: Raspberry Pi), this will refer to the I2C controller with
 *          its SCL line on the lowest-numbered GPIO pin.
 */
    #define I2CD I2CD1

/**
 * @brief   The number of the default I2C controller.
 *
 * @details For example: if I2CD is defined as I2CD1, then this would expand to `1`.
 */
    #define I2C_DEFAULT_DRIVER_NUMBER   1

  /* It is possible to set the I2C controller in single-controller setups as
   * the default controller, but could lead to awkward situations:
   * "why did I2CD work, but then I added a second controller, and it no longer works?"
   * Probably not worth it.
   * Here's what it looks like if we decided that it were worth it:
  #elif (BCM2835_I2C_CONTROLLER_COUNT_ == 1)
    // Explicit single-controller usage.
    // (Don't use derivatives like `BCM2835_I2C_BSC0_ENABLED_`, because
    // if `BCM2835_I2C_BSC0_ENABLED_ && !BCM2835_I2C_USE_I2C0`, it means
    // the caller didn't explicitly enable that controller, so it's not
    // a reasonable default.
    #if   BCM2835_I2C_USE_I2C0
      #define I2CD I2CD0
      #define I2C_DEFAULT_DRIVER_NUMBER 0
    #elif BCM2835_I2C_USE_I2C1
      #define I2CD I2CD1
      #define I2C_DEFAULT_DRIVER_NUMBER 1
    #elif BCM2835_I2C_USE_I2C2
      #define I2CD I2CD2
      #define I2C_DEFAULT_DRIVER_NUMBER 2
    #endif
  */
  #elif I2C_IGNORE_MISSING_BOARD
    // This branch exists to make it possible to ignore the below
    // "This board [...] does not have a default I2C driver" error.
  #else
    #error "This board for the BCM2835 does not have a default I2C driver assigned," \
      " so the I2CD macro is unavailable. Either provide a default for this board (ideally)," \
      " or if that's not practical, define the I2C_IGNORE_MISSING_BOARD macro to disable this" \
      " error and leave the I2CD macro undefined (ex: pass -DI2C_IGNORE_MISSING_BOARD to your" \
      " C compiler)."
  #endif

#endif


#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);

  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr, 
                                       const uint8_t *txbuf, size_t txbytes, 
                                       uint8_t *rxbuf, const uint8_t rxbytes, 
                                       systime_t timeout);

  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr, 
                                       uint8_t *rxbuf, size_t rxbytes, 
                                       systime_t timeout);

  void i2c_lld_serve_interrupt(I2CDriver *i2cp);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C */

#endif /* _I2C_LLD_H_ */

/** @} */
