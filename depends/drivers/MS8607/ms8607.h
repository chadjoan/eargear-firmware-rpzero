/**
 * \file ms8607.h
 *
 * \brief ms8607 Temperature, pressure and humidity sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 */

#ifndef MS8607_H_INCLUDED
#define MS8607_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Types

/*
typedef uint8_t ms8607_status;
#define ms8607_ok     ((ms8607_status)1)
#define ms8607_error  ((ms8607_status)0)
*/

// Enums

enum ms8607_humidity_i2c_master_mode {
	ms8607_i2c_hold,
	ms8607_i2c_no_hold
};

enum ms8607_status {
	ms8607_status_ok,
	ms8607_status_error_within_callback,
	ms8607_status_eeprom_is_zero, // Formerly ms8607_status_crc_error
	ms8607_status_eeprom_crc_error, // Formerly ms8607_status_crc_error
	ms8607_status_measurement_invalid, // Formerly ms8607_status_i2c_transfer_error
	ms8607_status_heater_on_error,
};

enum ms8607_humidity_resolution {
	ms8607_humidity_resolution_12b = 0,
	ms8607_humidity_resolution_8b,
	ms8607_humidity_resolution_10b,
	ms8607_humidity_resolution_11b
};

enum ms8607_battery_status {
	ms8607_battery_ok,
	ms8607_battery_low
};

enum ms8607_heater_status {
	ms8607_heater_off,
	ms8607_heater_on
};

enum ms8607_pressure_resolution {
	ms8607_pressure_resolution_osr_256 = 0,
	ms8607_pressure_resolution_osr_512,
	ms8607_pressure_resolution_osr_1024,
	ms8607_pressure_resolution_osr_2048,
	ms8607_pressure_resolution_osr_4096,
	ms8607_pressure_resolution_osr_8192
};

// Structs
/**
 * \brief  This structure is used by the MS8607 driver to specify I2C transfers
 *         for the callbacks that implement I2C functionality for the driver.
 */
typedef struct ms8607_i2c_controller_packet {
	/// \brief Address to peripheral device
	uint16_t address;

	/// \brief Length of data array
	uint16_t data_length;

	/// \brief Data array containing all data to be transferred
	uint8_t *data;
} ms8607_i2c_controller_packet;

/**
 * \brief    This structure allows the caller to provide implementations for the
 *           MS8607 driver's dependencies, which are mostly I2C functionality.
 *
 * \details  Some patterns are employed to keep the driver and callbacks
 *           flexible and thread-safe:
 *    - A `caller_context` pointer is passed from the caller into the MS8607
 *        driver, and then from the MS8607 driver into these callbacks. This is
 *        what allows the caller and their callbacks to communicate and persist
 *        data accross calls into the driver (and to do so without relying on
 *        global variables or thread-local-storage).
 *    - The return value is usually the `ms8607_status` enum, but only one of
 *        two possible enum values shall be returned from the callback:
 *        `ms8607_status_ok` and `ms8607_status_error_within_callback`.
 *    - Returning `ms8607_status_ok` indicates that the callback completed its
 *        operation (usually an I2C transaction) successfully. This tells the
 *        driver that it can continue working.
 *    - Returning `ms8607_status_error_within_callback` indicates that something
 *        went wrong. The driver will typically return from its own function
 *        immediately after receiving this error code from a callback. This
 *        is intended primarily as a way to provide the driver with "go / no-go"
 *        information, and nothing more specific than that. If the caller needs
 *        to return more specific error details from a failed callback,
 *        the `caller_context` argument should be used for this purpose.
 */
typedef struct ms8607_dependencies {
	/**
	 * \brief   Callback that shall implement I2C packet reading (receive|rx) functionality.
	 *
	 * \details This callback (caller-supplied function) shall read an I2C packet
	 *          from the I2C bus that the MS8607 sensor is connected to.
	 *          This allows the MS8607 driver to complete I2C transfers without
	 *          directly depending on any one specific I2C implementation.
	 *
	 *          This callback is required: the MS8607 driver will be unable to
	 *          retrieve readings from the MS8607 sensor without it.
	 *
	 * \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	 *        - ms8607_status_ok : I2C transfer completed successfully
	 *        - ms8607_status_error_within_callback : Problem with i2c transfer
	 */
	enum ms8607_status  (*i2c_controller_read_packet)(void *caller_context, ms8607_i2c_controller_packet *const);

	/**
	 * \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality.
	 *
	 * \details This callback (caller-supplied function) shall write an I2C packet
	 *          to the I2C bus that the MS8607 sensor is connected to.
	 *          This allows the MS8607 driver to complete I2C transfers without
	 *          directly depending on any one specific I2C implementation.
	 *
	 *          This callback is required: the MS8607 driver will be unable to
	 *          retrieve readings from the MS8607 sensor without it.
	 *
	 * \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	 *        - ms8607_status_ok : I2C transfer completed successfully
	 *        - ms8607_status_error_within_callback : Problem with i2c transfer
	 */
	enum ms8607_status  (*i2c_controller_write_packet)(void *caller_context, ms8607_i2c_controller_packet *const);

	/**
	 * \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality,
	 *          but does not transmit a "stop" bit at the end of the I2C packet.
	 *
	 * \details This callback (caller-supplied function) shall write an I2C packet
	 *          to the I2C bus that the MS8607 sensor is connected to.
	 *          Unlike the `i2c_controller_write_packet`  function, this version
	 *          shall NOT write a "stop" bit at the end of the packet.
	 *
	 *          If the caller's I2C implementation is not capable of this,
	 *          then it is recommended that the caller provide a stub function
	 *          that returns `ms8607_status_error_within_callback`. The caller
	 *          should then prevent any use of the MS8607's "hold" mode.
	 *
	 *          This callback is optional: the MS8607 driver can retrieve
	 *          readings from the MS8607 sensor without it, but it becomes
	 *          necessary if the sensor is used in "hold" mode
	 *          (`ms8607_humidity_i2c_master_mode : ms8607_i2c_hold`).
	 *
	 * \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	 *        - ms8607_status_ok : I2C transfer completed successfully
	 *        - ms8607_status_error_within_callback : Problem with i2c transfer
	 */
	enum ms8607_status  (*i2c_controller_write_packet_no_stop)(void *caller_context, ms8607_i2c_controller_packet *const);

	/**
	 * \brief   Callback that shall wait for the given number of milliseconds when called.
	 *
	 * \details If the caller is operating in a multi-threaded environment
	 *          (including software-based schedulers running on single-threaded
	 *          processors), then it is perfectly acceptable to yield this time
	 *          to other threads or fibers.
	 */
	void                (*delay_ms)(void *caller_context, uint32_t milliseconds); // TODO: this might disappear after the driver implements proper polling primitives
} ms8607_dependencies;

// Functions

/**
 * \brief Configures the caller's I2C controller to be used with the MS8607 device.
 *
 * \param[in] ms8607_dependencies : Struct with callbacks that implement I2C controller functions.
 */
void ms8607_init(const ms8607_dependencies*);

/**
 * \brief Check whether MS8607 device is connected
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 *
 * \return bool : status of MS8607
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool ms8607_is_connected(void* caller_context);

/**
 * \brief Reset the MS8607 device
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_reset(void* caller_context);

/**
 * \brief  Set Humidity sensor ADC resolution.
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 * \param[in] ms8607_humidity_resolution : Resolution requested
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_set_humidity_resolution(void *caller_context, enum ms8607_humidity_resolution);

/**
 * \brief Set Pressure sensor ADC resolution.
 *
 * \param[in] ms8607_pressure_resolution : Resolution requested
 *
 */
void ms8607_set_pressure_resolution(enum ms8607_pressure_resolution);

/**
 * \brief Set I2C master mode.
 *
 * \param[in] ms8607_i2c_master_mode : I2C mode
 *
 */
void ms8607_set_humidity_i2c_master_mode(enum ms8607_humidity_i2c_master_mode);

/**
 * \brief Reads the temperature, pressure and relative humidity value.
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 * \param[out] float* : degC temperature value
 * \param[out] float* : mbar pressure value
 * \param[out] float* : %RH Relative Humidity value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 *       - ms8607_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
 *       - ms8607_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
 *       - ms8607_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
 */
enum ms8607_status ms8607_read_temperature_pressure_humidity(void *caller_context, float *, float *, float *);

/**
 * \brief Provide battery status
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 * \param[out] ms8607_battery_status* : Battery status
 *                      - ms8607_battery_ok,
 *                      - ms8607_battery_low
 *
 * \return status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_get_battery_status(void *caller_context, enum ms8607_battery_status*);

/**
 * \brief Enable heater
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_enable_heater(void* caller_context);

/**
 * \brief Disable heater
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_disable_heater(void* caller_context);

/**
 * \brief Get heater status
 *
 * \param[in] void* caller_context : When this function calls any callbacks
 *         (function pointers) from the `ms8607_dependencies` structure,
 *         this will be passed directly to those callbacks' `caller_context`
 *         parameter.
 * \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
 *	                    - ms8607_heater_off,
 *                      - ms8607_heater_on
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_error_within_callback : Error occurred within a ms8607_dependencies function
 */
enum ms8607_status ms8607_get_heater_status(void* caller_context, enum ms8607_heater_status*);

/**
 * \brief Returns result of compensated humidity
 *        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
 *
 * \param[in] float - Actual temperature measured (degC)
 * \param[in] float - Actual relative humidity measured (%RH)
 * \param[out] float *- Compensated humidity (%RH).
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_heater_on_error : Cannot compute compensated humidity because heater is on
 */
enum ms8607_status ms8607_get_compensated_humidity( float, float, float*);

/**
 * \brief Returns the computed dew point
 *        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
 *
 * \param[in] float - Actual temperature measured (degC)
 * \param[in] float - Actual relative humidity measured (%RH)
 * \param[out] float *- Dew point temperature (DegC).
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_heater_on_error : Cannot compute dew point because heater is on
 */
enum ms8607_status ms8607_get_dew_point( float, float, float*);

/**
 * \brief   Returns a string describing the given error code.
 *
 * \details The caller should filter out any cases where `error_code` equals
 *          `ms8607_status_error_within_callback`. The "error within callback"
 *          return value is not returned from the ms8607 driver itself, so the
 *          ms8607 driver has no way to know what caused the error.
 *          This function will still return a valid string constant in those
 *          cases, but it will a very generic message and thus won't be
 *          specific enough for conclusive troubleshooting.
 *
 *          The `ms8607_status_error_within_callback` code is intended to be returned
 *          from function pointers (callbacks) provided to the `ms8607_dependencies`
 *          structure during ms8607 driver initialization with `ms8607_init(...)`.
 *          Since the caller provides these callbacks, it is the caller's
 *          responsibility to either report such errors from within the
 *          callbacks, or to track that error information separately
 *          (ex: by passing a custom struct pointer as the `caller_context` parameter,
 *          then using such a struct to persist more specific error information
 *          past the ms8607 driver function's return).
 *
 *          This function is thread-safe.
 *
 * \return  A non-NULL string describing the given error code.
 *          If the error code is not valid for any reason, the returned string
 *          will simply indicate that the error code was not valid.
 */
const char *ms8607_stringize_error(enum ms8607_status error_code);

#endif /* MS8607_H_INCLUDED */
