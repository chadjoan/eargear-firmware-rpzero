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
	ms8607_status_no_i2c_acknowledge,
	ms8607_status_i2c_transfer_error,
	ms8607_status_crc_error,
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
typedef struct ms8607_i2c_controller_packet {
	// Address to peripheral device
	uint16_t address;
	// Length of data array
	uint16_t data_length;
	// Data array containing all data to be transferred
	uint8_t *data;
} ms8607_i2c_controller_packet;

typedef struct ms8607_dependencies {
	enum ms8607_status  (*i2c_controller_read_packet)(void *caller_context, ms8607_i2c_controller_packet *const);
	enum ms8607_status  (*i2c_controller_write_packet)(void *caller_context, ms8607_i2c_controller_packet *const);
	enum ms8607_status  (*i2c_controller_write_packet_no_stop)(void *caller_context, ms8607_i2c_controller_packet *const);
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
 * \return bool : status of MS8607
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool ms8607_is_connected(void* caller_context);

/**
 * \brief Reset the MS8607 device
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms8607_status ms8607_reset(void* caller_context);

/**
 * \brief Set Humidity sensor ADC resolution.
 *
 * \param[in] ms8607_humidity_resolution : Resolution requested
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
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
 * \param[out] float* : degC temperature value
 * \param[out] float* : mbar pressure value
 * \param[out] float* : %RH Relative Humidity value
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms8607_status_crc_error : CRC check error
 */
enum ms8607_status ms8607_read_temperature_pressure_humidity(void *caller_context, float *, float *, float *);

/**
 * \brief Provide battery status
 *
 * \param[out] ms8607_battery_status* : Battery status
 *                      - ms8607_battery_ok,
 *                      - ms8607_battery_low
 *
 * \return status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms8607_status ms8607_get_battery_status(void *caller_context, enum ms8607_battery_status*);

/**
 * \brief Enable heater
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms8607_status ms8607_enable_heater(void* caller_context);

/**
 * \brief Disable heater
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms8607_status ms8607_disable_heater(void* caller_context);

/**
 * \brief Get heater status
 *
 * \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
 *	                    - ms8607_heater_off,
 *                      - ms8607_heater_on
 *
 * \return ms8607_status : status of MS8607
 *       - ms8607_status_ok : I2C transfer completed successfully
 *       - ms8607_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms8607_status_no_i2c_acknowledge : I2C did not acknowledge
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

#endif /* MS8607_H_INCLUDED */
