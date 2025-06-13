#ifndef DRV2605DRIVER_H_
#define DRV2605DRIVER_H_


#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "pin_config.h"

#define DRV2605_ADDR 0x5A ///< Device I2C address

/**
 *
 * \brief DRV2605l Haptic Driver for LRA and ERM library
 *	Uses I2C to communicate with the driver 
 *  Based in Adafruit DRV2605 library 
 *	https://github.com/adafruit/Adafruit_DRV2605_Library
 *
 */

/**
 *  \brief Driver input modes
 */
//@{
typedef enum _motInMode
{
	/* Internal trigger mode */
	INTTRIG			=		0x00,
	/* External trigger edge mode */
	EXTTRIGEDGE		=		0x01,
	/* External trigger level mode */
	EXTTRIGLEVEL	=		0x02,
	/* PWM or Analog input mode */
	PWMANALOG		=		0x03,
	/* Audio-to-Vibe mode */
	AUDIOTOVIBE		=		0x04,
	/* Real time playback mode */
	RTPLAYBACK		=		0x05,
	/* Diagnostics test mode */
	DIAGNOSTICS		=		0x06,
	/* Auto calibration mode */
	AUTOCAL			=		0x07
} motInMode;

/* Device Register map */
#define DRV2605_REG_STATUS			(uint8_t) 0x00
#define DRV2605_REG_MODE			(uint8_t) 0x01
#define DRV2605_REG_RTPIN			(uint8_t) 0x02
#define DRV2605_REG_LIBRARY			(uint8_t) 0x03
#define DRV2605_REG_WAVESEQ			(uint8_t) 0x04
#define DRV2605_REG_GO				(uint8_t) 0x0C
#define DRV2605_REG_OVERDRIVE		(uint8_t) 0x0D
#define DRV2605_REG_SUSTAINPOS		(uint8_t) 0x0E
#define DRV2605_REG_SUSTAINNEG		(uint8_t) 0x0F
#define DRV2605_REG_BREAK			(uint8_t) 0x10
#define DRV2605_REG_AUDIOMAX		(uint8_t) 0x13
#define DRV2605_REG_RATEDV			(uint8_t) 0x16
#define DRV2605_REG_CLAMPV			(uint8_t) 0x17
#define DRV2605_REG_FEEDBACK		(uint8_t) 0x1A
#define DRV2605_REG_CONTROL1		(uint8_t) 0x1B
#define DRV2605_REG_CONTROL2		(uint8_t) 0x1C
#define DRV2605_REG_CONTROL3		(uint8_t) 0x1D
#define DRV2605_REG_CONTROL4		(uint8_t) 0x1E
#define DRV2605_REG_CONTROL5		(uint8_t) 0x1F
#define DRV2605_REG_LRAOPENLOOPPERIOD	(uint8_t) 0x20

/* Enable pin port */
#define ENABLE_PIN					DRV_EN
/* I2C sync instance where the driver is connected */
#define I2C_CHANNEL					I2C_0

/**
 * \brief Enables and Initializes the motor driver, setting the default configuration. 
 *	
 *	Leaves the motor in active mode, ready to use.
 *	Default configuration:
 *		- Active Internal Trigger mode
 *		- No real-time playback
 *		- No overdrive
 *		- No sustain time offset
 *		- No brake time offset
 *		- Full-scale voltage level set to app. 0.7 V for Audio-to-Vibe mode
 *		- LRA mode
 *		- Open Loop mode
 *		- Memory Playback time of 1 ms
 *	\return Configuration status
 *	\retval true	The configuration was loaded successfully
 *	\retval false	The I2C communication was failed
 *
 */
bool mot_init(void);

/**
 * \brief Allows to write a register through I2C channel
 *	
 *	\param[in] reg		Register to write value
 *	\param[in] value	8-bit value to write in the given register
 *
 *	\return Write status
 *	\retval true	The register was written successfully
 *	\retval false	The I2C communication was failed
 *
 */
bool writeRegister8(uint8_t reg, uint8_t value);

/**
 * \brief Set the driver to off status
 *	
 *	Put the device in standby mode then, the enable pin is set low
 *
 */
void motorSetOffMode(void);

/**
 * \brief Set the driver to active status
 *	
 * Set up the enable pin, then the standby bit is clean.
 *
 */
void motorSetActiveMode(void);

/**
 * \brief Perform the driver auto calibration routine
 *	
 *	Setup the auto calibration mode and run the calibration for 1 second.
 *	Then, waits for the auto calibration ends.
 *	For a correct calibration, the LRA or ERM device must be connected.
 *	Some registers must be configured before the auto calibration.
 *	
 *	\return Auto calibration status
 *	\retval true	The auto calibration was successfully
 *	\retval false	The calibration was failure
 *
 */
bool autoCalibration(void);

/**
 *	\brief Configures the driver with the old VibeBrain settings
 *	
 *	This configuration must be done before the auto calibration
 *	Configuration:
 *		- Rated Voltage: 2816.96 mV
 *		- Overdrive Clamp Voltage: 3238.5 mV
 *		- Brake factor: 3x
 *		- ERM mode
 *		- Loop Gain: High Gain
 *		- Startup boost: Enabled
 *		- AC Couple: Disabled
 *		- Drive Time: 5 ms
 *		- Bidirectional input mode
 *		- Brake Stabilizer: Enabled
 *		- ERM mode: Open Loop
 *		- Input mode: PWM Input
 *	\return Configuration status
 *	\retval true	The configuration was set successfully
 *	\retval false	The I2C communication was failed
 *
 */
bool motConfig(void);

/**
 * \brief Set the driver input mode in Mode register
 *	
 *	Allows to modify the standby status of driver.
 *	\param[in] mode		Mode to set in driver
 *
 */
void motorSetInputMode(motInMode mode);

#endif /* DRV2605DRIVER_H_ */