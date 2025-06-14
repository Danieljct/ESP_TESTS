#ifndef SIGNAL_H_
#define SIGNAL_H_

#include <math.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "DRV2605driver.h" // ¡Este es crucial!
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "esp_log.h"


#ifndef SIGNAL_CFG
#define SIGNAL_CFG

// Precomputed 2*pi*f
#define	F1HZ8					11.30973354
#define F23HZ					144.51326190
// Nueva: Frecuencia de la portadora para la señal AM (200 Hz)
#define F200HZ                  (2.0 * M_PI * 200.0)


/* Timer and PWM related settings */
// Changed PWM_FREQ to a higher value suitable for ERM carrier, e.g., 20kHz
// Esta es la frecuencia de la portadora PWM que el ESP32 genera y envía al DRV2605.
// Debe ser lo suficientemente alta para ser inaudible y para que el DRV2605 la procese bien.
#define PWM_FREQ        10000 // Nueva: 20kHz frecuencia de portadora PWM para ERM
#define PWM_RES_BITS    LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)
#define PWM_CHANNEL     LEDC_CHANNEL_3
#define PWM_TIMER       LEDC_TIMER_1
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_GPIO        DRV_IN // Salida PWM del ESP32 hacia el pin IN del DRV2605

/**
 * \brief PWM timer defines
 */
//@{
/* Timer instance */
#define SIGNAL_TIMER_GROUP      TIMER_GROUP_0
#define SIGNAL_TIMER_IDX        TIMER_0
/* Timer clock frequency */
#define TIMER_BASE_CLK          80000000  // 80 MHz
#define SIGNAL_TIMER_DIVIDER    80
#define F_CLK                   (TIMER_BASE_CLK / SIGNAL_TIMER_DIVIDER)  // Frecuencia del reloj del temporizador
/* Timer IRQ number */
#define SIGNAL_TIMER_SCALE      (TIMER_BASE_CLK / SIGNAL_TIMER_DIVIDER)
#endif

/**
 * \brief Pulse settings struct
 */
//@{
typedef struct _pulse_data
{
	/* Pulse mode on/off */
	bool flag;
	/* Pulse period in miliseconds */
	uint16_t period;
	/* Pulse percentage duty cycle */
	uint8_t duty;
	/* Pulse duty cycle time in seconds */
	double dutytime;
	/* Pulse period in seconds */
	double periodtime;
} pulse_data_t;

/**
 * \brief Supported signals
 */
//@{
typedef enum _signal_t
{
	/* No signal */
	SIGNAL_OFF,
	/* Constant max value */
	SIGNAL_MAX,
	/* Sine of 23 Hz */
	SIGNAL_SIN23HZ,
	/* Square of 23 Hz */
	SIGNAL_SQR23HZ,
	/* Sine of 23 Hz multiplied by Sine of 1.8Hz */
	SIGNAL_SIN23HZ_SIN1HZ8,
	/* Square of 23 Hz multiplied by Square of 1.8Hz */
	SIGNAL_SQR23HZ_SQR1HZ8,
	/* Sine of 23 Hz multiplied by Square of 1.8Hz */
	SIGNAL_SIN23HZ_SQR1HZ8,
    /* New: AM Modulated Signal (Carrier 200Hz, Message 23Hz) */
    SIGNAL_AM_MODULATED // Nueva: señal AM modulada
} signal_t;




/**
 * \brief Initializes the signal generator
 *	
 *	Sets the PWM frequency.
 *	Initializes the time variable.
 *	Compute the time increment.
 *	Enables the timer interrupt.
 *	It needs the following timer configuration in Atmel Start:
 *		- PWM API: Lite: TC: PWM
 *		- Mode: 16-bit Waveform mode
 *		- WO/1: Generated signal pin
 *		- Waveform Generation Mode: MPWM
 *		- Some pair of values that accomplish CC0 > CC1 (Both will be overwritten)
 *		- Intenset: Overflow interrupt enable
 * *	\param[in] frequency:	PWM frequency in Hz
 *
 */
void SIGNAL_init(double frequency);

/**
 * \brief Select a signal to generate
 *	
 *	\param[in] mode:		Signal from signal_t
 *
 */
void SIGNAL_select(signal_t mode);

/**
 * \brief Compute the signal value for the current time
 *	
 *	According to selected mode, compute the signal for the current internal time.
 *
 *	\return Signal value between 0 and 1023
 *	
 *
 */
uint16_t SIGNAL_getValue(void);

void SIGNAL_update_time(void);

/**
 * \brief Set a duty cycle for PWM carrier signal	
 *
 *	\param[in] duty:		Duty Cycle between 0 and 1023
 *
 */
void SIGNAL_pwmSetDutyCycle(uint16_t dutycycle);

/**
 * \brief Set a given frequency to PWM
 *
 *	Set frequency to timer register.
 *	Clear duty cycle
 *		
 *	\param[in] freq:		Maximum timer count
 *
 */
void SIGNAL_pwmSetFreq(uint16_t freq);

/**
 * \brief Set the pulse flag
 *	
 *	Call to setPulseTiming is required before pulse enable
 *	
 *	\param[in] st:		true for pulse enable, false for pulse disable
 *
 */
void SIGNAL_setPulseFlag(bool st);

/**
 * \brief Configure pulse period and duty cycle
 *	
 *	\param[in] period_s:		Period in miliseconds
 *	\param[in] duty:			Duty cycle percentage			
 *
 */
void SIGNAL_setPulseTiming(uint16_t period_s, uint8_t duty);

/**
 * \brief Stop the signal generation
 *	
 *	Disables the PWM timer
 *	Clear the duty cycle
 *	Reset current_time variable
 *
 */
void SIGNAL_stop(void);

/**
 * \brief Start the signal generation
 *	
 *	Enables the PWM timer
 *	Set duty cycle to 512
 *	Reset current_time variable
 */
void SIGNAL_start(void);

/**
 * \brief Sets the next duty cycle to load 
 *	
 *	This function only sets a variable. The duty change is in timer interrupt	
 *
 *	\param[in] duty:		Next duty cycle to be set. The given value must be between 0 and 1023
 *
 */
void SIGNAL_setNextDuty(uint16_t duty);

/**
 * \brief Allows to write the current_time variable
 *	
 *	\param[in] val		New current time
 *
 */
void SIGNAL_setCurrentTime(double val);

/**
 * \brief Get the current_time value
 *	
 *	\return current_time value
 *
 */
double SIGNAL_getCurrentTime(void);

/**
 * \brief Clear the update flag
 *
 *	Used to mark an update as attended
 *
 */
void SIGNAL_clearUpdateFlag(void);

/**
 * \brief Get the status of the update flag
 *	
 *	\return update_flag status
 *	\retval true	A new value was computed
 *	\retval false	No new values available
 *
 */
bool SIGNAL_getUpdateFlag(void);

#endif /* SIGNAL_H_ */
