#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

//#define		PWM_FREQ			10000   /* PWM frequency in Hz */
#define		PULSE_PERIOD		6000    /* Pulse period in ms */
#define		PULSE_DUTY			50     /* Percentage of duty cycle of pulse */
#define		MAX_REPETITIONS		50      /* Maximum number of repetitions */
#define     devkit          0       /* Set to 1 for devkit, 0 for production board */

#if devkit
/* GPIO */
#define LED_R 9
#define LED_G 8
#define LED_B 7
#define GND 10

#define BUTTON_0 5
#define BUTTON_1 4
#define DRV_EN 6

/* I2C */
#define SDA_PIN 13
#define SCL_PIN 14

#define SDA_PIN_PMIC 18
#define SCL_PIN_PMIC 19
#endif
/* GPIO */
#define LED_R 21
#define LED_G 18
#define LED_B 17
//#define GND 10

#define BUTTON_0 15
#define BUTTON_1 16
#define DRV_EN 5
#define DRV_IN 12

/* I2C DRV */
#define SDA_PIN 6
#define SCL_PIN 7

#define CONFIG_MAX17055_I2C_SCL_GPIO           9//9   // GPIO pin para I2C SCL
#define CONFIG_MAX17055_I2C_SDA_GPIO           8//8   // GPIO pin para I2C SDA
#define CONFIG_MAX17055_I2C_FREQ_HZ          100000 // Frecuencia de reloj I2C (100 kHz)
#define I2C_MASTER_TX_BUF_LEN       0     // Longitud del buffer de transmisión (0 para master)
#define I2C_MASTER_RX_BUF_LEN       0     // Longitud del buffer de recepción (0 para master)

#define MAX17055_SENSOR_ADDR        0x36  // Dirección I2C del MAX17055


/* i2c lis3dh */
#define LIS3DH_I2C_ADDRESS 0x19
#define LIS3DH_I2C_SDA 3
#define LIS3DH_I2C_SCL 4
#define LIS3DH_I2C_FREQ 400000



#endif