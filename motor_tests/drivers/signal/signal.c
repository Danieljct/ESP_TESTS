#include "signal.h"
#include "freertos/FreeRTOS.h" // Para FreeRTOS
#include "freertos/task.h"     // Para tareas de FreeRTOS
#include "freertos/queue.h"    // Aunque usaremos notificaciones, es buena práctica tenerla
#include "esp_log.h"           // Para logs del ESP-IDF

static const char *TAG = "signal";

pulse_data_t pulse_data;


static ledc_channel_config_t pwm_channel;
static ledc_timer_config_t pwm_timer;

/* Timing control */
static bool timer_running = false;
static double current_time = 0;
static double time_increment = 1.0 / PWM_FREQ;
static signal_t signal_mode;
static bool update_flag = false; // Esta bandera ya no es estrictamente necesaria para la comunicación ISR-Task

/* GPTimer (sample timer) */
static gptimer_handle_t gptimer = NULL;

/* Handle para la tarea de procesamiento de señal */
static TaskHandle_t xSignalProcessingTaskHandle = NULL;


// Signal generators
uint16_t SIGNAL_sin23hz(double t)       { return 350 * sin(F23HZ * t) + 600; }
uint16_t SIGNAL_sqr23hz(double t)       { return (sin(F23HZ * t) >= 0) ? 1023 : 512; }
uint16_t SIGNAL_sin23hz_sin1hz8(double t) { return 767 + 255 * sin(F1HZ8 * t) * sin(F23HZ * t); }
uint16_t SIGNAL_sqr23hz_sqr1hz8(double t) { return (sin(F1HZ8 * t) >= 0) ? SIGNAL_sqr23hz(t) : 512; }
uint16_t SIGNAL_sin23hz_sqr1hz8(double t) { return (sin(F1HZ8 * t) >= 0) ? SIGNAL_sin23hz(t) : 512; }

// Nueva función para señal AM modulada
// Carrier: 200 Hz, Message: 23 Hz
uint16_t SIGNAL_am_modulated(double t) {
    // Frecuencia del mensaje: 23 Hz (F23HZ ya definido como 2*pi*23)
    // Frecuencia de la portadora: 200 Hz (F200HZ ya definido como 2*pi*200)

    // Normalizar la señal del mensaje a un rango de [0, 1]
    // La función sin() oscila entre -1 y 1. (sin(x) + 1.0) / 2.0 la escala a [0, 1].
    double normalized_message = (sin(F23HZ * t) + 1.0) / 2.0;

    // Normalizar la señal de la portadora a un rango de [0, 1]
    double normalized_carrier = (sin(F200HZ * t) + 1.0) / 2.0;

    // Definir el rango de amplitud (duty cycle) para la portadora modulada.
    // Estos valores controlan la intensidad mínima y máxima de la vibración.
    // Ajusta estos valores (0-1023) según la respuesta deseada de tu motor ERM.
    double min_amplitude_duty = 0.0;  // Amplitud mínima del ciclo de trabajo (0-1023)
    double max_amplitude_duty = 1023.0; // Amplitud máxima del ciclo de trabajo (0-1023)
    double amplitude_range = max_amplitude_duty - min_amplitude_duty;

    // La amplitud de la portadora (200Hz) es modulada por el mensaje (23Hz).
    // Esta 'amplitud modulada' varía entre min_amplitude_duty y max_amplitude_duty.
    double modulated_carrier_amplitude = min_amplitude_duty + normalized_message * amplitude_range;

    // Aplicar la forma de onda de la portadora (200Hz), escalada por la amplitud modulada.
    // El 'normalized_carrier' (0 a 1) asegura que la señal oscile dentro de la 'modulated_carrier_amplitude'.
    double final_value_float = modulated_carrier_amplitude * normalized_carrier;

    // Asegurar que el valor final del ciclo de trabajo esté dentro del rango [0, 1023].
    // Las operaciones anteriores deberían mantenerlo en este rango si min/max_amplitude_duty están bien definidos.
    if (final_value_float < 0.0) final_value_float = 0.0;
    if (final_value_float > 1023.0) final_value_float = 1023.0;

    return (uint16_t)final_value_float;
}


// Utility
void SIGNAL_clearUpdateFlag(void) { update_flag = false; } // Esta función ya no es relevante
bool SIGNAL_getUpdateFlag(void)   { return update_flag; }   // Esta función ya no es relevante
void SIGNAL_setCurrentTime(double val) { current_time = val; }
double SIGNAL_getCurrentTime(void) { return current_time; }

uint16_t SIGNAL_getValue(void) {
    switch (signal_mode) {
        case SIGNAL_SIN23HZ: return SIGNAL_sin23hz(current_time);
        case SIGNAL_SQR23HZ: return SIGNAL_sqr23hz(current_time);
        case SIGNAL_SIN23HZ_SIN1HZ8: return SIGNAL_sin23hz_sin1hz8(current_time);
        case SIGNAL_SQR23HZ_SQR1HZ8: return SIGNAL_sqr23hz_sqr1hz8(current_time);
        case SIGNAL_SIN23HZ_SQR1HZ8: return SIGNAL_sin23hz_sqr1hz8(current_time);
        case SIGNAL_AM_MODULATED: return SIGNAL_am_modulated(current_time); // Nueva señal AM
        case SIGNAL_MAX: return 512; // Valor constante si es SIGNAL_MAX (o ajusta si es necesario)
        default: return 0; // Valor por defecto si no hay señal seleccionada
    }
}

void SIGNAL_pwmSetDutyCycle(uint16_t duty) {
    uint32_t scaled = (duty > 1023 ? 1023 : duty);
    uint32_t ledc_duty = (scaled * PWM_MAX_DUTY) / 1023; // Escala a la resolución de bits real del PWM
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, ledc_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

// Función de callback del temporizador GPTimer (se ejecuta a PWM_FREQ)
// ¡Ahora es mucho más ligera!
bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    current_time += time_increment; // Incrementa el tiempo para la próxima muestra de la forma de onda
    
    // Notifica a la tarea de procesamiento que hay un nuevo valor para calcular
    if (xSignalProcessingTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xSignalProcessingTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    // Si una tarea de mayor prioridad ha sido despertada, se solicita un cambio de contexto
    return (xHigherPriorityTaskWoken == pdTRUE); 
}

// Tarea de FreeRTOS para procesar la señal y actualizar el PWM
static void signal_processing_task(void *pvParameters) {
    while (1) {
        // Espera una notificación de la ISR del GPTimer
        // No hay timeout, espera indefinidamente hasta que la ISR notifique
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // pdTRUE para limpiar la notificación después de tomarla

        // Calcula el nuevo valor del ciclo de trabajo de la señal seleccionada
        uint16_t val = SIGNAL_getValue();
        // Aplica el nuevo ciclo de trabajo al PWM
        SIGNAL_pwmSetDutyCycle(val);

        // Opcional: imprimir algo para depuración, pero no en producción si es muy frecuente
        // ESP_LOGI(TAG, "Duty Cycle updated: %d", val);
    }
}

// Inicialización del generador de señal
void SIGNAL_init(double frequency) {
    current_time = 0; // Reinicia el tiempo
    time_increment = 1.0 / frequency; // Calcula el incremento de tiempo basado en la frecuencia de muestreo

    // Configuración del temporizador LEDC para el PWM
    pwm_timer = (ledc_timer_config_t) {
        .duty_resolution = PWM_RES_BITS, // Resolución del ciclo de trabajo (ej. 10 bits)
        .freq_hz = PWM_FREQ, // Frecuencia de la portadora PWM (20 kHz)
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // Configuración del canal LEDC para el PWM
    pwm_channel = (ledc_channel_config_t) {
        .channel    = PWM_CHANNEL,
        .duty       = 0, // Inicia con ciclo de trabajo en 0
        .gpio_num   = PWM_GPIO, // Pin GPIO para la salida PWM
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&pwm_channel);

    // Creación de la tarea de procesamiento de señal
    // Se le da una prioridad y tamaño de stack adecuados
    xTaskCreate(signal_processing_task, "signal_proc", 4096, NULL, configMAX_PRIORITIES - 2, &xSignalProcessingTaskHandle);
    if (xSignalProcessingTaskHandle == NULL) {
        ESP_LOGE(TAG, "Failed to create signal processing task");
        return;
    }


    // Configuración del GPTimer para muestrear la forma de onda
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        // La resolución del temporizador debe ser alta para una buena precisión en el conteo
        // Si PWM_FREQ es 20kHz, y queremos que el ISR se dispare cada tick de PWM_FREQ,
        // entonces la resolución debe ser PWM_FREQ * N, donde N es el factor de conteo.
        // Aquí se usa PWM_FREQ * 1000 para una resolución de 20MHz.
        .resolution_hz = PWM_FREQ * 1000 
    };
    gptimer_new_timer(&timer_config, &gptimer);

    // Registro del callback ISR para el GPTimer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_isr_callback,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    // Configuración de la alarma del GPTimer
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0, // No recargar el contador automáticamente
        // alarm_count = (resolution_hz / PWM_FREQ) = (20,000,000 / 20,000) = 1000
        .alarm_count = 1000, 
        .flags.auto_reload_on_alarm = true // Re-activar la alarma automáticamente
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer); // Habilita el GPTimer
    gptimer_start(gptimer);  // Inicia el GPTimer

    timer_running = true;
    ESP_LOGI(TAG, "Signal generation initialized with PWM sampling rate of %f Hz", frequency);
}

// Establece el próximo ciclo de trabajo. (Esta función podría no ser necesaria si el ISR lo hace todo)
void SIGNAL_setNextDuty(uint16_t duty) {
    if (duty > 1023) duty = 1023; // Asegurar límites
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

// Selecciona el modo de señal
void SIGNAL_select(signal_t mode) {
    signal_mode = mode;
}

// Detiene la generación de señal
void SIGNAL_stop(void) {
    if (timer_running) {
        gptimer_stop(gptimer); // Detiene el GPTimer
        timer_running = false;
        // Suspende la tarea para que no intente procesar notificaciones
        if (xSignalProcessingTaskHandle != NULL) {
            vTaskSuspend(xSignalProcessingTaskHandle);
        }
    }
    current_time = 0.0; // Reinicia el tiempo
    // Opcional: poner el duty cycle a 0 para detener el motor
    SIGNAL_pwmSetDutyCycle(0);
}

// Inicia la generación de señal
void SIGNAL_start(void) {
    SIGNAL_stop(); // Asegura que se detenga y reinicie antes de empezar
    gptimer_start(gptimer); // Inicia el GPTimer
    timer_running = true;
    current_time = 0.0; // Reinicia el tiempo
    // Reanuda la tarea si estaba suspendida
    if (xSignalProcessingTaskHandle != NULL) {
        vTaskResume(xSignalProcessingTaskHandle);
    }
    // Opcional: establecer un duty cycle inicial si es necesario
    SIGNAL_pwmSetDutyCycle(512); // P.ej., un valor neutro o a la mitad
}


// Funciones relacionadas con el modo de pulso (si se usa)
void SIGNAL_setPulseTiming(uint16_t period_s, uint8_t duty)
{
    if ((period_s > 0) && (duty > 0) && (duty <= 100))
    {
        pulse_data.period = period_s;
        pulse_data.duty = duty;
        pulse_data.periodtime = 0.001 * period_s;
        pulse_data.dutytime = 0.01 * (duty * pulse_data.periodtime);
    }
    else
    {
        if (pulse_data.flag)
        {
            SIGNAL_stop();
        }
        pulse_data.flag = false;
        pulse_data.period = 0;
        pulse_data.duty = 0;
        pulse_data.dutytime = 0.0; 
    }
}
void SIGNAL_setPulseFlag(bool st)
{
    pulse_data.flag = (st && (pulse_data.period > 0));
}
