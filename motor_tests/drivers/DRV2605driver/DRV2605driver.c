#include "DRV2605driver.h"

#define TIMEOUT 200

static const char *TAG = "I2C_SLAVE";

static esp_err_t set_i2c(void){
    i2c_config_t i2c_conf = {};

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = SDA_PIN;
    i2c_conf.scl_io_num = SCL_PIN;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_conf.master.clk_speed = 400000;
    i2c_conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));


    return ESP_OK;
}


bool writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, DRV2605_ADDR, data, sizeof(data), pdMS_TO_TICKS(TIMEOUT));
    return ret == ESP_OK;
}
 
bool readRegister8(uint8_t reg, uint8_t *value)
{
    esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, DRV2605_ADDR, &reg, 1, value, 1, pdMS_TO_TICKS(TIMEOUT));
    return ret == ESP_OK;
}


esp_err_t init_gpio_mot(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<DRV_EN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    return gpio_config(&io_conf);
}

bool mot_init(void) {
    bool status = true;
    uint8_t regbuffer = 0;

    init_gpio_mot();
    gpio_set_level(DRV_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    set_i2c();


    // --- Reinicio y configuración mínima ---
    ESP_LOGI(TAG, "Configurando DRV2605 en modo mínimo para diagnóstico");

    // Poner en standby primero
    writeRegister8(DRV2605_REG_MODE, 0x40); // Standby

    // Salir de standby
    //writeRegister8(DRV2605_REG_MODE, 0x00); // Internal Trigger Mode
    status &= writeRegister8(DRV2605_REG_RTPIN, 0x00);
    // No overdrive
	status &= writeRegister8(DRV2605_REG_OVERDRIVE, 0x00);
	// Clean Sustain
	status &= writeRegister8(DRV2605_REG_SUSTAINPOS, 0x00);
	status &= writeRegister8(DRV2605_REG_SUSTAINNEG, 0x00);
	// Clean Break
	status &= writeRegister8(DRV2605_REG_BREAK,0x00);
	// Set audiomax
	status &= writeRegister8(DRV2605_REG_AUDIOMAX, 0x64);

    // Asegurar modo ERM
    if (!readRegister8(DRV2605_REG_FEEDBACK, &regbuffer)) {
        ESP_LOGE(TAG, "Fallo lectura FEEDBACK");
        return false;
    }
    ESP_LOGI(TAG, "FEEDBACK inicial: 0x%02X", regbuffer);
    //regbuffer &= ~(1 << 7);
    regbuffer |= 1<<7;
    writeRegister8(DRV2605_REG_FEEDBACK, regbuffer);

    // Set Open Loop
    status &= readRegister8(DRV2605_REG_CONTROL3, &regbuffer);
    ESP_LOGI(TAG, "CONTROL3 inicial: 0x%02X", regbuffer);
    regbuffer |= 0x22; // regbuffer |= 0x01;
	status &= writeRegister8(DRV2605_REG_CONTROL3, regbuffer);

    motConfig();



    // Volver a modo interno
    writeRegister8(DRV2605_REG_MODE, 0x00);

    return status;
}



bool autoCalibration(void)
{
    bool status = true;
    uint8_t regbuffer;

    // Set auto calibration mode
    status &= writeRegister8(DRV2605_REG_MODE, AUTOCAL);
    // Set auto calibration time to 1 s
    status &= writeRegister8(DRV2605_REG_CONTROL4, 0x30);
    // Set go register
    status &= writeRegister8(DRV2605_REG_GO, 0x1);
    // Wait for calibration to end
    do
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        readRegister8(DRV2605_REG_GO, &regbuffer);
    } while (regbuffer == 0x01);
    // Check calibration status
    status &= readRegister8(DRV2605_REG_STATUS, &regbuffer);
    if (regbuffer & 0x08)
    {
        status = false;
    }
    return status;
}



bool motConfig(void)
{
    bool status = true;
    uint8_t regbuffer;
autoCalibration();
    // Set rated voltage
    //status &= writeRegister8(DRV2605_REG_RATEDV, 0x85);
    // Set overdrive clamp voltage
    //status &= writeRegister8(DRV2605_REG_CLAMPV, 0x96);
    // Set brake factor, ERM mode and high gain
    status &= readRegister8(DRV2605_REG_FEEDBACK, &regbuffer);
    //regbuffer &= 0x03;
    //regbuffer |= 0x28;
    regbuffer |= (1<<7);//|(0<<4)|0x6; // Como en perception 1|ganancia feedback|6
    status &= writeRegister8(DRV2605_REG_FEEDBACK, regbuffer);
    // Set drive time and setup boost
    status &= readRegister8(DRV2605_REG_CONTROL1, &regbuffer);
    regbuffer &= 0x00;
    regbuffer |= 0x94;
    status &= writeRegister8(DRV2605_REG_CONTROL1, regbuffer);
    // Set bidirectional input mode and brake stabilizer
    status &= readRegister8(DRV2605_REG_CONTROL2, &regbuffer);
    regbuffer |= 0xC0;
 //  regbuffer = 0xF5;
    status &= writeRegister8(DRV2605_REG_CONTROL2, regbuffer);
    // Set ERM Open Loop and PWM input
    status &= readRegister8(DRV2605_REG_CONTROL3, &regbuffer);
    regbuffer |= 0x22;
    //regbuffer = 0x86;
    status &= writeRegister8(DRV2605_REG_CONTROL3, regbuffer);

    return status;
}


void motorSetActiveMode(void)
{
    ESP_LOGI(TAG, "motorSetActiveMode");
    gpio_set_level(DRV_EN, 1);
    uint8_t regbuffer;
    readRegister8(DRV2605_REG_MODE, &regbuffer);
    regbuffer &= 0xBF;
    writeRegister8(DRV2605_REG_MODE, regbuffer);
}

void motorSetOffMode(void)
{
    uint8_t regbuffer;
    readRegister8(DRV2605_REG_MODE, &regbuffer);
    regbuffer |= 0x40;
    writeRegister8(DRV2605_REG_MODE, regbuffer);
    gpio_set_level(DRV_EN, 0);
}

void motorSetInputMode(motInMode mode)
{
    writeRegister8(DRV2605_REG_MODE, mode);
}
