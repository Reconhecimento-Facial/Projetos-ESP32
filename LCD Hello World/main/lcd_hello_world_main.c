#include <i2c-lcd.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "lcd_hello_world";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 1000 // I2C timeout in ms

char buffer[50];
double num = 12.35;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main (void) {
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    lcd_init();
    lcd_clear();

    lcd_put_cur(0, 0);
    lcd_send_string("Hello, Softex!");

    lcd_put_cur(1, 0);
    lcd_send_string("from ESP32");
    // sprintf(buffer, "valor: %.2f", num);
    // lcd_put_cur(0, 0);
    // lcd_send_string(buffer);
}