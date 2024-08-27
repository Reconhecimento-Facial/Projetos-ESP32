#include <complex.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <rom/ets_sys.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "bme280.h"

#define TAG_BME280 "BME280"

#define SDA_PIN 21
#define SCL_PIN 22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

void i2c_master_init() // I2C master initialization
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE; // Error code

    esp_err_t espRc; // For error checking
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create a new command link handle to queue commands for I2C bus

    i2c_master_start(cmd); // Start condition on I2C bus
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true); // Write to the device address

    i2c_master_write_byte(cmd, reg_addr, true); // Write to the register address
    i2c_master_write(cmd, reg_data, cnt, true); // Write to the register data with the count of bytes to write
    i2c_master_stop(cmd); // Stop condition on I2C bus

    // Execute the command link to send the queued commands to the I2C bus
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    if(espRc == ESP_OK)
    {
        iError = SUCCESS; // If the command was successful, set the error code to success
    }
    else
    {
        iError = FAIL; // If the command was not successful, set the error code to fail
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32_t iError = BME280_INIT_VALUE; // Error code
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create a new command link handle to queue commands for I2C bus

    i2c_master_start(cmd); // Start condition on I2C bus
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true); // Write to the device address
    i2c_master_write_byte(cmd, reg_addr, true); // Write to the register address

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true); // Read from the device address

    if(cnt > 1)
    {
        // Read the data from the register and send an ACK to the device to continue reading
        i2c_master_read(cmd, reg_data, + cnt - 1, I2C_MASTER_ACK);
    }

    // Read the last byte of data from the register and send a NACK to the device to stop reading
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if(espRc == ESP_OK)
    {
        iError = SUCCESS; // If the command was successful, set the error code to success
    } else
    {
        iError = FAIL; // If the command was not successful, set the error code to fail
    }

    i2c_cmd_link_delete(cmd);

    return (s8_t)iError;
}


void BME280_delay_msek(u32 msek) // Delay function
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

void Publisher_Task(void *params) // Task to publish data to the MQTT broker
{
    struct bme280_t bme280 = { // BME280  I2C communication structure
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS1,
        .delay_msec = BME280_delay_msek
    };

    s32 com_rslt; // Communication result variable to store the result of the communication with the BME280 sensor
    s32 v_uncomp_pressure_s32; // Variable to store the uncompensated pressure value
    s32 v_uncomp_temperature_s32; // Variable to store the uncompensated temperature value
    s32 v_uncomp_humidity_s32; // Variable to store the uncompensated humidity value

    com_rslt = bme280_init(&bme280); // Initialize the BME280 sensor
    printf("com_rslt %d\n", com_rslt);

    // Set the oversampling for the temperature, pressure, and humidity
    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X); // Set the oversampling for the pressure to 16x
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_16X); // Set the oversampling for the temperature to 16x
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_16X); // Set the oversampling for the humidity to 16x

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS); // Set the standby time to 1 ms
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16); // Set the filter coefficient to 16

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE); // Set the power mode to normal mode

    if(com_rslt == SUCCESS)
    {
        while(true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second

            // Read the uncompensated pressure, temperature, and humidity values from the BME280 sensor
            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

            double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32); // Compensate the temperature
            char temperature_str[12];
            sprintf(temperature_str, "%.2f degC", temp);

            double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa to hPa
            char pressure_str[10];
            sprintf(pressure_str, "%.2f hPa", press);

            double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32); // Compensate the humidity
            char humidity_str[10];
            sprintf(humidity_str, "%.2f %%", hum);

            if(com_rslt == SUCCESS)
            {
                printf("Temperature: %s, Pressure: %s, Humidity: %s\n", temperature_str, pressure_str, humidity_str);
            } else
            {
                ESP_LOGE(TAG_BME280, "measure error, code: %d", com_rslt);
            }
        }
    }
    else
    {
        ESP_LOGE(TAG_BME280, "init or setting error, code: %d", com_rslt);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init(); // Initialize the non-volatile storage (NVS) library
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase the NVS partition if there are no free pages or a new version is found
        ret = nvs_flash_init(); // Initialize the NVS library again
    }
    ESP_ERROR_CHECK(ret);

    i2c_master_init(); // Initialize the I2C master

    // Read the data from the BME280 sensor and publish it to the MQTT broker
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
}

