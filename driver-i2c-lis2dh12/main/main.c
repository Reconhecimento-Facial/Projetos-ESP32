/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "lis2dh.h"
#include "unity.h"

#define I2C_MASTER_SCL_IO    22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0  /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0  /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000   /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static lis2dh12_handle_t lis2dh12 = NULL;

/**
 * @brief i2c master initialization
 */
static void lis2dh12_test_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    lis2dh12 = lis2dh12_create(i2c_bus, LIS2DH12_I2C_ADDRESS);
}

static void lis2dh12_test_deinit()
{
    lis2dh12_delete(&lis2dh12);
    i2c_bus_delete(&i2c_bus);
}

static void lis2dh12_test_get_data(void)
{
    uint8_t deviceid;
    uint16_t x_acc;
    uint16_t y_acc;
    uint16_t z_acc;
    int cnt = 100;
    lis2dh12_get_deviceid(lis2dh12, &deviceid);
    printf("LIS2DH12 device id is: %02x\n", deviceid);

    lis2dh12_config_t  lis2dh12_config;
    lis2dh12_get_config(lis2dh12, &lis2dh12_config);
    printf("temp_enable is: %02x\n", lis2dh12_config.temp_enable);
    printf("odr is: %02x\n", lis2dh12_config.odr);
    printf("option mode is: %02x\n", lis2dh12_config.opt_mode);
    printf("z_enable status is: %02x\n", lis2dh12_config.z_enable);
    printf("y_enable status is: %02x\n", lis2dh12_config.y_enable);
    printf("x_enable status is: %02x\n", lis2dh12_config.x_enable);
    printf("bdu_status status is: %02x\n", lis2dh12_config.bdu_status);
    printf("full scale is: %02x\n", lis2dh12_config.fs);

    lis2dh12_config.temp_enable = LIS2DH12_TEMP_DISABLE;
    lis2dh12_config.odr = LIS2DH12_ODR_1HZ;
    lis2dh12_config.opt_mode = LIS2DH12_OPT_NORMAL;
    lis2dh12_config.z_enable = LIS2DH12_ENABLE;
    lis2dh12_config.y_enable = LIS2DH12_ENABLE;
    lis2dh12_config.x_enable = LIS2DH12_ENABLE;
    lis2dh12_config.bdu_status = LIS2DH12_DISABLE;
    lis2dh12_config.fs = LIS2DH12_FS_16G;
    lis2dh12_set_config(lis2dh12, &lis2dh12_config);
    lis2dh12_acce_value_t lis2dh12_acce_value = {0};

    lis2dh12_get_config(lis2dh12, &lis2dh12_config);
    printf("temp_enable is: %02x\n", lis2dh12_config.temp_enable);
    printf("odr is: %02x\n", lis2dh12_config.odr);
    printf("option mode is: %02x\n", lis2dh12_config.opt_mode);
    printf("z_enable status is: %02x\n", lis2dh12_config.z_enable);
    printf("y_enable status is: %02x\n", lis2dh12_config.y_enable);
    printf("x_enable status is: %02x\n", lis2dh12_config.x_enable);
    printf("bdu_status status is: %02x\n", lis2dh12_config.bdu_status);
    printf("full scale is: %02x\n", lis2dh12_config.fs);

    while (cnt--) {
        printf("\n******************************************\n");
        lis2dh12_get_x_acc(lis2dh12, &x_acc);
        printf("X-axis acceleration is: %08x\n", x_acc);
        lis2dh12_get_y_acc(lis2dh12, &y_acc);
        printf("Y-axis acceleration is: %08x\n", y_acc);
        lis2dh12_get_z_acc(lis2dh12, &z_acc);
        printf("Z-axis acceleration is: %08x\n", z_acc);
        lis2dh12_get_acce(lis2dh12, &lis2dh12_acce_value);
        printf("x = %f y= %f z = %f\n", lis2dh12_acce_value.acce_x, lis2dh12_acce_value.acce_y, lis2dh12_acce_value.acce_z);
        printf("******************************************\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    lis2dh12_test_init();
    vTaskDelay(1000 / portTICK_RATE_MS);
    lis2dh12_test_get_data();
    lis2dh12_test_deinit();
}