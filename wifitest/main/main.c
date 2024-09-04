#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#include "wifi.h"
#include "http_client.h"

SemaphoreHandle_t wifi_on;


void handler_http_request(void *params)
{
    while(1) {
        if(xSemaphoreTake(wifi_on, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("Main Task", "WiFi is connected, starting HTTP request");

            https_request();
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_on = xSemaphoreCreateBinary();
    wifi_start();

    xTaskCreate(&handler_http_request, "http_request", 4096, NULL, 5, NULL);
}

