
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_log.h"
#include "wifi.h"
#include "mqtt_control.h"
#include "moisture_sensor.h"

float cal_abs(float a)
{
    if (a < 0)
        return -1 * a;
    return a;
}
void task_captura_humedad(void *args)
{
    float humedad_ant = 0;
    float humedad = 0;
    long ultima_captura = esp_timer_get_time()/1000;
    char data[20];
    while (1)
    {
        humedad = calcular_porcentaje_humedad();
        if(cal_abs(humedad_ant - humedad) > 0.5 || (esp_timer_get_time()/1000 - ultima_captura) > 60000){
            //ENVIAR
            //Agregar a la cola de envIo
            humedad_ant = humedad;
            ultima_captura = esp_timer_get_time()/1000;
            ESP_LOGI("t_Hu", "Cambio humedad %.4f", humedad);
            sprintf(data, "%.4f", humedad);
            publish_data("sensor-data", data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    initialize_wifi("luillidei", "luillidei");
    wait_wifi_Connection();
    init_adc_config();
    mqtt_app_start();

    while (!is_connected)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    xTaskCreate(task_captura_humedad, "task_captura", 2048, NULL, 1, NULL);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
