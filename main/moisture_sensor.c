#include "moisture_sensor.h"

//Initializa la conexion y los puertos con los sensores
esp_err_t init_adc_config()
{
    esp_err_t err = ESP_OK;
    err = adc1_config_width(ADC_WIDTH);
    if (err != ESP_OK)
    {
        ESP_LOGE("ADC", "Hubo un error al momento de asignar el ancho del adc");
    }
    err = adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    if (err != ESP_OK)
    {
        ESP_LOGE("ADC", "Hubo un error al configurar atenuacion del adc");
    }
    return err;
}
//Obtiene los datos del sensor y los preprocesa
int data_filter()
{
    uint16_t contador = 0;
    int sum_valores = 0;
    while( contador++ < NUM_SAMPLES){
        sum_valores += adc1_get_raw(ADC_CHANNEL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return sum_valores/NUM_SAMPLES;
}
//Calcula la humedad del lugar a traves de unos rangos
float calculate_humidity_percentage(){
    int valor_digital = data_filter();
    return -0.0444*valor_digital + 177.78;
}