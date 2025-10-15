#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

// Definición de los pines para el LED y el botón
#define LED    GPIO_NUM_15
#define BUTTON GPIO_NUM_21


void punto(void){ 
    gpio_set_level(LED, 1); // Encendemos el LED
    vTaskDelay(pdMS_TO_TICKS(200)); // Esperamos 0.2 segundos
    gpio_set_level(LED, 0); // Apagamos el LED
    vTaskDelay(pdMS_TO_TICKS(200)); // Esperamos 0.2 segundos

}

void raya(void){
    gpio_set_level(LED, 1); // Encendemos el LED
    vTaskDelay(pdMS_TO_TICKS(500)); // Esperamos 0.5 segundos
    gpio_set_level(LED, 0); // Apagamos el LED
    vTaskDelay(pdMS_TO_TICKS(500)); // Esperamos 0.5 segundos
}

void SOS(){
    printf("SOS\n"); 
    for (int i = 0; i <3; i ++) // Tres puntos
    {
        punto();    
    }
    for (int i = 0 ; i<3; i++) // Tres rayas
    {
      raya();  
    }
    for (int i = 0 ; i<3; i++){ // Tres puntos
    }
    for (int i = 0; i<3;i++)
    {
        punto();
    }
}
bool doubleClick(void)
{
    const TickType_t debounceDelay = pdMS_TO_TICKS(50);
    const TickType_t doubleClickTimeout = pdMS_TO_TICKS(400);
    TickType_t startTime;

    // Espera la primera pulsación
    while (gpio_get_level(BUTTON) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Pequeño delay para debounce
    vTaskDelay(debounceDelay);

    // Verifica si se soltó el botón
    while (gpio_get_level(BUTTON) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Marca el tiempo de la primera pulsación
    startTime = xTaskGetTickCount();

    // Espera la segunda pulsación dentro del tiempo permitido
    while ((xTaskGetTickCount() - startTime) < doubleClickTimeout)
    {

        if (gpio_get_level(BUTTON) == 0)
        {
            // Espera a que se suelte el botón nuevamente
            vTaskDelay(debounceDelay);
            while (gpio_get_level(BUTTON) == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            ESP_LOGI("BUTTON", "Double click detected");
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Evita WDT
    }

    return false; // No se detectó doble clic
}
void app_main(void)
{
    // Reinicia la configuración de los pines LED y botón
    gpio_reset_pin(LED);
    gpio_reset_pin(BUTTON);

    // Configura el pin del LED como salida
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    // Configura el pin del botón con resistencia pull-up
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);
    // Configura el pin del botón como entrada
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);

    // Bucle principal
    while(true)
    {
        // Lee el estado del botón (0 si está presionado, 1 si no)
        int status = gpio_get_level(BUTTON
    );

        // Si el botón está presionado (nivel bajo)
        if(status == false)
        {
         SOS() ; // Enciende el LED
        }
        else
        {
            gpio_set_level(LED, 0); // Apaga el LED
        }

        // Espera 20 ms antes de repetir
        vTaskDelay(pdMS_TO_TICKS(20)) ;
    }

}
