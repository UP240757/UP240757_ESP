#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
//#include "esp_attr.h"  

#define INT_PIN  GPIO_NUM_27
#define LED_PIN  GPIO_NUM_25

volatile uint8_t int_count = 0;
volatile bool ejecutar_sos = false;
volatile TickType_t last_press_time = 0; 


void sos_signal(void)
{
    printf("Ejecutando señal SOS...\n");

    for (int i = 0; i < 3; i++) { 
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(800));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int i = 0; i < 3; i++) { 
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    printf("SOS completado.\n\n");
}


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    TickType_t current_time = xTaskGetTickCountFromISR();  
    if ((current_time - last_press_time) > pdMS_TO_TICKS(250)) 
    {
        int_count++;
        if (int_count >= 3)
        {
            ejecutar_sos = true;
            int_count = 0;
        }
        last_press_time = current_time;
    }
}


void app_main(void)
{
   
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);


    gpio_reset_pin(INT_PIN);
    gpio_set_direction(INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(INT_PIN, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_PIN, gpio_isr_handler, NULL);

    printf("Listo: presiona el botón 3 veces para ejecutar SOS.\n");

    while (1)
    {
        if (ejecutar_sos)
        {
            sos_signal();
            ejecutar_sos = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

