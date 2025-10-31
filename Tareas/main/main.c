#include <stdio.h>                      // Cabeceras estándar
#include "freertos/FreeRTOS.h"          // API de FreeRTOS
#include "freertos/task.h"              // Manejo de tareas (xTaskCreate, vTaskDelay, etc.)
#include "driver/gpio.h"                // Control de pines GPIO del ESP32
#include "esp_log.h"                    // Logging (ESP_LOGI, ESP_LOGW, ESP_LOGE)

#define ROJO GPIO_NUM_18                // Pin para LED rojo
#define VERDE GPIO_NUM_19               // Pin para LED verde
#define AZUL GPIO_NUM_16                // Pin para LED azul

// Constantes de tiempo para las tareas (milisegundos)
#define TIEMPO_TASK1 500                // Duración encendido/apagado del LED rojo
#define TIEMPO_TASK2 1000               // Duración encendido/apagado del LED verde
#define TIEMPO_TASK3 1500               // Duración encendido/apagado del LED azul

// Etiquetas para los logs de cada tarea
static const char *T1 = "Task 1";
static const char *T2 = "Task 2";
static const char *T3 = "Task 3";

// Tarea que controla el LED rojo
void RojoTask(void *pvParameters)
{
    while (1)
    {
        gpio_set_level(ROJO, 1);                    // Encender LED rojo
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK1));    // Esperar TIEMPO_TASK1 milisegundos
        gpio_set_level(ROJO, 0);                    // Apagar LED rojo
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK1));    // Esperar TIEMPO_TASK1 milisegundos
        ESP_LOGI(T1, "Rojo encendido");             // Log informativo (nivel INFO)
    }
}

// Tarea que controla el LED verde
void VerdeTask(void *pvParameters)
{
    while (1)
    {
        gpio_set_level(VERDE, 1);                   // Encender LED verde
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK2));    // Esperar TIEMPO_TASK2 milisegundos
        gpio_set_level(VERDE, 0);                   // Apagar LED verde
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK2));    // Esperar TIEMPO_TASK2 milisegundos
        ESP_LOGW(T2, "Verde encendido");            // Log de advertencia (nivel WARN)
    }
}

// Tarea que controla el LED azul
void AzulTask(void *pvParameters)
{
    while (1)
    {
        gpio_set_level(AZUL, 1);                    // Encender LED azul
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK3));    // Esperar TIEMPO_TASK3 milisegundos
        gpio_set_level(AZUL, 0);                    // Apagar LED azul
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_TASK3));    // Esperar TIEMPO_TASK3 milisegundos
        ESP_LOGE(T3, "Azul encendido");             // Log de error (nivel ERROR)
    }
}

// Configuración inicial de los pines GPIO usados (salidas)
void configIO(void){
    gpio_set_direction(ROJO, GPIO_MODE_OUTPUT);     // Configurar pin rojo como salida
    gpio_set_direction(VERDE, GPIO_MODE_OUTPUT);    // Configurar pin verde como salida
    gpio_set_direction(AZUL, GPIO_MODE_OUTPUT);     // Configurar pin azul como salida
}

// Punto de entrada de la aplicación en ESP-IDF
void app_main(void)
{
    configIO();                                     // Inicializar GPIO

    // Crear las tres tareas con stack de 2048 bytes y prioridad 1
    xTaskCreatePinnedToCore(RojoTask, "Tarea Rojo", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(VerdeTask, "Tarea Verde", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(AzulTask, "Tarea Azul", 2048, NULL, 1, NULL, 1);
}