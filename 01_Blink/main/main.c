#include "driver/gpio.h" // Esta librería es necesaria para manejar los pines GPIO, en el CMake debemos agregar la dependencia "driver"
#include "freertos/FreeRTOS.h" // Esta librería es necesaria para usar FreeRTOS 
#include "freertos/task.h" // Esta librería es necesaria para usar las tareas de FreeRTOS


#define LED GPIO_NUM_23 // Definimos el pin donde está conectado el LED
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

void
{  SOS()
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

void app_main(void) // Función principal del programa
{
    gpio_reset_pin(LED); // Reseteamos el pin del LED
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); // Configuramos el pin del LED como salida

    while (1) // Bucle infinito
    { 
        gpio_reset_pin(BOTON);
        gpio_set_direction(BOTON, GPIO_MODE_OUTPUT);
        gpio_pullup_en(BOTON); // Activamos resistencia pull-up interna

        int estadoAnterior = 1; // Boton suelto al inicio
        while(1){
            int estado = gpio_get_level(BOTON);
            if(estado == 0 && estadoAnterior == 1){ // flancode bajada
                SOS();
        }

        estadoAnterior = estado;
        vTaskDelay(pdMS_TO_TICKS(50)); // debounce
        }
    }
        SOS();
        
        
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperamos 2 segundos
    }
}



