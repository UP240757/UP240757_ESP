#include <stdio.h> // Incluye la biblioteca estándar de entrada/salida
#include "freertos/FreeRTOS.h" // Incluye la biblioteca principal de FreeRTOS
#include "driver/gpio.h" // Incluye el controlador para GPIO
#include "driver/ledc.h" // Incluye el controlador para LEDC (PWM)
#include "esp_adc/adc_oneshot.h" // Incluye el controlador ADC en modo de una sola toma

#define AIN1 GPIO_NUM_18
#define AIN2 GPIO_NUM_19
#define PWMA GPIO_NUM_21

int adc_value = 0; // Variable para almacenar el valor leído del ADC
adc_oneshot_unit_handle_t adc1_handle; // Manejador para la unidad ADC1
int adc_raw = 0; // Variable para almacenar el valor crudo del ADC
#define pot GPIO_NUM_36

int duty = 0;

void configuracion(void){
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // Identificador de la unidad ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE, // Deshabilita el modo ULP
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle); // Inicializa el ADC

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12, // Atenuación de 11dB
        .bitwidth = ADC_BITWIDTH_12, // Ancho de datos de 12 bits
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &channel_config); // Configura el canal 0 del ADC1
}

esp_err_t configureGpio(void)
{
    // Configure GPIO pins for input and output modes
    gpio_reset_pin(AIN1);   // Reset AIN1 pin
    gpio_reset_pin(AIN2);   // Reset AIN2 pin
    gpio_reset_pin(PWMA);   // Reset PWMA pin
    gpio_set_direction(AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWMA, GPIO_MODE_OUTPUT);
    return ESP_OK; // Return success
}

void setupPWM(void)
{
    // Configuración del canal PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT, // Resolución de 8 bits
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000, // Frecuencia de 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false // No desconfigurar el temporizador
    };
    ledc_timer_config(&ledc_timer);

    // Configuración del canal A
    ledc_channel_config_t ledc_channel_A = {
        .gpio_num = PWMA, // Primero el GPIO
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD, // Deshabilitar el modo de sueño
        .flags = {
            .output_invert = 0 // No invertir la salida
        }};
    ledc_channel_config(&ledc_channel_A);
    ledc_fade_func_install(0); // Instala la función de desvanecimiento
}

void app_main(void) // Función principal de la aplicación
{
    configureGpio(); // Configura los pines GPIO
    setupPWM(); // Configura el PWM 
    configuracion();

    gpio_set_level(AIN1, 1); // Establece AIN1 en alto
    gpio_set_level(AIN2, 0); // Establece AIN2 en


    while(1) // Bucle infinito
    {

         esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw);

         
        duty = (int)((adc_raw/4095.0)*255);
        printf("%d, %d\n", adc_raw, duty);

        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 0); // Actualiza el duty cycle
        //printf("Duty: %d\n", i); // Imprime el valor actual del duty cycle
        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 10 ms
       

    }
}