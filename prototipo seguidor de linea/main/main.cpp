#include "cabeceras.h"
QTR8A sensor;
uint16_t sensor_values[SENSOR_COUNT];
uint16_t position;



//void leerlinea(void *pvParam){}
//void control(void *pvParam){}
//void moverse(int speedLeft, int speedRight){}



void configureGpio(void)
{
    // Configure GPIO pins for input and output modes
    gpio_reset_pin(VERDE); // Reset LED pin
    gpio_reset_pin(ROJO);  // Reset LED1 pin
    gpio_reset_pin(RDY);   // Reset button pin
    gpio_reset_pin(AZUL);  // Reset button pin
    gpio_reset_pin(AIN1);  // Reset AIN1 pin
    gpio_reset_pin(AIN2);  // Reset AIN2 pin
    gpio_reset_pin(PWMA);  // Reset PWMA pin
    gpio_reset_pin(BIN1);  // Reset BIN1 pin
    gpio_reset_pin(BIN2);  // Reset BIN2 pin
    gpio_reset_pin(PWMB);  // Reset PWMB pin
    gpio_reset_pin(IR);    // Reset IR pin
    gpio_reset_pin(ISVM);  // Reset ISVM pin
    gpio_set_direction(VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(ROJO, GPIO_MODE_OUTPUT);
    gpio_set_direction(AZUL, GPIO_MODE_OUTPUT);
    gpio_set_direction(RDY, GPIO_MODE_INPUT);  // Set button pin as input
    gpio_set_pull_mode(RDY, GPIO_PULLUP_ONLY); // Activa el pull-up interno
    gpio_set_direction(CAL, GPIO_MODE_INPUT);  // Set button pin as input
    gpio_set_pull_mode(CAL, GPIO_PULLUP_ONLY); // Activa el pull-up interno
    gpio_set_direction(ISVM, GPIO_MODE_INPUT); // Set button pin as input
    gpio_set_direction(IR, GPIO_MODE_OUTPUT);  // Set button pin as input
    gpio_set_direction(AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWMA, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWMB, GPIO_MODE_OUTPUT);

}


esp_err_t createSensor(void)
{
    // Initialize the sensor with the specified pins
    sensor.setSensorPins({D1, D2, D3, D4, D5, D6, D7, D8}, SENSOR_COUNT, THRESHOLD);
    sensor.setTypeAnalog();            // Set the sensor type to Analog
    sensor.setTimeout(2500);           // Set the timeout for Analog sensors
    sensor.setSamplesPerSensor(15);    // Set the number of samples per person
    sensor.setEmitterPin(IR);          //Set the emitter pin for the sensor
    return ESP_OK; // Return Success

}


esp_err_t calibrateSensor(void)
{
    sensor.calibrate(QTRReadMode:: On);
    printf("Calibracion iniciada...\n");
    gpio_set_level(VERDE, 1);
    for (uint16_t i = 0; i < 150; ++i)
    {
        sensor.calibrate(); // calibrate the sensor
        printf("%d\n", i);
        vTaskDelay(pdMS_TO_TICKS(10)); // Retardo de 10ms entre lecturas
    }
    printf("calibracion finalizada\n");
    gpio_set_level(VERDE, 0);

    return ESP_OK; // Return Success


}


void getMAXMin(){
    for(int i=0; i < SENSOR_COUNT; i++ )
    printf("%d\t", sensor.calibrationOn.minimum[i]);
    printf("\n");
    for(int i= 0; i < SENSOR_COUNT; i++)
    printf("%d\t", sensor.calibrationOn.maximum[i]);
    printf("\n");

}




extern "C" void app_main(void)
{
    configureGpio();
    createSensor();


    while(gpio_get_level(CAL) == 1) //Boton sin presionar
    { vTaskDelay(pdMS_TO_TICKS(10));

    }
    calibrateSensor();

    while (1){
        position = sensor.readLineBlack(sensor_values);
        for(int i= 0; i<SENSOR_COUNT; i++)
         printf("%d\t", sensor_values[i]);
        printf("P: %d\n", position);
        vTaskDelay(pdMS_TO_TICKS(100));

    }
    
    }
    






