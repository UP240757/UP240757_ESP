#include <stdio.h>
#include "freertos/FreeRTOS.h"

// pines del infrarrojo 
#define D1 GPIO_NUM_36
#define D2 GPIO_NUM_39
#define D3 GPIO_NUM_34
#define D4 GPIO_NUM_35
#define D5 GPIO_NUM_32
#define D6 GPIO_NUM_33
#define D7 GPIO_NUM_25
#define D8 GPIO_NUM_26

//pines de motor- Derecho
#define PWMB GPIO_NUM_19
#define BIN2 GPIO_NUM_18
#define BIN1 GPIO_NUM_5 

 //pines de motor- Izquierdo
#define PWMA GPIO_NUM_4
#define AIN2 GPIO_NUM_17
#define AIN1 GPIO_NUM_16
 
//pines para botones
#define CAL GPIO_NUM_13
#define RDY GPIO_NUM_13


//pines de leds
#define VERDE GPIO_NUM_27
#define ROJO GPIO_NUM_14
#define AZUL GPIO_NUM_2

