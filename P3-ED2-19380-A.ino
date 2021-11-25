//********************************************************************************
// Universidad del Valle de Guatemala
// BE3015: Electrnónica Digital 2
// José David Méndez 19380
// Proyecto No.3 ESP32
//********************************************************************************

//**********************************************************************************************************************
// Librerías
//**********************************************************************************************************************
#include <Arduino.h>
// Llamado de librerias para poder configurar el sensor MAX30102/30105
#include <Wire.h> //Iniciar comunicación I2DC
#include "MAX30105.h"
#include "spo2_algorithm.h"
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
//Llamado de libreria para poder hacer funcionar el Neopixel
#include <Adafruit_NeoPixel.h>
//**********************************************************************************************************************
// Defincion de Pines y otros
//**********************************************************************************************************************
#define PIN  25 // Pin a la cual se conectara el Neopixel 
#define NUMPIXELS  8 //Numero de leds que cuenta el Neopixel 
//**********************************************************************************************************************
// Prototipo de Funciones
//**********************************************************************************************************************
void BPMYSPO2(void); // Prototipo de función encargada de enviar el valor BPM y SPO2 a la tiva C mediante comunicación USART.
void leerSensor(void); // Prototipo de función encargada de leer el valor BPM y SPO2 del sensor.
void Neopixel1(void); //Prototipo de funcion que se encarga de encender el neopixel cuando se manda a leer un dato.
void Neopixel2(void); //Prototipo de funcion que se encarga de encender el neopixel cuando se guarda un dato en la SD.
void Neopixel3(void); //Prototipo de funcion que se encarga de encender el neopixel cuando no se guarde ni se envíe ningun dato
//**********************************************************************************************************************
// Variables Globales
//**********************************************************************************************************************
//**********************************************************************************************************************
// Configuracion
//**********************************************************************************************************************
//**********************************************************************************************************************
// LOOP
//**********************************************************************************************************************
//**********************************************************************************************************************
// Fuinciones
//**********************************************************************************************************************
}
