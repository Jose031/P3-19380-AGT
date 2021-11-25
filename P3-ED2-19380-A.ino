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
String estadoBoton = ""; // Variable con la cual se recibe la instrucción por parte de la tiva c para enviar el valor BPM y SPO2.
int estado = 0;
//Variables globales para el funcionamiento del sensor
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//**********************************************************************************************************************
// Configuracion
//**********************************************************************************************************************
void setup()
{
  Serial.begin(115200); // Velocidad con la que se trabja la comunicación serial del canal 0.
  Serial2.begin(115200);  // Velocidad con la que se trabaja la comunicación serial del canal 2.

  //Inicializar Neopixel
  pixels.begin();
  pixels.setBrightness(40);
  //*********************************Configuración Sensor*********************************
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //*****************************************************************************************
}
//**********************************************************************************************************************
// LOOP
//**********************************************************************************************************************
//**********************************************************************************************************************
// Fuinciones
//**********************************************************************************************************************
void BPMYSPO2(void)
{
  if (Serial2.available() > 0) // Condición que permite verficar si hay bytes disponibles en el buffer de registro.
  {
    estadoBoton = Serial2.readStringUntil('\n'); //Lectura de la instrucción enviada por la Tiva C.
    estado = estadoBoton.toInt(); //Conversión  de la instrcción a un entero.
    Serial.print("Estado: ");
    Serial.print(estado);
  }

  if ((estado) == 1) // Condición que al identificar la instrucción de la Tiva C envía el ultimo valor leido por el sensor de BPM y SPO2.
  {
    Serial.println(" Se ha leido el valor con exito: ");
    Serial.print("BPM:");
    Serial.print(heartRate);
    Serial.print(" / ");
    Serial.print("SPO2:");
    Serial.println(spo2);
    Serial2.print("BPM:");
    Serial2.print(heartRate);
    Serial2.print(" / ");
    Serial2.print("SPO2:");
    Serial2.println(spo2);
    Neopixel1(); //Se llama a la función que permitira enceder el neopixel al enviar un valor del sensor. 
    // El utlimo valor de BPM es enviado por el módulo UART 2 a la Tiva C.
    estado = 0; //Se hace cero el estado para que se envie solo se envíe un valor de BPM y SPO2 por cada instrucción enviada por la Tiva C.
  }
  if (estado == 2) { //Condición que verifica que se está gurdando un dato en la memoria SD. 
    Serial.println(" Se ha guardado el valor con exito: ");
    Neopixel2(); //Se llama a la función que permitira enceder el neopixel al enviar un valor del sensor. 
    estado = 0; //Se hace cero el estado para que se envie solo se envíe un valor de BPM y SPO2 por cada instrucción enviada por la Tiva C.
  }
  if (estado == 0) { //Condición que evalua que no se está enviando una intrucción por parte de la Tiva c. 
    Serial.println();
    Neopixel3(); //Se llama a la función que permitira enceder el neopixel al enviar un valor del sensor. 
    
  }
}
void leerSensor() // Función encargada de leer los valores del sensor. 
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second

  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}
void Neopixel1(void) {
}
