/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This demo shows heart rate and SPO2 levels.

  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.

  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include "getData.h"
#include "spo2_algorithm.h"

#define MAX_BRIGHTNESS 255
#define LED_BUFFER 100

uint32_t irBuffer[LED_BUFFER];  //infrared LED sensor data
uint32_t redBuffer[LED_BUFFER]; //red LED sensor data

void getSpo2AndHeartRateInit(MAX30105* sensor, int32_t* spo2, int8_t* validSPO2, int32_t* heartRate, int8_t* validHeartRate)
{
    //read the first 100 samples, and determine the signal range
    for (byte i = 0; i < LED_BUFFER; i++) {
        while (!MAX30105_available(sensor)) //do we have new data?
            MAX30105_check(sensor); //Check the sensor for new data

        redBuffer[i] = MAX30105_getRed(sensor);
        irBuffer[i] = MAX30105_getIR(sensor);
        MAX30105_nextSample(sensor); //We're finished with this sample so move to next sample
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, LED_BUFFER, redBuffer, spo2, validSPO2, heartRate, validHeartRate);
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
void getSpo2AndHeartRateContinuous(MAX30105* sensor, int32_t* spo2, int8_t* validSPO2, int32_t* heartRate, int8_t* validHeartRate)
{
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < LED_BUFFER; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < LED_BUFFER; i++) {
        while (!MAX30105_available(sensor)) //do we have new data?
            MAX30105_check(sensor); //Check the sensor for new data

        redBuffer[i] = MAX30105_getRed(sensor);
        irBuffer[i] = MAX30105_getIR(sensor);
        MAX30105_nextSample(sensor); //We're finished with this sample so move to next sample
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, LED_BUFFER, redBuffer, spo2, validSPO2, heartRate, validHeartRate);
}