/*************************************************** 
 This is a library written for the Maxim MAX30105 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/i2c.h>

//Define the size of the I2C buffer
#define I2C_BUFFER_LENGTH 32

#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro

typedef uint8_t byte;

typedef struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  byte head;
  byte tail;
} sense_struct; //This is our circular buffer of readings from the sensor

typedef struct {
  const struct i2c_dt_spec* i2cDriver;
  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  uint8_t revisionID;
  sense_struct sense;
} MAX30105;


bool MAX30105_init(MAX30105* sensor);

uint32_t MAX30105_getRed(MAX30105* sensor); //Returns immediate red value
uint32_t MAX30105_getIR(MAX30105* sensor); //Returns immediate IR value
uint32_t MAX30105_getGreen(MAX30105* sensor); //Returns immediate green value
bool MAX30105_safeCheck(MAX30105* sensor, uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

// Configuration
void MAX30105_softReset(MAX30105* sensor);
void MAX30105_shutDown(MAX30105* sensor);
void MAX30105_wakeUp(MAX30105* sensor);

void MAX30105_setLEDMode(MAX30105* sensor, uint8_t mode);

void MAX30105_setADCRange(MAX30105* sensor, uint8_t adcRange);
void MAX30105_setSampleRate(MAX30105* sensor, uint8_t sampleRate);
void MAX30105_setPulseWidth(MAX30105* sensor, uint8_t pulseWidth);

void MAX30105_setPulseAmplitudeRed(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeIR(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeGreen(MAX30105* sensor, uint8_t amplitude);
void MAX30105_setPulseAmplitudeProximity(MAX30105* sensor, uint8_t amplitude);

void MAX30105_setProximityThreshold(MAX30105* sensor, uint8_t threshMSB);

//Multi-led configuration mode (page 22)
void MAX30105_enableSlot(MAX30105* sensor, uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
void MAX30105_disableSlots(MAX30105* sensor);

// Data Collection

//Interrupts (page 13, 14)
uint8_t MAX30105_getINT1(MAX30105* sensor); //Returns the main interrupt group
uint8_t MAX30105_getINT2(MAX30105* sensor); //Returns the temp ready interrupt
void MAX30105_enableAFULL(MAX30105* sensor); //Enable/disable individual interrupts
void MAX30105_disableAFULL(MAX30105* sensor);
void MAX30105_enableDATARDY(MAX30105* sensor);
void MAX30105_disableDATARDY(MAX30105* sensor);
void MAX30105_enableALCOVF(MAX30105* sensor);
void MAX30105_disableALCOVF(MAX30105* sensor);
void MAX30105_enablePROXINT(MAX30105* sensor);
void MAX30105_disablePROXINT(MAX30105* sensor);
void MAX30105_enableDIETEMPRDY(MAX30105* sensor);
void MAX30105_disableDIETEMPRDY(MAX30105* sensor);

//FIFO Configuration (page 18)
void MAX30105_setFIFOAverage(MAX30105* sensor, uint8_t numberOfSamples);
void MAX30105_enableFIFORollover(MAX30105* sensor);
void MAX30105_disableFIFORollover(MAX30105* sensor);
void MAX30105_setFIFOAlmostFull(MAX30105* sensor, uint8_t numberOfSamples);

//FIFO Reading
uint16_t MAX30105_check(MAX30105* sensor); //Checks for new data and fills FIFO
uint8_t MAX30105_available(MAX30105* sensor); //Tells caller how many new samples are available (head - tail)
void MAX30105_nextSample(MAX30105* sensor); //Advances the tail of the sense array
uint32_t MAX30105_getFIFORed(MAX30105* sensor); //Returns the FIFO sample pointed to by tail
uint32_t MAX30105_getFIFOIR(MAX30105* sensor); //Returns the FIFO sample pointed to by tail
uint32_t MAX30105_getFIFOGreen(MAX30105* sensor); //Returns the FIFO sample pointed to by tail

uint8_t MAX30105_getWritePointer(MAX30105* sensor);
uint8_t MAX30105_getReadPointer(MAX30105* sensor);
void MAX30105_clearFIFO(MAX30105* sensor); //Sets the read/write pointers to zero

//Proximity Mode Interrupt Threshold
void setPROXINTTHRESH(uint8_t val);

// Die Temperature
float MAX30105_readTemperature(MAX30105* sensor);
float MAX30105_readTemperatureF(MAX30105* sensor);

// Detecting ID/Revision
uint8_t MAX30105_readPartID(MAX30105* sensor);

// Setup the IC with user selectable settings
void MAX30105_setup(MAX30105* sensor, byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange);

void MAX30105_readRevisionID(MAX30105* sensor);

void MAX30105_bitMask(MAX30105* sensor, uint8_t reg, uint8_t mask, uint8_t thing);
