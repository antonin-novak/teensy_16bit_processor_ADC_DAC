
/* Audio Processor for Teensy with ADC and external DAC
 * Copyright (c) 2022, Oliver Munroe, Stephane Letourneur, Antonin Novak
 * contact: antonin.novak(at)univ-lemans.fr
 * 
 * Laboratoire d'Acoustique de l'Université du Mans (LAUM), 
 * UMR 6613, Institut d'Acoustique - Graduate School (IA-GS), 
 * CNRS, Le Mans Université, France 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <ADC.h>
#include <ADC_util.h>
#include <SPI.h>

//Input (ADC) initialisation
const int readPin1 = A0;
const int readPin2 = A2;
const float VrefADC = 3.3;
const int ADCAverages = 1;
volatile uint16_t valADC0 = 0, valADC1 = 0;
volatile uint8_t VALADC0Ready = false, VALADC1Ready = false;
volatile float input1 = 0.0, input2 = 0.0;
const int resolutionADC = 16;
const float conversionConstADC = VrefADC/((1<<resolutionADC)-1);


//Output (DAC) signal initialisation
const int slaveSelectPin = 25;
const int DIO = 2;
const int LDAC = 24;
const float VrefDAC = 2.5;
volatile uint16_t valDAC = 0;
volatile float val4DACOut = 0.0;
const int resolutionDAC = 16;
const float conversionConstDAC = ((1<<resolutionDAC)-1)/VrefDAC;

// Timing
const float sampleRateHz = 48000.0;
const int PmodDA3SPIMHz = 50;
const long PmodDA3SPIHz = PmodDA3SPIMHz*1000*1000;

// Using the Pedvide ADC Library on Github
ADC *adc = new ADC(); // adc object

void setup(void)
{
  // Declare needed pins as inputs or outputs
  pinMode(readPin1, INPUT);
  pinMode(readPin2, INPUT);
  pinMode (slaveSelectPin, OUTPUT);
  pinMode(DIO, OUTPUT);
  pinMode (LDAC, OUTPUT);

  // When using the Pedvide ADC library we can set more options
  adc->adc0->setAveraging(ADCAverages); // set number of averages
  adc->adc0->setResolution(resolutionADC); // set bits of resolution
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); // Set voltage reference for ADC.
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); // change the sampling speed
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed

  adc->adc0->stopPDB();
  adc->adc0->startSingleRead(readPin1);
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc0->startPDB(sampleRateHz); //frequency in Hz
  
  adc->adc1->setAveraging(ADCAverages); // set number of averages
  adc->adc1->setResolution(resolutionADC); // set bits of resolution
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3); // Set voltage reference for ADC.
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); // change the sampling speed
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed

  adc->adc1->stopPDB();
  adc->adc1->startSingleRead(readPin2); 
  adc->adc1->enableInterrupts(adc1_isr);
  adc->adc1->startPDB(sampleRateHz); //frequency in Hz

  
  NVIC_SET_PRIORITY(IRQ_USBOTG, 200);
  // initialise the SPI channel
  SPI.begin();
  SPI.beginTransaction(SPISettings(PmodDA3SPIHz, MSBFIRST, SPI_MODE0));
}


void Operations(void)
{  
  // read ADC Value and convert to voltage
  input1 = (valADC0 * conversionConstADC - 1.625);
  input2 = (valADC1 * conversionConstADC - 1.625);

  // do the Signal Processing (example: mean value of both inputs)
  val4DACOut = input1*0.5 + input2*0.5;

  // convert the output value for DAC
  valDAC = ((val4DACOut+1.25)*conversionConstDAC);

  // send the value using the SPI
  uint8_t SPIBuff[2] = {0};
  SPIBuff[0] = valDAC >> 8;
  SPIBuff[1] = valDAC & 0xFF;
 
  digitalWrite(LDAC,HIGH); 
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(SPIBuff, 2);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin,HIGH);
  digitalWrite(LDAC,LOW);
}

void loop(void)
{
  if (VALADC0Ready && VALADC1Ready){
    Operations();
    VALADC0Ready = false;
    VALADC1Ready = false;
  }
}

void adc0_isr() {
    valADC0 = (uint16_t)adc->adc0->readSingle();
    VALADC0Ready = true;
}

void adc1_isr() {
    valADC1 = (uint16_t)adc->adc1->readSingle();
    VALADC1Ready = true;
}
