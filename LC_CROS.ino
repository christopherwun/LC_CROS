/*  
  File/Sketch Name: LC_CROS

  Version No.: v1.0  Created 9 April, 2023

  Original Authors: Christopher Wun, Risa Pollak (UPenn)
  
  (Includes code modified from Clyde A. Lettsome, PhD, PE, MEM - more info here: https://clydelettsome.com/blog/2020/06/07/my-weekend-project-musical-note-detector-using-an-arduino/)

  Description: This code/sketch was created as a prototype for low-cost CROS hearing aids. In this form, it gathers frequency of sounds in its
  surroundings and determines the musical note played. It then plays this frequency back through a passive busser. In addition, it outputs the
  "guess note", (from A to G) determined based on the frequency of the pitch. Future versions may include more detailed sound processing.
*/

//*****************************************************************
//Definitions for Sound Processing
//*****************************************************************
#define  SAMPLES 128             //Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY  2048 //Fs = Based on Nyquist, must be 2 times the highest expected frequency.
#define  OFFSETSAMPLES 40  //used for calabrating purposes
#define TUNER -3    //Adjust  until C3 is 130.50
#define micPin A0 // pin for mic

const int sampleWindow = 50;                              // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
float samplingPeriod;
unsigned long microSeconds;
float storedNoteFreq[12]  = {130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185, 196, 207.65, 220, 233.08,  246.94};
int sumOffSet = 0;
int offSet[OFFSETSAMPLES]; //create offset  vector
int avgOffSet; //create offset vector
float  maxValue, minValue;
int toneDetected = 0;

//*****************************************************************
//Definitions for Adafruit
//*****************************************************************
#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include "BluefruitConfig.h"
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
int in1 = 10;
int in2 = 11;
int in3 = 5;
int in4 = 6;
#define trigPin 3
#define echoPin 2
char dir = 10;

//*****************************************************************
//SETUP
//*****************************************************************
void setup() {
  Serial.begin(9600);  //9600 Baud rate for the Serial Monitor
  //---------------------------------------------------------------
  //SETUP FOR ADAFRUIT
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);
  //Serial.println("example code");
  //Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  //Serial.println(F("---------------------------------------"));
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
      }
    }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  Serial.println(F("Requesting Bluefruit info:"));
  /* Print Bluefruit information */
  ble.info();
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  //Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
  ble.verbose(false);  // debug info is a little annoying after this point!
  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
  //---------------------------------------------------------------
  //SETUP FOR SOUND PROCESSING
  Serial.println(F("Calibrating. Please do not play any notes during calibration."));
  for (int i = 0; i < OFFSETSAMPLES; i++){
    Serial.print("calibration testing");
    offSet[i] = analogRead(micPin); //Reads  the value from analog pin 0 (A0), quantize it and save it as a real term.
    sumOffSet = sumOffSet + offSet[i];
    }

  delay(5000);
  Serial.println(F("Done calibrating."));
}
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
//*****************************************************************
//LOOP 
//*****************************************************************
void loop() {
  //---------------------------------------------------------------
  //LOOP FOR SOUND PROCESSING
  //---------------------------------------------------------------
  int  X[SAMPLES]; //create vector of size SAMPLES to hold real values
  float autoCorr[SAMPLES];  //create vector of size SAMPLES to hold imaginary values
  float  maxValue, minValue;
  int i, k, periodEnd, periodBegin,  period, adjuster, noteLocation, octaveRange;
  long  sum;
  int thresh = 0;
  int numOfCycles = 0;
  float signalFrequency, signalFrequency2,  signalFrequency3, signalFrequencyGuess, total;
  byte state_machine = 0;
  int  samplesPerPeriod;
  samplesPerPeriod  = 0;
  maxValue = 0;

  //*****************************************************************
  //Prepare to accept input from A0
  //*****************************************************************
  avgOffSet = round(sumOffSet / OFFSETSAMPLES);
  Serial.println("Play  your note!");
  delay(100);  //pause for for reaction time

  //*****************************************************************
  //Collect  SAMPLES samples from A0 with sample period of samplingPeriod
  //*****************************************************************
  samplingPeriod = 1.0 / SAMPLING_FREQUENCY; //Period in microseconds
  for (i  = 0; i < SAMPLES; i++){
    microSeconds = micros();    //Returns the number  of microseconds since the Arduino board began running the current script.
    X[i]  = analogRead(micPin); //Reads the value from analog pin 0 (A0), quantize it and save  it as a real term.
    /*remaining wait time between samples if necessary  in seconds */
    //while (micros() < (microSeconds + (samplingPeriod * 1000000))) {} CAN THIS BE REMOVED DELETE IF IT WORKS
    }

  //*****************************************************************
  //Autocorrelation Function
  //*****************************************************************
  for (i = 0; i < SAMPLES; i++){ //i=delay
    sum = 0;
    for (k =  0; k < SAMPLES - i; k++){ //Match signal with delayed signal
      sum  = sum + (((X[k]) - avgOffSet) * ((X[k + i]) - avgOffSet)); //X[k] is the signal  and X[k+i] is the delayed version
      }
    autoCorr[i] = sum / SAMPLES;

    // First Peak Detect State Machine
    if (state_machine==0 && i == 0) {
      thresh = autoCorr[i] * 0.5;
      state_machine = 1;
      }
    else if (state_machine == 1 && i>0 && thresh < autoCorr[i] && (autoCorr[i]-autoCorr[i-1])>0){  //state_machine=1, find 1 period for using first cycle
      maxValue  = autoCorr[i];
      }
    else if (state_machine == 1&& i>0 && thresh  < autoCorr[i-1] && maxValue == autoCorr[i-1] && (autoCorr[i]-autoCorr[i-1])<=0){
      periodBegin = i-1;
      state_machine = 2;
      numOfCycles  = 1;
      samplesPerPeriod = (periodBegin - 0);
      period = samplesPerPeriod;
      adjuster = TUNER+(50.04 * exp(-0.102 * samplesPerPeriod)); 
      signalFrequency  = ((SAMPLING_FREQUENCY) / (samplesPerPeriod))-adjuster; // f = fs/N
      }
    else if (state_machine == 2 && i>0 && thresh < autoCorr[i] && (autoCorr[i]-autoCorr[i-1])>0) { //state_machine=2, find 2 periods for 1st and 2nd cycle
      maxValue  = autoCorr[i]; }
    else if (state_machine == 2&& i>0 && thresh < autoCorr[i-1]  && maxValue == autoCorr[i-1] && (autoCorr[i]-autoCorr[i-1])<=0){
      periodEnd  = i-1;
      state_machine = 3;
      numOfCycles = 2;
      samplesPerPeriod  = (periodEnd - 0);
      signalFrequency2 = ((numOfCycles*SAMPLING_FREQUENCY)  / (samplesPerPeriod))-adjuster; // f = (2*fs)/(2*N)
      maxValue = 0;
      }
    else if (state_machine == 3 && i>0 && thresh < autoCorr[i] && (autoCorr[i]-autoCorr[i-1])>0) { //state_machine=3, find 3 periods for 1st, 2nd and 3rd cycle
      maxValue  = autoCorr[i]; 
      }
    else if (state_machine == 3&& i>0 && thresh < autoCorr[i-1]  && maxValue == autoCorr[i-1] && (autoCorr[i]-autoCorr[i-1])<=0) {
      periodEnd  = i-1;
      state_machine = 4;
      numOfCycles = 3;
      samplesPerPeriod  = (periodEnd - 0);
      signalFrequency3 = ((numOfCycles*SAMPLING_FREQUENCY)  / (samplesPerPeriod))-adjuster; // f = (3*fs)/(3*N)
      }
    }

  //*****************************************************************
  //Result Analysis
  //*****************************************************************
  if (samplesPerPeriod == 0) {
    Serial.println("Hmm..... I am not sure.  Are you trying to trick me?");

//    // bluetooth notification
//    if (toneDetected == 1) {
//      ble.print("AT+BLEUARTTX=");
//      ble.println("tone ended");
//      toneDetected = 0;
//    }
    
    }
  else { 

    // bluetooth notification
//    if (toneDetected == 0) {
//      ble.print("AT+BLEUARTTX=");
//      ble.println("tone detected");
//      toneDetected = 1;
//    }
    
    //prepare the weighting  function
    total = 0;
    if (signalFrequency !=0) {
      total  = 1;
      }
    if(signalFrequency2 !=0) {
      total = total +  2;
      }
    if (signalFrequency3 !=0) {
      total = total + 3;
      }

    //calculate the frequency using the weighting function
    signalFrequencyGuess  = 2*(((1/total) * signalFrequency) + ((2/total) * signalFrequency2) + ((3/total) *  signalFrequency3)); //find a weighted frequency
    Serial.print("The note you  played is approximately ");
    Serial.print(signalFrequencyGuess);     //Print  the frequency guess.
    Serial.println(" Hz.");
    
    //Find the closest note
    minValue = 10000000;
    noteLocation = 0;
    for (i = 0; i < 12; i++) {
      if(minValue> abs(signalFrequencyGuess-storedNoteFreq[i])) {
        minValue = abs(signalFrequencyGuess-storedNoteFreq[i]);
        noteLocation  = i;
        }
      }
    
  //plays the tone through the buzzer
  tone(5, signalFrequencyGuess, 1000);
  
  //Print the note to bluefruit

    ble.print("AT+BLEUARTTX=");
    ble.println("I think you played ");
    Serial.print("test");
    if(noteLocation==0)
    { 
      ble.print("AT+BLEUARTTX=");
      ble.println("C");
      ble.println("  ");
      Serial.print("C");
    }  
    else if(noteLocation==1)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("C#");
      ble.println("  ");
    }
    else if(noteLocation==2)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("D");
      ble.println("  ");
    }
    else if(noteLocation==3)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("D#");
      ble.println("  ");
    }
    else if(noteLocation==4)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("E");
      ble.println("  ");
    }
    else if(noteLocation==5)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("F");
      ble.println("  ");
    }
    else if(noteLocation==6)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("F#");
      ble.println("  ");
    }
    else if(noteLocation==7)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("G");
      ble.println("  ");
    }
    else if(noteLocation==8)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("G#");
      ble.println("  ");
    }
    else if(noteLocation==9)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("A");
      ble.println("  ");
    }
    else if(noteLocation==10)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("A#");
      ble.println("  ");
    }
    else if(noteLocation==11)
    {
      ble.print("AT+BLEUARTTX=");
      ble.println("B");
      ble.println("  ");
    }
  }
}
