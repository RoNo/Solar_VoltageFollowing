/*
 Solar Voltage following
 Copyright (c) 2015 Rolf Noellenburg.  All right reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "RCReceive.h"
#include <Servo.h>

//Voltage threshold in centivolts
int U_max = 620;
int U_min = 480;

//Stepsize in PWMcouts per loop
int x = 1;            //upwards
int y = 2;            //downwards


//Pin layout
const byte PIN_RC = 2;//Output speed controler
int VoltagePin = A4;  //Voltage divider
int CurrentPin = A5;  //Current sensor

RCReceive rcReceiver;  // Receiver
Servo Controllerpin;   //Speed controler

int RCin;
int RCout = 0;

int Uticks;
float U;
int Iticks;
int I_0 = 516;
float I;
unsigned int Pticks_C;
unsigned int Pticks;
float P;

float U_control;//comanded voltage

int control;
byte slowloop = 0;

// System Timers
unsigned long fast_loopTimer = 0;		// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds = 0;		// Delta Time in miliseconds
int mainLoop_count = 0;


void setup() {
  rcReceiver.attachInt(PIN_RC);
  Controllerpin.attach(5);                    // Controller on pin D5
  Controllerpin.writeMicroseconds(1020);
  Serial.begin(57600);
  Serial.println("Voltage following");
  Serial.println("Rolf Noellenburg");
  Serial.println("version 0.52");
  delay(3000);
}

void loop() {
  if (rcReceiver.hasNP() && !rcReceiver.hasError()) {
    doWork();
  } 
  else if (rcReceiver.hasError()) {
    doWork();
  }
}

void doWork() {
  //loopTimer of 4 equals 200Hz (1000/5msec)
  if (millis()-fast_loopTimer > 4) {
    deltaMiliSeconds = millis() - fast_loopTimer;
    fast_loopTimer = millis();
    fast_loop();
  }
}
void fast_loop()
{
  byte value = rcReceiver.getValue(); //Values between 0 - 255
  RCin = map(value, 0, 125, 1000, 2000);  // map to msec
  U_control = map(value, 126, 255, U_max, U_min);  // map to desired voltage

  Uticks = analogRead(VoltagePin);
  Iticks = analogRead(CurrentPin);
  Pticks = (Iticks - I_0) * Uticks;
  Pticks_C = Pticks;

  U = Uticks*11.17/809;
  I = (Iticks-I_0)*0.02642;
  P = I * U;

  // Voltage following
  if (U<U_control/100){
    control = RCout -y;
  }
  else{
    control = RCout +x;
  }

  // Set RCout depending on RCin
  if (RCin>1500){
    RCout=control;
    RCout = constrain(RCout, 1000, 2000);
  }
  else{
    RCout = RCin;
  }

  //write to controller;
  Controllerpin.writeMicroseconds(RCout);

  slowloop++;
  if (slowloop == 49)
  {
    slowloop=0;
    Serial.print ("  FLT: ");
    Serial.print (deltaMiliSeconds);
    //Serial.print ("  U-Ticks: ");
    //Serial.print (Uticks);
    Serial.print ("  U: ");
    Serial.print (U);
    Serial.print ("V  U_in: ");
    Serial.print (U_control);
    Serial.print ("  I: ");
    Serial.print (I);
    Serial.print ("  IT: ");
    Serial.print (Iticks);
    //Serial.print ("A  P-Ticks: ");
    //Serial.print (Pticks);
    Serial.print ("  P: ");
    Serial.print (P);
    Serial.print ("W  RCin: ");          
    Serial.print (RCin);
    Serial.print ("  RCout: ");
    Serial.print (RCout);
    Serial.println(""); 
  }
}
