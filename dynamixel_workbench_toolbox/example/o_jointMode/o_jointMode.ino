/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include <DynamixelWorkbench.h>

#define DXL_BUS_SERIAL1 "1"  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 "2"  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 "3"  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define BAUDRATE         57600

#define DXL_ID           1

DynamixelToolbox toolbox;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  toolbox.begin("XM", "3", BAUDRATE);
  toolbox.ping(DXL_ID);

  toolbox.jointMode(DXL_ID);
}

void loop() 
{
  toolbox.goalPosition(DXL_ID, 1048);

  delay(1000);

  toolbox.goalPosition(DXL_ID, 3048);

  delay(1000);
}