/*
  MIT License

  Copyright (c) 2022 Medical Open World, Pablo Sánchez Bergasa

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/
#include <Arduino.h>
#include "main.h"
#include "PID.h"

double HeaterPIDOutput;
double skinControlPIDInput;
double airControlPIDInput;
double humidityControlPIDOutput;
int humidifierTimeCycle = 5000;
unsigned long windowStartTime;

extern in3ator_parameters in3;
extern MAM_in3ator_Humidifier in3_hum;
extern bool humidifierState, humidifierStateChange;

double Kp[numPID] = {KP_SKIN, KP_AIR, KP_HUMIDITY};
double Ki[numPID] = {KI_SKIN, KI_AIR, KI_HUMIDITY};
double Kd[numPID] = {KD_SKIN, KD_AIR, KD_HUMIDITY};

PID airControlPID(&airControlPIDInput, &HeaterPIDOutput, &in3.desiredControlTemperature, Kp[airPID], Ki[airPID], Kd[airPID], P_ON_E, DIRECT);
PID skinControlPID(&skinControlPIDInput, &HeaterPIDOutput, &in3.desiredControlTemperature, Kp[skinPID], Ki[skinPID], Kd[skinPID], P_ON_E, DIRECT);
PID humidityControlPID(&in3.humidity[ROOM_DIGITAL_HUM_SENSOR], &humidityControlPIDOutput, &in3.desiredControlHumidity, Kp[humidityPID], Ki[humidityPID], Kd[humidityPID], P_ON_E, DIRECT);

void PIDInit()
{
  airControlPID.SetMode(MANUAL);
  skinControlPID.SetMode(MANUAL);
  humidityControlPID.SetMode(MANUAL);
}

void PIDHandler()
{
  if (airControlPID.GetMode() == AUTOMATIC)
  {
    airControlPIDInput = in3.temperature[ROOM_DIGITAL_TEMP_SENSOR];
    airControlPID.Compute();
    ledcWrite(HEATER_PWM_CHANNEL, HeaterPIDOutput && ongoingCriticalAlarm());
  }
  if (skinControlPID.GetMode() == AUTOMATIC)
  {
    skinControlPIDInput = in3.temperature[SKIN_SENSOR];
    skinControlPID.Compute();
    ledcWrite(HEATER_PWM_CHANNEL, HeaterPIDOutput && ongoingCriticalAlarm());
  }
  if (humidityControlPID.GetMode() == AUTOMATIC)
  {
    humidityControlPID.Compute();
    if (millis() - windowStartTime > humidifierTimeCycle)
    { // time to shift the Relay Window
      windowStartTime += humidifierTimeCycle;
    }
    if (humidityControlPIDOutput < millis() - windowStartTime)
    {
      if (humidifierState || humidifierStateChange)
      {
        in3_hum.turn(OFF);
        humidifierStateChange = false;
      }
      humidifierState = false;
    }
    else
    {
      if (!humidifierState || humidifierStateChange)
      {
        in3_hum.turn(ON);
        humidifierStateChange = false;
      }
      humidifierState = true;
    }
  }
}

void startPID(byte var)
{
  switch (var)
  {
  case airPID:
    airControlPID.SetOutputLimits(false, HEATER_MAX_PWM);
    airControlPID.SetTunings(Kp[airPID], Ki[airPID], Kd[airPID]);
    airControlPID.SetControllerDirection(DIRECT);
    airControlPID.SetSampleTime(PID_TEMPERATURE_SAMPLE_TIME);
    airControlPID.SetMode(AUTOMATIC);
    break;
  case skinPID:
    skinControlPID.SetOutputLimits(false, HEATER_MAX_PWM);
    skinControlPID.SetTunings(Kp[skinPID], Ki[skinPID], Kd[skinPID]);
    skinControlPID.SetControllerDirection(DIRECT);
    airControlPID.SetSampleTime(PID_TEMPERATURE_SAMPLE_TIME);
    skinControlPID.SetMode(AUTOMATIC);
    break;
  case humidityPID:
    humidifierStateChange = true;
    windowStartTime = millis();
    humidityControlPID.SetTunings(Kp[humidityPID], Ki[humidityPID], Kd[humidityPID]);
    humidityControlPID.SetControllerDirection(DIRECT);
    humidityControlPID.SetOutputLimits(humidifierTimeCycle * humidifierDutyCycleMin / 100, humidifierTimeCycle * humidifierDutyCycleMax / 100);
    airControlPID.SetSampleTime(PID_HUMIDITY_SAMPLE_TIME);
    humidityControlPID.SetMode(AUTOMATIC);
    break;
  }
}

void stopPID(byte var)
{
  switch (var)
  {
  case airPID:
    airControlPID.SetMode(MANUAL);
    break;
  case skinPID:
    skinControlPID.SetMode(MANUAL);
    break;
  case humidityPID:
    humidityControlPID.SetMode(MANUAL);
  }
}
