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

extern TwoWire *wire;
extern MAM_in3ator_Humidifier in3_hum;
extern Adafruit_ILI9341 tft;
extern SHTC3 mySHTC3; // Declare an instance of the SHTC3 class
extern RotaryEncoder encoder;
extern Beastdevices_INA3221 digitalCurrentSensor;

extern bool WIFI_EN;
extern long lastDebugUpdate;
extern long loopCounts;
extern int page;
extern int temperature_filter; // amount of temperature samples to filter
extern long lastNTCmeasurement[numNTC];

extern double errorTemperature[numSensors], temperatureCalibrationPoint;
extern double ReferenceTemperatureRange, ReferenceTemperatureLow;
extern double provisionalReferenceTemperatureLow;
extern double fineTuneSkinTemperature, fineTuneAirTemperature;
extern double RawTemperatureLow[numSensors], RawTemperatureRange[numSensors];
extern double provisionalRawTemperatureLow[numSensors];
extern double temperatureMax[numSensors], temperatureMin[numSensors];
extern int temperatureArray[numNTC][analog_temperature_filter]; // variable to handle each NTC with the array of last samples (only for NTC)
extern int temperature_array_pos;                               // temperature sensor number turn to measure
extern float diffSkinTemperature, diffAirTemperature;           // difference between measured temperature and user input real temperature
extern bool humidifierState, humidifierStateChange;
extern int previousHumidity; // previous sampled humidity
extern float diffHumidity;   // difference between measured humidity and user input real humidity

extern byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and they have different check rates
extern byte encoderRate;
extern byte encoderCount;
extern bool encPulseDetected;
extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;
extern bool digitalCurrentSensorPresent;

extern float instantTemperature[secondOrder_filter];
extern float previousTemperature[secondOrder_filter];

// room variables
extern const float minDesiredTemp[2]; // minimum allowed temperature to be set
extern const float maxDesiredTemp[2]; // maximum allowed temperature to be set
extern const int presetTemp[2];       // preset baby skin temperature

extern boolean A_set;
extern boolean B_set;
extern int encoderpinA;                 // pin  encoder A
extern int encoderpinB;                 // pin  encoder B
extern bool encPulsed, encPulsedBefore; // encoder switch status
extern bool updateUIData;
extern volatile int EncMove;            // moved encoder
extern volatile int lastEncMove;        // moved last encoder
extern volatile int EncMoveOrientation; // set to -1 to increase values clockwise
extern int last_encoder_move;           // moved encoder
extern long encoder_debounce_time;      // in milliseconds, debounce time in encoder to filter signal bounces
extern long last_encPulsed;             // last time encoder was pulsed

// Text Graphic position variables
extern int humidityX;
extern int humidityY;
extern int temperatureX;
extern int temperatureY;
extern int ypos;
extern bool print_text;
extern int initialSensorPosition;
extern bool pos_text[8];

extern bool enableSet;
extern float temperaturePercentage, temperatureAtStart;
extern float humidityPercentage, humidityAtStart;
extern int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX, humBarPosY;
extern int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
extern bool autoLock;             // setting that enables backlight switch OFF after a given time of no user actions
extern long lastbacklightHandler; // last time there was a encoder movement or pulse
extern long sensorsUpdatePeriod;

extern bool selected;
extern char cstring[128];
extern char *textToWrite;
extern char *words[12];
extern char *helpMessage;
extern byte bar_pos;
extern byte menu_rows;
extern byte length;
extern long lastGraphicSensorsUpdate;
extern long lastSensorsUpdate;
extern bool enableSetProcess;
extern long blinking;
extern bool state_blink;
extern bool blinkSetMessageState;
extern long lastBlinkSetMessage;



extern double HeaterPIDOutput;
extern double skinControlPIDInput;
extern double airControlPIDInput;
extern double humidityControlPIDOutput;
extern int humidifierTimeCycle;
extern unsigned long windowStartTime;

extern PID airControlPID;
extern PID skinControlPID;
extern PID humidityControlPID;

extern in3ator_parameters in3;

void userInterfaceHandler(int UI_page)
{
  if (EncMove)
  {
    if (!selected)
    {
      if (EncMove < 0)
      {
        EncMove++;
        if (UI_page == mainMenuPage)
        {
          enableSetProcess = enableSet;
        }
        else
        {
          enableSetProcess = true;
        }
        if (bar_pos < menu_rows - !enableSetProcess)
        {
          eraseBar(menu_rows, bar_pos);
          bar_pos++;
          updateBar(menu_rows, bar_pos);
        }
      }
      else
      {
        EncMove--;
        if (bar_pos > 1)
        {
          eraseBar(menu_rows, bar_pos);
          bar_pos--;
          updateBar(menu_rows, bar_pos);
        }
      }
      ypos = graphicHeight(bar_pos - 1);
    }
  }
  if (!GPIORead(ENC_SWITCH))
  {
    selected = !selected;
    if (menu_rows)
    {
      if (selected)
      {
        tft.fillRect(0, (tft.height() - height_heading) * (bar_pos - 1) / menu_rows + height_heading, width_select, (tft.height() - height_heading) / menu_rows, COLOR_CHOSEN);
      }
      else
      {
        tft.fillRect(0, (tft.height() - height_heading) * (bar_pos - 1) / menu_rows + height_heading, width_select, (tft.height() - height_heading) / menu_rows, WHITE);
      }
      for (int i = 2; i <= menu_rows; i++)
      {
        tft.fillRect(0, (tft.height() - height_heading) * (i - 1) / menu_rows + height_heading - 1, tft.height(), width_indentation, WHITE); // mejorable
      }
    }
    if (!encoderContinuousPress(UI_page))
    {
      switch (UI_page)
      {
      case mainMenuPage:
        switch (bar_pos - graphicTextOffset)
        {
        case controlModeGraphicPosition:
          in3.controlMode = !in3.controlMode;
          EEPROM.write(EEPROM_CONTROL_MODE, in3.controlMode);
          EEPROM.commit();
          UI_mainMenu();
          break;
        case temperatureGraphicPosition:
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              if (!in3.temperatureControl)
              {
                in3.temperatureControl = true;
                drawRightString(convertStringToChar(cstring, initialSensorsValue), initialSensorPosition, temperatureY, textFontSize);
                setTextColor(COLOR_MENU_TEXT);
                drawFloat(in3.desiredControlTemperature, 1, temperatureX - 65, temperatureY, textFontSize);
                enableSet = true;
              }
              if (EncMove > 0)
              {
                if (in3.desiredControlTemperature > minDesiredTemp[in3.controlMode])
                {
                  updateUIData = true;
                }
              }
              else
              {
                if (in3.desiredControlTemperature < maxDesiredTemp[in3.controlMode])
                {
                  updateUIData = true;
                }
              }
              if (updateUIData)
              {
                setTextColor(COLOR_MENU);
                drawFloat(in3.desiredControlTemperature, 1, temperatureX - 65, temperatureY, textFontSize);
                in3.desiredControlTemperature -= float(EncMove) * stepTemperatureIncrement;
                setTextColor(COLOR_MENU_TEXT);
                drawFloat(in3.desiredControlTemperature, 1, temperatureX - 65, temperatureY, textFontSize);
              }
              EncMove = false;
              updateUIData = false;
            }
          }
          EEPROM.write(EEPROM_DESIRED_CONTROL_MODE, in3.desiredControlTemperature);
          EEPROM.commit();
          drawStartMessage(enableSet, menu_rows, helpMessage);
          break;
        case humidityGraphicPosition:
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              if (!in3.humidityControl)
              {
                in3.humidityControl = true;
                setTextColor(COLOR_MENU);
                drawRightString(convertStringToChar(cstring, initialSensorsValue), initialSensorPosition, humidityY, textFontSize);
                setTextColor(COLOR_MENU_TEXT);
                drawCentreNumber(in3.desiredControlHumidity, humidityX - 65, humidityY);
                enableSet = true;
              }
              if (EncMove > 0)
              {
                if (in3.desiredControlHumidity > minHum)
                {
                  updateUIData = true;
                }
              }
              else
              {
                if (in3.desiredControlHumidity < maxHum)
                {
                  updateUIData = true;
                }
              }
              if (updateUIData)
              {
                setTextColor(COLOR_MENU);
                drawCentreNumber(in3.desiredControlHumidity, humidityX - 65, humidityY);
                in3.desiredControlHumidity -= (EncMove)*stepHumidityIncrement;
                setTextColor(COLOR_MENU_TEXT);
                drawCentreNumber(in3.desiredControlHumidity, humidityX - 65, humidityY);
              }
            }
            EncMove = false;
            updateUIData = false;
          }
          EEPROM.write(EEPROM_DESIRED_CONTROL_HUMIDITY, in3.desiredControlHumidity);
          EEPROM.commit();
          drawStartMessage(enableSet, menu_rows, helpMessage);
          break;
        case LEDGraphicPosition:
          in3.phototherapy = !in3.phototherapy;
          setTextColor(COLOR_MENU);
          if (in3.phototherapy)
          {
            drawRightString(convertStringToChar(cstring, "OFF"), unitPosition, ypos, textFontSize);
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "ON"), unitPosition, ypos, textFontSize);
          }
          setTextColor(COLOR_MENU_TEXT);
          if (in3.phototherapy)
          {
            drawRightString(convertStringToChar(cstring, "ON"), unitPosition, ypos, textFontSize);
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "OFF"), unitPosition, ypos, textFontSize);
          }
          GPIOWrite(PHOTOTHERAPY, in3.phototherapy);
          break;
        case settingsGraphicPosition:
          UI_settings();
          break;
        case startGraphicPosition:
          UI_actuatorsProgress();
          break;
        }
        break;
      case settingsPage:
        switch (bar_pos - graphicTextOffset)
        {
        case languageGraphicPosition:
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              setTextColor(COLOR_MENU);
              switch (in3.language)
              {
              case spanish:
                textToWrite = convertStringToChar(cstring, "SPA");
                break;
              case english:
                textToWrite = convertStringToChar(cstring, "ENG");
                break;
              case french:
                textToWrite = convertStringToChar(cstring, "FRA");
                break;
              case portuguese:
                textToWrite = convertStringToChar(cstring, "POR");
                break;
              }
              drawRightString(textToWrite, unitPosition, ypos, textFontSize);
              in3.language -= EncMove;
              if (in3.language < 0)
              {
                in3.language = numLanguages - 1;
              }
              if (in3.language >= numLanguages)
              {
                in3.language = false;
              }
              setTextColor(COLOR_MENU_TEXT);
              switch (in3.language)
              {
              case spanish:
                textToWrite = convertStringToChar(cstring, "SPA");
                break;
              case english:
                textToWrite = convertStringToChar(cstring, "ENG");
                break;
              case french:
                textToWrite = convertStringToChar(cstring, "FRA");
                break;
              case portuguese:
                textToWrite = convertStringToChar(cstring, "POR");
                break;
              }
              drawRightString(textToWrite, unitPosition, ypos, textFontSize);
              EncMove = false;
            }
          }
          EEPROM.write(EEPROM_LANGUAGE, in3.language);
          EEPROM.commit();
          UI_settings();
          break;
        case serialNumberGraphicPosition:
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              setTextColor(COLOR_MENU);
              drawRightNumber(in3.serialNumber, unitPosition, ypos);
              in3.serialNumber -= EncMove;
              EEPROM.write(EEPROM_SERIAL_NUMBER, in3.serialNumber);
              setTextColor(COLOR_MENU_TEXT);
              drawRightNumber(in3.serialNumber, unitPosition, ypos);
            }
            EncMove = false;
          }
          EEPROM.commit();
          break;
        case WifiENGraphicPosition:
          WIFI_EN = !WIFI_EN;
          if (WIFI_EN)
          {
            wifiInit();
          }
          else
          {
            wifiDisable();
          }
          EEPROM.write(EEPROM_WIFI_EN, WIFI_EN);
          EEPROM.commit();
          setTextColor(COLOR_MENU);
          if (WIFI_EN)
          {
            drawRightString(convertStringToChar(cstring, "OFF"), unitPosition, ypos, textFontSize);
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "ON"), unitPosition, ypos, textFontSize);
          }
          setTextColor(COLOR_MENU_TEXT);
          if (WIFI_EN)
          {
            drawRightString(convertStringToChar(cstring, "ON"), unitPosition, ypos, textFontSize);
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "OFF"), unitPosition, ypos, textFontSize);
          }
          break;
        case setdefaultValuesGraphicPosition:
          loaddefaultValues();
          if (WIFI_EN)
          {
            wifiInit();
          }
          else
          {
            wifiDisable();
          }
          UI_settings();
          break;
        case HWTestGraphicPosition:
          initHardware(true);
          UI_settings();
          break;
        case calibrateGraphicPosition:
          UI_calibration();
          break;
        }
        break;
      case calibrateSensorsPage:
        switch (bar_pos - graphicTextOffset)
        {
        case twoPointCalibrationGraphicPosition:
          firstPointCalibration();
          break;
        case fineTuneCalibrationGraphicPosition:
          fineTuneCalibration();
          break;
        case autoCalibrationGraphicPosition:
          autoCalibration();
          break;
        case restartCalibrationGraphicPosition:
          recapVariables();
          UI_calibration();
          break;
        }
        break;
      case fineTuneCalibrationPage:
        switch (bar_pos - graphicTextOffset)
        {
        case temperatureCalibrationGraphicPosition:
          errorTemperature[skinSensor] = false;
          diffSkinTemperature = in3.temperature[skinSensor];
          diffAirTemperature = in3.temperature[airSensor];
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              setTextColor(COLOR_MENU);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              setTextColor(COLOR_MENU_TEXT);
              diffSkinTemperature += EncMove * (0.1);
              diffAirTemperature += EncMove * (0.1);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              EncMove = false;
            }
          }
          break;
        case setCalibrationGraphicPosition:
          fineTuneSkinTemperature = diffSkinTemperature - in3.temperature[skinSensor];
          fineTuneAirTemperature = diffAirTemperature - in3.temperature[airSensor];
          log("[CALIBRATION] -> Fine tune Skin value is " + String(fineTuneSkinTemperature));
          log("[CALIBRATION] -> Fine tune Air value is " + String(fineTuneAirTemperature));
          EEPROM.writeFloat(EEPROM_FINE_TUNE_TEMP_SKIN, fineTuneSkinTemperature);
          EEPROM.writeFloat(EEPROM_FINE_TUNE_TEMP_AIR, fineTuneAirTemperature);
          EEPROM.commit();
          UI_mainMenu();
          break;
        }
        break;
      case firstPointCalibrationPage:
        clearCalibrationValues();
        switch (bar_pos - graphicTextOffset)
        {
        case temperatureCalibrationGraphicPosition:
          errorTemperature[skinSensor] = false;
          diffSkinTemperature = in3.temperature[skinSensor];
          diffAirTemperature = in3.temperature[airSensor];
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              setTextColor(COLOR_MENU);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              setTextColor(COLOR_MENU_TEXT);
              diffSkinTemperature += EncMove * (0.1);
              diffAirTemperature += EncMove * (0.1);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              EncMove = false;
            }
          }
          break;
        case setCalibrationGraphicPosition:
          provisionalReferenceTemperatureLow = diffSkinTemperature;
          provisionalRawTemperatureLow[skinSensor] = in3.temperature[skinSensor];
          log("[CALIBRATION] -> Low reference point is " + String(provisionalReferenceTemperatureLow) + ", low raw skin point is " + String(provisionalRawTemperatureLow[skinSensor]));
          secondPointCalibration();
          break;
        }
        break;
      case secondPointCalibrationPage:
        switch (bar_pos - graphicTextOffset)
        {
        case temperatureCalibrationGraphicPosition:
          diffSkinTemperature = in3.temperature[skinSensor];
          while (GPIORead(ENC_SWITCH))
          {
            updateData();
            if (EncMove)
            {
              setTextColor(COLOR_MENU);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              setTextColor(COLOR_MENU_TEXT);
              diffSkinTemperature += EncMove * (0.1);
              drawFloat(diffSkinTemperature, 1, valuePosition, ypos, textFontSize);
              EncMove = false;
              log("difTemp: " + String(diffSkinTemperature));
            }
          }
          break;
        case setCalibrationGraphicPosition:
          ReferenceTemperatureLow = provisionalReferenceTemperatureLow;
          RawTemperatureLow[skinSensor] = provisionalRawTemperatureLow[skinSensor];
          ReferenceTemperatureRange = diffSkinTemperature - ReferenceTemperatureLow;
          if (RawTemperatureRange[skinSensor])
          {
            RawTemperatureRange[skinSensor] = (in3.temperature[skinSensor] - RawTemperatureLow[skinSensor]);
            log("calibration factors: " + String(RawTemperatureLow[skinSensor]) + "," + String(RawTemperatureRange[skinSensor]) + "," + String(ReferenceTemperatureRange) + "," + String(ReferenceTemperatureLow));
            saveCalibrationToEEPROM();
          }
          else
          {
            log("[CALIBRATION] -> ERROR -> DIVIDE BY ZERO");
          }
          UI_settings();
          break;
        }
        break;
      case autoCalibrationPage:
        break;
      }
      selected = false;
      if (menu_rows)
      {
        tft.fillRect(0, (tft.height() - height_heading) * (bar_pos - 1) / menu_rows + height_heading, width_select, (tft.height() - height_heading) / menu_rows, WHITE);
      }
      encoderContinuousPress(UI_page);
      vTaskDelay(debounceTime / portTICK_PERIOD_MS);
    }
  }
}

bool encoderContinuousPress(int UI_page)
{
  updateData();
  if (UI_page == mainMenuPage)
  {
    long timePressed = millis();
    while (!GPIORead(ENC_SWITCH))
    {
      updateData();
      if (millis() - timePressed > timePressToSettings)
      {
        UI_settings();
        return (true);
      }
    }
  }
  else
  {
    return (back_mode());
  }
  return false;
}

int getYpos(int UI_menu_rows, byte row)
{
  row++; // because it starts at zero
  if (UI_menu_rows)
  {
    return ((tft.height() - height_heading) / (2 * UI_menu_rows) + (row - 1) * (tft.height() - height_heading) / (menu_rows) + letter_height);
  }
  return false;
}

void checkSetMessage(int UI_page)
{
  int compareTime;
  if (blinkSetMessageState)
  {
    compareTime = blinkTimeON;
  }
  else
  {
    compareTime = blinkTimeOFF;
  }
  if (millis() - lastBlinkSetMessage > compareTime)
  {
    lastBlinkSetMessage = millis();
    blinkSetMessageState = !blinkSetMessageState;
    if (blinkSetMessageState)
    {
      setTextColor(COLOR_WARNING_TEXT);
    }
    else
    {
      setTextColor(COLOR_MENU);
    }
    if (page == mainMenuPage)
    {
      drawHelpMessage(in3.language);
    }
    drawCentreString(helpMessage, width_select + (tft.width() - width_select) / 2, getYpos(menu_rows, startGraphicPosition), textFontSize);
  }
}

bool back_mode()
{
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
  last_encPulsed = millis();
  byte back_bar = false;
  while (!GPIORead(ENC_SWITCH))
  {
    updateData();
    if (millis() - last_encPulsed > time_back_wait)
    {
      back_bar++;
      tft.drawLine(width_back - back_bar, 0, width_back - back_bar, height_heading, COLOR_MENU);
    }
    if (back_bar == width_back)
    {
      UI_mainMenu();
      return true;
    }
    vTaskDelay(((time_back_draw + time_back_wait) / width_back) / portTICK_PERIOD_MS);
  }
  if (millis() - last_encPulsed > time_back_wait)
  {
    drawBack();
  }
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
  return (false);
}
