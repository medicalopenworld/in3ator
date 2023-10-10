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

extern bool WIFI_EN;
extern long lastDebugUpdate;
extern long loopCounts;
extern int page;

extern double errorTemperature[SENSOR_TEMP_QTY], temperatureCalibrationPoint;
extern double ReferenceTemperatureRange, ReferenceTemperatureLow;
extern double provisionalReferenceTemperatureLow;
extern double fineTuneSkinTemperature;
extern double RawTemperatureLow[SENSOR_TEMP_QTY],
    RawTemperatureRange[SENSOR_TEMP_QTY];
extern double provisionalRawTemperatureLow[SENSOR_TEMP_QTY];
extern int temperature_array_pos; // temperature sensor number turn to measure
extern float diffSkinTemperature,
    diffAirTemperature; // difference between measured temperature and user
                        // input real temperature
extern bool humidifierState, humidifierStateChange;
extern int previousHumidity; // previous sampled humidity
extern float diffHumidity;   // difference between measured humidity and user
                             // input real humidity

extern byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and
// they have different check rates
extern byte encoderRate;
extern byte encoderCount;

extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;

// room variables

extern boolean A_set;
extern boolean B_set;
extern int encoderpinA;                 // pin  encoder A
extern int encoderpinB;                 // pin  encoder B
extern bool encPulsed, encPulsedBefore; // encoder switch status
extern bool updateUIData;
extern volatile int EncMove;     // moved encoder
extern volatile int lastEncMove; // moved last encoder
extern volatile int
    EncMoveOrientation;            // set to -1 to increase values clockwise
extern int last_encoder_move;      // moved encoder
extern long encoder_debounce_time; // in milliseconds, debounce time in encoder
                                   // to filter signal bounces
extern long last_encPulsed;        // last time encoder was pulsed

// Text Graphic position variables
extern int humidityX;
extern int humidityY;
extern int temperatureX;
extern int temperatureY;
extern int separatorTopYPos, separatorMidYPos, separatorBotYPos;
extern int ypos;
extern bool print_text;
extern int initialSensorPosition;
extern bool pos_text[8];

extern bool enableSet;
extern float temperaturePercentage, temperatureAtStart;
extern float humidityPercentage, humidityAtStart;
extern int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX,
    humBarPosY;
extern int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
extern bool autoLock; // setting that enables backlight switch OFF after a
                      // given time of no user actions
extern long
    lastbacklightHandler; // last time there was a encoder movement or pulse
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

void setTextColor(int16_t color) { screenTextColor = color; }

void setBackgroundColor(int16_t color) { screenTextBackgroundColor = color; }

int16_t getBackgroundColor() { return screenTextBackgroundColor; }

void setSensorsGraphicPosition(int UI_page)
{
  switch (UI_page)
  {
  case mainMenuPage:
    humidityX = tft.width() - 50;
    humidityY = graphicHeight(humidityGraphicPosition);
    temperatureX = tft.width() - 79;
    temperatureY = graphicHeight(temperatureGraphicPosition);
    break;
  case actuatorsProgressPage:
    barWidth = tft.width() / 4 * 2;
    barHeight = 20;
    tempBarPosX = tft.width() / 2;
    tempBarPosY = tft.height() / 3 - 11;
    humBarPosX = tempBarPosX;
    humBarPosY = tft.height() * 3 / 4 + 15;
    temperatureX = letter_width;
    temperatureY = tempBarPosY - barHeight / 2;
    humidityX = 4 * letter_width;
    humidityY = humBarPosY - barHeight / 2;
    separatorTopYPos = tft.height() / 3 + 5;
    separatorMidYPos = tft.height() * 2 / 3 - 10;
    break;
  }
}
int16_t drawString(char *string, int16_t poX, int16_t poY, int16_t size)
{
  int16_t sumX = 0;
  int16_t width = 8;
  int16_t gap = -2;

  while (*string)
  {
    tft.drawChar(poX, poY, *string, screenTextColor, screenTextBackgroundColor,
                 size);
    *string++;
    poX += (width + gap) * size; /* Move cursor right       */
  }
  return sumX;
}

void drawHeading(int UI_page, int UI_serialNumber)
{
  tft.fillRect(0, 0, tft.width(), height_heading, COLOR_HEADING);
  if (UKRAINE_MODE)
  {
    tft.fillRect(0, height_heading / 2, tft.width(), height_heading / 2,
                 YELLOW);
  }
  if (UI_page != mainMenuPage)
  {
    drawBack();
  }
  setTextColor(COLOR_MENU);
  drawCentreString(convertStringToChar(cstring, "in3_"),
                   tft.width() / 2 - 2 * letter_width - 10, headint_text_height,
                   textFontSize);
  drawCentreNumber(UI_serialNumber, tft.width() / 2, headint_text_height);
  drawCentreString(
      convertStringToChar(cstring, String(FWversion) + "/" + HWversion),
      tft.width() - 4 * letter_width, headint_text_height, textFontSize);
}

void updateHeadingEvent(byte Event, bool event_status)
{
  uint16_t currentBackgroundColor = getBackgroundColor();
  setBackgroundColor(COLOR_HEADING);
  if (event_status)
  {
    setTextColor(COLOR_MENU);
  }
  else
  {
    setTextColor(COLOR_HEADING);
  }
  switch (Event)
  {
  case EVENT_2G:
    drawCentreString(convertStringToChar(cstring, "2G"), EVENT_2G_UI_POS,
                     headint_text_height, textFontSize);
    break;
  case EVENT_WIFI:
    drawCentreString(convertStringToChar(cstring, "W"), EVENT_WIFI_UI_POS,
                     headint_text_height, textFontSize);
    break;
  case EVENT_SERVER_CONNECTION:
    drawCentreString(convertStringToChar(cstring, "S"),
                     EVENT_SERVER_CONNECTION_UI_POS, headint_text_height,
                     textFontSize);
    break;
  case EVENT_OTA_ONGOING:
    drawCentreString(convertStringToChar(cstring, "O"),
                     EVENT_OTA_ONGOING_UI_POS, headint_text_height,
                     textFontSize);
    break;
  default:
    break;
  }
  setBackgroundColor(currentBackgroundColor);
}

void UI_updateConnectivityEvents()
{
  updateHeadingEvent(EVENT_2G, GPRSIsAttached());
  updateHeadingEvent(EVENT_WIFI, WIFIIsConnected());
  updateHeadingEvent(EVENT_SERVER_CONNECTION,
                     GPRSIsConnectedToServer() || WIFIIsConnectedToServer());
}

void eraseBar(int UI_menu_rows, int bar_pos)
{
  if (UI_menu_rows)
  {
    tft.fillRect(
        0,
        (tft.height() - height_heading) * (bar_pos - 1) / UI_menu_rows +
            height_heading,
        width_select, (tft.height() - height_heading) / UI_menu_rows,
        COLOR_BAR);
  }
}

void updateBar(int UI_menu_rows, int bar_pos)
{
  if (UI_menu_rows)
  {
    tft.fillRect(
        0,
        (tft.height() - height_heading) * (bar_pos - 1) / UI_menu_rows +
            height_heading,
        width_select, (tft.height() - height_heading) / UI_menu_rows,
        COLOR_SELECTED);
    for (int i = 2; i <= UI_menu_rows; i++)
    {
      tft.fillRect(0,
                   (tft.height() - height_heading) * (i - 1) / UI_menu_rows +
                       height_heading - 1,
                   tft.height(), width_indentation, WHITE); // mejorable
    }
  }
}

int16_t drawNumber(long long_num, int16_t poX, int16_t poY, int16_t size)
{
  char tmp[10];
  if (long_num < 0)
    sprintf(tmp, "%li", long_num);
  else
    sprintf(tmp, "%lu", long_num);
  return drawString(tmp, poX, poY, size);
}

void drawBack()
{
  tft.fillRect(0, 0, width_back, height_heading, COLOR_HEADING);
  tft.drawRect(0, 0, width_back, height_heading, BLACK);
  tft.fillTriangle(arrow_height, height_heading / 2, width_back / 2,
                   arrow_height, width_back / 2, height_heading - arrow_height,
                   COLOR_ARROW);
  tft.fillRect(width_back / 2, height_heading / 2 - arrow_tail,
               width_back / 2 - arrow_height, arrow_tail, COLOR_ARROW);
  tft.fillRect(width_back / 2, height_heading / 2,
               width_back / 2 - arrow_height, arrow_tail, COLOR_ARROW);
}

void drawRightNumber(int n, int x, int i)
{
  length = true;
  for (long k = 10; k <= n; k *= 10)
  {
    length++;
  }
  drawNumber(n, x - length * 14, i, textFontSize);
}

void drawIntroMessage()
{
  byte numWords = 3;
  switch (in3.language)
  {
  case english:
    words[0] = convertStringToChar("Welcome to in3ator");
    words[1] = convertStringToChar("");
    words[2] = convertStringToChar("Saving lives");
    break;
  case spanish:
    words[0] = convertStringToChar("Bienvenido a in3");
    words[1] = convertStringToChar("");
    words[2] = convertStringToChar("Salvando vidas");
    break;
  case french:
    words[0] = convertStringToChar("Bienvenue a in3");
    words[1] = convertStringToChar("");
    words[2] = convertStringToChar("Sauver des vies");
    break;
  case portuguese:
    words[0] = convertStringToChar("Bem-vindo ao");
    words[1] = convertStringToChar("");
    words[2] = convertStringToChar("Salvando vidas");
    break;
  }
  for (int i = false; i < numWords; i++)
  {
    drawCentreString(words[i], tft.width() / 2,
                     tft.height() * (1 + i) / (2 + numWords), textFontSize);
  }
}

void drawAlarmMessage(char *alertMessage)
{
  setTextColor(COLOR_WARNING_TEXT);
  drawCentreString(alertMessage,
                   width_select + (tft.width() - width_select) / 2,
                   headint_text_height, textFontSize);
}

void drawHumidityUnits()
{
  drawRightString(convertStringToChar(cstring, "/"), separatorPosition, ypos,
                  textFontSize);
  drawRightString(convertStringToChar(cstring, "%"), unitPosition, ypos,
                  textFontSize);
}

int decimalDigits(long n)
{
  int length = true;
  for (long k = 10; k <= n; k *= 10)
  {
    length++;
  }
  return (length);
}

void drawCentreNumber(int n, int x, int i)
{
  drawNumber(n, x - decimalDigits(n) * 7, i, textFontSize);
}

int hexDigits(long n)
{
  int length = true;
  for (long k = 16; k <= n; k *= 16)
  {
    length++;
  }
  return (length);
}

void drawStop()
{
  switch (in3.language)
  {
  case spanish:
    textToWrite = convertStringToChar(cstring, "Pulsa 2 seg para salir");
    break;
  case english:
    textToWrite = convertStringToChar(cstring, "Press 2 sec to go back");
    break;
  case french:
    textToWrite = convertStringToChar(cstring, "appuyez 2 sec pour voler");
    break;
  case portuguese:
    textToWrite =
        convertStringToChar(cstring, "Pressione 2 segundos para sair");
    break;
  }
  drawCentreString(textToWrite, tft.width() / 2, tft.height() - letter_height,
                   textFontSize);
}

int graphicHeight(int position)
{
  if (menu_rows)
  {
    return ((tft.height() - height_heading) / (2 * menu_rows) +
            position * (tft.height() - height_heading) / (menu_rows) +
            letter_height);
  }
  return false;
}

void drawSelectedTemperature(float temperatureToDraw,
                             float previousTemperatureDrawn)
{
  setTextColor(COLOR_MENU);
  drawFloat(previousTemperatureDrawn, 1, temperatureX, temperatureY,
            textFontSize);
  setTextColor(COLOR_MENU_TEXT);
  drawFloat(temperatureToDraw, 1, temperatureX, temperatureY, textFontSize);
}

void drawUnselectedTemperature(float temperatureToDraw,
                               float previousTemperatureDrawn)
{
  tft.setTextColor(COLOR_MENU);
  drawFloat(previousTemperatureDrawn, 1, tft.width() / 2 - 20,
            tft.height() / 2 + 10, textFontSize);
  tft.setTextColor(COLOR_MENU_TEXT);
  drawFloat(temperatureToDraw, 1, tft.width() / 2 - 20, tft.height() / 2 + 10,
            textFontSize);
}

void drawHumidity(int UI_humidity, int UI_previousHumdity)
{
  setTextColor(COLOR_MENU);
  drawCentreNumber(UI_previousHumdity, humidityX, humidityY);
  setTextColor(COLOR_MENU_TEXT);
  drawCentreNumber(UI_humidity, humidityX, humidityY);
}

void drawHelpMessage(byte UI_language)
{
  switch (UI_language)
  {
  case english:
    helpMessage = convertStringToChar(cstring, "Set desired parameters");
    break;
  case spanish:
    helpMessage = convertStringToChar(cstring, "Introduce parametros");
    break;
  case french:
    helpMessage = convertStringToChar(cstring, "Entrer parametres");
    break;
  case portuguese:
    helpMessage = convertStringToChar(cstring, "Insira os parametros");
    break;
  }
}

void drawStartMessage(bool UI_enableSet, int UI_menu_rows,
                      char *UI_helpMessage)
{
  if (UI_enableSet)
  {
    setTextColor(COLOR_MENU);
    drawHelpMessage(in3.language);
    drawCentreString(helpMessage,
                     width_select + (tft.width() - width_select) / 2,
                     getYpos(UI_menu_rows, startGraphicPosition), textFontSize);
    setTextColor(COLOR_MENU_TEXT);
    switch (in3.language)
    {
    case spanish:
      words[startGraphicPosition] = convertStringToChar(cstring, "Empezar");
      break;
    case english:
      words[startGraphicPosition] = convertStringToChar(cstring, "Start");
      break;
    case french:
      words[startGraphicPosition] = convertStringToChar(cstring, "Debut");
      break;
    case portuguese:
      words[startGraphicPosition] = convertStringToChar(cstring, "Comecar");
      break;
    }
    drawCentreString(words[startGraphicPosition],
                     width_select + (tft.width() - width_select) / 2,
                     getYpos(UI_menu_rows, startGraphicPosition), textFontSize);
  }
}

void drawActuatorsSeparators()
{
  tft.fillRect(0, separatorTopYPos, tft.width(), barThickness, COLOR_FRAME_BAR);
  tft.fillRect(0, separatorMidYPos, tft.width(), barThickness, COLOR_FRAME_BAR);
}

void printLoadingTemperatureBar(double UI_desiredControlTemperature)
{
  drawFloat(UI_desiredControlTemperature, 1, tft.width() - 5 * letter_width,
            temperatureY, textFontSize);
  for (int i = true; i <= barThickness; i++)
  {
    tft.drawRect(tempBarPosX - barWidth / 2 - i,
                 tempBarPosY - barHeight / 2 - i, barWidth + i * 2,
                 barHeight + i * 2, COLOR_FRAME_BAR);
  }
}

void printLoadingHumidityBar(int UI_desiredControlHumidity)
{
  drawFloat(UI_desiredControlHumidity, 1, humBarPosX + barWidth / 2 + 10,
            humidityY, textFontSize);
  for (int i = true; i <= barThickness; i++)
  {
    tft.drawRect(humBarPosX - barWidth / 2 - i, humBarPosY - barHeight / 2 - i,
                 barWidth + i * 2, barHeight + i * 2, COLOR_FRAME_BAR);
  }
}

void updateLoadingTemperatureBar(float prev, float actual)
{
  if (prev != actual)
  {
    float diff = (actual - prev) / 100;
    int color;
    float barX;
    int barY, barDiffWidth;
    barX = tempBarPosX - (barWidth / 2) * (1 - prev / 50);
    barY = tempBarPosY - barHeight / 2;
    barDiffWidth = barWidth * abs(diff) + 1;
    if (diff > 0)
    {
      color = COLOR_LOADING_BAR;
    }
    else
    {
      color = COLOR_MENU;
      barX -= barDiffWidth - 1;
    }
    tft.fillRect(barX, barY, barDiffWidth, barHeight, color);
  }
}

void updateLoadingHumidityBar(float prev, float actual)
{
  if (prev != actual)
  {
    float diff = (actual - prev) / 100;
    int color;
    float barX;
    int barY, barDiffWidth;
    barX = humBarPosX - (barWidth / 2) * (1 - prev / 50);
    barY = humBarPosY - barHeight / 2;
    barDiffWidth = barWidth * abs(diff) + 1;
    if (diff > 0)
    {
      color = COLOR_LOADING_BAR;
    }
    else
    {
      color = COLOR_MENU;
      barX -= barDiffWidth - 1;
    }
    tft.fillRect(barX, barY, barDiffWidth, barHeight, color);
  }
}

void blinkGoBackMessage()
{
  if (millis() - blinking > 1000)
  {
    blinking = millis();
    state_blink = !state_blink;
    if (state_blink)
    {
      setTextColor(ILI9341_ORANGE);
    }
    else
    {
      setTextColor(COLOR_MENU);
      blinking += 400;
    }
    drawStop();
  }
}

int16_t drawCentreString(char *string, int16_t dX, int16_t poY, int16_t size)
{
  int16_t sumX = 0;
  int16_t len = 0;
  char *pointer = string;
  int16_t width = 8;
  int16_t gap = -2;

  while (*pointer)
  {
    len += (width + gap);
    *pointer++;
  }
  len = len * size;
  int16_t poX = dX - len / 2;
  if (poX < 0)
    poX = 0;

  while (*string)
  {
    tft.drawChar(poX, poY, *string, screenTextColor, screenTextBackgroundColor,
                 size);
    *string++;
    poX += (width + gap) * size; /* Move cursor right       */
  }
  return sumX;
}

int16_t drawRightString(char *string, int16_t dX, int16_t poY, int16_t size)
{
  int16_t sumX = 0;
  int16_t len = 0;
  char *pointer = string;
  int16_t width = 8;
  int16_t gap = -2;

  while (*pointer)
  {
    len += (width + gap);
    *pointer++;
  }

  len = len * size;
  int16_t poX = dX - len;

  if (poX < 0)
    poX = 0;

  while (*string)
  {
    tft.drawChar(poX, poY, *string, screenTextColor, screenTextBackgroundColor,
                 size);
    *string++;
    poX += (width + gap) * size; /* Move cursor right       */
  }

  return sumX;
}

int16_t drawFloat(float floatNumber, int16_t decimal, int16_t poX, int16_t poY,
                  int16_t size)
{
  unsigned long temp = 0;
  float decy = 0.0;
  float rounding = 0.5;
  float eep = 0.000001;
  int16_t sumX = 0;
  char negativeSymbol[] = "-";
  char decimalSymbol[] = ".";
  int16_t width = 8;
  int16_t gap = -2;
  int r = 1;

  if (floatNumber - 0.0 < eep) // floatNumber < 0
  {
    drawString(negativeSymbol, poX, poY, size);
    floatNumber = -floatNumber;
    poX += (width + gap) * size;
    sumX += (width + gap) * size;
  }

  for (unsigned char i = 0; i < decimal; ++i)
  {
    rounding /= 10.0;
  }
  floatNumber += rounding;
  temp = (long)floatNumber;
  drawNumber(temp, poX, poY, size);
  while (temp > 9)
  {
    temp /= 10;
    r++;
  }
  temp = (long)floatNumber;
  poX += (width + gap) * size * r;
  sumX += (width + gap) * size;

  if (decimal > 0)
  {
    drawString(decimalSymbol, poX, poY, size);
    poX += (width + gap) * size; /* Move cursor right            */
    sumX += (width + gap) * size;
  }
  else
  {
    return sumX;
  }

  decy = floatNumber - temp;
  for (unsigned char i = 0; i < decimal; i++)
  {
    decy *= 10;  /* for the next decimal         */
    temp = decy; /* get the decimal              */
    drawNumber(temp, poX, poY, size);

    poX += (width + gap) * size; /* Move cursor right            */
    sumX += (width + gap) * size;
    decy -= temp;
  }
  return sumX;
}

char *convertStringToChar(String input)
{
  char *cstr = new char[input.length() + 1];
  strcpy(cstr, input.c_str());
  return cstr;
}

char *convertStringToChar(char *arrayInput, String input)
{
  strcpy(arrayInput, input.c_str());
  return arrayInput;
}

void drawTemperatureUnits()
{
  drawRightString(convertStringToChar(cstring, "/"), separatorPosition, ypos,
                  textFontSize);
  drawRightString(convertStringToChar(cstring, "C"), unitPosition, ypos,
                  textFontSize);
}

void loadlogo()
{
  tft.setTextSize(1);
  if (UKRAINE_MODE)
  {
    tft.fillScreen(BLUE);
    tft.fillRect(0, tft.height() / 2, tft.width(), tft.height() / 2, YELLOW);
    setTextColor(WHITE);
  }
  else
  {
    tft.fillScreen(introBackColor);
    tft.fillScreen(introBackColor);
    setTextColor(introTextColor);
  }
  drawIntroMessage();
}

/*
   Function pending to complete
*/
void drawHardwareErrorMessage(long error, bool criticalError,
                              bool calibrationError)
{
  tft.fillScreen(introBackColor);
  tft.setTextColor(
      introTextColor); // use tft. because tft.print is configured by it
  tft.setCursor(tft.width() / 4 - hexDigits(error) * 16, tft.height() / 10);
  tft.setTextSize(3);
  if (error || criticalError || calibrationError)
  {
    tft.print("HW error:");
    tft.println(error, HEX);
    tft.println();
    if (criticalError || calibrationError)
    {
      tft.setTextSize(3);
      if (criticalError)
      {
        tft.println("WIRING ERROR");
      }
      if (calibrationError)
      {
        tft.println("CALIBRATION ERROR");
      }
      tft.println();
    }
    switch (in3.language)
    {
    case spanish:
      textToWrite = convertStringToChar(cstring, " Por favor contacta");
      break;
    case portuguese:
      textToWrite =
          convertStringToChar(cstring, " Por favor entre em contato");
      break;
    case english:
      textToWrite = convertStringToChar(cstring, " Please contact");
      break;
    case french:
      textToWrite =
          convertStringToChar(cstring, " S'il vous plait contactez");
      break;
    }
    tft.println(textToWrite);
    tft.setTextSize(2);
    tft.println("  medicalopenworld.org");
  }
  else
  {
    tft.print("SUCCESS :)");
  }
  tft.println();
  tft.println();
  tft.setTextSize(2);
  tft.print(" ");
  switch (in3.language)
  {
  case spanish:
    textToWrite = convertStringToChar(cstring, "Presione para continuar");
    break;
  case portuguese:
    textToWrite = convertStringToChar(cstring, "Pressione para continuar");
    break;
  case english:
    textToWrite = convertStringToChar(cstring, "Press to continue");
    break;
  case french:
    textToWrite = convertStringToChar(cstring, "Appuyez pour continuer");
    break;
  }
  tft.println(textToWrite);
}

void graphics(uint8_t UI_page, uint8_t UI_language, uint8_t UI_print_text,
              uint8_t UI_menu_rows, uint8_t UI_var_0, uint8_t UI_var_1)
{
  setTextColor(COLOR_MENU_TEXT);
  if (!UI_page)
  {
    tft.fillRect(width_select, height_heading, tft.width() - width_select,
                 tft.height() - height_heading, COLOR_MENU);
  }
  else
  {
    tft.fillRect(0, height_heading, tft.width(), tft.height() - height_heading,
                 COLOR_MENU);
  }
  if (UI_print_text)
  {
    if (UI_menu_rows)
    {
      tft.fillRect(0, height_heading, width_select,
                   (tft.height() - height_heading) / UI_menu_rows,
                   COLOR_SELECTED);
    }
  }
  for (int i = 2; i <= UI_menu_rows; i++)
  {
    if (UI_menu_rows)
    {
      tft.fillRect(0,
                   (tft.height() - height_heading) * (i - 1) / UI_menu_rows +
                       height_heading,
                   width_select, (tft.height() - height_heading) / UI_menu_rows,
                   COLOR_BAR);
      tft.fillRect(0,
                   (tft.height() - height_heading) * (i - 1) / UI_menu_rows +
                       height_heading - 1,
                   tft.width(), width_indentation, WHITE);
    }
  }
  tft.drawRect(0, tft.height() - 1, width_select, tft.height() - 1, COLOR_MENU);
  if (UI_print_text)
  {
    tft.setTextSize(1);
    for (int i = false; i < UI_menu_rows; i++)
    {
      ypos = graphicHeight(i);
      if (!pos_text[i])
      {
        drawString(words[i], width_select + side_gap, ypos, textFontSize);
      }
      else if (pos_text[i])
      {
        drawCentreString(words[i],
                         width_select + (tft.width() - width_select) / 2, ypos,
                         textFontSize);
      }
      switch (UI_page)
      {
      case mainMenuPage:
        switch (i)
        {
        case controlModeGraphicPosition:
          if (UI_var_0)
          {
            switch (in3.language)
            {
            case spanish:
              textToWrite = convertStringToChar(cstring, "AIRE");
              break;
            case english:
              textToWrite = convertStringToChar(cstring, "AIR");
              break;
            case french:
              textToWrite = convertStringToChar(cstring, "AIR");
              break;
            case portuguese:
              textToWrite = convertStringToChar(cstring, "AR");
              break;
            }
            drawRightString(textToWrite, unitPosition, ypos, textFontSize);
          }
          else
          {
            switch (in3.language)
            {
            case spanish:
              textToWrite = convertStringToChar(cstring, "PIEL");
              break;
            case english:
              textToWrite = convertStringToChar(cstring, "SKIN");
              break;
            case french:
              textToWrite = convertStringToChar(cstring, "PEAU");
              break;
            case portuguese:
              textToWrite = convertStringToChar(cstring, "PELE");
              break;
            }
            drawRightString(textToWrite, unitPosition, ypos, textFontSize);
          }
          break;
        case temperatureGraphicPosition:
          drawTemperatureUnits();
          drawRightString(convertStringToChar(cstring, initialSensorsValue),
                          initialSensorPosition, temperatureY,
                          textFontSize);
          break;
        case LEDGraphicPosition:
          if (UI_var_1)
          {
            drawRightString(convertStringToChar(cstring, "ON"),
                            unitPosition, ypos, textFontSize);
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "OFF"),
                            unitPosition, ypos, textFontSize);
          }
          break;
        case humidityGraphicPosition:
          drawHumidityUnits();
          drawRightString(convertStringToChar(cstring, initialSensorsValue),
                          initialSensorPosition, humidityY, textFontSize);
          break;
        }
        break;
      case settingsPage:
        switch (i)
        {
        case languageGraphicPosition:
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
          break;
        case serialNumberGraphicPosition:
          drawRightNumber(UI_var_0, unitPosition, ypos);
          break;
        case WifiENGraphicPosition:
          if (UI_var_1)
          {
            drawRightString(convertStringToChar(cstring, "ON"),
                            unitPosition, ypos, textFontSize);
            if (WiFi.status() == WL_CONNECTED)
            {
              drawCentreString(
                  convertStringToChar(cstring,
                                      String(WiFi.localIP()[0]) + "." +
                                          String(WiFi.localIP()[1]) + "." +
                                          String(WiFi.localIP()[2]) + "." +
                                          String(WiFi.localIP()[3])),
                  tft.width() / 2, ypos, textFontSize);
            }
          }
          else
          {
            drawRightString(convertStringToChar(cstring, "OFF"),
                            unitPosition, ypos, textFontSize);
          }
        }
        break;
      }
    }
  }
}