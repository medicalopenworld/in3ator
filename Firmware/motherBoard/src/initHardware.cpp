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
extern SHTC3 mySHTC3;  // Declare an instance of the SHTC3 class
extern Adafruit_SHT4x sht4;
extern RotaryEncoder encoder;
extern Beastdevices_INA3221 mainDigitalCurrentSensor;
extern Beastdevices_INA3221 secundaryDigitalCurrentSensor;

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
extern float diffSkinTemperature, diffAirTemperature; // difference between measured temperature and user input real temperature
extern bool humidifierState, humidifierStateChange;
extern int previousHumidity;  // previous sampled humidity
extern float diffHumidity;    // difference between measured humidity and user
                              // input real humidity

extern byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and
// they have different check rates
extern byte encoderRate;
extern byte encoderCount;
extern bool encPulseDetected;
extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;
extern bool ambientSensorPresent;
extern bool digitalCurrentSensorPresent[2];

// room variables
extern bool controlAlgorithm;

extern boolean A_set;
extern boolean B_set;
extern int encoderpinA;                  // pin  encoder A
extern int encoderpinB;                  // pin  encoder B
extern bool encPulsed, encPulsedBefore;  // encoder switch status
extern bool updateUIData;
extern volatile int EncMove;      // moved encoder
extern volatile int lastEncMove;  // moved last encoder
extern volatile int
    EncMoveOrientation;             // set to -1 to increase values clockwise
extern int last_encoder_move;       // moved encoder
extern long encoder_debounce_time;  // in milliseconds, debounce time in encoder
                                    // to filter signal bounces
extern long last_encPulsed;         // last time encoder was pulsed

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
extern int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX,
    humBarPosY;
extern int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
extern bool autoLock;  // setting that enables backlight switch OFF after a
                       // given time of no user actions
extern long
    lastbacklightHandler;  // last time there was a encoder movement or pulse
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

extern double Kp[numPID], Ki[numPID], Kd[numPID];
extern PID airControlPID;
extern PID skinControlPID;
extern PID humidityControlPID;

extern int ScreenBacklightMode;

#define testMode false
#define operativeMode true

#define CURRENT_STABILIZE_TIME_DEFAULT 700
#define CURRENT_STABILIZE_TIME_HEATER 1000

#define NTC_BABY_MIN 1
#define NTC_BABY_MAX 60
#define DIG_TEMP_ROOM_MIN 1
#define DIG_TEMP_ROOM_MAX 60
#define DIG_HUM_ROOM_MIN 1
#define DIG_HUM_ROOM_MAX 100

#if (HW_NUM != 6)
#define HEATER_CONSUMPTION_MIN 1.5
#define FAN_CONSUMPTION_MIN 0.03
#define PHOTOTHERAPY_CONSUMPTION_MIN 0.3
#define HUMIDIFIER_CONSUMPTION_MIN 0.07

#define HEATER_CONSUMPTION_MAX 20
#define FAN_CONSUMPTION_MAX 0.8
#define PHOTOTHERAPY_CONSUMPTION_MAX 3
#define HUMIDIFIER_CONSUMPTION_MAX 0.8

#define STANDBY_CONSUMPTION_MIN 0
#define STANDBY_CONSUMPTION_MAX 1

#define SCREEN_CONSUMPTION_MIN 0.005
#define SCREEN_CONSUMPTION_MAX 1

#define BUZZER_CONSUMPTION_MIN 0
#else
#define HEATER_CONSUMPTION_MIN 0
#define FAN_CONSUMPTION_MIN 0
#define PHOTOTHERAPY_CONSUMPTION_MIN 0
#define HUMIDIFIER_CONSUMPTION_MIN 0

#define HEATER_CONSUMPTION_MAX 10000
#define FAN_CONSUMPTION_MAX 10000
#define PHOTOTHERAPY_CONSUMPTION_MAX 10000
#define HUMIDIFIER_CONSUMPTION_MAX 10000

#define STANDBY_CONSUMPTION_MIN 0
#define STANDBY_CONSUMPTION_MAX 10

#define SCREEN_CONSUMPTION_MIN 0
#define SCREEN_CONSUMPTION_MAX 10

#define BUZZER_CONSUMPTION_MIN 0
#endif

long HW_error = false;
long lastTFTCheck;

extern in3ator_parameters in3;
TCA9555 TCA(0x20);

void initDebug() {
  Serial.begin(115200);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  log("in3ator debug uart, version v" + String(FWversion) + "/" +
      String(HWversion) + ", SN: " + String(in3.serialNumber));
}

void initPWMGPIO() {
  log("[HW] -> Initialiting PWM GPIOs");
  ledcSetup(SCREENBACKLIGHT_PWM_CHANNEL, DEFAULT_PWM_FREQUENCY,
            DEFAULT_PWM_RESOLUTION);
  ledcSetup(HEATER_PWM_CHANNEL, DEFAULT_PWM_FREQUENCY, DEFAULT_PWM_RESOLUTION);
  ledcSetup(BUZZER_PWM_CHANNEL, DEFAULT_PWM_FREQUENCY, DEFAULT_PWM_RESOLUTION);
  ledcAttachPin(SCREENBACKLIGHT, SCREENBACKLIGHT_PWM_CHANNEL);
  ledcAttachPin(HEATER, HEATER_PWM_CHANNEL);
  ledcAttachPin(BUZZER, BUZZER_PWM_CHANNEL);
  ledcWrite(SCREENBACKLIGHT_PWM_CHANNEL, false);
  ledcWrite(HEATER_PWM_CHANNEL, false);
  ledcWrite(BUZZER_PWM_CHANNEL, false);
#if (HW_NUM == 8)
  ledcSetup(HUMIDIFIER_PWM_CHANNEL, HUMIDIFIER_PWM_FREQUENCY,
            DEFAULT_PWM_RESOLUTION);
  ledcAttachPin(HUMIDIFIER_CTL, HUMIDIFIER_PWM_CHANNEL);
  ledcWrite(HUMIDIFIER_CTL, false);
#endif
  log("[HW] -> PWM GPIOs initialized");
}

void initGPIO() {
  log("[HW] -> Initializing GPIOs");
#if (HW_NUM == 6)
  TCA.begin();
  for (int pin = 0; pin < 16; pin++) {
    TCA.setPolarity(pin, false);
  }
  initPin(UNUSED_GPIO_EXP0, OUTPUT);
  initPin(UNUSED_GPIO_EXP1, OUTPUT);
  initPin(UNUSED_GPIO_EXP2, OUTPUT);
  initPin(UNUSED_GPIO_EXP3, OUTPUT);
  GPIOWrite(UNUSED_GPIO_EXP0, HIGH);
  GPIOWrite(UNUSED_GPIO_EXP1, HIGH);
  GPIOWrite(UNUSED_GPIO_EXP2, HIGH);
  GPIOWrite(UNUSED_GPIO_EXP3, HIGH);
  initPin(GPRS_EN, OUTPUT);
  GPIOWrite(GPRS_EN, HIGH);
  initPin(HUMIDIFIER_CTL, OUTPUT);
  GPIOWrite(HUMIDIFIER_CTL, LOW);
#elif (HW_NUM == 8)
  initPin(HUMIDIFIER_PWM, OUTPUT);
#endif
#if (HW_NUM >= 10)
  initPin(ON_OFF_SWITCH, INPUT);
  initPin(FAN_SPEED_FEEDBACK, INPUT_PULLUP);
#endif
  initPin(PHOTOTHERAPY, OUTPUT);
  initPin(GPRS_PWRKEY, OUTPUT);
  initPin(encoderpinA, INPUT_PULLUP);
  initPin(encoderpinB, INPUT_PULLUP);
  initPin(ENC_SWITCH, INPUT_PULLUP);
  initPin(TFT_CS, OUTPUT);
  initPin(PHOTOTHERAPY, OUTPUT);
  GPIOWrite(PHOTOTHERAPY, LOW);
  initPin(FAN, OUTPUT);
  initPin(HEATER, OUTPUT);
  initPin(BUZZER, OUTPUT);
  initPin(SCREENBACKLIGHT, OUTPUT);
  initPin(ACTUATORS_EN, OUTPUT);
  GPIOWrite(ACTUATORS_EN, HIGH);
  GPIOWrite(PHOTOTHERAPY, LOW);
  GPIOWrite(FAN, LOW);
  // initPin(ON_OFF_SWITCH, INPUT);
  initPWMGPIO();
  log("[HW] -> GPIOs initilialized");
}

void initInterrupts() {
  attachInterrupt(ENC_SWITCH, encSwitchHandler, CHANGE);
  attachInterrupt(ENC_A, encoderISR, CHANGE);
  attachInterrupt(ENC_B, encoderISR, CHANGE);

#if (HW_NUM >= 10)
  attachInterrupt(FAN_SPEED_FEEDBACK, fanEncoderISR, CHANGE);
  attachInterrupt(ON_OFF_SWITCH, ON_OFF_Switch_ISR, FALLING);
#endif
}

void initRoomSensor() {
  roomSensorPresent = false;
  wire->beginTransmission(ROOM_SENSOR_I2C_ADDRESS);
  roomSensorPresent = !(wire->endTransmission());
  if (roomSensorPresent == true) {
    log("[HW] -> Room sensor succesfully found, initializing...");
    mySHTC3.begin(Wire);
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
  }
}

void initAmbientSensor() {
  ambientSensorPresent = false;
  wire->beginTransmission(AMBIENT_SENSOR_I2C_ADDRESS);
  ambientSensorPresent = !(wire->endTransmission());
  if (ambientSensorPresent == true) {
    log("[HW] -> Ambient sensor succesfully found, initializing...");
    sht4.begin(&Wire);
  }
}
void initCurrentSensor(bool currentSensor) {
  if (currentSensor == MAIN) {
    log("[HW] -> Initialiting MAIN current sensor");
    wire->beginTransmission(MAIN_DIGITAL_CURRENT_SENSOR_I2C_ADDRESS);
  } else {
    log("[HW] -> Initialiting SECUNDARY current sensor");
    wire->beginTransmission(SECUNDARY_DIGITAL_CURRENT_SENSOR_I2C_ADDRESS);
  }
  if (!(wire->endTransmission())) {
    digitalCurrentSensorPresent[currentSensor] = true;
    log("[HW] ->digital sensor detected");
    if (currentSensor == MAIN) {
      mainDigitalCurrentSensor.begin();
      mainDigitalCurrentSensor.reset();
      // Set shunt resistors to 10 mOhm for all channels
      mainDigitalCurrentSensor.setShuntRes(SYSTEM_SHUNT, PHOTOTHERAPY_SHUNT,
                                           FAN_SHUNT);
      mainDigitalCurrentSensor.setShuntConversionTime(
          INA3221_REG_CONF_CT_140US);
      mainDigitalCurrentSensor.setAveragingMode(INA3221_REG_CONF_AVG_128);
    } else {
      digitalCurrentSensorPresent[currentSensor] = true;
      secundaryDigitalCurrentSensor.begin();
      secundaryDigitalCurrentSensor.reset();
      // Set shunt resistors to 10 mOhm for all channels
      secundaryDigitalCurrentSensor.setShuntRes(HEATER_SHUNT, USB_SHUNT,
                                                BATTERY_SHUNT);
      secundaryDigitalCurrentSensor.setShuntConversionTime(
          INA3221_REG_CONF_CT_140US);
      secundaryDigitalCurrentSensor.setAveragingMode(INA3221_REG_CONF_AVG_128);
    }
  } else {
    log("[HW] -> no digital sensor detected");
  }
}

void initPowerAlarm() {}

void initI2C() {
  log("[HW] -> Initializing i2c port");
  Wire.begin(I2C_SDA, I2C_SCL);
  wire = &Wire;
  log("[HW] -> I2c port initialized");
}

void addErrorToVar(long &errorVar, int error) { errorVar |= (1 << error); }

void initSensors() {
  long error = HW_error;
  log("[HW] -> Initialiting sensors");
  initCurrentSensor(MAIN);
  initCurrentSensor(SECUNDARY);
  initRoomSensor();
  initAmbientSensor();
  // sensors verification
  for (int i = 0; i <= NTC_SAMPLES_TEST; i++) {
    while (!measureNTCTemperature())
      ;
  }
  if (in3.temperature[SKIN_SENSOR] < NTC_BABY_MIN) {
    log("[HW] -> Fail -> NTC temperature is lower than expected");
    addErrorToVar(HW_error, NTC_BABY_MIN_ERROR);
  }
  if (in3.temperature[SKIN_SENSOR] > NTC_BABY_MAX) {
    log("[HW] -> Fail -> NTC temperature is higher than expected");
    addErrorToVar(HW_error, NTC_BABY_MAX_ERROR);
  }
  if (updateRoomSensor()) {
    if (in3.temperature[ROOM_DIGITAL_TEMP_SENSOR] < DIG_TEMP_ROOM_MIN) {
      log("[HW] -> Fail -> Room temperature is lower than expected");
      addErrorToVar(HW_error, DIG_TEMP_ROOM_MIN_ERROR);
    }
    if (in3.temperature[ROOM_DIGITAL_TEMP_SENSOR] > DIG_TEMP_ROOM_MAX) {
      log("[HW] -> Fail -> Room temperature is higher than expected");
      addErrorToVar(HW_error, DIG_TEMP_ROOM_MAX_ERROR);
    }
    if (in3.humidity[ROOM_DIGITAL_HUM_SENSOR] < DIG_HUM_ROOM_MIN) {
      log("[HW] -> Fail -> Room humidity is lower than expected");
      addErrorToVar(HW_error, DIG_HUM_ROOM_MIN_ERROR);
    }
    if (in3.humidity[ROOM_DIGITAL_HUM_SENSOR] > DIG_HUM_ROOM_MAX) {
      log("[HW] -> Fail -> Room humidity is higher than expected");
      addErrorToVar(HW_error, DIG_HUM_ROOM_MAX_ERROR);
    }
  } else {
    addErrorToVar(HW_error, DIGITAL_SENSOR_NOTFOUND);
    log("[HW] -> Fail -> No room sensor found");
  }
  if (error == HW_error) {
    log("[HW] -> OK -> Sensors are working as expected");
  }
}

void standByCurrentTest() {
  long error = HW_error;
  float testCurrent;
  log("[HW] -> Measuring standby current...");

  testCurrent = measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL);
  if (testCurrent < STANDBY_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, DEFECTIVE_CURRENT_SENSOR);
    log("[HW] -> Fail -> Defective current sensor");
  }
  if (testCurrent > STANDBY_CONSUMPTION_MAX) {
    addErrorToVar(HW_error, STANDBY_CONSUMPTION_MAX_ERROR);
    log("[HW] -> Fail -> Maximum stanby current exceeded");
  }
  if (error == HW_error) {
    log("[HW] -> OK -> Current sensor is working as expected: " +
        String(testCurrent) + " Amps");
  } else {
    log("[HW] -> Fail -> test current is " + String(testCurrent) + " Amps");
  }
  in3.system_current_standby_test = testCurrent;
}

void initSenseCircuit() { standByCurrentTest(); }

void initializeTFT() {
  tft.setController(DISPLAY_CONTROLLER_IC);
  tft.begin(DISPLAY_SPI_CLK);
  tft.setRotation(DISPLAY_DEFAULT_ROTATION);
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x");
  Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x");
  Serial.println(x, HEX);
}

void initTFT() {
  long error = HW_error;
  float testCurrent, offsetCurrent;
  int backlight_start_value, backlight_end_value;
  offsetCurrent = measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL);
#if (HW_NUM == 6)
  initPin(TOUCH_CS, OUTPUT);
  initPin(SD_CS, OUTPUT);
  initPin(TFT_RST, OUTPUT);
  initPin(TFT_CS_EXP, OUTPUT);
  initPin(TFT_DC, OUTPUT);
  GPIOWrite(TOUCH_CS, HIGH);
  GPIOWrite(SD_CS, HIGH);
  GPIOWrite(TFT_CS_EXP, LOW);
  GPIOWrite(TFT_RST, LOW);  // alternating HIGH/LOW
  delay(5);
  GPIOWrite(TFT_RST, HIGH);  // alternating HIGH/LOW
  delay(5);
#endif
  initializeTFT();
  loadlogo();
  if (BACKLIGHT_CONTROL == DIRECT_BACKLIGHT_CONTROL) {
    backlight_start_value = false;
    backlight_end_value = BACKLIGHT_POWER_DEFAULT;
  } else {
    backlight_start_value = BACKLIGHT_POWER_DEFAULT;
    backlight_end_value = false;
  }
  for (int i = backlight_start_value; i < backlight_end_value; i++) {
    ledcWrite(SCREENBACKLIGHT_PWM_CHANNEL, i);
    vTaskDelay(BACKLIGHT_DELAY / portTICK_PERIOD_MS);
    if (BACKLIGHT_CONTROL == INVERTED_BACKLIGHT_CONTROL) {
      i -= 2;
    }
  }
  vTaskDelay(INIT_TFT_DELAY / portTICK_PERIOD_MS);
  testCurrent =
      measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL) - offsetCurrent;
  if (testCurrent < SCREEN_CONSUMPTION_MIN) {
    // addErrorToVar(HW_error, DEFECTIVE_SCREEN;
    log("[HW] -> WARNING -> Screen current is not high enough");
  }
  if (testCurrent > SCREEN_CONSUMPTION_MAX) {
    // addErrorToVar(HW_error, DEFECTIVE_SCREEN;
    log("[HW] -> WARNING -> Screen current exceeded");
  }
  if (error == HW_error) {
    log("[HW] -> OK -> Screen is working as expected: " + String(testCurrent) +
        " Amps");
  } else {
    log("[HW] -> Fail -> test current is " + String(testCurrent) + " Amps");
  }
  in3.display_current_test = testCurrent;
}

void initBuzzer() {
  long error = HW_error;
  float testCurrent, offsetCurrent;

  offsetCurrent = measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL);
  ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_HALF_PWM);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  testCurrent =
      measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL) - offsetCurrent;
  ledcWrite(BUZZER_PWM_CHANNEL, false);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  if (testCurrent < BUZZER_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, DEFECTIVE_BUZZER);
    log("[HW] -> Fail -> Buzzer current is not high enough");
  }
  if (error == HW_error) {
    log("[HW] -> OK -> Buzzer is working as expected: " + String(testCurrent) +
        " Amps");
  } else {
    log("[HW] -> Fail -> test current is " + String(testCurrent) + " Amps");
  }
  in3.buzzer_current_test = testCurrent;
}

bool actuatorsTest() {
  long error = HW_error;
  log("[HW] -> Checking actuators...");

  float testCurrent, offsetCurrent;
  offsetCurrent = measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL);
  ledcWrite(HEATER_PWM_CHANNEL, PWM_MAX_VALUE);
  vTaskDelay(CURRENT_STABILIZE_TIME_HEATER / portTICK_PERIOD_MS);
  testCurrent =
      measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL) - offsetCurrent;
  log("[HW] -> Heater current consumption: " + String(testCurrent) + " Amps");
  in3.heater_current_test = testCurrent;
  ledcWrite(HEATER_PWM_CHANNEL, 0);
  if (testCurrent < HEATER_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, HEATER_CONSUMPTION_MIN_ERROR);
    log("[HW] -> Fail -> Heater current consumption is too low");
    setAlarm(HEATER_ISSUE_ALARM);
    return (true);
  }
  if (testCurrent > HEATER_CONSUMPTION_MAX) {
    addErrorToVar(HW_error, HEATER_CONSUMPTION_MAX_ERROR);
    log("[HW] -> Fail -> Heater current consumption is too high");
    setAlarm(HEATER_ISSUE_ALARM);
    return (true);
  }
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  offsetCurrent = measureMeanConsumption(MAIN, PHOTOTHERAPY_SHUNT_CHANNEL);
  GPIOWrite(PHOTOTHERAPY, HIGH);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  testCurrent =
      measureMeanConsumption(MAIN, PHOTOTHERAPY_SHUNT_CHANNEL) - offsetCurrent;
  GPIOWrite(PHOTOTHERAPY, LOW);
  log("[HW] -> Phototherapy current consumption: " + String(testCurrent) +
      " Amps");
  in3.phototherapy_current_test = testCurrent;
  if (testCurrent < PHOTOTHERAPY_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, PHOTOTHERAPY_CONSUMPTION_MIN_ERROR);
    log("[HW] -> Fail -> PHOTOTHERAPY current consumption is too low");
  }
  if (testCurrent > PHOTOTHERAPY_CONSUMPTION_MAX) {
    addErrorToVar(HW_error, PHOTOTHERAPY_CONSUMPTION_MAX_ERROR);
    log("[HW] -> Fail -> PHOTOTHERAPY current consumption is too high");
    return (true);
  }
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  offsetCurrent = measureMeanConsumption(
      MAIN, SYSTEM_SHUNT_CHANNEL);  // <- UPDATE THIS CODE TO ASK I2C DATA
  in3_hum.turn(ON);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  testCurrent = measureMeanConsumption(MAIN, SYSTEM_SHUNT_CHANNEL) -
                offsetCurrent;  // <- UPDATE THIS CODE TO ASK I2C DATA
  log("[HW] -> Humidifier current consumption: " + String(testCurrent) +
      " Amps");
  in3.humidifier_current_test = testCurrent;
  in3_hum.turn(OFF);
  if (testCurrent < HUMIDIFIER_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, HUMIDIFIER_CONSUMPTION_MIN_ERROR);
    log("[HW] -> Fail -> HUMIDIFIER current consumption is too low");
  }
  if (testCurrent > HUMIDIFIER_CONSUMPTION_MAX) {
    addErrorToVar(HW_error, HUMIDIFIER_CONSUMPTION_MAX_ERROR);
    log("[HW] -> Fail -> HUMIDIFIER current consumption is too high");
    return (true);
  }
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  offsetCurrent = measureMeanConsumption(MAIN, FAN_SHUNT_CHANNEL);
  GPIOWrite(FAN, HIGH);
  vTaskDelay(CURRENT_STABILIZE_TIME_DEFAULT / portTICK_PERIOD_MS);
  testCurrent = measureMeanConsumption(MAIN, FAN_SHUNT_CHANNEL) - offsetCurrent;
  log("[HW] -> FAN consumption: " + String(testCurrent) + " Amps");
  in3.fan_current_test = testCurrent;
  GPIOWrite(FAN, LOW);
  if (testCurrent < FAN_CONSUMPTION_MIN) {
    addErrorToVar(HW_error, FAN_CONSUMPTION_MIN_ERROR);
    log("[HW] -> Fail -> Fan current consumption is too low");
    return (true);
  }
  if (testCurrent > FAN_CONSUMPTION_MAX &&
      testCurrent > FAN_MAX_CURRENT_OVERRIDE * FAN_CONSUMPTION_MAX * 2) {
    addErrorToVar(HW_error, FAN_CONSUMPTION_MAX_ERROR);
    log("[HW] -> Fail -> Fan current consumption is too high");
    return (true);
  }
  if (error == HW_error) {
    log("[HW] -> OK -> Actuators are working as expected");
  } else {
    log("[HW] -> Fail -> Some actuators are not working as expected");
  }
  return (false);
}

bool initActuators() {
#if (HW_NUM <= 6)
  in3_hum.begin(HUMIDIFIER_BINARY, HUMIDIFIER_CTL);
#elif (HW_NUM <= 8)
  in3_hum.begin(HUMIDIFIER_PWM, HUMIDIFIER_CTL);
#else
  in3_hum.begin();
#endif
  return (actuatorsTest());
}

void initPin(uint8_t GPIO, uint8_t Mode) {
  if (GPIO < GPIO_EXP_BASE) {
    pinMode(GPIO, Mode);
  } else {
    TCA.pinMode(GPIO - GPIO_EXP_BASE, Mode);
  }
}

void GPIOWrite(uint8_t GPIO, uint8_t Mode) {
  if (GPIO < GPIO_EXP_BASE) {
    digitalWrite(GPIO, Mode);
  } else {
    log("[HW] -> TCA9355 writing pin" + String(GPIO - GPIO_EXP_BASE) + " -> " +
        String(Mode));
    if (!TCA.digitalWrite(GPIO - GPIO_EXP_BASE, Mode)) {
      log("[HW] -> TCA9355 WRITE ERROR");
    }
  }
}

bool GPIORead(uint8_t GPIO) {
  if (GPIO < GPIO_EXP_BASE) {
    return (digitalRead(GPIO));
  } else {
    return (TCA.digitalRead(GPIO - GPIO_EXP_BASE));
  }
}

void initHardware(bool printOutputTest) {
  initDebug();
  // brownOutConfig(false);
  initEEPROM();
  initI2C();
  initGPIO();
  initSensors();
  log("[HW] -> Initialiting hardware");
  initSenseCircuit();
  initTFT();
  initPowerAlarm();
  initBuzzer();
  initInterrupts();
  PIDInit();
  in3.HW_critical_error = initActuators();
  if (WIFI_EN) {
    wifiInit();
  }
  if (!HW_error) {
    log("[HW] -> HARDWARE OK");
  } else {
    log("[HW] -> HARDWARE TEST FAIL");
    log("[HW] -> HARDWARE ERROR CODE:" + String(HW_error, HEX));
  }
  in3.HW_test_error_code = HW_error;
  if (printOutputTest || in3.HW_critical_error || in3.calibrationError) {
    log("[HW] -> PRINTING ERROR TO USER");
    drawHardwareErrorMessage(HW_error, in3.HW_critical_error,
                             in3.calibrationError);
    while (GPIORead(ENC_SWITCH)) {
      updateData();
    }
  }
  buzzerTone(2, buzzerStandbyToneDuration, buzzerStandbyTone);
  watchdogInit(WDT_TIMEOUT);
  initAlarms();
  GPIOWrite(ACTUATORS_EN, LOW);
}