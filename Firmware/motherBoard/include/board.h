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

#define HW_NUM 14
#define HW_REVISION 'A'
#define HWversion String(HW_NUM) + "." + String(HW_REVISION)
#define FWversion "14.2"
#define WIFI_NAME "In3ator"
#define CURRENT_FIRMWARE_TITLE "in3ator"

#define DEFAULT_WIFI_EN ON

#if (HW_NUM <= 8)
#define DISPLAY_SPI_CLK SPI_CLOCK_DIV16
#elif (HW_NUM == 9)
#define DISPLAY_SPI_CLK SPI_CLOCK_DIV16
#elif (HW_NUM >= 10)
#define DISPLAY_SPI_CLK SPI_CLOCK_DIV16
#endif

#if (HW_NUM <= 8)
#define ANALOG_TO_AMP_FACTOR 0.2
#define CURRENT_MEASURES_AMOUNT 20
#endif
#if (HW_NUM <= 6)
#define HUMIDIFIER_INTERFACE HUMIDIFIER_BINARY
#elif (HW_NUM <= 8)
#define HUMIDIFIER_INTERFACE HUMIDIFIER_PWM
#else
// Hardware
#define HUMIDIFIER_INTERFACE HUMIDIFIER_I2C
#endif

#define GPIO_EXP_BASE 100 // To differentiate with ESP32 GPIO
#if (HW_NUM > 14)
// PINOUT
#define TOUCH_SENSOR 1
#define TOUCH_SENSOR_SEL 2
#define BUZZER 5
#define LED 7
#define I2C_SDA 8
#define I2C_SCL 9
#define BABY_NTC_PIN 10
#define FAN 12
#define PHOTOTHERAPY 13
#define ACTUATORS_EN 14
#define HEATER 16
#define AFE44XX_CS 21
#define FAN_SPEED_FEEDBACK 38
#define AFE4490_ADC_READY 45
#define SERIAL2_RX_PIN 48
#define SERIAL2_TX_PIN 47

#define FAKE_PIN 46

#define SCREENBACKLIGHT FAKE_PIN
#define ON_OFF_SWITCH FAKE_PIN
#define AFE44XX_PWDN_PIN FAKE_PIN
#define GPRS_PWRKEY FAKE_PIN
#define TFT_DC FAKE_PIN
#define ENC_SWITCH FAKE_PIN
#define ENC_A FAKE_PIN
#define ENC_B FAKE_PIN
#define TFT_CS FAKE_PIN

#elif (HW_NUM == 14)
// PINOUT
#define TFT_DC 0
#define AFE44XX_CS 2
#define ENC_SWITCH 4
#define BUZZER 5
#define FAN 12
#define PHOTOTHERAPY 13
#define ACTUATORS_EN 14
#define TFT_CS 15
#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 25
#define HEATER 27
#define TOUCH_SENSOR_SEL 26
#define TOUCH_SENSOR 32
#define SCREENBACKLIGHT 33
#define ENC_B 34
#define FAN_SPEED_FEEDBACK 35
#define BABY_NTC_PIN 39
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
#define AFE4490_ADC_READY NULL
#define ON_OFF_SWITCH NULL
#define AFE44XX_PWDN_PIN NULL
#define GPRS_PWRKEY NULL

#elif (HW_NUM >= 13)
// PINOUT
// #define TFT_DC 0
#define AFE44XX_CS 2
#define ENC_SWITCH 4
#define BUZZER 5
#define FAN 12
#define PHOTOTHERAPY 13
#define ACTUATORS_EN 14
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
// #define TFT_CS 15
#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 25
#define GPRS_PWRKEY 26
#define HEATER 27
#define SCREENBACKLIGHT 33
#define ENC_B 32
#define FAN_SPEED_FEEDBACK 35
#define BABY_NTC_PIN 39
#define AFE4490_ADC_READY NULL
#define ON_OFF_SWITCH NULL
#define AFE44XX_PWDN_PIN NULL

// #define DISPLAY_CONTROLLER_IC ST7789V_CONTROLLER

#elif (HW_NUM >= 9)
// PINOUT
#define TFT_DC 0
#define AFE44XX_CS 2
#define ENC_SWITCH 4
#define BUZZER 5
#define FAN 12
#define PHOTOTHERAPY 13
#define ACTUATORS_EN 14
#define TFT_CS 15
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 25
#define GPRS_PWRKEY 26
#define HEATER 27
#define ENC_B 32
#define SCREENBACKLIGHT 33
#define ON_OFF_SWITCH 34
#define FAN_SPEED_FEEDBACK 35
#define AFE4490_ADC_READY 36
#define BABY_NTC_PIN 39
#define AFE44XX_PWDN_PIN NULL

// #define DISPLAY_CONTROLLER_IC ST7789V_CONTROLLER
#elif (HW_NUM == 8)
#define TFT_DC 0
#define ENC_SWITCH 4
#define BUZZER 5
#define FAN 12
#define PHOTOTHERAPY 13
#define HUMIDIFIER_CTL 14
#define TFT_CS 15
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 25
#define GPRS_PWRKEY 26
#define HEATER 27
#define ENC_B 32
#define SCREENBACKLIGHT 33
#define ACTUATORS_EN 34 // fake pin
#define SYSTEM_CURRENT_SENSOR 36
#define BABY_NTC_PIN 39

#define DISPLAY_CONTROLLER_IC ST7789V_CONTROLLER

#else
// Hardware
// PINOUT

#define AFE44XX_CS 2
#define TFT_DC 4
#define BUZZER 5
#define TFT_CS 15 // fake GPIO
#define SERIAL2_RX_PIN 16
#define SERIAL2_TX_PIN 17
#define I2C_SDA 21
#define I2C_SCL 22
#define ENC_A 25
#define ENC_SWITCH 26
#define HEATER 27
#define ENC_B 32
#define SCREENBACKLIGHT 33
#define BABY_NTC_PIN 34
#define AIR_NTC_PIN 35
#define SYSTEM_CURRENT_SENSOR 36
// #define SYSTEM_SHUNT 36

#define GPIO_EXP_0 0 + GPIO_EXP_BASE
#define GPIO_EXP_1 1 + GPIO_EXP_BASE
#define GPIO_EXP_2 2 + GPIO_EXP_BASE
#define GPIO_EXP_3 3 + GPIO_EXP_BASE
#define GPIO_EXP_4 4 + GPIO_EXP_BASE
#define GPIO_EXP_5 5 + GPIO_EXP_BASE
#define GPIO_EXP_6 6 + GPIO_EXP_BASE
#define GPIO_EXP_7 7 + GPIO_EXP_BASE
#define GPIO_EXP_8 8 + GPIO_EXP_BASE
#define GPIO_EXP_9 9 + GPIO_EXP_BASE
#define GPIO_EXP_10 10 + GPIO_EXP_BASE
#define GPIO_EXP_11 11 + GPIO_EXP_BASE
#define GPIO_EXP_12 12 + GPIO_EXP_BASE
#define GPIO_EXP_13 13 + GPIO_EXP_BASE
#define GPIO_EXP_14 14 + GPIO_EXP_BASE
#define GPIO_EXP_15 15 + GPIO_EXP_BASE

#define UNUSED_GPIO_EXP0 GPIO_EXP_5
#define UNUSED_GPIO_EXP1 GPIO_EXP_12
#define UNUSED_GPIO_EXP2 GPIO_EXP_14
#define UNUSED_GPIO_EXP3 GPIO_EXP_15

#define ACTUATORS_EN GPIO_EXP_0
#define GPRS_EN GPIO_EXP_1
#define SD_CS GPIO_EXP_2
#define FAN GPIO_EXP_3
#define HUMIDIFIER_CTL GPIO_EXP_6
#define PHOTOTHERAPY GPIO_EXP_7
#define GPRS_PWRKEY GPIO_EXP_8
#define TFT_CS_EXP GPIO_EXP_9
#define TOUCH_IRQ GPIO_EXP_10
#define TOUCH_CS GPIO_EXP_11
#define TFT_RST GPIO_EXP_13

#define DISPLAY_CONTROLLER_IC ILI9341_CONTROLLER

#endif

#if ARDUINO_USB_MODE == 1
#define debugSerial Serial0
#else
#define debugSerial Serial
#endif

// number assignment of each environmental sensor for later call in variable
#define SKIN_SENSOR 0
#define NTC_QTY 1 // number of NTC
#define ROOM_DIGITAL_TEMP_SENSOR 1
#define AMBIENT_DIGITAL_TEMP_SENSOR 2
#define SENSOR_TEMP_QTY 3 // number of total temperature sensors in system
#define ROOM_DIGITAL_HUM_SENSOR 0
#define AMBIENT_DIGITAL_HUM_SENSOR 1
#define SENSOR_HUM_QTY 2 // number of total humidity sensors in system

#define SYSTEM_SHUNT_CHANNEL INA3221_CH1
#define PHOTOTHERAPY_SHUNT_CHANNEL INA3221_CH2
#define FAN_SHUNT_CHANNEL INA3221_CH3

#define HEATER_SHUNT_CHANNEL INA3221_CH1
#define USB_SHUNT_CHANNEL INA3221_CH2
#define BATTERY_SHUNT_CHANNEL INA3221_CH3

#define HUMIDIFIER_SHUNT 1

#define SDCard false
#define SYSTEM_SHUNT 2 // miliohms
#define FAN_SHUNT 100  // miliohms
#if (HW_NUM >= 12)
#define PHOTOTHERAPY_SHUNT 82 // miliohms
#else
#define PHOTOTHERAPY_SHUNT 20 // miliohms
#endif
#define BATTERY_SHUNT 27000 // miliohms
#define USB_SHUNT 100       // miliohms
#define HEATER_SHUNT 2      // miliohms

#define DISPLAY_DEFAULT_ROTATION 3

#define SCREENBACKLIGHT_PWM_CHANNEL 0
#define BUZZER_PWM_CHANNEL 1
#define HEATER_PWM_CHANNEL 2
#define FAN_PWM_CHANNEL 3
#define PHOTOTHERAPY_PWM_CHANNEL 4

#define HUMIDIFIER_PWM_CHANNEL 4
#define DEFAULT_PWM_RESOLUTION 8
#define LOW_PWM_FREQUENCY 32
#define DEFAULT_PWM_FREQUENCY 2000
#define HUMIDIFIER_PWM_FREQUENCY 109000

#define maxADCvalue 4095
#define maxDACvalue 4095
// #define PWM_MAX_VALUE maxADCvalue
#define PWM_MAX_VALUE (pow(2, DEFAULT_PWM_RESOLUTION) - 1)
#define FAN_PWM PWM_MAX_VALUE

#if (ADC_READ_FUNCTION == MILLIVOTSREAD_ADC)
#define ADC_TO_DISCARD_MIN 500  // in mV
#define ADC_TO_DISCARD_MAX 2500 // in mV
#else
#define ADC_TO_DISCARD_MIN maxADCvalue / 5     // in ADC points
#define ADC_TO_DISCARD_MAX maxADCvalue * 4 / 5 // in ADC points
#endif

#define DIG_TEMP_TO_DISCARD_MAX 60
#define DIG_TEMP_TO_DISCARD_MIN 5

#define BL_NORMAL 0
#define BL_POWERSAVE 1

#define HEATER_MAX_PWM PWM_MAX_VALUE
#define HEATER_HALF_PWR PWM_MAX_VALUE / 2
#define HEATER_START_PWM 5

#define BUZZER_MAX_PWM PWM_MAX_VALUE
#define BUZZER_HALF_PWM PWM_MAX_VALUE / 2

#define DIRECT_BACKLIGHT_CONTROL true
#define INVERTED_BACKLIGHT_CONTROL false

#define MIN_SYSTEM_VOLTAGE_TRIGGER 0
#define MAX_SYSTEM_VOLTAGE_TRIGGER 8

#if (HW_NUM <= 8 || (HW_NUM == 9 && HW_REVISION == 'A'))
#define SCREEN_BRIGHTNESS_FACTOR                                               \
  0.1 // Max brightness will be multiplied by this constant
#define BACKLIGHT_POWER_SAFE_PERCENTAGE 0.6
#define BACKLIGHT_CONTROL INVERTED_BACKLIGHT_CONTROL
#else
#define SCREEN_BRIGHTNESS_FACTOR                                               \
  0.7 // Max brightness will be multiplied by this constant
#define BACKLIGHT_POWER_SAFE_PERCENTAGE 0.3
#define BACKLIGHT_CONTROL DIRECT_BACKLIGHT_CONTROL
#endif

#define BACKLIGHT_POWER_SAFE PWM_MAX_VALUE *BACKLIGHT_POWER_SAFE_PERCENTAGE
#define BACKLIGHT_POWER_DEFAULT PWM_MAX_VALUE *SCREEN_BRIGHTNESS_FACTOR
