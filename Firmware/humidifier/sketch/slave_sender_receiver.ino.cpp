#include <Arduino.h>
#line 1 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
/* Wire Slave Receiver
  by Wi6Labs

  Demonstrates use of the Wire library.
  Receives/sends data as an I2C/TWI slave device.
  Refer to the "Wire Master Reader Writer" example for use with this.

  Created 27 June 2017
  Updated 14 August 2017
  - this example is now common to all STM32 boards

  This example code is in the public domain.
*/

#define MIN_TIME_LOW_SIGNAL 10
#define SDA PA12
#define SCL PA11

#include <Wire.h>
TwoWire Wire2(SDA, SCL);
//HardwareSerial Serial2(PA3, PA2);

#define LED PB3
#define WORKING_MODE PA2
#define WORKING_INPUT_SIGNAL PA3
#define I2C_ADDR  2
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
bool humidifierWorkingSignal;
bool humidifierWorkingMode;
long lastHighPulse;

#line 33 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
void setup();
#line 72 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
void loop();
#line 89 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
void receiveEvent(int howMany);
#line 111 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
void requestEvent();
#line 1 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void initHAL();
#line 9 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
#line 36 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void SystemClock_Config(void);
#line 69 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
static void MX_TIM1_Init(void);
#line 108 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void MX_TIM3_Init(void);
#line 33 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
void setup()
{
  initHAL();
  pinMode(WORKING_MODE, INPUT_PULLUP);
  pinMode(WORKING_INPUT_SIGNAL, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(SCL, INPUT);
  digitalWrite(LED, HIGH);
  humidifierWorkingSignal = digitalRead(WORKING_INPUT_SIGNAL);
  humidifierWorkingMode = digitalRead(WORKING_MODE);
  if (!humidifierWorkingMode) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    digitalWrite(LED, LOW);
    while (1);
  }
  if (humidifierWorkingSignal) {
    Wire2.begin(I2C_ADDR);         // join i2c bus with address #4
    Wire2.onRequest(requestEvent); // register event
    Wire2.onReceive(receiveEvent); // register event+
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
  }
  else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED, LOW);
      delay(100);
      digitalWrite(LED, HIGH);
      delay(100);
    }
  }
  /*
    Serial2.begin(115200);           // start serial for output
    Serial2.println("I2C TEST");
  */

}

void loop()
{
  if (!humidifierWorkingSignal) {
    if (digitalRead(SCL)) {
      lastHighPulse = millis();
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
      digitalWrite(LED, LOW);
    }
    else if (millis() - lastHighPulse > MIN_TIME_LOW_SIGNAL) {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      digitalWrite(LED, HIGH);
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  //Serial2.println("received event");
  while (1 < Wire2.available()) // loop through all but the last
  {
    char c = Wire2.read();     // receive byte as a character
    //Serial2.print(c);          // print the character
  }
  int x = Wire2.read();        // receive byte as an integer
  //Serial2.println(x);          // print the integer
  if (x == 0) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    digitalWrite(LED, HIGH);
  }
  if (x == 1) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    digitalWrite(LED, LOW);
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  //Wire2.write("hello\n");  // respond with message of 6 bytes
  // as expected by master
  //Serial2.println("requested Message");
}

#line 1 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void initHAL() {
  HAL_Init();
  SystemClock_Config();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  MX_TIM1_Init();
  MX_TIM3_Init();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (htim->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */

    /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
      PB0     ------> TIM3_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */

    /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
     in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 255;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 62499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
      != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };
  TIM_OC_InitTypeDef sConfigOC = { 0 };

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 145;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
      != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
      != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

