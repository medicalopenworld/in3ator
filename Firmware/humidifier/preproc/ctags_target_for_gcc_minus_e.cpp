# 1 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
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
# 19 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
# 20 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino" 2
TwoWire Wire2((((((((((0xC0 + 1) + 1) + 1) + 1) + 1) + 1) + 1) + 1) + 1), ((((((((0xC0 + 1) + 1) + 1) + 1) + 1) + 1) + 1) + 1));
//HardwareSerial Serial2(PA3, PA2);





TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
bool humidifierWorkingSignal;
bool humidifierWorkingMode;
long lastHighPulse;

void setup()
{
  initHAL();
  pinMode(((0xC0 + 1) + 1), 0x2);
  pinMode((((0xC0 + 1) + 1) + 1), 0x2);
  pinMode(17, 0x1);
  pinMode(((((((((0xC0 + 1) + 1) + 1) + 1) + 1) + 1) + 1) + 1), 0x0);
  digitalWrite(17, 0x1);
  humidifierWorkingSignal = digitalRead((((0xC0 + 1) + 1) + 1));
  humidifierWorkingMode = digitalRead(((0xC0 + 1) + 1));
  if (!humidifierWorkingMode) {
    HAL_TIM_PWM_Start(&htim3, 0x00000008U /*!< Capture/compare channel 3 identifier      */);
    digitalWrite(17, 0x0);
    while (1);
  }
  if (humidifierWorkingSignal) {
    Wire2.begin(2); // join i2c bus with address #4
    Wire2.onRequest(requestEvent); // register event
    Wire2.onReceive(receiveEvent); // register event+
    digitalWrite(17, 0x0);
    delay(100);
    digitalWrite(17, 0x1);
    delay(100);
  }
  else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(17, 0x0);
      delay(100);
      digitalWrite(17, 0x1);
      delay(100);
    }
  }
  /*

    Serial2.begin(115200);           // start serial for output

    Serial2.println("I2C TEST");

  */
# 70 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\slave_sender_receiver.ino"
}

void loop()
{
  if (!humidifierWorkingSignal) {
    if (digitalRead(((((((((0xC0 + 1) + 1) + 1) + 1) + 1) + 1) + 1) + 1))) {
      lastHighPulse = millis();
      HAL_TIM_PWM_Start(&htim3, 0x00000008U /*!< Capture/compare channel 3 identifier      */);
      digitalWrite(17, 0x0);
    }
    else if (millis() - lastHighPulse > 10) {
      HAL_TIM_PWM_Stop(&htim3, 0x00000008U /*!< Capture/compare channel 3 identifier      */);
      digitalWrite(17, 0x1);
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
    char c = Wire2.read(); // receive byte as a character
    //Serial2.print(c);          // print the character
  }
  int x = Wire2.read(); // receive byte as an integer
  //Serial2.println(x);          // print the integer
  if (x == 0) {
    HAL_TIM_PWM_Stop(&htim3, 0x00000008U /*!< Capture/compare channel 3 identifier      */);
    digitalWrite(17, 0x1);
  }
  if (x == 1) {
    HAL_TIM_PWM_Start(&htim3, 0x00000008U /*!< Capture/compare channel 3 identifier      */);
    digitalWrite(17, 0x0);
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
# 1 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
void initHAL() {
  HAL_Init();
  SystemClock_Config();
  do { volatile /*!< Defines 'read / write' permissions */ uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */ + 0x00020000UL) + 0x00001000UL))->IOPENR) |= ((0x1UL << (1U)) /*!< 0x00000002 */)); /* Delay after an RCC peripheral clock enabling */ tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */ + 0x00020000UL) + 0x00001000UL))->IOPENR) & ((0x1UL << (1U)) /*!< 0x00000002 */)); (void)tmpreg /* To avoid gcc/g++ warnings */; } while(0U);
  MX_TIM1_Init();
  MX_TIM3_Init();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (htim->Instance == ((TIM_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */) + 0x00000400UL)))
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */

    /* USER CODE END TIM3_MspPostInit 0 */

    do { volatile /*!< Defines 'read / write' permissions */ uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */ + 0x00020000UL) + 0x00001000UL))->IOPENR) |= ((0x1UL << (1U)) /*!< 0x00000002 */)); /* Delay after an RCC peripheral clock enabling */ tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */ + 0x00020000UL) + 0x00001000UL))->IOPENR) & ((0x1UL << (1U)) /*!< 0x00000002 */)); (void)tmpreg /* To avoid gcc/g++ warnings */; } while(0U);
    /**TIM3 GPIO Configuration

      PB0     ------> TIM3_CH3

    */
# 22 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
    GPIO_InitStruct.Pin = ((uint16_t)0x0001) /* Pin 0 selected    */;
    GPIO_InitStruct.Mode = ((0x2uL << 0u) | (0x0uL << 4u)) /*!< Alternate Function Push Pull Mode                                  */;
    GPIO_InitStruct.Pull = 0x00000000u /*!< No Pull-up or Pull-down activation  */;
    GPIO_InitStruct.Speed = 0x00000000u /*!< Low speed       */;
    GPIO_InitStruct.Alternate = ((uint8_t)0x01) /*!< TIM3 Alternate Function mapping */;
    HAL_GPIO_Init(((GPIO_TypeDef *) ((0x50000000UL) /*!< IOPORT base address */ + 0x00000400UL)), &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */

    /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage

  */
# 42 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
  HAL_PWREx_ControlVoltageScaling((0x1UL << (9U)) /*!< Voltage scaling bit 0 */ /*!< Voltage scaling range 1 */);

  /** Initializes the RCC Oscillators according to the specified parameters

     in the RCC_OscInitTypeDef structure.

  */
# 47 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
  RCC_OscInitStruct.OscillatorType = 0x00000002U /*!< HSI to configure */;
  RCC_OscInitStruct.HSIState = (0x1UL << (8U)) /*!< 0x00000100 */ /*!< Internal High Speed clock enable */ /*!< HSI clock activation */;
  RCC_OscInitStruct.HSIDiv = 0x00000000U /*!< HSI clock is not divided */;
  RCC_OscInitStruct.HSICalibrationValue = 64U /*!< Default HSI calibration trimming value */;
  RCC_OscInitStruct.PLL.PLLState = 0x00000000U /*!< PLL configuration unchanged */;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1) { };
  }

  /** Initializes the CPU, AHB and APB buses clocks

  */
# 58 "C:\\Users\\Administrator\\Desktop\\slave_sender_receiver\\HAL.ino"
  RCC_ClkInitStruct.ClockType = 0x00000002U /*!< HCLK to configure */ | 0x00000001U /*!< SYSCLK to configure */
                                | 0x00000004U /*!< PCLK1 to configure */;
  RCC_ClkInitStruct.SYSCLKSource = 0x00000000U /*!< HSI selection as system clock */;
  RCC_ClkInitStruct.AHBCLKDivider = 0x00000000U /*!< SYSCLK not divided */;
  RCC_ClkInitStruct.APB1CLKDivider = 0x00000000U /*!< HCLK not divided */;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 0x00000000UL /*!< FLASH Zero wait state */) != HAL_OK) {
    while (1) { };
  }
}

static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = ((TIM_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */) + 0x00012C00UL));
  htim1.Init.Prescaler = 255;
  htim1.Init.CounterMode = 0x00000000U /*!< Counter used as up-counter   */;
  htim1.Init.Period = 62499;
  htim1.Init.ClockDivision = 0x00000000U /*!< Clock division: tDTS=tCK_INT   */;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = 0x00000000U /*!< TIMx_ARR register is not buffered */;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    while (1) { };
  }
  sClockSourceConfig.ClockSource = (0x1UL << (12U)) /*!< 0x00001000 */ /*!< Internal clock source                                 */;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    while (1) { };
  }
  sMasterConfig.MasterOutputTrigger = 0x00000000U /*!< TIMx_EGR.UG bit is used as trigger output (TRGO)              */;
  sMasterConfig.MasterOutputTrigger2 = 0x00000000U /*!< TIMx_EGR.UG bit is used as trigger output (TRGO2)              */;
  sMasterConfig.MasterSlaveMode = 0x00000000U /*!< Master/slave mode is selected */;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
      != HAL_OK) {
    while (1) { };
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
  htim3.Instance = ((TIM_TypeDef *) (((0x40000000UL) /*!< Peripheral base address */) + 0x00000400UL));
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = 0x00000000U /*!< Counter used as up-counter   */;
  htim3.Init.Period = 145;
  htim3.Init.ClockDivision = 0x00000000U /*!< Clock division: tDTS=tCK_INT   */;
  htim3.Init.AutoReloadPreload = 0x00000000U /*!< TIMx_ARR register is not buffered */;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    while (1) { };
  }
  sClockSourceConfig.ClockSource = (0x1UL << (12U)) /*!< 0x00001000 */ /*!< Internal clock source                                 */;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    while (1) { };
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    while (1) { };
  }
  sMasterConfig.MasterOutputTrigger = 0x00000000U /*!< TIMx_EGR.UG bit is used as trigger output (TRGO)              */;
  sMasterConfig.MasterSlaveMode = 0x00000000U /*!< Master/slave mode is selected */;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
      != HAL_OK) {
    while (1) { };
  }
  sConfigOC.OCMode = ((0x0004UL << (4U)) /*!< 0x00000040 */ | (0x0002UL << (4U)) /*!< 0x00000020 */) /*!< PWM mode 1                             */;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = 0x00000000U /*!< Capture/Compare output polarity  */;
  sConfigOC.OCFastMode = 0x00000000U /*!< Output Compare fast disable */;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, 0x00000008U /*!< Capture/compare channel 3 identifier      */)
      != HAL_OK) {
    while (1) { };
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}
