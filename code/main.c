#include "main.h"
#include "cmsis_os.h"

UART_HandleTypeDef huart1;
uint8_t rx_data;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

osThreadId servocloseHandle;
osThreadId pickup1Handle;
osThreadId pickup2Handle;
osThreadId conveyorHandle;
osThreadId pickdownHandle;
osThreadId servoopenHandle;

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	volatile uint32_t step_count;
	volatile uint32_t target_steps;
} StepperMotor;

StepperMotor pickup;
StepperMotor lead;
StepperMotor con;
StepperMotor dig;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);

void Servo_close(void const * argument);
void Pickup1(void const * argument);
void Pickup2(void const * argument);
void Conveyor(void const * argument);
void Pickdown(void const * argument);
void Servo_open(void const * argument);

void Servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void Stepper_Move(StepperMotor *motor, uint32_t steps);

volatile int m = 0;
volatile int i = 0;
int pick_steps = 400;
int lead_steps = 3200;
int con_steps = 300;
int dig_steps = 1000;

#define PICKUP_DIR PICK_DIR_Pin
#define LEAD_DIR LEAD_DIR_Pin
#define CON_DIR CON_DIR_Pin
#define DIG_DIR DIG_DIR_Pin

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();

MX_USART1_UART_Init();

/* Start UART interrupt */
HAL_UART_Receive_IT(&huart1, &rx_data, 1);


  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  /* Create the thread(s) */
  /* definition and creation of servoclose */
  osThreadDef(servoclose, Servo_close, osPriorityNormal, 0, 128);
  servocloseHandle = osThreadCreate(osThread(servoclose), NULL);

  osThreadDef(pickup1, Pickup1, osPriorityNormal, 0, 128);
  pickup1Handle = osThreadCreate(osThread(pickup1), NULL);

  osThreadDef(pickup2, Pickup2, osPriorityNormal, 0, 128);
  pickup2Handle = osThreadCreate(osThread(pickup2), NULL);

  osThreadDef(conveyor, Conveyor, osPriorityNormal, 0, 350);
  conveyorHandle = osThreadCreate(osThread(conveyor), NULL);

  osThreadDef(pickdown, Pickdown, osPriorityNormal, 0, 128);
  pickdownHandle = osThreadCreate(osThread(pickdown), NULL);

  osThreadDef(servoopen, Servo_open, osPriorityNormal, 0, 128);
  servoopenHandle = osThreadCreate(osThread(servoopen), NULL);

  pickup.htim = &htim5;
  pickup.channel = TIM_CHANNEL_1;
  pickup.target_steps = 0;

  dig.htim = &htim2;
  dig.channel = TIM_CHANNEL_1;
  dig.target_steps = 0;

  lead.htim = &htim3;
  lead.channel = TIM_CHANNEL_1;
  lead.target_steps = 0;

  con.htim = &htim4;
  con.channel = TIM_CHANNEL_1;
  con.target_steps = 0;

  osSignalSet(servocloseHandle, 0x01);
  osKernelStart();

  while (1)
  {
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  HAL_UART_Init(&huart1);
}


void Servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
    uint32_t pulse_length = 210 + (angle * (1050 - 210) / 180);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}

void Stepper_Move(StepperMotor *motor, uint32_t steps)
{
    if(motor->target_steps == 0)   // Only start if idle
    {
        motor->target_steps = steps;
        motor->step_count = 0;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        if(dig.target_steps > 0)
        {
            dig.step_count++;

            if(dig.step_count >= dig.target_steps)
            {
            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                dig.target_steps = 0;
            }
        }
    }
    if(htim->Instance == TIM3)
    {
        if(lead.target_steps > 0)
        {
            lead.step_count++;

            if(lead.step_count >= lead.target_steps)
            {
            	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                lead.target_steps = 0;
            }
        }
    }
    if(htim->Instance == TIM4)
    {
        if(con.target_steps > 0)
        {
            con.step_count++;

            if(con.step_count >= con.target_steps)
            {
            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
                con.target_steps = 0;
            }
        }
    }
    if(htim->Instance == TIM5)
    {
        if(pickup.target_steps > 0)
        {
            pickup.step_count++;

            if(pickup.step_count >= pickup.target_steps)
            {
            	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                pickup.target_steps = 0;
            }
        }
    }
}

void Servo_close(void const * argument)
{
  for(;;)
  {
	  osSignalWait(0x01, osWaitForever);
	  Servo_angle(&htim8, TIM_CHANNEL_1, 90);
	  osDelay(500);
	  osSignalSet(pickdownHandle, 0x01);
  }
}

void Pickdown(void const * argument)
{
  for(;;)
  {
	osSignalWait(0x01, osWaitForever);
	m++;
    HAL_GPIO_WritePin(GPIOA, PICKUP_DIR, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, DIG_DIR, GPIO_PIN_RESET);
    osDelay(2);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 100);

    Stepper_Move(&pickup, pick_steps);
    Stepper_Move(&dig, dig_steps);

    while(pickup.target_steps != 0 || dig.target_steps != 0)
    {
        osDelay(1);
    }
	osSignalSet(servoopenHandle, 0x01);
  }
}

void Servo_open(void const * argument)
{
  for(;;)
  {
	osSignalWait(0x01, osWaitForever);
	Servo_angle(&htim8, TIM_CHANNEL_1, 0);
	osDelay(500);
	i++;
	if(i==8 || i==16)
	{
	    osSignalSet(conveyorHandle,0x01);

	    if(i==16)
	        i=0;
	}
	else if(i<8)
	{
	    osSignalSet(pickup1Handle,0x01);
	}
	else
	{
	    osSignalSet(pickup2Handle,0x01);
	}
	osDelay(1);
  }
}

void Pickup1(void const * argument)
{
  for(;;)
  {
    osSignalWait(0x01, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, PICKUP_DIR, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, DIG_DIR, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, LEAD_DIR, GPIO_PIN_SET);
    osDelay(2);

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);

    Stepper_Move(&pickup, pick_steps);
    Stepper_Move(&dig, dig_steps);
    Stepper_Move(&lead, lead_steps);
    while(pickup.target_steps != 0 || dig.target_steps != 0 || lead.target_steps != 0)
    {
        osDelay(1);
    }
	osSignalSet(servocloseHandle, 0x01);
  }
}

void Conveyor(void const * argument)
{
  for(;;)
  {
    osSignalWait(0x01, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, PICKUP_DIR, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, DIG_DIR, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, CON_DIR, GPIO_PIN_RESET);
    osDelay(2);

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);

    Stepper_Move(&pickup, pick_steps);
    Stepper_Move(&dig, dig_steps);
    Stepper_Move(&con, con_steps);

    while(pickup.target_steps != 0 || dig.target_steps != 0 || con.target_steps != 0)
    {
        osDelay(1);
    }
	osSignalSet(servocloseHandle, 0x01);
  }
}

void Pickup2(void const * argument)
{
  for(;;)
  {
	osSignalWait(0x01, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, PICKUP_DIR, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, DIG_DIR, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, LEAD_DIR, GPIO_PIN_RESET);
    osDelay(2);

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);

    Stepper_Move(&pickup,pick_steps);
    Stepper_Move(&dig, dig_steps);
    Stepper_Move(&lead, lead_steps);

    while(pickup.target_steps != 0 || dig.target_steps != 0 || lead.target_steps != 0)
    {
        osDelay(1);
    }
	osSignalSet(servocloseHandle, 0x01);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    if(rx_data == 'A')    // 7 kHz
    {
              // ARR = 142 → 7 kHz
      __HAL_TIM_SET_AUTORELOAD(&htim2, 2399);
      __HAL_TIM_SET_AUTORELOAD(&htim3, 2399);
      __HAL_TIM_SET_AUTORELOAD(&htim4, 2399);
      __HAL_TIM_SET_AUTORELOAD(&htim5, 2399);

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1200);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1200);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1200);
         }

    else if(rx_data == 'B')   // 5 kHz (default)
    {
           // ARR = 199 already set → no change needed
           __HAL_TIM_SET_AUTORELOAD(&htim2, 199);
           __HAL_TIM_SET_AUTORELOAD(&htim3, 199);
           __HAL_TIM_SET_AUTORELOAD(&htim4, 199);
           __HAL_TIM_SET_AUTORELOAD(&htim5, 199);

         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
         __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 100);

    }

    else if(rx_data == 'C')   // 9 kHz
    {
      // ARR = 110 → 9 kHz
      __HAL_TIM_SET_AUTORELOAD(&htim2, 1199);
      __HAL_TIM_SET_AUTORELOAD(&htim3, 1199);
      __HAL_TIM_SET_AUTORELOAD(&htim4, 1199);
      __HAL_TIM_SET_AUTORELOAD(&htim5, 1199);

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 600);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 600);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 600);
    }

    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);

    /* Reset counters (VERY IMPORTANT for smooth change) */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    /* Restart UART interrupt */
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig 	= {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 199;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PICK_DIR_Pin|DIG_DIR_Pin|LEAD_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CON_DIR_GPIO_Port, CON_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PICK_DIR_Pin DIG_DIR_Pin LEAD_DIR_Pin */
  GPIO_InitStruct.Pin = PICK_DIR_Pin|DIG_DIR_Pin|LEAD_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CON_DIR_Pin */
  GPIO_InitStruct.Pin = CON_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CON_DIR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
