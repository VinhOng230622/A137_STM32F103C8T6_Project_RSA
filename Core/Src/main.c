/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Project: 4WD Robot Car with STM32 & L298N
// Structure:
// (LF_Motor) \           / (RF_Motor)
//             \         /
//              \_______/
//              /       \
//             /         \
// (LB_Motor) /           \ (RB_Motor)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    // Port và Pin cho các chân hướng (GPIO)
    GPIO_TypeDef* Port_IN;
    uint16_t      Pin_IN1;
    uint16_t      Pin_IN2;
    uint16_t      Pin_IN3;
    uint16_t      Pin_IN4;

    // Timer và Channel cho xung PWM (ENA, ENB)
    TIM_HandleTypeDef* htim;
    uint32_t           Channel_A;
    uint32_t           Channel_B;

    uint32_t           Max_Duty;  // Giá trị ARR (Period)
} L298_Config;

typedef struct {
    GPIO_TypeDef* Port_IN;
    uint16_t      Pin_IN1;
    uint16_t      Pin_IN2;
    uint16_t      Pin_IN3;
    uint16_t      Pin_IN4;
    uint16_t      Pin_IN5;

}TCRT_5000_Config;

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t Channel;
} ECHO_PIN;

typedef struct {
    GPIO_TypeDef* Port_IN;
    uint16_t TRIG_PIN;
    ECHO_PIN echo;
} HC_SR_04_Config;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Temp_ECHO_PORT GPIOB         //temporary value for copy from Youtube
#define Temp_ECHO_PIN GPIO_PIN_6

// Pin Definitions for Port A
#define IN1_LF_PIN GPIO_PIN_4
#define IN2_LF_PIN GPIO_PIN_5
#define IN3_LB_PIN GPIO_PIN_6
#define IN4_LB_PIN GPIO_PIN_7

// Pin Definitions for Port B
#define IN1_RF_PIN GPIO_PIN_0
#define IN2_RF_PIN GPIO_PIN_1
#define IN3_RB_PIN GPIO_PIN_10
#define IN4_RB_PIN GPIO_PIN_11


#define	Left_Point GPIO_PIN_3
#define	LM_Point GPIO_PIN_15
#define	Middle_Point GPIO_PIN_14
#define	RM_Point GPIO_PIN_13
#define	Right_Point GPIO_PIN_12


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char buff[100];

void mPrint(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buff, sizeof(buff), format, args);

    va_end(args);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 100);
    }
}


// Khởi tạo Struct cho Driver bên Trái (LF & LB)
L298_Config L298_Left = {
    .Port_IN   = GPIOA,
    .Pin_IN1   = IN1_LF_PIN,
    .Pin_IN2   = IN2_LF_PIN,
    .Pin_IN3   = IN3_LB_PIN,
    .Pin_IN4   = IN4_LB_PIN,
    .htim      = &htim2,
    .Channel_A = TIM_CHANNEL_1, // ENA
    .Channel_B = TIM_CHANNEL_2, // ENB
    .Max_Duty  = 99             // Khớp với ARR trong MX_TIM2_Init
};

// Khởi tạo Struct cho Driver bên Phải (RF & RB)
L298_Config L298_Right = {
    .Port_IN   = GPIOB,
    .Pin_IN1   = IN1_RF_PIN,
    .Pin_IN2   = IN2_RF_PIN,
    .Pin_IN3   = IN3_RB_PIN,
    .Pin_IN4   = IN4_RB_PIN,
    .htim      = &htim2,
    .Channel_A = TIM_CHANNEL_3,
    .Channel_B = TIM_CHANNEL_4,
    .Max_Duty  = 99
};

// Khởi tạo Struct cho TCRT5000
TCRT_5000_Config TCRT5000 = {
	.Port_IN = GPIOB,
	.Pin_IN1	=	Left_Point,
	.Pin_IN2	=	LM_Point,
	.Pin_IN3	=	Middle_Point,
	.Pin_IN4	=	RM_Point,
	.Pin_IN5	=	Right_Point,
};

// Khởi tạo Struct cho Sensor HC-SR-04
HC_SR_04_Config HC_SR_04_ = {
    .Port_IN = GPIOB,
    .TRIG_PIN = GPIO_PIN_7,
    .echo = {
        .htim = &htim4,
        .Channel = TIM_CHANNEL_3
    }
};


volatile uint32_t Value1 = 0;
volatile uint32_t Value2 = 0;
volatile uint32_t Difference = 0;
volatile uint32_t pMillis = 0;
volatile uint8_t  Is_First_Captured = 0;
volatile float    Distance = 0;
uint8_t s1; // Trái ngoài
uint8_t s2;// Trái trong
uint8_t s3; // Giữa
uint8_t s4;
uint8_t s5;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//void L298_SetSpeed(L298_Config *dev, int16_t speed_A, int16_t speed_B);
//void L298_Stop(L298_Config *dev);
//void LineFollower_Logic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PWM_MAX 99

void SetOneMotor(GPIO_TypeDef* port, uint16_t pin1, uint16_t pin2,
                 TIM_HandleTypeDef *htim, uint32_t channel,
                 int speed)
{
    if(speed > PWM_MAX) speed = PWM_MAX;
    if(speed < -PWM_MAX) speed = -PWM_MAX;

    if(speed > 0)
    {
        HAL_GPIO_WritePin(port, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port, pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(htim, channel, speed);
    }
    else if(speed < 0)
    {
        HAL_GPIO_WritePin(port, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(htim, channel, -speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
    }
}


// code điều chỉnh cả thẩy 4 động cơ
void SetMotor4(int fl, int lb, int fr, int rb)
{
    // ===== LEFT SIDE =====
    SetOneMotor(L298_Left.Port_IN, L298_Left.Pin_IN1, L298_Left.Pin_IN2,L298_Left.htim, L298_Left.Channel_A, fl); // Front Left
    SetOneMotor(L298_Left.Port_IN, L298_Left.Pin_IN3, L298_Left.Pin_IN4,L298_Left.htim, L298_Left.Channel_B, lb); // Back Left
    // ===== RIGHT SIDE =====
    SetOneMotor(L298_Right.Port_IN, L298_Right.Pin_IN1, L298_Right.Pin_IN2,L298_Right.htim, L298_Right.Channel_A, fr); // Front Right
    SetOneMotor(L298_Right.Port_IN, L298_Right.Pin_IN3, L298_Right.Pin_IN4,L298_Right.htim, L298_Right.Channel_B, rb); // Back Right
}

uint8_t s1;
uint8_t s2;
uint8_t s3;
uint8_t s4;
uint8_t s5;

int error = 0;

int Read_Line_Error(void)
{
    s1 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN1); // L
    s2 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN2); // LM
    s3 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN3); // M
    s4 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN4); // RM
    s5 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN5); // R

    // ===== Nếu logic bị ngược thì mở dòng này =====
    s1 = !s1; s2 = !s2; s3 = !s3; s4 = !s4; s5 = !s5;
if(s1 == s2 && s2 == s3 && s3 == s4 && s4 == s5 )
{
 int error = -1;
 return error;
}
else{
    int error = 0;
    int count = 0;

    if(s1) { error += -2; count++; }
    if(s2) { error += -1; count++; }
    if(s3) { error +=  0; count++; }
    if(s4) { error +=  1; count++; }
    if(s5) { error +=  2; count++; }

    static int last_error = 0;

    if(count != 0)
    {
        error = error / count;
        last_error = error;
    }
    else
    {
        // Mất line → quay theo hướng cũ
        if(last_error < 0) error = -3;
        else error = 3;
    }

    return error;}
}

void SetMotor(int left, int right)
{
    SetMotor4(left, left, right, right);
}



void Navigate(void)
{

	int error = Read_Line_Error();

	if (error == -1)
	{
		SetMotor(0,0);
	}
	else
	{
    int base = 80;   // <= PWM_MAX (99)
    int Kp = 20;

    int left  = base + Kp * error;
    int right = base - Kp * error;

    SetMotor(left, right);}
}




/**
  * @brief Cài đặt tốc độ và hướng cho 1 cụm L298
  * speed: -99 đến 99 (âm là lùi, dương là tiến)
  */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM cho tất cả các kênh trên TIM2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim2);


  // Start PWM cho tất cả các kênh trên TIM4
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  LineFollower_Logic();
//	  HAL_Delay(5);
//	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);

    /* USER CODE END WHILE */
	  Navigate();
	  HAL_Delay(1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

/**
 *
 *
 *
 * @brief Đọc khoảng cách từ cảm biến HC-SR04 (Tính bằng cm)
 */
float HCSR04_Read(void) {
    uint32_t local_time = 0;
    uint32_t timeout = 0;

    // 1. Trigger 10us
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_SET);
    for(int i = 0; i < 720; i++) __NOP();  // ~10us @72MHz
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_RESET);

    // 2. Chờ ECHO lên HIGH
    timeout = 0;
    while (HAL_GPIO_ReadPin(HC_SR_04_.Port_IN, GPIO_PIN_8) == GPIO_PIN_RESET) {
        if (timeout++ > 30000) return -1; // Không nhận được echo
    }

    // 3. Đếm thời gian ECHO HIGH
    timeout = 0;
    while (HAL_GPIO_ReadPin(HC_SR_04_.Port_IN, GPIO_PIN_8) == GPIO_PIN_SET) {
        local_time++;
        if (timeout++ > 30000) break; // tránh treo
    }

    // 4. Tính khoảng cách
    return (float)local_time * 0.01715f;
}

void HCSR04_Trigger(void) {
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_SET);
    // Delay 10us chuẩn dùng NOP để tránh compiler tối ưu xóa mất
    for(int i=0; i<500; i++) __NOP();
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_RESET);
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM4) // Kiểm tra đúng Timer 4
//    {
//        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Đúng Channel 3
//        {
//            if (Is_First_Captured == 0) // Nếu là cạnh lên (Bắt đầu xung Echo)
//            {
//                IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // Đọc giá trị Timer lần 1
//                Is_First_Captured = 1;  // Đặt cờ đã xong lần 1
//
//                // Đổi cấu hình sang bắt cạnh xuống (Falling edge) cho lần tiếp theo
//                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
//            }
//            else if (Is_First_Captured == 1) // Nếu là cạnh xuống (Kết thúc xung Echo)
//            {
//                IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // Đọc giá trị Timer lần 2
//                __HAL_TIM_SET_COUNTER(htim, 0); // Reset counter về 0 để chuẩn bị cho lần đo sau
//
//                // Tính toán độ lệch thời gian
//                if (IC_Val2 > IC_Val1) {
//                    Difference = IC_Val2 - IC_Val1;
//                } else {
//                    Difference = (0xFFFF - IC_Val1) + IC_Val2; // Xử lý khi tràn Timer (nếu có)
//                }
//
//                // Với Prescaler = 71, mỗi tick = 1us.
//                // Công thức: Khoảng cách = (Thời gian * Vận tốc âm thanh) / 2
//                // Distance = (Difference * 0.0343) / 2 = Difference * 0.01715
//                Distance = (float)Difference * 0.01715;
//
//                Is_First_Captured = 0; // Reset cờ
//
//                // Đổi cấu hình về lại bắt cạnh lên (Rising edge)
//                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
//
//                // Tạm dừng ngắt để tránh nhiễu, khi nào cần đo mới gọi lại trong Trigger
//                __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
//            }
//        }
//    }
//}

/**
 * @brief Logic dò line kết hợp tránh vật cản
 */
//void LineFollower_Logic(void) {
//    // Đọc trạng thái 5 mắt (1 là thấy line đen, 0 là trắng)
//    uint8_t s1 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN1); // Trái ngoài
//    uint8_t s2 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN2); // Trái trong
//    uint8_t s3 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN3); // Giữa
//    uint8_t s4 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN4); // Phải trong
//    uint8_t s5 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN5); // Phải ngoài
//
//    // 1. Check vật cản trước
//    float d = HCSR04_Read();
//    HAL_Delay(60);
//
//    if (d < 15.0 && d > 0) { // Nếu vật cản < 15cm
//        L298_Drive(&L298_Left, 0, 0);  // Dừng xe
//        L298_Drive(&L298_Right, 0, 0);
//        return;
//    }
//
//    // 2. Logic dò line cơ bản (Simple IF-ELSE)
//    if (s3 == 1 && s2 == 0 && s4 == 0) {
//        // Tiến thẳng
//        L298_Drive(&L298_Left, 40, 40);
//        L298_Drive(&L298_Right, 40, 40);
//    }
//    else if (s2 == 1 || s1 == 1) {
//        // Lệch trái -> Rẽ trái (Bánh trái chậm, bánh phải nhanh)
//        L298_Drive(&L298_Left, 10, 10);
//        L298_Drive(&L298_Right, 50, 50);
//    }
//    else if (s4 == 1 || s5 == 1) {
//        // Lệch phải -> Rẽ phải
//        L298_Drive(&L298_Left, 50, 50);
//        L298_Drive(&L298_Right, 10, 10);
//    }
//    else {
//        // Mất line -> Dừng hoặc quay lại tìm
//        L298_Drive(&L298_Left, 20, 20);
//        L298_Drive(&L298_Right, 20, 20);
//    }
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
