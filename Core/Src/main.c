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
	uint16_t	TRIG_PIN;
	uint16_t	ECHO_PIN;
}HC_SR_04_Config;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Pin Definitions for Port B
#define IN1_LF_PIN GPIO_PIN_0
#define IN2_LF_PIN GPIO_PIN_1
#define IN3_LB_PIN GPIO_PIN_10
#define IN4_LB_PIN GPIO_PIN_11

#define IN1_RF_PIN GPIO_PIN_12
#define IN2_RF_PIN GPIO_PIN_13
#define IN3_RB_PIN GPIO_PIN_14
#define IN4_RB_PIN GPIO_PIN_15

#define	Left_Point GPIO_PIN_4
#define	LM_Point GPIO_PIN_5
#define	Middle_Point GPIO_PIN_6
#define	RM_Point GPIO_PIN_7
#define	Right_Point GPIO_PIN_8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

// Khởi tạo Struct cho Driver bên Trái (LF & LB)
L298_Config L298_Left = {
    .Port_IN   = GPIOB,
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
	.Port_IN = GPIOA,
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
		.ECHO_PIN = GPIO_PIN_8,
};


volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t  Is_First_Captured = 0;
volatile float    Distance = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void L298_SetSpeed(L298_Config *dev, int16_t speed_A, int16_t speed_B);
void L298_Stop(L298_Config *dev);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Cài đặt tốc độ và hướng cho 1 cụm L298
  * speed: -99 đến 99 (âm là lùi, dương là tiến)
  */
void L298_Drive(L298_Config *dev, int16_t speed_A, int16_t speed_B) {
    // Điều khiển Motor A (LF hoặc RF)
    if (speed_A > 0) {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN2, GPIO_PIN_RESET);
    } else if (speed_A < 0) {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN2, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN2, GPIO_PIN_RESET);
    }

    // Điều khiển Motor B (LB hoặc RB)
    if (speed_B > 0) {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN4, GPIO_PIN_RESET);
    } else if (speed_B < 0) {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN4, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->Port_IN, dev->Pin_IN4, GPIO_PIN_RESET);
    }

    // Cập nhật PWM (giá trị tuyệt đối)
    uint32_t duty_A = (speed_A < 0) ? -speed_A : speed_A;
    uint32_t duty_B = (speed_B < 0) ? -speed_B : speed_B;

    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel_A, duty_A);
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel_B, duty_B);
}

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
  /* USER CODE BEGIN 2 */
  // Start PWM cho tất cả các kênh trên TIM2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LineFollower_Logic();
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

/**
 * @brief Đọc khoảng cách từ cảm biến HC-SR04 (Tính bằng cm)
 */
float HCSR04_Read(void) {
    uint32_t local_time = 0;

    // 1. Tạo xung Trigger 10us
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_SET);
    HAL_Delay(0); // Delay cực nhỏ hoặc dùng timer delay
    for(int i=0; i<450; i++); // Ước lượng ~10us cho F103 72MHz
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_RESET);

    // 2. Chờ Echo lên mức HIGH (Bắt đầu đo)
    while (HAL_GPIO_ReadPin(HC_SR_04_.Port_IN, HC_SR_04_.ECHO_PIN) == GPIO_PIN_RESET);

    // 3. Đếm thời gian Echo ở mức HIGH
    while (HAL_GPIO_ReadPin(HC_SR_04_.Port_IN, HC_SR_04_.ECHO_PIN) == GPIO_PIN_SET) {
        local_time++;
        if (local_time > 30000) break; // Timeout để tránh treo
    }

    // 4. Tính toán khoảng cách (Hệ số thực nghiệm cho F103)
    return (float)local_time * 0.017;
}

void HCSR04_Trigger(void) {
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_SET);
    // Delay 10us chuẩn dùng NOP để tránh compiler tối ưu xóa mất
    for(int i=0; i<500; i++) __NOP();
    HAL_GPIO_WritePin(HC_SR_04_.Port_IN, HC_SR_04_.TRIG_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) // Kiểm tra đúng Timer 4
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Đúng Channel 3
        {
            if (Is_First_Captured == 0) // Nếu là cạnh lên (Bắt đầu xung Echo)
            {
                IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // Đọc giá trị Timer lần 1
                Is_First_Captured = 1;  // Đặt cờ đã xong lần 1

                // Đổi cấu hình sang bắt cạnh xuống (Falling edge) cho lần tiếp theo
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else if (Is_First_Captured == 1) // Nếu là cạnh xuống (Kết thúc xung Echo)
            {
                IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // Đọc giá trị Timer lần 2
                __HAL_TIM_SET_COUNTER(htim, 0); // Reset counter về 0 để chuẩn bị cho lần đo sau

                // Tính toán độ lệch thời gian
                if (IC_Val2 > IC_Val1) {
                    Difference = IC_Val2 - IC_Val1;
                } else {
                    Difference = (0xFFFF - IC_Val1) + IC_Val2; // Xử lý khi tràn Timer (nếu có)
                }

                // Với Prescaler = 71, mỗi tick = 1us.
                // Công thức: Khoảng cách = (Thời gian * Vận tốc âm thanh) / 2
                // Distance = (Difference * 0.0343) / 2 = Difference * 0.01715
                Distance = (float)Difference * 0.01715;

                Is_First_Captured = 0; // Reset cờ

                // Đổi cấu hình về lại bắt cạnh lên (Rising edge)
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

                // Tạm dừng ngắt để tránh nhiễu, khi nào cần đo mới gọi lại trong Trigger
                __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
            }
        }
    }
}

/**
 * @brief Logic dò line kết hợp tránh vật cản
 */
void LineFollower_Logic(void) {
    // Đọc trạng thái 5 mắt (1 là thấy line đen, 0 là trắng)
    uint8_t s1 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN1); // Trái ngoài
    uint8_t s2 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN2); // Trái trong
    uint8_t s3 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN3); // Giữa
    uint8_t s4 = HAL_GPIO_ReadPin(TCRT
    		5000.Port_IN, TCRT5000.Pin_IN4); // Phải trong
    uint8_t s5 = HAL_GPIO_ReadPin(TCRT5000.Port_IN, TCRT5000.Pin_IN5); // Phải ngoài

    // 1. Check vật cản trước
    if (HCSR04_Read() < 15.0) { // Nếu vật cản < 15cm
        L298_Drive(&L298_Left, 0, 0);  // Dừng xe
        L298_Drive(&L298_Right, 0, 0);
        return;
    }

    // 2. Logic dò line cơ bản (Simple IF-ELSE)
    if (s3 == 1 && s2 == 0 && s4 == 0) {
        // Tiến thẳng
        L298_Drive(&L298_Left, 40, 40);
        L298_Drive(&L298_Right, 40, 40);
    }
    else if (s2 == 1 || s1 == 1) {
        // Lệch trái -> Rẽ trái (Bánh trái chậm, bánh phải nhanh)
        L298_Drive(&L298_Left, 10, 10);
        L298_Drive(&L298_Right, 50, 50);
    }
    else if (s4 == 1 || s5 == 1) {
        // Lệch phải -> Rẽ phải
        L298_Drive(&L298_Left, 50, 50);
        L298_Drive(&L298_Right, 10, 10);
    }
    else {
        // Mất line -> Dừng hoặc quay lại tìm
        L298_Drive(&L298_Left, 20, 20);
        L298_Drive(&L298_Right, 20, 20);
    }
}
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
