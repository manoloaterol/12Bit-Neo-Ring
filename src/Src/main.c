/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_BRIGHTNESS 255  // Max led brightness
#define SAMPLES 20          // ADC samples
#define AVERAGE_SAMPLES 5   // Number of saples to compute the average

// OP-AMP rail aperture. 
// Values from 0.1 to 1
// Less values, more sesitivity
#define RAIL_APERTURE 0.7   

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */

uint8_t mode = 0;      // visualization mode
uint32_t lastTick = 0; // used to avoid button bounce effect

// Max ADC value
uint16_t max_value = 2047 * RAIL_APERTURE;

// specific to control led effects
uint16_t lastValues[AVERAGE_SAMPLES] = {0};
uint8_t lastValueCount = 0;
uint8_t globalHue = 0;
uint8_t hueOffset = 120;

// led rotation
uint8_t rotation_delay = 20;
uint8_t rotation_count = 0;
uint8_t rotation_step = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void addLastValue(uint16_t value);
uint16_t getAverage();

void mode_0(uint16_t value);
void mode_1(uint16_t value);
void mode_2(uint16_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_DMA_Init();
    MX_ADC_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    volatile uint16_t rawValues[SAMPLES]; // Values from ADC

    // Clear leds
    ws2812_init(&htim3);
    ws2812_fillBlack();
    ws2812_update();

    /* Uncomment to Test current consumption */
    // setWHOLEcolor(255, 255, 255);
    // ws2812_update();
    // HAL_Delay(6000);
    /******/

    // Start ADC conversion in DMA mode.
    // DMA is configured in circular mode with continuous requests
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)rawValues, SAMPLES);

    // Delay duration
    uint8_t delay = 10;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // calculate ADC value average from specified samples
        volatile uint16_t average = rawValues[0];
        for (uint8_t i = 1; i < SAMPLES; i++)
        {
            average = (average + rawValues[i]);
        }
        average /= SAMPLES;

        // Execute visualization mode
        switch (mode)
        {
        case 0:
            mode_0(average);
            break;
        case 1:
            mode_1(average);
            break;
        case 2:
            mode_2(average);
            break;
        default:
            break;
        }

        HAL_Delay(delay);

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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x2000090E;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
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

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 59;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel4_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin : PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * Interrupt callback 
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_7)
    {
        // Check clock ticks between interrupts
        // to avoid bounce effect
        uint32_t tick = HAL_GetTick();
        uint32_t diff = tick - lastTick;
        if (diff > 200)
        {
            ws2812_fillBlack();
            mode++;
            if (mode > 2)
            {
                mode = 0;
            }
        }

        lastTick = tick;
    }
}

/**
 * Add value to compute average
 */
void addLastValue(uint16_t value) {
    lastValues[lastValueCount] = value;
    lastValueCount++;

    if(lastValueCount >= AVERAGE_SAMPLES) {
        lastValueCount = 0;
    }
}

/**
 * Get the average value
 */
uint16_t getAverage() {
    uint16_t average = 0;
    for(uint8_t i = 0; i<AVERAGE_SAMPLES; i++) {
        average += lastValues[i];
    }

    return abs(average/AVERAGE_SAMPLES);
}

/**
 * Mode 0 - Sound Pressure Gauge
 */
void mode_0(uint16_t value)
{
    // Difference between the average value and center position
    // of the op-amp signal. ADC computes in 12bit(4096), so
    // center position hava a value of 4095/2 ~= 2047
    uint16_t tempValue = abs(2047 - value);

    // Equivalent brightness
    uint8_t brightness = floor(tempValue * MAX_BRIGHTNESS) / max_value;
    
    // LED position that corresponds to the value
    uint8_t position = abs((tempValue * LED_COUNT) / max_value);


    // Check difference from last value to change the Hue
    int diff = value - getAverage();
    if (diff > 1)
    {
        if (globalHue < 235)
        {
            globalHue += 2;
        }
    }
    else if (diff < -1)
    {
        if (globalHue > 3)
        {
            globalHue -= 2;
        }
    }
    addLastValue(value);

    // Apply LED values and display data
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        if (i < position)
        {
            ws2812_setLEDhue(i, globalHue + hueOffset + (i * 2), 255, 255);
        }
        else
        {
            ws2812_setLEDfade(i, 1.2);
        }
    }
    ws2812_update();
}

/**
 * Mode 1 - Half ring centered and mirrored
 */
void mode_1(uint16_t value)
{
    uint16_t tempValue = abs(2047 - value);
    uint8_t brightness = floor(tempValue * MAX_BRIGHTNESS) / max_value;
    uint8_t position = abs((tempValue * 4) / max_value);

    int diff = value - getAverage();
    if (diff > 1)
    {
        if (globalHue < 235)
        {
            globalHue += 2;
        }
    }
    else if (diff < -1)
    {
        if (globalHue > 5)
        {
            globalHue -= 2;
        }
    }
    addLastValue(value);


    for (uint8_t i = 0; i < 4; i++)
    {
        if (i < position)
        {
            ws2812_setLEDhue(i, globalHue + hueOffset + (i * 2), 255, 255);
        }
        else
        {
            ws2812_setLEDfade(i, 1.2);
        }
    }

    // mirror
    ws2812_mirrorFirstQuarter();

    ws2812_update();
}

/**
 * Mode 2 - Same as Mode 1, but with rotation
 */
void mode_2(uint16_t value)
{
    uint16_t tempValue = abs(2047 - value);
    uint8_t brightness = floor(tempValue * MAX_BRIGHTNESS) / max_value;
    uint8_t position = abs((tempValue * 4) / max_value);

    int diff = value - getAverage();
    if (diff > 1)
    {
        if (globalHue < 235)
        {
            globalHue += 2;
        }
    }
    else if (diff < -1)
    {
        if (globalHue > 5)
        {
            globalHue -= 2;
        }
    }
    addLastValue(value);


    for (uint8_t i = 0; i < 4; i++)
    {
        if (i < position)
        {
            ws2812_setLEDhue(i, globalHue + hueOffset + (i * 2), 255, 255);
        }
        else
        {
            ws2812_setLEDfade(i, 1.2);
        }
    }

    // mirror
    ws2812_mirrorFirstQuarter();

    if (rotation_count == rotation_delay)
    {
        rotation_count = 0;
        rotation_step++;
        if (rotation_step >= LED_COUNT)
        {
            rotation_step = 0;
        }
    }
    rotation_count++;

    ws2812_shift(rotation_step);
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
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
