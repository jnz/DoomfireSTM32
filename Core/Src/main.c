/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 Jan Zwiener
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stm32f429i_discovery_lcd.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define WIDTH  240
#define HEIGHT 320
#define BPP 4
// Framebuffer memory location is defined in LCD_FRAME_BUFFER define

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA2D_HandleTypeDef hdma2d;
LTDC_HandleTypeDef hltdc;
SDRAM_HandleTypeDef hsdram1;
HCD_HandleTypeDef hhcd_USB_OTG_HS;
RNG_HandleTypeDef hrng;

static uint32_t g_firepalette[0xFF];
static int PALETTE_SIZE; // number of valid entries in g_firepalette
static uint8_t g_firebuf[WIDTH * HEIGHT]; // more like "flamebuf" :-)
static int LCD_LAYER;

/* Private function prototypes -----------------------------------------------*/
static void init(void);
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/*
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
*/

static __inline unsigned M_Random(void);
void defaultTask(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_DMA_Init();
    MX_USART1_UART_Init();
    // MX_GPIO_Init();
    // MX_DMA2D_Init();
    // MX_FMC_Init();
    // MX_LTDC_Init();
    hrng.Instance = RNG;
    HAL_RNG_Init(&hrng);

    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
    BSP_LCD_Init();
    LCD_LAYER = 1;
    BSP_LCD_LayerDefaultInit(LCD_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_LAYER);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
    // BSP_LCD_Clear(LCD_COLOR_BLACK);

    /* DMA2D setup */
    hdma2d.Init.Mode         = DMA2D_M2M_PFC;
    hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
    hdma2d.Init.OutputOffset = 0;
    hdma2d.Instance = DMA2D;
    hdma2d.LayerCfg[LCD_LAYER].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[LCD_LAYER].InputAlpha = 0xFF; // only for A8 or A4
    hdma2d.LayerCfg[LCD_LAYER].InputColorMode = DMA2D_INPUT_L8;
    hdma2d.LayerCfg[LCD_LAYER].InputOffset = 0;
    HAL_DMA2D_Init(&hdma2d);
    HAL_DMA2D_ConfigLayer(&hdma2d, LCD_LAYER);

    /* Load the color palette */
    init();
    DMA2D_CLUTCfgTypeDef CLUTCfg;
    CLUTCfg.Size = 0xFF; // PALETTE_SIZE;
    CLUTCfg.CLUTColorMode = DMA2D_CCM_ARGB8888;
    CLUTCfg.pCLUT = g_firepalette;
    HAL_DMA2D_CLUTStartLoad(&hdma2d, &CLUTCfg, DMA2D_FOREGROUND_LAYER);
    while (HAL_DMA2D_PollForTransfer(&hdma2d, 16) == HAL_BUSY) {}

    /* Enable cycle counter, to use the cycle counter
     * as cheap random number generator */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Main task */
    defaultTask();

    while (1) {} /* should never end up here */
}

static void init(void)
{
    PALETTE_SIZE = 0;

    g_firepalette[PALETTE_SIZE++] = 0xFF070707;
    g_firepalette[PALETTE_SIZE++] = 0xFF1F0707;
    g_firepalette[PALETTE_SIZE++] = 0xFF2F0F07;
    g_firepalette[PALETTE_SIZE++] = 0xFF470F07;
    g_firepalette[PALETTE_SIZE++] = 0xFF571707;
    g_firepalette[PALETTE_SIZE++] = 0xFF671F07;
    g_firepalette[PALETTE_SIZE++] = 0xFF771F07;
    g_firepalette[PALETTE_SIZE++] = 0xFF8F2707;
    g_firepalette[PALETTE_SIZE++] = 0xFF9F2F07;
    g_firepalette[PALETTE_SIZE++] = 0xFFAF3F07;
    g_firepalette[PALETTE_SIZE++] = 0xFFBF4707;
    g_firepalette[PALETTE_SIZE++] = 0xFFC74707;
    g_firepalette[PALETTE_SIZE++] = 0xFFDF4F07;
    g_firepalette[PALETTE_SIZE++] = 0xFFDF5707;
    g_firepalette[PALETTE_SIZE++] = 0xFFDF5707;
    g_firepalette[PALETTE_SIZE++] = 0xFFD75F07;
    g_firepalette[PALETTE_SIZE++] = 0xFFD75F07;
    g_firepalette[PALETTE_SIZE++] = 0xFFD7670F;
    g_firepalette[PALETTE_SIZE++] = 0xFFCF6F0F;
    g_firepalette[PALETTE_SIZE++] = 0xFFCF770F;
    g_firepalette[PALETTE_SIZE++] = 0xFFCF7F0F;
    g_firepalette[PALETTE_SIZE++] = 0xFFCF8717;
    g_firepalette[PALETTE_SIZE++] = 0xFFC78717;
    g_firepalette[PALETTE_SIZE++] = 0xFFC78F17;
    g_firepalette[PALETTE_SIZE++] = 0xFFC7971F;
    g_firepalette[PALETTE_SIZE++] = 0xFFBF9F1F;
    g_firepalette[PALETTE_SIZE++] = 0xFFBF9F1F;
    g_firepalette[PALETTE_SIZE++] = 0xFFBFA727;
    g_firepalette[PALETTE_SIZE++] = 0xFFBFA727;
    g_firepalette[PALETTE_SIZE++] = 0xFFBFAF2F;
    g_firepalette[PALETTE_SIZE++] = 0xFFB7AF2F;
    g_firepalette[PALETTE_SIZE++] = 0xFFB7B72F;
    g_firepalette[PALETTE_SIZE++] = 0xFFB7B737;
    g_firepalette[PALETTE_SIZE++] = 0xFFCFCF6F;
    g_firepalette[PALETTE_SIZE++] = 0xFFDFDF9F;
    g_firepalette[PALETTE_SIZE++] = 0xFFEFEFC7;
    g_firepalette[PALETTE_SIZE++] = 0xFFFFFFFF;

    /* initialize bottom of screen with white */
    int y = HEIGHT - 1;
    for (int x = 0; x < WIDTH; x++)
    {
        g_firebuf[y * WIDTH + x] = PALETTE_SIZE-1;
    }

}

static void sleep(uint32_t Delay)
{
    const uint32_t tickstart = HAL_GetTick();
    while((HAL_GetTick() - tickstart) < Delay)
    {
        __WFE();
    }
}

void defaultTask(void)
{
    // volatile uint32_t* fb = (uint32_t*)LCD_FRAME_BUFFER;
    int frameTimeMs = 0;
    uint8_t outputBuffer[128];
    uint32_t epoch;

    for(epoch=0;;epoch++)
    {

        int x, y;
        uint32_t tickStart = HAL_GetTick();
        for (x = 0; x < WIDTH; x++)
        {
            for (y = 2; y < HEIGHT; y++)
            {
                const uint8_t rnd = hrng.Instance->DR % 3; // HW Random instead of M_Random()
                const uint32_t from = y * WIDTH + x;
                const uint32_t to = from - rnd + 1;
                g_firebuf[to - WIDTH] = g_firebuf[from] - (rnd&(g_firebuf[from]>0));
            }
        }

        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
        if (HAL_DMA2D_Start(&hdma2d, (uint32_t)g_firebuf, (uint32_t)LCD_FRAME_BUFFER, WIDTH, HEIGHT) == HAL_OK)
        {
        }

        frameTimeMs = (int)(HAL_GetTick() - tickStart);
        int timeleft = 16 - frameTimeMs;
        if (timeleft > 0)
        {
            sleep(timeleft);
        }

        if (epoch % 60 == 0 && HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
        {
            const int bytesInBuffer =
                    snprintf((char*)outputBuffer, sizeof(outputBuffer), "T %i ms\r\n", frameTimeMs);
            HAL_UART_Transmit(&huart1, outputBuffer, bytesInBuffer, 32);
            // HAL_UART_Transmit_DMA(&huart1, outputBuffer, bytesInBuffer); // WARNING DMA not configured properly
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
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
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

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
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    /* RNG Peripheral clock enable */
    __RNG_CLK_ENABLE();
}

#if 0
/* FMC initialization function */
static void MX_FMC_Init(void)
{

    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_SDRAM_TimingTypeDef SdramTiming = {0};

    /* USER CODE BEGIN FMC_Init 1 */

    /* USER CODE END FMC_Init 1 */

    /** Perform the SDRAM1 memory initialization sequence
     */
    hsdram1.Instance = FMC_SDRAM_DEVICE;
    /* hsdram1.Init */
    hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
    hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
    hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
    hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
    hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
    hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
    hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
    hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
    /* SdramTiming */
    SdramTiming.LoadToActiveDelay = 2;
    SdramTiming.ExitSelfRefreshDelay = 7;
    SdramTiming.SelfRefreshTime = 4;
    SdramTiming.RowCycleDelay = 7;
    SdramTiming.WriteRecoveryTime = 3;
    SdramTiming.RPDelay = 2;
    SdramTiming.RCDDelay = 2;

    if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
    {
        Error_Handler( );
    }

    /* USER CODE BEGIN FMC_Init 2 */

    /* USER CODE END FMC_Init 2 */
}
#endif

#if 0
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
    GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
    GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ACP_RST_Pin */
    GPIO_InitStruct.Pin = ACP_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OC_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOOT1_Pin */
    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : TE_Pin */
    GPIO_InitStruct.Pin = TE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
    GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : LD3_Pin LD4_Pin */
    GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}
#endif

#if 0
/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void)
{

    /* USER CODE BEGIN DMA2D_Init 0 */

    /* USER CODE END DMA2D_Init 0 */

    /* USER CODE BEGIN DMA2D_Init 1 */

    /* USER CODE END DMA2D_Init 1 */
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_M2M;
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
    hdma2d.Init.OutputOffset = 0;
    hdma2d.LayerCfg[1].InputOffset = 0;
    hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
    hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[1].InputAlpha = 0;
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DMA2D_Init 2 */

    /* USER CODE END DMA2D_Init 2 */

}
#endif


#if 0
/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void)
{

    /* USER CODE BEGIN LTDC_Init 0 */

    /* USER CODE END LTDC_Init 0 */

    LTDC_LayerCfgTypeDef pLayerCfg = {0};

    /* USER CODE BEGIN LTDC_Init 1 */

    /* USER CODE END LTDC_Init 1 */
    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = 9;
    hltdc.Init.VerticalSync = 1;
    hltdc.Init.AccumulatedHBP = 29;
    hltdc.Init.AccumulatedVBP = 3;
    hltdc.Init.AccumulatedActiveW = 269;
    hltdc.Init.AccumulatedActiveH = 323;
    hltdc.Init.TotalWidth = 279;
    hltdc.Init.TotalHeigh = 327;
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;
    if (HAL_LTDC_Init(&hltdc) != HAL_OK)
    {
        Error_Handler();
    }
    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = 240;
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = 320;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    pLayerCfg.Alpha = 255;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    pLayerCfg.FBStartAdress = 0xD0000000;
    pLayerCfg.ImageWidth = 240;
    pLayerCfg.ImageHeight = 320;
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN LTDC_Init 2 */

    /* USER CODE END LTDC_Init 2 */

}
#endif

#ifdef  USE_FULL_ASSERT
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

static const unsigned char g_rndtable[256] = {
    0,   8, 109, 220, 222, 241, 149, 107,  75, 248, 254, 140,  16,  66 ,
    74,  21, 211,  47,  80, 242, 154,  27, 205, 128, 161,  89,  77,  36 ,
    95, 110,  85,  48, 212, 140, 211, 249,  22,  79, 200,  50,  28, 188 ,
    52, 140, 202, 120,  68, 145,  62,  70, 184, 190,  91, 197, 152, 224 ,
    149, 104,  25, 178, 252, 182, 202, 182, 141, 197,   4,  81, 181, 242 ,
    145,  42,  39, 227, 156, 198, 225, 193, 219,  93, 122, 175, 249,   0 ,
    175, 143,  70, 239,  46, 246, 163,  53, 163, 109, 168, 135,   2, 235 ,
    25,  92,  20, 145, 138,  77,  69, 166,  78, 176, 173, 212, 166, 113 ,
    94, 161,  41,  50, 239,  49, 111, 164,  70,  60,   2,  37, 171,  75 ,
    136, 156,  11,  56,  42, 146, 138, 229,  73, 146,  77,  61,  98, 196 ,
    135, 106,  63, 197, 195,  86,  96, 203, 113, 101, 170, 247, 181, 113 ,
    80, 250, 108,   7, 255, 237, 129, 226,  79, 107, 112, 166, 103, 241 ,
    24, 223, 239, 120, 198,  58,  60,  82, 128,   3, 184,  66, 143, 224 ,
    145, 224,  81, 206, 163,  45,  63,  90, 168, 114,  59,  33, 159,  95 ,
    28, 139, 123,  98, 125, 196,  15,  70, 194, 253,  54,  14, 109, 226 ,
    71,  17, 161,  93, 186,  87, 244, 138,  20,  52, 123, 251,  26,  36 ,
    17,  46,  52, 231, 232,  76,  31, 221,  84,  37, 216, 165, 212, 106 ,
    197, 242,  98,  43,  39, 175, 254, 145, 190,  84, 118, 222, 187, 136 ,
    120, 163, 236, 249
};
static int rndindex;
static __inline unsigned M_Random(void)
{
    rndindex = (rndindex+1)&0xff;
    return g_rndtable[rndindex] + (DWT->CYCCNT & 0xff);
}
