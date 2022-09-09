/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2022 Jan Zwiener (jan@zwiener.org)
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

#define WIDTH  240      // horizontal screen size in pixel
#define HEIGHT 320      // vertical screen size in pixel

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA2D_HandleTypeDef hdma2d;
LTDC_HandleTypeDef hltdc;
SDRAM_HandleTypeDef hsdram1;
RNG_HandleTypeDef hrng;

static uint32_t g_firepalette[0xFF]; // fire color palette definition
static int PALETTE_SIZE; // number of valid entries in g_firepalette
static uint8_t g_flamebuf[WIDTH * HEIGHT]; // simulation buffer, 1 byte/pixel
static int LCD_LAYER; // active display layer

/* Private function prototypes -----------------------------------------------*/
static void initPalette(void);
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

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
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_DMA_Init();
    MX_USART1_UART_Init();
    hrng.Instance = RNG; // Setup hardware random number generator
    HAL_RNG_Init(&hrng);

    /* Display setup */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
    BSP_LCD_Init();
    LCD_LAYER = 1;
    BSP_LCD_LayerDefaultInit(LCD_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_LAYER);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

    /* ChromART (DMA2D) setup */
    hdma2d.Init.Mode         = DMA2D_M2M_PFC; // convert 8bit palette colors to 32bit ARGB888
    hdma2d.Init.ColorMode    = DMA2D_ARGB8888; // destination color format
    hdma2d.Init.OutputOffset = 0;
    hdma2d.Instance = DMA2D;
    hdma2d.LayerCfg[LCD_LAYER].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[LCD_LAYER].InputAlpha = 0xFF; // N/A only for A8 or A4
    hdma2d.LayerCfg[LCD_LAYER].InputColorMode = DMA2D_INPUT_L8; // source format (1 byte/pixel)
    hdma2d.LayerCfg[LCD_LAYER].InputOffset = 0;
    HAL_DMA2D_Init(&hdma2d);
    HAL_DMA2D_ConfigLayer(&hdma2d, LCD_LAYER);

    /* Load the color palette (CLUT) */
    initPalette(); // prepare palette before uploading it to DMA2D
    DMA2D_CLUTCfgTypeDef CLUTCfg;
    CLUTCfg.Size = sizeof(g_firepalette)/sizeof(g_firepalette[0]); // 256 palette entries
    CLUTCfg.CLUTColorMode = DMA2D_CCM_ARGB8888;
    CLUTCfg.pCLUT = g_firepalette;
    HAL_DMA2D_CLUTStartLoad(&hdma2d, &CLUTCfg, DMA2D_FOREGROUND_LAYER); // start loading of palette
    while (HAL_DMA2D_PollForTransfer(&hdma2d, 16) == HAL_BUSY) {} // wait until palette is loaded

    /* Enable CPU cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // access the cycle counter at: DWT->CYCCNT

    /* Run Main task */
    defaultTask();

    while (1) {} /* should never end up here */
}

static void initPalette(void)
{
    PALETTE_SIZE = 0;

    // Format: ARGB8888 (ALPHA is always 0xFF), RED, GREEN, BLUE
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

    /* initialize bottom of screen with white to start a fire */
    int y = HEIGHT - 1;
    for (int x = 0; x < WIDTH; x++)
    {
        g_flamebuf[y * WIDTH + x] = PALETTE_SIZE-1;
    }

}

/* Using the Systick 1000 Hz millisecond timer to sleep */
static void sleep(uint32_t delayMs)
{
    const uint32_t tickstart = HAL_GetTick();
    while((HAL_GetTick() - tickstart) < delayMs)
    {
        __WFE(); // save a bit of power while we are waiting
    }
}

/* Fire simulation is inside this function */
void defaultTask(void)
{
    const int setpointframeTimeMs = 16; // desired frametime in ms

    volatile uint32_t* fb = (uint32_t*)LCD_FRAME_BUFFER; // output framebuffer WIDTH*HEIGHT*4 (one uint32 per pixel ARGB8888)
    int frameTimeMs = 0; // current frametime in ms
    uint8_t uartAsciiOutput[128]; // debug ASCII output buffer for UART sending

    for(uint32_t epoch=0;;epoch++)
    {
        int x, y;
        uint32_t tickStart = HAL_GetTick();
        for (x = 0; x < WIDTH; x++)
        {
            for (y = 2; y < HEIGHT; y++)
            {
                const uint8_t rnd = hrng.Instance->DR % 3; // Get a random number from the HW random number generator
                const uint32_t from = y * WIDTH + x;
                const uint32_t to = from - rnd + 1;
                g_flamebuf[to - WIDTH] = g_flamebuf[from] - (rnd&(g_flamebuf[from]>0));
            }
        }

        // Use DMA2D to copy the content of g_flamebuf to the framebuffer
        // This is faster than manually updating the LCD frame buffer.
        HAL_DMA2D_PollForTransfer(&hdma2d, setpointframeTimeMs); // make sure DMA2D is ready for the next transfer
        HAL_DMA2D_Start(&hdma2d, (uint32_t)g_flamebuf, (uint32_t)fb, WIDTH, HEIGHT);

        frameTimeMs = (int)(HAL_GetTick() - tickStart);
        const int timeleft = setpointframeTimeMs - frameTimeMs;
        if (timeleft > 0)
        {
            sleep(timeleft);
        }

        // Send the frametime in milliseconds via ASCII over UART to a host PC for debugging/optimization
        if (epoch % 60 == 0 && HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
        {
            const int bytesInBuffer =
                    snprintf((char*)uartAsciiOutput, sizeof(uartAsciiOutput), "T %i ms\r\n", frameTimeMs);
            HAL_UART_Transmit(&huart1, uartAsciiOutput, bytesInBuffer, 32);
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
    __RNG_CLK_ENABLE(); /* RNG Peripheral clock enable */
}

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
