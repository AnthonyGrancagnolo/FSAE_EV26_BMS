/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <stdint.h>

/* Redirect printf to USART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* CS control */
#define CS_PIN   GPIO_PIN_4
#define CS_PORT  GPIOA
static inline void CS_Select(void)   { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); }
static inline void CS_Deselect(void) { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);    }

/* LED for activity (assumes user LED on PA5, adjust if different) */
#define LED_PIN   GPIO_PIN_5
#define LED_PORT  GPIOA
static inline void LED_On(void)  { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   }
static inline void LED_Off(void) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }

/* SPI transfer 16-bit */
static uint16_t SPI_Transfer16(uint16_t tx)
{
    uint16_t rx = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx, (uint8_t*)&rx, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return 0xFFFF; // indicates error
    }
    return rx;
}

/* WAKE pulse (if board uses PB0) */
static void WakePulse(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(2);
    printf("Wake pulse done.\r\n");
}

/* Change SPI mode */
static HAL_StatusTypeDef SPI_SetMode(uint32_t cpol, uint32_t cpha)
{
    HAL_SPI_DeInit(&hspi1);

    hspi1.Init.CLKPolarity = cpol;
    hspi1.Init.CLKPhase    = cpha;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    CS_Deselect();

    printf("\r\n=== Nucleo → ISOSPI1A → BMS Board Test ===\r\n");
    printf("CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7, UART=USART2\r\n");

    WakePulse(); // optional

    /* Test two SPI modes */
    struct { uint32_t cpol, cpha; const char* name; } modes[] =
    {
        { SPI_POLARITY_LOW, SPI_PHASE_1EDGE, "Mode 0 (CPOL=0, CPHA=0)" },
        { SPI_POLARITY_LOW, SPI_PHASE_2EDGE, "Mode 1 (CPOL=0, CPHA=1)" }
    };

    uint16_t testPatterns[] = { 0xAAAA, 0x5555, 0xC000, 0x3FFF, 0x8000, 0x8100 };
    size_t numPatterns = sizeof(testPatterns)/sizeof(testPatterns[0]);

    for (int m = 0; m < 2; m++)
    {
        printf("\r\n--- Testing %s ---\r\n", modes[m].name);
        if (SPI_SetMode(modes[m].cpol, modes[m].cpha) != HAL_OK)
        {
            printf("Failed to set SPI mode.\r\n");
            continue;
        }
        HAL_Delay(5);

        for (size_t p = 0; p < numPatterns; p++)
        {
            uint16_t tx = testPatterns[p];

            CS_Select();
            uint16_t rx = SPI_Transfer16(tx);
            CS_Deselect();

            LED_On();
            HAL_Delay(20);
            LED_Off();

            if (rx == 0xFFFF)
            {
                printf("Sent 0x%04X → SPI error\n", tx);
            }
            else
            {
                printf("Sent 0x%04X → Received 0x%04X\n", tx, rx);
            }

            HAL_Delay(200);
        }

        printf("Burst send of 0x%04X x8 (watch CS/SCK/MOSI/MISO on scope)\n", testPatterns[0]);
        for (int i = 0; i < 8; i++)
        {
            CS_Select();
            uint16_t rx = SPI_Transfer16(testPatterns[0]);
            CS_Deselect();
            printf(" Burst %d → 0x%04X\n", i, rx);
            HAL_Delay(50);
        }
    }

    printf("\r\n--- Automated tests done, entering continuous read loop ---\n");

    /* Back to your “read cell” scenario */
    // Set mode as needed
    SPI_SetMode(SPI_POLARITY_LOW, SPI_PHASE_2EDGE); // Mode 1 for your device (if that is correct)
    HAL_Delay(5);

    while (1)
    {
        uint16_t cmd = 0xC000; // read command placeholder
        CS_Select();
        uint16_t rx = SPI_Transfer16(cmd);
        CS_Deselect();

        printf("ReadCmd 0x%04X → 0x%04X\n", cmd, rx);

        HAL_Delay(300);
    }
}

/* SystemClock_Config */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef  RCC_ClkInitStruct = {0};

    RCC_OscInitTypeDef hO = {0};
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // stay here
    }
}
