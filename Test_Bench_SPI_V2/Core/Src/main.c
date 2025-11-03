#include "stm32f0xx_hal.h"
#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// ========== Function Prototypes ==========
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void L9963E_WAKEUP(void);
int8_t L9963E_Init(void);
int8_t L9963E_ReadReg(uint8_t reg, uint16_t *data);
void UART_Print(const char *msg);
void Error_Handler(void);

// ========== UART Print ==========
void UART_Print(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// ========== L9963E Wake ==========
void L9963E_WAKEUP(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS high
    HAL_Delay(10);
    UART_Print("Wake pulse done.\r\n");
}

// ========== L9963E Read Register ==========
int8_t L9963E_ReadReg(uint8_t reg, uint16_t *data) {
    uint8_t tx[3] = {reg, 0x00, 0x00};
    uint8_t rx[3] = {0};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 100) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    *data = (rx[1] << 8) | rx[2];
    return 0;
}

// ========== L9963E Init ==========
int8_t L9963E_Init(void) {
    uint16_t devID = 0;
    if (L9963E_ReadReg(0x00, &devID) == 0) {
        char buf[64];
        sprintf(buf, "DEV_ID = 0x%04X\r\n", devID);
        UART_Print(buf);
        return 0;
    } else {
        UART_Print("DEV_ID read failed.\r\n");
        return -1;
    }
}

// ========== MAIN ==========
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    UART_Print("=== Nucleo → ISO-SPI → L9963E Test ===\r\n");

    L9963E_WAKEUP();
    L9963E_Init();

    uint16_t cell1 = 0;
    while (1) {
        if (L9963E_ReadReg(0x01, &cell1) == 0) {
            char msg[64];
            sprintf(msg, "Cell1 raw = 0x%04X\r\n", cell1);
            UART_Print(msg);
        } else {
            UART_Print("SPI read error\r\n");
        }
        HAL_Delay(500);
    }
}

// ========== GPIO Init ==========
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // SPI CS
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// ========== System Clock ==========
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

// ========== Error Handler ==========
void Error_Handler(void) {
    UART_Print("Error_Handler invoked\r\n");
    __disable_irq();
    while (1);
}
