/* main.c - Wake + DEV_GEN_CFG + FSM poll (slow, robust startup) */

#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* === GPIO aliases (adjust pins if your wiring differs) === */
#define MOSI_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define MOSI_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define SCK_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define SCK_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define CS_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define MISO_READ() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)

#define DIS_LOW()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define DIS_HIGH()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define TXEN_LOW()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define TXEN_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define BNE_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)

/* L9963E addresses */
#define DEV_GEN_CFG_ADDR 0x01
#define FSM_ADDR         0x12

extern UART_HandleTypeDef huart2;

/* slow bitbang (very conservative delays for bring-up) */
void spiBitbangTransfer_slow(uint8_t* tx, uint8_t* rx, int len)
{
    for (int b = 0; b < len; b++)
    {
        uint8_t txByte = tx[b];
        uint8_t rxByte = 0;
        for (int bit = 7; bit >= 0; bit--)
        {
            if (txByte & (1 << bit)) MOSI_HIGH();
            else MOSI_LOW();

            for (volatile int d = 0; d < 5000; d++); // long pre-clock delay

            SCK_HIGH(); // rising edge - sample MISO after this (CPHA=1)
            for (volatile int d = 0; d < 5000; d++);
            if (MISO_READ() == GPIO_PIN_SET) rxByte |= (1 << bit);

            SCK_LOW();
            for (volatile int d = 0; d < 5000; d++); // post-clock
        }
        rx[b] = rxByte;
    }
}

/* basic UART helper */
void uartSend(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* init GPIO + UART (no CubeMX dependency) */
void initPeripherals(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* SPI outputs: PA4 (CS), PA5 (SCK), PA7 (MOSI) */
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* MISO input PA6 */
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Control pins PC1 (TXEN), PC2 (DIS) as outputs */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* BNE input PC0 */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP; /* weak pull-up prevents floating reads */
    HAL_GPIO_Init(GPIOC, &gpio);

    /* default lines */
    CS_HIGH();
    SCK_LOW();
    MOSI_LOW();
    TXEN_LOW();
    DIS_HIGH();

    /* UART2 init */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        /* UART init failed — hang */
        while (1);
    }
}

/* helper: do a whole-frame transaction with pre/post CS delays using slow bitbang */
void doTransfer_slow(uint8_t *tx, uint8_t *rx, int len)
{
    CS_LOW();
    for (volatile int d=0; d<2000; d++); // pre-CS clock settle (>=300ns)
    spiBitbangTransfer_slow(tx, rx, len);
    for (volatile int d=0; d<4000; d++); // post-transfer settle (>=2us)
    CS_HIGH();
}

/* send many dummy bytes (0xFF) to wake iso link and remote side */
void send_wakeup_pulse(void)
{
    uint8_t txbuf[8];
    uint8_t rxbuf[8];
    for (int i=0;i<8;i++) txbuf[i] = 0xFF;
    doTransfer_slow(txbuf, rxbuf, 8);
}

/* attempt to read FIFO frames if BNE asserted */
void readFrameWhenBNE(void)
{
    char msg[128];
    if (BNE_READ() == GPIO_PIN_SET)
    {
        /* read several bytes (try 8) to capture full frame */
        uint8_t tx[8]; uint8_t rx[8];
        for (int i=0;i<8;i++) tx[i] = 0x00; // dummy bytes to clock out rx
        doTransfer_slow(tx, rx, 8);
        int n = snprintf(msg, sizeof(msg), "FRAME RX:");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, HAL_MAX_DELAY);
        for (int i=0;i<8;i++)
        {
            n = snprintf(msg, sizeof(msg), " %02X", rx[i]);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, HAL_MAX_DELAY);
        }
        uartSend("\r\n");
    }
}

int main(void)
{
    HAL_Init();
    initPeripherals();

    char buf[160];
    uint8_t tx[4], rx[4];

    uartSend("\r\n=== POWBMS slow bring-up test ===\r\n");

    /* --- Required safe power/wake sequence --- */
    /* Ensure DIS toggling and delays to re-engage BNE after any previous forced state */
    DIS_HIGH();      /* start disabled */
    TXEN_LOW();
    HAL_Delay(50);

    /* Toggle DIS to ensure trimming/config re-engage on next wake */
    DIS_HIGH(); HAL_Delay(10);
    DIS_LOW();  HAL_Delay(10);

    /* Now enable transmitter */
    TXEN_HIGH();
    HAL_Delay(20);

    /* Optional: small extra toggle to be robust */
    // DIS_HIGH(); HAL_Delay(5); DIS_LOW(); HAL_Delay(5);

    /* Give ISO link a wake pulse: several 0xFF bytes slowly */
    uartSend("Sending wakeup pulse...\r\n");
    send_wakeup_pulse();
    HAL_Delay(20);

    /* Try writing DEV_GEN_CFG = 0x0001 (attempt to request normal mode) */
    tx[0] = DEV_GEN_CFG_ADDR;
    tx[1] = 0x01; // request normalish config -- try enabling features
    tx[2] = 0x00;
    tx[3] = 0x00;
    doTransfer_slow(tx, rx, 4);
    snprintf(buf, sizeof(buf), "DEV_GEN_CFG TX=%02X %02X %02X %02X RX=%02X %02X %02X %02X BNE=%d\r\n",
             tx[0],tx[1],tx[2],tx[3], rx[0],rx[1],rx[2],rx[3], (BNE_READ()==GPIO_PIN_SET));
    uartSend(buf);
    HAL_Delay(50);

    /* Now main loop: poll FSM and watch BNE */
    while (1)
    {
        /* slow read FSM */
        tx[0] = FSM_ADDR;
        tx[1] = 0x00;
        tx[2] = 0x00;
        tx[3] = 0x00;
        doTransfer_slow(tx, rx, 4);

        snprintf(buf, sizeof(buf),
                 "FSM RX=%02X %02X %02X %02X  BNE=%d\r\n",
                 rx[0], rx[1], rx[2], rx[3],
                 (BNE_READ()==GPIO_PIN_SET));
        uartSend(buf);

        /* if BNE is high, try to read queued frame(s) */
        if (BNE_READ() == GPIO_PIN_SET)
        {
            uartSend("BNE asserted — reading frame(s)...\r\n");
            readFrameWhenBNE();
        }

        HAL_Delay(500);
    }
}
