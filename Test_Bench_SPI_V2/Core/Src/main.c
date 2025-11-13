/* main.c - single-file BMS slow-bitbang + L9963 frame builder/CRC + BNE wait/receive
 *
 * Usage:
 *  - NUCLEO-F091RC (adjust if different)
 *  - Connect: PA4=NCS, PA5=SCK, PA6=MISO, PA7=MOSI
 *             PC0=BNE (input), PC1=TXEN (output), PC2=DIS (output)
 *  - Ensure common ground between all boards, ISO devkit VDD=5V and VIO=3.3V,
 *    and VTREF ~3.3V at the BMS board.
 *
 * This file uses bit-banged SPI mode 3 for full control and uses the
 * L9963 CRC/frame conventions you supplied.
 *
 * Keep huart2 provided by your project (CubeMX) — it's extern here.
 */

#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ========== CONFIG ========== */
#define RUN_LOOPBACK_CHECK 1
#define FORCE_TRANSCEIVER_ALWAYS 1

/* ========== GPIO aliases (change if wiring differs) ========== */
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

/* L9963 addresses */
#define DEV_GEN_CFG_ADDR 0x01
#define FSM_ADDR         0x12
#define VCELL1_ADDR      0x21
#define VCELL_LAST_ADDR  0x2E

/* Extern UART handle from CubeMX project */
extern UART_HandleTypeDef huart2;

/* UART helper */
static void uartSend(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* ------------ CRC LUT & calc (copied/adapted from your driver) ------------ */
static uint8_t crc6_lut[64] = {
    0x0,  0x19, 0x32, 0x2b, 0x3d, 0x24, 0x0f, 0x16,
    0x23, 0x3a, 0x11, 0x08, 0x1e, 0x07, 0x2c, 0x35,
    0x1f, 0x06, 0x2d, 0x34, 0x22, 0x3b, 0x10, 0x09,
    0x3c, 0x25, 0x0e, 0x17, 0x01, 0x18, 0x33, 0x2a,
    0x3e, 0x27, 0x0c, 0x15, 0x03, 0x1a, 0x31, 0x28,
    0x1d, 0x04, 0x2f, 0x36, 0x20, 0x39, 0x12, 0x0b,
    0x21, 0x38, 0x13, 0x0a, 0x1c, 0x05, 0x2e, 0x37,
    0x02, 0x1b, 0x30, 0x29, 0x3f, 0x26, 0x0d, 0x14
};

/* constants */
#define WORD_LEN           40UL
#define CRC_LEN            6UL
#define CRC_POLY           (uint64_t)0x0000000000000059ULL
#define CRC_SEED           (uint64_t)0x0000000000000038ULL
#define CRC_INIT_SEED_MASK (CRC_SEED << (WORD_LEN - CRC_LEN))
#define CRC_INIT_MASK      (CRC_POLY << (WORD_LEN - CRC_LEN - 1))
#define FIRST_BIT_MASK     ((uint64_t)1 << (WORD_LEN - 1))

static uint8_t L9963_crc_calc(uint64_t InputWord) {
    uint64_t TestBitMask;
    uint64_t CRCMask;
    uint8_t BitCount;
    uint8_t crc = 0;

    /* Clear CRC field and xor seed mask */
    InputWord = (InputWord & 0xFFFFFFFFC0ULL) ^ CRC_INIT_SEED_MASK;

    TestBitMask = FIRST_BIT_MASK;
    CRCMask     = CRC_INIT_MASK;
    BitCount    = WORD_LEN % CRC_LEN;
    while (0 != BitCount--) {
        if (0 != (InputWord & TestBitMask)) {
            InputWord ^= CRCMask;
        }
        CRCMask >>= 1;
        TestBitMask >>= 1;
    }

    for (int8_t i = WORD_LEN - (WORD_LEN % CRC_LEN) - CRC_LEN; i > 0; i -= CRC_LEN) {
        crc = crc6_lut[((InputWord >> i) & 0x3F) ^ crc];
    }

    return crc;
}

/* ------------------- frame helpers ------------------- */
/* Build 40-bit frame into 5-byte array (big-endian as wire expects)
   Parameters:
     pa:      1 (primary => request)
     rw_burst: 0 read-reg, 1 write/reg-burst (per usage)
     devid:   device id (1..N)
     addr_command: register address or command
     data:    16-bit-ish data payload (fits in 24-bit field if needed)
*/
static void build_frame_5(uint8_t *out, uint8_t pa, uint8_t rw_burst, uint8_t devid,
                          uint8_t addr_command, uint32_t data) {
    /* Frame layout in "frame.val" union used by your driver:
       Bits: [39..0] = pa(1) | rw_burst(1) | devid(6) | addr(6) | data(20?) | crc6
       The original driver uses a union; we will assemble a 40-bit value accordingly.
       For safety, we pack the frame similarly: (pa << 39) ... etc. */

    uint64_t frame = 0;
    /* Place fields roughly matching the original packing:
       We'll follow the same order used in their build: pa (1 bit) at MSB, rw_burst next, devid, addr, data
       We must ensure crc field (6 LSBs) cleared before calc.
    */
    frame = ((uint64_t)(pa & 0x1) << 39)
          | ((uint64_t)(rw_burst & 0x1) << 38)
          | ((uint64_t)(devid & 0x3F) << 32)    /* 6 bits devid */
          | ((uint64_t)(addr_command & 0x3F) << 26) /* 6 bits addr */
          | ((uint64_t)(data & 0x03FFFFFF) << 6);   /* data occupies next bits (up to 26 bits) */

    /* compute CRC6 */
    uint8_t crc = L9963_crc_calc(frame);
    /* place CRC in lowest 6 bits */
    frame |= (uint64_t)(crc & 0x3F);

    /* Switch endianness to wire order expected by remote (driver used switch_endianness) */
    /* driver switched bytes: out[0]=in[4], out[1]=in[3], ... out[4]=in[0] */
    uint8_t tmp[5];
    tmp[0] = (frame >> 32) & 0xFF;
    tmp[1] = (frame >> 24) & 0xFF;
    tmp[2] = (frame >> 16) & 0xFF;
    tmp[3] = (frame >> 8) & 0xFF;
    tmp[4] = (frame >> 0) & 0xFF;
    /* reverse order to match your driver's switch_endianness */
    out[0] = tmp[4];
    out[1] = tmp[3];
    out[2] = tmp[2];
    out[3] = tmp[1];
    out[4] = tmp[0];
}

/* ------------------- low-level SPI bitbang (Mode 3) ------------------- */
/* Conservative timing loops for bring-up */
static void delay_short(void) {
    for (volatile int d = 0; d < 2000; ++d) { __asm volatile ("nop"); }
}

/* Transmit N bytes (no MISO sampling) while asserting TXEN for the transmitter */
static void spi_transmit_bytes(const uint8_t *tx, int len) {
    /* According to L9963 driver pattern: TXEN_HIGH before CS low and transmission; TXEN_LOW before CS high */
    TXEN_HIGH();
    CS_LOW();
    for (int b = 0; b < len; ++b) {
        uint8_t byte = tx[b];
        for (int bit = 7; bit >= 0; --bit) {
            if (byte & (1 << bit)) MOSI_HIGH(); else MOSI_LOW();
            delay_short();
            /* falling edge */
            SCK_LOW();
            delay_short();
            /* rising edge */
            SCK_HIGH();
            delay_short();
        }
    }
    /* complete */
    CS_HIGH();
    TXEN_LOW();
    /* ensure idle CPOL=1 */
    SCK_HIGH();
}

/* Receive len bytes: the usual flow is to assert TXEN_LOW (so remote can drive) and then wait for BNE,
   but when called we will clock '0x00' while reading MISO. CS is controlled outside by caller when needed. */
static void spi_receive_bytes(uint8_t *rx, int len) {
    /* This function clocks bytes while reading MISO. Caller should handle CS and TXEN levels. */
    for (int b = 0; b < len; ++b) {
        uint8_t rbyte = 0;
        for (int bit = 7; bit >= 0; --bit) {
            MOSI_LOW(); /* drive 0 while receiving */
            delay_short();
            SCK_LOW();
            delay_short();
            SCK_HIGH();
            if (MISO_READ() == GPIO_PIN_SET) rbyte |= (1 << bit);
            delay_short();
        }
        rx[b] = rbyte;
    }
}

/* ------------------ higher-level driver-style wrappers ------------------ */

/* _spi_transmit wrapper like _L9963E_DRV_spi_transmit: sets TXEN + CS, sends 5 bytes */
static void drv_spi_transmit_5(uint8_t *raw5) {
    /* Keep TXEN high during transmit; remote expects TXEN during frame transmission */
    TXEN_HIGH();
    CS_LOW();
    spi_transmit_bytes(raw5, 5); /* note: spi_transmit_bytes asserts TXEN and CS; still safe */
    /* spi_transmit_bytes toggles CS high and TXEN low already, so nothing more to do */
}

/* wait for BNE and read 5 bytes; returns 0 on OK, non-zero on timeout or crc error.
   timeout_ms: milliseconds timeout
   device_id: expected device id in frame (we'll check after reading)
   out_frame: 5-byte raw frame output (wire order) if non-null
*/
static int drv_wait_and_receive_5(uint8_t expected_devid, uint8_t *out_frame, uint32_t timeout_ms) {
    const uint32_t start = HAL_GetTick();
    uint8_t raw[5];
    uint8_t frame_swapped[5];
    uint8_t ok = 0;

    /* Ensure TXEN low during wait/read */
    TXEN_LOW();

    while (1) {
        if (BNE_READ() == GPIO_PIN_SET) {
            /* Read 5 bytes */
            CS_LOW();
            spi_receive_bytes(raw, 5); /* reads MISO while clocking */
            CS_HIGH();

            /* Switch endianness back to internal ordering as driver did:
               raw wire order -> frame.val (driver used switch_endianness raw->frame.val)
               That mapping was: out[0]=in[4]; out[1]=in[3]; out[2]=in[2]; out[3]=in[1]; out[4]=in[0];
               So to reconstruct frame.val bytes, reverse same mapping.
            */
            frame_swapped[0] = raw[4];
            frame_swapped[1] = raw[3];
            frame_swapped[2] = raw[2];
            frame_swapped[3] = raw[1];
            frame_swapped[4] = raw[0];

            /* compute 40-bit value for CRC check */
            uint64_t val = ((uint64_t)frame_swapped[0] << 32) |
                           ((uint64_t)frame_swapped[1] << 24) |
                           ((uint64_t)frame_swapped[2] << 16) |
                           ((uint64_t)frame_swapped[3] << 8) |
                           ((uint64_t)frame_swapped[4]);

            /* Extract devid from same bit positions used in build_frame_5 */
            /* In build_frame_5 we used: devid at bits [37..32] (6 bits). Because we placed pa<<39, rw<<38, devid<<32 */
            uint8_t devid = (uint8_t)((val >> 32) & 0x3F); /* note val is already big-endian arrangement */

            /* Validate CRC: LSB 6 bits are CRC (in frame.val) */
            uint8_t crc_calc = L9963_crc_calc(val);
            uint8_t crc_frame = (uint8_t)(val & 0x3F);
            if (crc_calc != crc_frame) {
                /* CRC bad - signal error but keep trying until timeout */
                char buf[80];
                int n = snprintf(buf, sizeof(buf), "RX CRC FAIL (got %02X calc %02X) devid=%u\r\n", crc_frame, crc_calc, devid);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
                /* continue waiting until timeout */
            } else {
                /* CRC ok: check device id */
                if (devid == expected_devid || expected_devid == 0) {
                    ok = 1;
                    if (out_frame) {
                        /* return raw bytes in wire order for caller convenience */
                        for (int i=0;i<5;++i) out_frame[i] = raw[i];
                    }
                    break;
                } else {
                    /* Not the device we expect; loop again until timeout */
                    char buf[64];
                    int n = snprintf(buf, sizeof(buf), "RX devid %u != expected %u\r\n", devid, expected_devid);
                    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
                }
            }
        }

        if ((HAL_GetTick() - start) >= timeout_ms) {
            return -1; /* timeout */
        }
    }

    /* after read, raise TXEN (driver behavior) */
    TXEN_HIGH();
    return ok ? 0 : -2;
}

/* Build and send a reg-read (pa=1 rw_burst=0 devid, addr) and wait for response */
static int drv_reg_read(uint8_t devid, uint8_t address, uint8_t *out_payload32, uint32_t timeout_ms) {
    uint8_t raw5[5];
    build_frame_5(raw5, 1, 0, devid, address, 0);
    /* transmit frame: follow driver: TXEN_HIGH + CS_LOW + transmission + TXEN_LOW + CS_HIGH */
    spi_transmit_bytes(raw5, 5);
    /* Wait and receive a frame for that device */
    uint8_t rx5[5];
    int r = drv_wait_and_receive_5(devid, rx5, timeout_ms);
    if (r != 0) return r;

    /* convert rx5 into bytes in driver frame order (switch_endianness) */
    uint8_t swapped[5];
    swapped[0] = rx5[4];
    swapped[1] = rx5[3];
    swapped[2] = rx5[2];
    swapped[3] = rx5[1];
    swapped[4] = rx5[0];
    /* Extract data field (we placed data starting at bit 6 up to bit ~29 depending) */
    uint32_t data = ((uint32_t)swapped[1] << 24) | ((uint32_t)swapped[2] << 16) | ((uint32_t)swapped[3] << 8) | ((uint32_t)swapped[4]);
    /* In your earlier code you used rx[2]<<8 | rx[3] as raw16; caller can parse as needed */
    if (out_payload32) * (uint32_t*)out_payload32 = data;
    return 0;
}

/* ------------------- MCU init + diagnostics ------------------- */
static void SystemClock_Config_local(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) while(1);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) while(1);
}

/* init periph pins & UART (no HAL SPI usage) */
static void initPeripherals_local(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* outputs: CS=PA4, SCK=PA5, MOSI=PA7 */
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

    /* Control pins PC1(TXEN) PC2(DIS) -> outputs */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* BNE PC0 input with pulldown to avoid floating mid-rail */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* defaults for Mode3: idle SCK high */
    CS_HIGH();
    SCK_HIGH();
    MOSI_LOW();
    TXEN_LOW();
    DIS_HIGH(); /* transceiver disabled by default */

    /* UART2 init (respects extern huart2 fields provided in CubeMX) */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) while(1);
}

/* Diagnostic wake sequence: toggles DIS/TXEN and sends long 0xFF stream while sampling MISO */
static void diagnostic_wake_local(void) {
    char msg[160];
    snprintf(msg, sizeof(msg), "INIT PINS: DIS=%d TXEN=%d BNE=%d MISO=%d\r\n",
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2),
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1),
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0),
             (int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
    uartSend(msg);

#if FORCE_TRANSCEIVER_ALWAYS
    DIS_LOW();
    TXEN_HIGH();
    HAL_Delay(80);
    uartSend("DEBUG: FORCED DIS_LOW() + TXEN_HIGH() (transceiver held enabled)\r\n");
#else
    /* the toggle sequence your datasheet suggests to re-engage trimming */
    DIS_HIGH(); TXEN_LOW();
    HAL_Delay(200);
    DIS_LOW();
    HAL_Delay(250);
    TXEN_HIGH();
    HAL_Delay(100);
    uartSend("Normal wake toggle completed.\r\n");
#endif

    /* Send a long stream of 0xFF (32 bytes) and sample MISO while doing it */
    uartSend("Starting wake with 32x0xFF - observing MISO samples...\r\n");
    uint8_t txbuf[32];
    for (int i=0;i<32;i++) txbuf[i] = 0xFF;

    /* We'll manually bitbang 32 bytes but sample MISO and print aggregated results */
    uint8_t rxbuf[8];
    memset(rxbuf, 0, sizeof(rxbuf));
    CS_LOW();
    TXEN_HIGH(); /* drive line while sending wake */
    for (int b=0;b<32;b++) {
        uint8_t rxbyte = 0;
        uint8_t tx = txbuf[b];
        for (int bit=7; bit>=0; --bit) {
            if (tx & (1<<bit)) MOSI_HIGH(); else MOSI_LOW();
            delay_short();
            SCK_LOW();
            delay_short();
            SCK_HIGH();
            if (MISO_READ() == GPIO_PIN_SET) rxbyte |= (1<<bit);
            delay_short();
        }
        rxbuf[b & 7] = rxbyte;
        if ((b & 3) == 3) {
            char tmp[80];
            int n = snprintf(tmp, sizeof(tmp), "WOKE BYTES %02d..%02d MISO=%02X BNE=%d\r\n",
                             b-3,b, rxbuf[b & 7], (int)(BNE_READ()==GPIO_PIN_SET));
            HAL_UART_Transmit(&huart2, (uint8_t*)tmp, n, HAL_MAX_DELAY);
        }
    }
    CS_HIGH();
    TXEN_LOW();
    HAL_Delay(120);

    snprintf(msg, sizeof(msg), "POST WAKE: DIS=%d TXEN=%d BNE=%d MISO_sample=%02X %02X %02X %02X\r\n",
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2),
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1),
             (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0),
             rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
    uartSend(msg);
}

/* Loopback check requires you to physically jumper PA7->PA6 */
static int loopback_check_local(void) {
#if !RUN_LOOPBACK_CHECK
    return 0;
#else
    uartSend("Loopback check: short PA7->PA6 with jumper and wait...\r\n");
    HAL_Delay(400);
    uint8_t tx[4] = {0xA5, 0x5A, 0xFF, 0x00};
    uint8_t rx[4] = {0};
    doTransfer:
    /* simple 4 byte transfer using doTransfer approach: CS low, bitbang, CS high */
    CS_LOW();
    for (int b=0;b<4;b++) {
        uint8_t txb = tx[b];
        uint8_t rxb = 0;
        for (int bit=7;bit>=0;--bit) {
            if (txb & (1<<bit)) MOSI_HIGH(); else MOSI_LOW();
            delay_short();
            SCK_LOW();
            delay_short();
            SCK_HIGH();
            if (MISO_READ() == GPIO_PIN_SET) rxb |= (1<<bit);
            delay_short();
        }
        rx[b] = rxb;
    }
    CS_HIGH();
    int ok = 1;
    for (int i=0;i<4;i++) if (rx[i] != tx[i]) ok = 0;
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "LOOPBACK TX: %02X %02X %02X %02X RX: %02X %02X %02X %02X OK=%d\r\n",
                     tx[0],tx[1],tx[2],tx[3], rx[0],rx[1],rx[2],rx[3], ok);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
    return ok ? 0 : -1;
#endif
}

/* ------------------------- main ------------------------- */
int main(void) {
    HAL_Init();
    SystemClock_Config_local();
    initPeripherals_local();

    uartSend("\r\n=== POWBMS L9963 minimal driver (Mode3 bitbang) ===\r\n");

    if (RUN_LOOPBACK_CHECK) {
        if (loopback_check_local() != 0) {
            uartSend("Loopback failed - check PA7->PA6 jumper and wiring/timing.\r\n");
            /* continue anyway to allow manual debug */
        }
    }

    diagnostic_wake_local();

    /* Try a simple DEV_GEN_CFG read (device id 1 example) */
    {
        uint8_t raw5[5];
        build_frame_5(raw5, 1, 0, 1, DEV_GEN_CFG_ADDR, 0);
        /* Transmit */
        spi_transmit_bytes(raw5, 5);
        /* Wait for reply */
        uint8_t out5[5];
        int r = drv_wait_and_receive_5(1, out5, 200);
        if (r == 0) {
            char buf[128];
            int n = snprintf(buf, sizeof(buf), "DEV_GEN_CFG RX raw: %02X %02X %02X %02X %02X\r\n",
                             out5[0],out5[1],out5[2],out5[3],out5[4]);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
        } else {
            if (r == -1) uartSend("DEV_GEN_CFG: timeout waiting for BNE/response\r\n");
            else uartSend("DEV_GEN_CFG: error receiving frame\r\n");
        }
    }

    /* Main loop: poll FSM and read cells periodically */
    int loop = 0;
    for (;;) {
        /* Build FSM read frame device 1 */
        uint8_t txf[5];
        build_frame_5(txf, 1, 0, 1, FSM_ADDR, 0);
        spi_transmit_bytes(txf, 5);

        uint8_t resp[5];
        int rr = drv_wait_and_receive_5(1, resp, 50); /* short wait */
        if (rr == 0) {
            char buf[128];
            int n = snprintf(buf, sizeof(buf), "FSM RX raw: %02X %02X %02X %02X %02X  BNE=%d\r\n",
                             resp[0],resp[1],resp[2],resp[3],resp[4], (int)(BNE_READ()==GPIO_PIN_SET));
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
        } else {
            /* print heartbeat without frame data */
            char buf[64];
            int n = snprintf(buf, sizeof(buf), "FSM: no frame (BNE=%d)\r\n", (int)(BNE_READ()==GPIO_PIN_SET));
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
        }

        if (BNE_READ() == GPIO_PIN_SET) {
            uartSend("BNE asserted - reading frame(s)...\r\n");
            uint8_t frame[5];
            if (drv_wait_and_receive_5(0, frame, 200) == 0) {
                char buf[96];
                int n = snprintf(buf, sizeof(buf), "FRAME RX: %02X %02X %02X %02X %02X\r\n",
                                 frame[0],frame[1],frame[2],frame[3],frame[4]);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
            } else {
                uartSend("FRAME READ ERR or TIMEOUT\r\n");
            }
        }

        /* read battery cell regs every 10 loops */
        if ((loop++ % 10) == 0) {
            uartSend("Reading cell registers (raw) ...\r\n");
            for (uint8_t addr = VCELL1_ADDR; addr <= VCELL_LAST_ADDR; ++addr) {
                uint8_t raw5[5];
                build_frame_5(raw5, 1, 0, 1, addr, 0);
                spi_transmit_bytes(raw5, 5);
                uint8_t rcv[5];
                int r = drv_wait_and_receive_5(1, rcv, 80);
                if (r == 0) {
                    char buf[96];
                    int n = snprintf(buf, sizeof(buf), "VCELL @0x%02X = %02X %02X %02X %02X %02X\r\n",
                                     addr, rcv[0], rcv[1], rcv[2], rcv[3], rcv[4]);
                    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
                } else {
                    char buf[64];
                    int n = snprintf(buf, sizeof(buf), "VCELL@%02X read failed (r=%d)\r\n", addr, r);
                    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
                }
                HAL_Delay(30);
            }
        }

        HAL_Delay(500);
    }
    /* not reached */
    return 0;
}
