#ifndef L9963_H
#define L9963_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

/* =======================
   Configuration Defines
   ======================= */

#define FRAME_SIZE              5
#define SPI_TRANSMIT_TIMEOUT    10

/* =======================
   External SPI Handle
   ======================= */

extern SPI_HandleTypeDef hspi3;

/* =======================
   Function Prototypes
   ======================= */

/**
 * @brief Initialize L9963T control pins
 */
void L9963T_Init(void);

/**
 * @brief Send isoSPI wakeup sequence to L9963E
 */
void L9963E_Wakeup(void);

/**
 * @brief Transmit frame via L9963T
 */
void l9963TL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                          uint8_t *pData,
                          uint16_t Size,
                          uint32_t Timeout);

/**
 * @brief Wait for response and receive frame
 */
void L9963TL_SPI_WaitAndRecieve(SPI_HandleTypeDef *hspi,
                                uint8_t *pData,
                                uint16_t Size,
                                uint32_t Timeout);

/**
 * @brief Initialize L9963E chain
 */
void L9963E_Init(uint8_t n_slaves);

#ifdef __cplusplus
}
#endif

#endif /* INC_L9963_H_ */
