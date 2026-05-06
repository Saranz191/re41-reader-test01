#ifndef RE41_SPI_H
#define RE41_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define RE41_REGISTER_ADDRESS_MASK  0x3Fu
#define RE41_SPI_DEBUG_TRACE_SIZE   129u

extern volatile uint8_t g_re41_debug_last_tx[RE41_SPI_DEBUG_TRACE_SIZE];
extern volatile uint8_t g_re41_debug_last_rx[RE41_SPI_DEBUG_TRACE_SIZE];
extern volatile uint16_t g_re41_debug_last_len;
extern volatile uint8_t g_re41_debug_last_status;
extern volatile uint8_t g_re41_debug_last_addr;
extern volatile uint8_t g_re41_debug_last_value;
extern volatile uint32_t g_re41_debug_last_error;
extern volatile uint32_t g_re41_debug_last_sr;

void RE41_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef RE41_SPI_ReadRegister(uint8_t address, uint8_t *value);
HAL_StatusTypeDef RE41_SPI_WriteRegister(uint8_t address, uint8_t value);
HAL_StatusTypeDef RE41_SPI_ReadRegisters(const uint8_t *addresses, uint16_t count, uint8_t *values);
HAL_StatusTypeDef RE41_SPI_WriteBytes(uint8_t address, const uint8_t *data, uint16_t count);
HAL_StatusTypeDef RE41_SPI_TransferRaw(const uint8_t *tx, uint16_t count, uint8_t *rx);
HAL_StatusTypeDef RE41_SPI_Analyze(uint8_t *report, uint16_t report_len);

#ifdef __cplusplus
}
#endif

#endif /* RE41_SPI_H */
