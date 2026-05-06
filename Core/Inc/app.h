#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

void App_Init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
