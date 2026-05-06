#include "re41_spi.h"

#include "main.h"

#define RE41_SPI_TIMEOUT_MS  20u
#define RE41_SPI_READ_BIT    0x80u
#define RE41_SPI_NCS_DELAY_CYCLES  8u
#define RE41_CMDREG          0x01u
#define RE41_FIFODATA        0x02u
#define RE41_FIFOLEN         0x04u
#define RE41_IRQEN           0x06u
#define RE41_IRQFLAG         0x07u
#define RE41_CTRLREG         0x09u
#define RE41_IDLED_CMD       0x00u
#define RE41_ALL_IRQ         0x3Fu
#define RE41_LOALERT_IRQ     0x01u
#define RE41_FLUSHFIFO_BIT   0x01u

static SPI_HandleTypeDef *s_re41_hspi;
static uint8_t s_re41_tx_buffer[RE41_SPI_DEBUG_TRACE_SIZE];
static uint8_t s_re41_rx_buffer[RE41_SPI_DEBUG_TRACE_SIZE];

volatile uint8_t g_re41_debug_last_tx[RE41_SPI_DEBUG_TRACE_SIZE];
volatile uint8_t g_re41_debug_last_rx[RE41_SPI_DEBUG_TRACE_SIZE];
volatile uint16_t g_re41_debug_last_len;
volatile uint8_t g_re41_debug_last_status;
volatile uint8_t g_re41_debug_last_addr;
volatile uint8_t g_re41_debug_last_value;
volatile uint32_t g_re41_debug_last_error;
volatile uint32_t g_re41_debug_last_sr;

static uint8_t RE41_SPI_CommandByte(uint8_t address, uint8_t is_read)
{
  uint8_t command = (uint8_t)((address & RE41_REGISTER_ADDRESS_MASK) << 1);

  if (is_read != 0u)
  {
    command |= RE41_SPI_READ_BIT;
  }

  return command;
}

static void RE41_SPI_DelayAfterNcsEdge(void)
{
  volatile uint32_t i;

  for (i = 0u; i < RE41_SPI_NCS_DELAY_CYCLES; i++)
  {
    __NOP();
  }
}

static void RE41_SPI_Select(void)
{
  HAL_GPIO_WritePin(RE41_NCS_GPIO_Port, RE41_NCS_Pin, GPIO_PIN_RESET);
  RE41_SPI_DelayAfterNcsEdge();
}

static void RE41_SPI_Deselect(void)
{
  RE41_SPI_DelayAfterNcsEdge();
  HAL_GPIO_WritePin(RE41_NCS_GPIO_Port, RE41_NCS_Pin, GPIO_PIN_SET);
}

static void RE41_SPI_ClearDebugTrace(void)
{
  uint16_t i;

  for (i = 0u; i < RE41_SPI_DEBUG_TRACE_SIZE; i++)
  {
    g_re41_debug_last_tx[i] = 0u;
    g_re41_debug_last_rx[i] = 0u;
  }
}

static void RE41_SPI_RecordDebugTrace(const uint8_t *tx,
                                      const uint8_t *rx,
                                      uint16_t len,
                                      HAL_StatusTypeDef status,
                                      uint8_t address,
                                      uint8_t value)
{
  uint16_t i;
  uint16_t copy_len = len;

  if (copy_len > RE41_SPI_DEBUG_TRACE_SIZE)
  {
    copy_len = RE41_SPI_DEBUG_TRACE_SIZE;
  }

  RE41_SPI_ClearDebugTrace();
  for (i = 0u; i < copy_len; i++)
  {
    if (tx != NULL)
    {
      g_re41_debug_last_tx[i] = tx[i];
    }

    if (rx != NULL)
    {
      g_re41_debug_last_rx[i] = rx[i];
    }
  }

  g_re41_debug_last_len = copy_len;
  g_re41_debug_last_status = (uint8_t)status;
  g_re41_debug_last_addr = address;
  g_re41_debug_last_value = value;
  g_re41_debug_last_error = (s_re41_hspi != NULL) ? s_re41_hspi->ErrorCode : 0u;
  g_re41_debug_last_sr = (s_re41_hspi != NULL) ? s_re41_hspi->Instance->SR : 0u;
}

static void RE41_SPI_ClearPeripheralRx(void)
{
  if (s_re41_hspi != NULL)
  {
    __HAL_SPI_CLEAR_OVRFLAG(s_re41_hspi);
  }
}

static HAL_StatusTypeDef RE41_SPI_TransferBytes(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  if ((s_re41_hspi == NULL) || (tx == NULL) || (rx == NULL) || (len == 0u))
  {
    return HAL_ERROR;
  }

  s_re41_hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  return HAL_SPI_TransmitReceive(s_re41_hspi, (uint8_t *)tx, rx, len, HAL_MAX_DELAY);
}

void RE41_SPI_Init(SPI_HandleTypeDef *hspi)
{
  s_re41_hspi = hspi;
  RE41_SPI_Deselect();
  HAL_GPIO_WritePin(RE41_PDRST_GPIO_Port, RE41_PDRST_Pin, GPIO_PIN_SET);
  HAL_Delay(10u);
  HAL_GPIO_WritePin(RE41_PDRST_GPIO_Port, RE41_PDRST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10u);
  RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_OK, 0u, 0u);
}

HAL_StatusTypeDef RE41_SPI_ReadRegister(uint8_t address, uint8_t *value)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2] = {0u, 0u};
  uint8_t read_value = 0u;

  if ((s_re41_hspi == NULL) || (value == NULL))
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, address, 0u);
    return HAL_ERROR;
  }

  tx_buffer[0] = RE41_SPI_CommandByte(address, 1u);
  tx_buffer[1] = 0x00u;

  RE41_SPI_ClearPeripheralRx();
  RE41_SPI_Select();
  status = HAL_SPI_TransmitReceive(s_re41_hspi, tx_buffer, rx_buffer, 2u, HAL_MAX_DELAY);
  RE41_SPI_Deselect();

  if (status == HAL_OK)
  {
    read_value = rx_buffer[1];
    *value = read_value;
  }

  RE41_SPI_RecordDebugTrace(tx_buffer, rx_buffer, 2u, status, address, read_value);
  return status;
}

HAL_StatusTypeDef RE41_SPI_WriteRegister(uint8_t address, uint8_t value)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2] = {0u, 0u};

  if (s_re41_hspi == NULL)
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, address, value);
    return HAL_ERROR;
  }

  tx_buffer[0] = RE41_SPI_CommandByte(address, 0u);
  tx_buffer[1] = value;

  RE41_SPI_ClearPeripheralRx();
  RE41_SPI_Select();
  status = HAL_SPI_TransmitReceive(s_re41_hspi, tx_buffer, rx_buffer, 2u, HAL_MAX_DELAY);
  RE41_SPI_Deselect();

  RE41_SPI_RecordDebugTrace(tx_buffer, rx_buffer, 2u, status, address, value);
  return status;
}

HAL_StatusTypeDef RE41_SPI_ReadRegisters(const uint8_t *addresses, uint16_t count, uint8_t *values)
{
  HAL_StatusTypeDef status;
  uint16_t i;
  uint16_t transfer_len;
  uint8_t debug_value = 0u;

  if ((s_re41_hspi == NULL) || (addresses == NULL) || (values == NULL) ||
      (count == 0u) || (count >= RE41_SPI_DEBUG_TRACE_SIZE))
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, 0u, 0u);
    return HAL_ERROR;
  }

  for (i = 0u; i < count; i++)
  {
    s_re41_tx_buffer[i] = RE41_SPI_CommandByte(addresses[i], 1u);
    s_re41_rx_buffer[i] = 0u;
  }
  s_re41_tx_buffer[count] = 0x00u;
  s_re41_rx_buffer[count] = 0u;
  transfer_len = (uint16_t)(count + 1u);

  RE41_SPI_ClearPeripheralRx();
  RE41_SPI_Select();
  status = RE41_SPI_TransferBytes(s_re41_tx_buffer, s_re41_rx_buffer, transfer_len);
  RE41_SPI_Deselect();

  if (status == HAL_OK)
  {
    for (i = 0u; i < count; i++)
    {
      values[i] = s_re41_rx_buffer[i + 1u];
    }
    debug_value = values[0];
  }

  RE41_SPI_RecordDebugTrace(s_re41_tx_buffer,
                            s_re41_rx_buffer,
                            transfer_len,
                            status,
                            addresses[0],
                            debug_value);
  return status;
}

HAL_StatusTypeDef RE41_SPI_WriteBytes(uint8_t address, const uint8_t *data, uint16_t count)
{
  HAL_StatusTypeDef status;
  uint16_t i;
  uint16_t transfer_len;

  if ((s_re41_hspi == NULL) || (data == NULL) ||
      (count == 0u) || (count >= RE41_SPI_DEBUG_TRACE_SIZE))
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, address, 0u);
    return HAL_ERROR;
  }

  s_re41_tx_buffer[0] = RE41_SPI_CommandByte(address, 0u);
  s_re41_rx_buffer[0] = 0u;
  for (i = 0u; i < count; i++)
  {
    s_re41_tx_buffer[i + 1u] = data[i];
    s_re41_rx_buffer[i + 1u] = 0u;
  }
  transfer_len = (uint16_t)(count + 1u);

  RE41_SPI_ClearPeripheralRx();
  RE41_SPI_Select();
  status = RE41_SPI_TransferBytes(s_re41_tx_buffer, s_re41_rx_buffer, transfer_len);
  RE41_SPI_Deselect();

  RE41_SPI_RecordDebugTrace(s_re41_tx_buffer,
                            s_re41_rx_buffer,
                            transfer_len,
                            status,
                            address,
                            data[0]);
  return status;
}

HAL_StatusTypeDef RE41_SPI_TransferRaw(const uint8_t *tx, uint16_t count, uint8_t *rx)
{
  HAL_StatusTypeDef status;
  uint16_t i;
  uint8_t debug_addr = 0u;
  uint8_t debug_value = 0u;

  if ((s_re41_hspi == NULL) || (tx == NULL) || (rx == NULL) ||
      (count == 0u) || (count > RE41_SPI_DEBUG_TRACE_SIZE))
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, 0u, 0u);
    return HAL_ERROR;
  }

  for (i = 0u; i < count; i++)
  {
    s_re41_tx_buffer[i] = tx[i];
    s_re41_rx_buffer[i] = 0u;
  }

  RE41_SPI_ClearPeripheralRx();
  RE41_SPI_Select();
  status = RE41_SPI_TransferBytes(s_re41_tx_buffer, s_re41_rx_buffer, count);
  RE41_SPI_Deselect();

  if (status == HAL_OK)
  {
    for (i = 0u; i < count; i++)
    {
      rx[i] = s_re41_rx_buffer[i];
    }
    debug_addr = s_re41_tx_buffer[0];
    debug_value = s_re41_rx_buffer[count - 1u];
  }

  RE41_SPI_RecordDebugTrace(s_re41_tx_buffer,
                            s_re41_rx_buffer,
                            count,
                            status,
                            debug_addr,
                            debug_value);
  return status;
}

HAL_StatusTypeDef RE41_SPI_Analyze(uint8_t *report, uint16_t report_len)
{
  HAL_StatusTypeDef status;
  uint8_t reg_value = 0u;
  uint8_t fifo_len_before = 0u;
  uint8_t fifo_len_after = 0u;
  uint8_t fifo_data = 0u;
  uint8_t irq_flag = 0u;

  if ((report == NULL) || (report_len < 4u))
  {
    RE41_SPI_RecordDebugTrace(NULL, NULL, 0u, HAL_ERROR, 0u, 0u);
    return HAL_ERROR;
  }

  status = RE41_SPI_WriteRegister(RE41_IRQEN, RE41_ALL_IRQ);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_WriteRegister(RE41_IRQFLAG, RE41_ALL_IRQ);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_WriteRegister(RE41_CMDREG, RE41_IDLED_CMD);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_ReadRegister(RE41_CTRLREG, &reg_value);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_WriteRegister(RE41_CTRLREG, (uint8_t)(reg_value | RE41_FLUSHFIFO_BIT));
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_ReadRegister(RE41_FIFOLEN, &fifo_len_before);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_WriteRegister(RE41_FIFODATA, 0xAAu);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_ReadRegister(RE41_FIFOLEN, &fifo_len_after);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_ReadRegister(RE41_FIFODATA, &fifo_data);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RE41_SPI_ReadRegister(RE41_IRQFLAG, &irq_flag);
  if (status != HAL_OK)
  {
    return status;
  }

  report[0] = fifo_len_before;
  report[1] = fifo_len_after;
  report[2] = fifo_data;
  report[3] = (uint8_t)(irq_flag & RE41_LOALERT_IRQ);
  return HAL_OK;
}
