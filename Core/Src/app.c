#include "app.h"

#include "protocol_fsm.h"
#include "re41_spi.h"

#include <string.h>

#define APP_UART_RX_BUFFER_SIZE          256u
#define APP_UART_IDLE_RX_BUFFER_SIZE     256u
#define APP_UART_TX_MAX_DATA_LEN         (PROTOCOL_MAX_DATA_LEN + 1u)
#define APP_UART_TX_MAX_FRAME_LEN        (1u + 2u + PROTOCOL_MIN_PAYLOAD_LEN + APP_UART_TX_MAX_DATA_LEN + 1u)
#define APP_UART_TX_TIMEOUT_MS           100u
#define APP_DEBUG_RX_BYTE_DELAY_MS       0u
#define APP_DEBUG_RX_TRACE_SIZE          32u

#define APP_CATEGORY_RE41                0x0001u
#define APP_CATEGORY_PROTOCOL_ERROR      0xFFFFu

#define APP_CMD_RE41_READ_REG            0x0001u
#define APP_CMD_RE41_WRITE_REG           0x0002u
#define APP_CMD_RE41_READ_REG_LIST       0x0003u
#define APP_CMD_RE41_WRITE_BYTES         0x0004u
#define APP_CMD_RE41_SPI_RAW             0x0005u
#define APP_CMD_RE41_ANALYZE             0x0006u
#define APP_CMD_PROTOCOL_ERROR           0x0001u

#define APP_STATUS_OK                    0x00u
#define APP_STATUS_INVALID_LENGTH        0x01u
#define APP_STATUS_UNSUPPORTED_CATEGORY  0x02u
#define APP_STATUS_UNSUPPORTED_COMMAND   0x03u
#define APP_STATUS_SPI_ERROR             0x04u
#define APP_STATUS_UART_RX_OVERFLOW      0x05u
#define APP_STATUS_LRC_ERROR             0x06u

typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_DISPATCH,
  APP_STATE_REPLY
} AppState_t;

static UART_HandleTypeDef *s_huart;
static ProtocolFsm_t s_protocol_fsm;
static ProtocolPacket_t s_request;
static AppState_t s_state = APP_STATE_IDLE;

static uint8_t s_uart_idle_rx_buffer[APP_UART_IDLE_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_buffer[APP_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint8_t s_rx_overflow;

static uint8_t s_response_data[APP_UART_TX_MAX_DATA_LEN];
static uint16_t s_response_category;
static uint16_t s_response_command;
static uint16_t s_response_data_len;

volatile uint8_t g_app_debug_rx_trace[APP_DEBUG_RX_TRACE_SIZE];
volatile uint8_t g_app_debug_rx_trace_index;
volatile uint8_t g_app_debug_last_rx_byte;
volatile uint8_t g_app_debug_last_processed_byte;
volatile uint8_t g_app_debug_last_event;
volatile uint8_t g_app_debug_last_state;
volatile uint16_t g_app_debug_last_length;
volatile uint8_t g_app_debug_last_lrc;
volatile uint16_t g_app_debug_packet_ready_count;
volatile uint16_t g_app_debug_lrc_error_count;
volatile uint16_t g_app_debug_length_error_count;
volatile uint16_t g_app_debug_irq_rx_count;
volatile uint16_t g_app_debug_uart_error_count;
volatile uint16_t g_app_debug_rx_overflow_count;
volatile uint16_t g_app_debug_rx_head;
volatile uint16_t g_app_debug_rx_tail;
volatile uint8_t g_app_debug_tx_frame[APP_UART_TX_MAX_FRAME_LEN];
volatile uint16_t g_app_debug_tx_len;
volatile uint8_t g_app_debug_tx_status;
volatile uint16_t g_app_debug_tx_count;

static void App_RecordRxByteFromIsr(uint8_t byte)
{
  g_app_debug_last_rx_byte = byte;
  g_app_debug_rx_trace[g_app_debug_rx_trace_index] = byte;
  g_app_debug_rx_trace_index = (uint8_t)((g_app_debug_rx_trace_index + 1u) % APP_DEBUG_RX_TRACE_SIZE);
  g_app_debug_irq_rx_count++;
}

static void App_StartUartReceiveToIdle(void)
{
  uint8_t dummy;

  if (s_huart != NULL)
  {
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE) != 0u)
    {
      dummy = (uint8_t)(s_huart->Instance->RDR & 0xFFu);
      (void)dummy;
    }

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    (void)HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_uart_idle_rx_buffer, APP_UART_IDLE_RX_BUFFER_SIZE);
  }
}

static uint8_t App_RxBufferPushFromIsr(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((s_rx_head + 1u) % APP_UART_RX_BUFFER_SIZE);

  if (next_head == s_rx_tail)
  {
    s_rx_overflow = 1u;
    g_app_debug_rx_overflow_count++;
    return 0u;
  }

  s_rx_buffer[s_rx_head] = byte;
  s_rx_head = next_head;
  g_app_debug_rx_head = s_rx_head;
  return 1u;
}

static uint8_t App_RxBufferPop(uint8_t *byte)
{
  if (s_rx_head == s_rx_tail)
  {
    return 0u;
  }

  *byte = s_rx_buffer[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) % APP_UART_RX_BUFFER_SIZE);
  g_app_debug_rx_tail = s_rx_tail;
  return 1u;
}

static void App_SetResponse(uint16_t category, uint16_t command, const uint8_t *data, uint16_t data_len)
{
  s_response_category = category;
  s_response_command = command;
  s_response_data_len = data_len;

  if ((data != NULL) && (data != s_response_data) && (data_len > 0u))
  {
    memcpy(s_response_data, data, data_len);
  }
}

static void App_SetStatusResponse(const ProtocolPacket_t *request, uint8_t status)
{
  s_response_data[0] = status;
  App_SetResponse(request->category, request->command, s_response_data, 1u);
}

static void App_SetProtocolErrorResponse(uint8_t status)
{
  s_response_data[0] = status;
  App_SetResponse(APP_CATEGORY_PROTOCOL_ERROR, APP_CMD_PROTOCOL_ERROR, s_response_data, 1u);
}

static void App_TransmitResponse(void)
{
  uint8_t frame[APP_UART_TX_MAX_FRAME_LEN];
  uint16_t payload_len = (uint16_t)(PROTOCOL_MIN_PAYLOAD_LEN + s_response_data_len);
  uint16_t index = 0u;
  uint8_t lrc;
  uint16_t i;
  HAL_StatusTypeDef tx_status;

  if ((s_huart == NULL) || (s_response_data_len > APP_UART_TX_MAX_DATA_LEN))
  {
    return;
  }

  frame[index++] = PROTOCOL_HEADER;
  frame[index++] = (uint8_t)(payload_len >> 8);
  frame[index++] = (uint8_t)(payload_len & 0xFFu);
  frame[index++] = (uint8_t)(s_response_category >> 8);
  frame[index++] = (uint8_t)(s_response_category & 0xFFu);
  frame[index++] = (uint8_t)(s_response_command >> 8);
  frame[index++] = (uint8_t)(s_response_command & 0xFFu);

  for (i = 0u; i < s_response_data_len; i++)
  {
    frame[index++] = s_response_data[i];
  }

  lrc = frame[1];
  for (i = 2u; i < index; i++)
  {
    lrc ^= frame[i];
  }
  frame[index++] = lrc;

  for (i = 0u; i < index; i++)
  {
    g_app_debug_tx_frame[i] = frame[i];
  }
  g_app_debug_tx_len = index;

  tx_status = HAL_UART_Transmit(s_huart, frame, index, APP_UART_TX_TIMEOUT_MS);
  g_app_debug_tx_status = (uint8_t)tx_status;
  if (tx_status == HAL_OK)
  {
    g_app_debug_tx_count++;
  }
}

static void App_HandleRe41Command(const ProtocolPacket_t *request)
{
  HAL_StatusTypeDef spi_status = HAL_OK;
  uint8_t value = 0u;

  switch (request->command)
  {
    case APP_CMD_RE41_READ_REG:
      if (request->data_len != 1u)
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      spi_status = RE41_SPI_ReadRegister(request->data[0], &value);
      s_response_data[0] = (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR;
      s_response_data[1] = value;
      App_SetResponse(request->category, request->command, s_response_data, 2u);
      break;

    case APP_CMD_RE41_WRITE_REG:
      if (request->data_len != 2u)
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      spi_status = RE41_SPI_WriteRegister(request->data[0], request->data[1]);
      App_SetStatusResponse(request, (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR);
      break;

    case APP_CMD_RE41_READ_REG_LIST:
      if ((request->data_len == 0u) || (request->data_len > PROTOCOL_MAX_DATA_LEN))
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      spi_status = RE41_SPI_ReadRegisters(request->data, request->data_len, &s_response_data[1]);
      s_response_data[0] = (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR;
      App_SetResponse(request->category, request->command, s_response_data, (uint16_t)(request->data_len + 1u));
      break;

    case APP_CMD_RE41_WRITE_BYTES:
      if (request->data_len < 2u)
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      spi_status = RE41_SPI_WriteBytes(request->data[0], &request->data[1], (uint16_t)(request->data_len - 1u));
      App_SetStatusResponse(request, (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR);
      break;

    case APP_CMD_RE41_SPI_RAW:
      if ((request->data_len == 0u) || (request->data_len > PROTOCOL_MAX_DATA_LEN))
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      spi_status = RE41_SPI_TransferRaw(request->data, request->data_len, &s_response_data[1]);
      s_response_data[0] = (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR;
      App_SetResponse(request->category, request->command, s_response_data, (uint16_t)(request->data_len + 1u));
      break;

    case APP_CMD_RE41_ANALYZE:
      if (request->data_len != 0u)
      {
        App_SetStatusResponse(request, APP_STATUS_INVALID_LENGTH);
        break;
      }

      s_response_data[1] = 0u;
      s_response_data[2] = 0u;
      s_response_data[3] = 0u;
      s_response_data[4] = 0u;
      spi_status = RE41_SPI_Analyze(&s_response_data[1], 4u);
      s_response_data[0] = (spi_status == HAL_OK) ? APP_STATUS_OK : APP_STATUS_SPI_ERROR;
      App_SetResponse(request->category, request->command, s_response_data, 5u);
      break;

    default:
      App_SetStatusResponse(request, APP_STATUS_UNSUPPORTED_COMMAND);
      break;
  }
}

static void App_RunStateMachine(void)
{
  switch (s_state)
  {
    case APP_STATE_DISPATCH:
      if (s_request.category == APP_CATEGORY_RE41)
      {
        App_HandleRe41Command(&s_request);
      }
      else
      {
        App_SetStatusResponse(&s_request, APP_STATUS_UNSUPPORTED_CATEGORY);
      }
      s_state = APP_STATE_REPLY;
      break;

    case APP_STATE_REPLY:
      App_TransmitResponse();
      s_state = APP_STATE_IDLE;
      break;

    case APP_STATE_IDLE:
    default:
      break;
  }
}

static void App_RunStateMachineToIdle(void)
{
  uint8_t guard = 0u;

  while ((s_state != APP_STATE_IDLE) && (guard < 3u))
  {
    App_RunStateMachine();
    guard++;
  }
}

void App_Init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi)
{
  s_huart = huart;
  s_rx_head = 0u;
  s_rx_tail = 0u;
  s_rx_overflow = 0u;
  s_state = APP_STATE_IDLE;
  ProtocolFsm_Init(&s_protocol_fsm);
  RE41_SPI_Init(hspi);
  App_StartUartReceiveToIdle();
}

void App_Process(void)
{
  uint8_t byte;
  ProtocolPacket_t packet;
  ProtocolEvent_t event;

  if (s_rx_overflow != 0u)
  {
    s_rx_overflow = 0u;
    App_SetProtocolErrorResponse(APP_STATUS_UART_RX_OVERFLOW);
    s_state = APP_STATE_REPLY;
    App_RunStateMachineToIdle();
  }

  while (App_RxBufferPop(&byte) != 0u)
  {
    if (APP_DEBUG_RX_BYTE_DELAY_MS > 0u)
    {
      HAL_Delay(APP_DEBUG_RX_BYTE_DELAY_MS);
    }

    event = ProtocolFsm_InputByte(&s_protocol_fsm, byte, &packet);
    g_app_debug_last_processed_byte = byte;
    g_app_debug_last_event = (uint8_t)event;
    g_app_debug_last_state = (uint8_t)s_protocol_fsm.state;
    g_app_debug_last_length = s_protocol_fsm.length;
    g_app_debug_last_lrc = s_protocol_fsm.lrc;

    if (event == PROTOCOL_EVENT_PACKET_READY)
    {
      g_app_debug_packet_ready_count++;
      s_request = packet;
      s_state = APP_STATE_DISPATCH;
      App_RunStateMachineToIdle();
    }
    else if (event == PROTOCOL_EVENT_LENGTH_ERROR)
    {
      g_app_debug_length_error_count++;
      App_SetProtocolErrorResponse(APP_STATUS_INVALID_LENGTH);
      s_state = APP_STATE_REPLY;
      App_RunStateMachineToIdle();
    }
    else if (event == PROTOCOL_EVENT_LRC_ERROR)
    {
      g_app_debug_lrc_error_count++;
      App_SetProtocolErrorResponse(APP_STATUS_LRC_ERROR);
      s_state = APP_STATE_REPLY;
      App_RunStateMachineToIdle();
    }
    else
    {
      /* No complete frame yet. */
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if ((huart == NULL) || (huart != s_huart))
  {
    return;
  }

  for (i = 0u; i < Size; i++)
  {
    App_RecordRxByteFromIsr(s_uart_idle_rx_buffer[i]);
    (void)App_RxBufferPushFromIsr(s_uart_idle_rx_buffer[i]);
  }

  App_StartUartReceiveToIdle();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_huart)
  {
    g_app_debug_uart_error_count++;
    App_StartUartReceiveToIdle();
  }
}
