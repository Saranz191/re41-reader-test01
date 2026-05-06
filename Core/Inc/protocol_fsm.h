#ifndef PROTOCOL_FSM_H
#define PROTOCOL_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PROTOCOL_HEADER              0xAAu
#define PROTOCOL_MIN_PAYLOAD_LEN     4u
#define PROTOCOL_MAX_DATA_LEN        128u
#define PROTOCOL_MAX_PAYLOAD_LEN     (PROTOCOL_MIN_PAYLOAD_LEN + PROTOCOL_MAX_DATA_LEN)

typedef enum
{
  PROTOCOL_EVENT_NONE = 0,
  PROTOCOL_EVENT_PACKET_READY,
  PROTOCOL_EVENT_LENGTH_ERROR,
  PROTOCOL_EVENT_LRC_ERROR
} ProtocolEvent_t;

typedef struct
{
  uint16_t category;
  uint16_t command;
  uint16_t data_len;
  uint8_t data[PROTOCOL_MAX_DATA_LEN];
} ProtocolPacket_t;

typedef enum
{
  PROTOCOL_STATE_WAIT_HEADER = 0,
  PROTOCOL_STATE_LEN_MSB,
  PROTOCOL_STATE_LEN_LSB,
  PROTOCOL_STATE_PAYLOAD,
  PROTOCOL_STATE_LRC
} ProtocolState_t;

typedef struct
{
  ProtocolState_t state;
  uint16_t length;
  uint16_t payload_index;
  uint8_t lrc;
  ProtocolPacket_t packet;
} ProtocolFsm_t;

void ProtocolFsm_Init(ProtocolFsm_t *fsm);
ProtocolEvent_t ProtocolFsm_InputByte(ProtocolFsm_t *fsm, uint8_t byte, ProtocolPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_FSM_H */
