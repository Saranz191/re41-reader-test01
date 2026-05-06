#include "protocol_fsm.h"

#include <string.h>

static void ProtocolFsm_Reset(ProtocolFsm_t *fsm)
{
  fsm->state = PROTOCOL_STATE_WAIT_HEADER;
  fsm->length = 0u;
  fsm->payload_index = 0u;
  fsm->lrc = 0u;
  memset(&fsm->packet, 0, sizeof(fsm->packet));
}

void ProtocolFsm_Init(ProtocolFsm_t *fsm)
{
  ProtocolFsm_Reset(fsm);
}

ProtocolEvent_t ProtocolFsm_InputByte(ProtocolFsm_t *fsm, uint8_t byte, ProtocolPacket_t *packet)
{
  switch (fsm->state)
  {
    case PROTOCOL_STATE_WAIT_HEADER:
      if (byte == PROTOCOL_HEADER)
      {
        fsm->state = PROTOCOL_STATE_LEN_MSB;
        fsm->length = 0u;
        fsm->payload_index = 0u;
        fsm->lrc = 0u;
        memset(&fsm->packet, 0, sizeof(fsm->packet));
      }
      break;

    case PROTOCOL_STATE_LEN_MSB:
      fsm->length = ((uint16_t)byte << 8);
      fsm->lrc = byte;
      fsm->state = PROTOCOL_STATE_LEN_LSB;
      break;

    case PROTOCOL_STATE_LEN_LSB:
      fsm->length |= byte;
      fsm->lrc ^= byte;
      if ((fsm->length < PROTOCOL_MIN_PAYLOAD_LEN) || (fsm->length > PROTOCOL_MAX_PAYLOAD_LEN))
      {
        ProtocolFsm_Reset(fsm);
        return PROTOCOL_EVENT_LENGTH_ERROR;
      }
      fsm->packet.data_len = (uint16_t)(fsm->length - PROTOCOL_MIN_PAYLOAD_LEN);
      fsm->state = PROTOCOL_STATE_PAYLOAD;
      break;

    case PROTOCOL_STATE_PAYLOAD:
      fsm->lrc ^= byte;
      if (fsm->payload_index == 0u)
      {
        fsm->packet.category = ((uint16_t)byte << 8);
      }
      else if (fsm->payload_index == 1u)
      {
        fsm->packet.category |= byte;
      }
      else if (fsm->payload_index == 2u)
      {
        fsm->packet.command = ((uint16_t)byte << 8);
      }
      else if (fsm->payload_index == 3u)
      {
        fsm->packet.command |= byte;
      }
      else
      {
        fsm->packet.data[fsm->payload_index - PROTOCOL_MIN_PAYLOAD_LEN] = byte;
      }

      fsm->payload_index++;
      if (fsm->payload_index >= fsm->length)
      {
        fsm->state = PROTOCOL_STATE_LRC;
      }
      break;

    case PROTOCOL_STATE_LRC:
      if (byte == fsm->lrc)
      {
        *packet = fsm->packet;
        ProtocolFsm_Reset(fsm);
        return PROTOCOL_EVENT_PACKET_READY;
      }
      ProtocolFsm_Reset(fsm);
      return PROTOCOL_EVENT_LRC_ERROR;

    default:
      ProtocolFsm_Reset(fsm);
      break;
  }

  return PROTOCOL_EVENT_NONE;
}
