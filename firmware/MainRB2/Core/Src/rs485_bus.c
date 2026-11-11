/**
  ******************************************************************************
  * @file    rs485_bus.c
  * @brief   Portable half-duplex RS-485 framed binary protocol, circular DMA RX
  *          parser, TC-gated RE/DE transceiver direction control, UART error
  *          recovery, and multi-wheel command / telemetry TDMA helpers.
  ******************************************************************************
  */

#include "rs485_bus.h"
#include <string.h>

static void rs485_set_dir_rx(const RS485_Bus_t *bus)
{
  if ((bus != NULL) && (bus->re_de_port != NULL))
  {
    HAL_GPIO_WritePin(bus->re_de_port, bus->re_de_pin, GPIO_PIN_RESET);
  }
}

static void rs485_set_dir_tx(const RS485_Bus_t *bus)
{
  if ((bus != NULL) && (bus->re_de_port != NULL))
  {
    HAL_GPIO_WritePin(bus->re_de_port, bus->re_de_pin, GPIO_PIN_SET);
  }
}

static void rs485_uart_flush_rx(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }
#if defined(UNIT_TEST_HOST)
  __HAL_UART_FLUSH_DRREGISTER(huart);
#elif defined(STM32F405xx) || defined(STM32F4)
  (void)__HAL_UART_FLUSH_DRREGISTER(huart);
#else
  __HAL_UART_FLUSH_DRREGISTER(huart);
#endif
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
}

static uint16_t rs485_read_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

static uint32_t rs485_read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8U) |
         ((uint32_t)p[2] << 16U) |
         ((uint32_t)p[3] << 24U);
}

static void rs485_write_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8U) & 0xFFU);
}

static void rs485_write_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8U) & 0xFFU);
  p[2] = (uint8_t)((v >> 16U) & 0xFFU);
  p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

uint16_t RS485_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  if (data == NULL)
  {
    return crc;
  }

  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8U;
    for (uint8_t b = 0U; b < 8U; b++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1U);
      }
    }
  }
  return crc;
}

static bool rs485_is_src_allowed(const RS485_Bus_t *bus, uint8_t src_id)
{
  if (src_id == bus->local_id)
  {
    return false;
  }

  if (bus->local_id == RS485_ADDR_MASTER)
  {
    return (src_id >= RS485_ADDR_SLAVE_MIN) && (src_id <= RS485_ADDR_SLAVE_MAX);
  }

  return (src_id == RS485_ADDR_MASTER);
}

static void rs485_dispatch_valid_frame(RS485_Bus_t *bus, const uint8_t *buf, uint32_t now_ms)
{
  uint8_t version  = buf[2];
  uint8_t src_id   = buf[3];
  uint8_t dst_id   = buf[4];
  uint8_t msg_type = buf[5];
  uint8_t seq      = buf[6];
  uint8_t len      = buf[7];

  if ((version != RS485_PROTO_VERSION) || (len > RS485_MAX_PAYLOAD))
  {
    return;
  }

  if (!rs485_is_src_allowed(bus, src_id))
  {
    bus->stats.rejected_src_frames++;
    return;
  }

  if ((dst_id != bus->local_id) && (dst_id != RS485_ADDR_BCAST))
  {
    bus->stats.rejected_dst_frames++;
    return;
  }

  bus->last_rx.version  = version;
  bus->last_rx.src_id   = src_id;
  bus->last_rx.dst_id   = dst_id;
  bus->last_rx.msg_type = msg_type;
  bus->last_rx.seq      = seq;
  bus->last_rx.len      = len;
  if (len > 0U)
  {
    memcpy(bus->last_rx.payload, &buf[RS485_HEADER_SIZE], len);
  }
  if (len < RS485_MAX_PAYLOAD)
  {
    memset(&bus->last_rx.payload[len], 0, RS485_MAX_PAYLOAD - len);
  }

  bus->rx_updated = 1U;
  bus->has_ever_rx = 1U;
  bus->last_rx_tick_ms = now_ms;
  bus->stats.rx_ok_frames++;

  if (bus->rx_callback != NULL)
  {
    bus->rx_callback(bus, &bus->last_rx, bus->rx_callback_ctx);
  }
}

static void rs485_resync_from_frame_buf(RS485_Bus_t *bus, uint32_t now_ms)
{
  uint8_t saved[RS485_MAX_FRAME_SIZE];
  uint8_t avail = bus->frame_pos;
  uint8_t s = 1U;

  if (avail <= 1U)
  {
    bus->frame_pos = 0U;
    return;
  }

  memcpy(saved, bus->frame_buf, avail);
  bus->frame_pos = 0U;

  while (s < avail)
  {
    if (saved[s] != RS485_SOF0)
    {
      s++;
      continue;
    }

    uint8_t rem = (uint8_t)(avail - s);
    if (rem == 1U)
    {
      bus->frame_buf[0] = RS485_SOF0;
      bus->frame_pos = 1U;
      return;
    }

    if (saved[s + 1U] != RS485_SOF1)
    {
      s++;
      continue;
    }

    if (rem == 2U)
    {
      bus->frame_buf[0] = RS485_SOF0;
      bus->frame_buf[1] = RS485_SOF1;
      bus->frame_pos = 2U;
      return;
    }

    if (saved[s + 2U] != RS485_PROTO_VERSION)
    {
      s++;
      continue;
    }

    if (rem < RS485_HEADER_SIZE)
    {
      memcpy(bus->frame_buf, &saved[s], rem);
      bus->frame_pos = rem;
      return;
    }

    uint8_t plen = saved[s + 7U];
    if (plen > RS485_MAX_PAYLOAD)
    {
      bus->stats.len_err_frames++;
      s++;
      continue;
    }

    uint8_t total = (uint8_t)(RS485_HEADER_SIZE + plen + RS485_CRC_SIZE);
    if (rem < total)
    {
      memcpy(bus->frame_buf, &saved[s], rem);
      bus->frame_pos = rem;
      return;
    }

    uint16_t crc_calc = RS485_Crc16Ccitt(&saved[s + 2U], (uint16_t)(6U + plen));
    uint16_t crc_rx = ((uint16_t)saved[s + 8U + plen] << 8U) |
                      (uint16_t)saved[s + 9U + plen];

    if (crc_calc == crc_rx)
    {
      rs485_dispatch_valid_frame(bus, &saved[s], now_ms);
      s = (uint8_t)(s + total);
    }
    else
    {
      bus->stats.crc_err_frames++;
      s++;
    }
  }
}

static void rs485_feed_byte(RS485_Bus_t *bus, uint8_t byte, uint32_t now_ms)
{
  uint8_t pos = bus->frame_pos;

  if (pos == 0U)
  {
    if (byte == RS485_SOF0)
    {
      bus->frame_buf[0] = byte;
      bus->frame_pos = 1U;
    }
    return;
  }

  if (pos == 1U)
  {
    if (byte == RS485_SOF1)
    {
      bus->frame_buf[1] = byte;
      bus->frame_pos = 2U;
    }
    else if (byte == RS485_SOF0)
    {
      bus->frame_pos = 1U;
    }
    else
    {
      bus->frame_pos = 0U;
    }
    return;
  }

  if (pos >= RS485_MAX_FRAME_SIZE)
  {
    bus->frame_pos = 0U;
    return;
  }

  bus->frame_buf[pos] = byte;
  bus->frame_pos = (uint8_t)(pos + 1U);

  if (pos == 2U)
  {
    if (byte != RS485_PROTO_VERSION)
    {
      rs485_resync_from_frame_buf(bus, now_ms);
    }
    return;
  }

  if (pos == 7U)
  {
    if (byte > RS485_MAX_PAYLOAD)
    {
      bus->stats.len_err_frames++;
      rs485_resync_from_frame_buf(bus, now_ms);
    }
    return;
  }

  if (bus->frame_pos >= RS485_HEADER_SIZE)
  {
    uint8_t plen = bus->frame_buf[7];
    uint8_t total = (uint8_t)(RS485_HEADER_SIZE + plen + RS485_CRC_SIZE);
    if (bus->frame_pos == total)
    {
      uint16_t crc_calc = RS485_Crc16Ccitt(&bus->frame_buf[2], (uint16_t)(6U + plen));
      uint16_t crc_rx = ((uint16_t)bus->frame_buf[8U + plen] << 8U) |
                        (uint16_t)bus->frame_buf[9U + plen];

      if (crc_calc == crc_rx)
      {
        bus->frame_pos = 0U;
        rs485_dispatch_valid_frame(bus, bus->frame_buf, now_ms);
      }
      else
      {
        bus->stats.crc_err_frames++;
        rs485_resync_from_frame_buf(bus, now_ms);
      }
    }
  }
}

static uint16_t rs485_encode_raw(uint8_t *out,
                                 uint8_t src_id,
                                 uint8_t dst_id,
                                 uint8_t msg_type,
                                 uint8_t seq,
                                 const uint8_t *payload,
                                 uint8_t len)
{
  out[0] = RS485_SOF0;
  out[1] = RS485_SOF1;
  out[2] = RS485_PROTO_VERSION;
  out[3] = src_id;
  out[4] = dst_id;
  out[5] = msg_type;
  out[6] = seq;
  out[7] = len;

  if (len > 0U)
  {
    memcpy(&out[RS485_HEADER_SIZE], payload, len);
  }

  uint16_t crc = RS485_Crc16Ccitt(&out[2], (uint16_t)(6U + len));
  out[8U + len] = (uint8_t)(crc >> 8U);
  out[9U + len] = (uint8_t)(crc & 0xFFU);

  return (uint16_t)(RS485_HEADER_SIZE + len + RS485_CRC_SIZE);
}

static HAL_StatusTypeDef rs485_transmit_raw(RS485_Bus_t *bus, uint16_t total_len, uint32_t now_ms)
{
  if ((bus == NULL) || (bus->huart == NULL) || (total_len == 0U) || (total_len > RS485_MAX_FRAME_SIZE))
  {
    return HAL_ERROR;
  }

  if ((bus->tx_active != 0U) || (bus->huart->gState != HAL_UART_STATE_READY))
  {
    bus->stats.tx_busy_drops++;
    return HAL_BUSY;
  }

  rs485_set_dir_tx(bus);

  HAL_StatusTypeDef st;
  if (bus->huart->hdmatx != NULL)
  {
    bus->tx_active = 1U;
    bus->tx_start_ms = now_ms;
    st = HAL_UART_Transmit_DMA(bus->huart, bus->tx_buf, total_len);
    if (st != HAL_OK)
    {
      bus->tx_active = 0U;
      rs485_set_dir_rx(bus);
    }
  }
  else
  {
    st = HAL_UART_Transmit(bus->huart, bus->tx_buf, total_len, 25U);
    rs485_set_dir_rx(bus);
  }

  if (st == HAL_OK)
  {
    bus->stats.tx_ok_frames++;
  }
  else if (st == HAL_BUSY)
  {
    bus->stats.tx_busy_drops++;
  }
  return st;
}

void RS485_Init(RS485_Bus_t *bus,
                UART_HandleTypeDef *huart,
                GPIO_TypeDef *re_de_port,
                uint16_t re_de_pin,
                uint8_t local_id)
{
  if (bus == NULL)
  {
    return;
  }

  memset(bus, 0, sizeof(*bus));
  bus->huart = huart;
  bus->re_de_port = re_de_port;
  bus->re_de_pin = re_de_pin;
  bus->local_id = local_id;
  rs485_set_dir_rx(bus);
}

void RS485_SetLocalId(RS485_Bus_t *bus, uint8_t local_id)
{
  if (bus != NULL)
  {
    bus->local_id = local_id;
  }
}

void RS485_SetRxCallback(RS485_Bus_t *bus, RS485_RxCallback_t cb, void *ctx)
{
  if (bus == NULL)
  {
    return;
  }
  bus->rx_callback = cb;
  bus->rx_callback_ctx = ctx;
}

HAL_StatusTypeDef RS485_StartRx(RS485_Bus_t *bus)
{
  if ((bus == NULL) || (bus->huart == NULL))
  {
    return HAL_ERROR;
  }

  bus->rx_read_pos = 0U;
  bus->frame_pos = 0U;
  memset(bus->rx_dma_buf, 0, sizeof(bus->rx_dma_buf));
  if (bus->tx_active == 0U)
  {
    rs485_set_dir_rx(bus);
  }

  if (bus->huart->hdmarx == NULL)
  {
    return HAL_OK;
  }

  (void)HAL_UART_AbortReceive(bus->huart);
  rs485_uart_flush_rx(bus->huart);
  return HAL_UART_Receive_DMA(bus->huart, bus->rx_dma_buf, RS485_RX_BUF_SIZE);
}

void RS485_FeedBytes(RS485_Bus_t *bus, const uint8_t *data, uint16_t len, uint32_t now_ms)
{
  if ((bus == NULL) || (data == NULL))
  {
    return;
  }
  for (uint16_t i = 0U; i < len; i++)
  {
    rs485_feed_byte(bus, data[i], now_ms);
  }
}

void RS485_ProcessRx(RS485_Bus_t *bus, uint32_t now_ms)
{
  if ((bus == NULL) || (bus->huart == NULL) || (bus->huart->hdmarx == NULL))
  {
    return;
  }

  if (bus->huart->RxState == HAL_UART_STATE_READY)
  {
    (void)RS485_StartRx(bus);
    return;
  }

  uint32_t remaining = __HAL_DMA_GET_COUNTER(bus->huart->hdmarx);
  if (remaining > RS485_RX_BUF_SIZE)
  {
    remaining = RS485_RX_BUF_SIZE;
  }

  uint16_t write_pos = (uint16_t)(RS485_RX_BUF_SIZE - remaining);
  if (write_pos >= RS485_RX_BUF_SIZE)
  {
    write_pos = 0U;
  }

  while (bus->rx_read_pos != write_pos)
  {
    uint8_t byte = bus->rx_dma_buf[bus->rx_read_pos];
    bus->rx_read_pos++;
    if (bus->rx_read_pos >= RS485_RX_BUF_SIZE)
    {
      bus->rx_read_pos = 0U;
    }
    rs485_feed_byte(bus, byte, now_ms);
  }
}

void RS485_TickMs(RS485_Bus_t *bus, uint32_t now_ms)
{
  if ((bus == NULL) || (bus->huart == NULL))
  {
    return;
  }

  /* Watchdog so RE_DE can never stay wedged TX-High on a missed UART TC IRQ */
  if (bus->tx_active != 0U)
  {
    uint32_t elapsed = (uint32_t)(now_ms - bus->tx_start_ms);
    if ((elapsed >= RS485_TX_WATCHDOG_MS) ||
        ((bus->huart->gState == HAL_UART_STATE_READY) && (elapsed >= 4U)))
    {
      if (bus->huart->gState != HAL_UART_STATE_READY)
      {
        (void)HAL_UART_AbortTransmit(bus->huart);
      }
      bus->tx_active = 0U;
      rs485_set_dir_rx(bus);
      bus->stats.tx_wdog_recoveries++;
    }
  }

  /* Release deferred slave reply only after guaranteed bus-idle turnaround margin */
  if ((bus->pending_reply_valid != 0U) &&
      (bus->tx_active == 0U) &&
      (bus->huart->gState == HAL_UART_STATE_READY) &&
      ((uint32_t)(now_ms - bus->pending_reply_rx_ms) >= RS485_SLAVE_TURNAROUND_MS))
  {
    uint8_t len = bus->pending_reply_len;
    bus->pending_reply_valid = 0U;
    if ((len > 0U) && (len <= RS485_MAX_FRAME_SIZE))
    {
      memcpy(bus->tx_buf, bus->pending_reply_buf, len);
      (void)rs485_transmit_raw(bus, len, now_ms);
    }
  }
}

void RS485_TxCpltCallback(RS485_Bus_t *bus, UART_HandleTypeDef *huart)
{
  if ((bus == NULL) || (bus->huart == NULL) || (huart == NULL) ||
      (huart->Instance != bus->huart->Instance))
  {
    return;
  }

  bus->tx_active = 0U;
  rs485_set_dir_rx(bus);
}

void RS485_UartErrorCallback(RS485_Bus_t *bus, UART_HandleTypeDef *huart)
{
  if ((bus == NULL) || (bus->huart == NULL) || (huart == NULL) ||
      (huart->Instance != bus->huart->Instance))
  {
    return;
  }

  bus->stats.uart_err_recoveries++;
  rs485_uart_flush_rx(bus->huart);
  if (bus->huart->hdmarx != NULL)
  {
    (void)RS485_StartRx(bus);
  }
}

bool RS485_PopLatestFrame(RS485_Bus_t *bus, RS485_Frame_t *out_frame)
{
  if ((bus == NULL) || (out_frame == NULL) || (bus->rx_updated == 0U))
  {
    return false;
  }
  *out_frame = bus->last_rx;
  bus->rx_updated = 0U;
  return true;
}

bool RS485_IsLinkAlive(const RS485_Bus_t *bus, uint32_t now_ms, uint32_t timeout_ms)
{
  if ((bus == NULL) || (bus->has_ever_rx == 0U))
  {
    return false;
  }
  return ((uint32_t)(now_ms - bus->last_rx_tick_ms) <= timeout_ms);
}

HAL_StatusTypeDef RS485_SendFrame(RS485_Bus_t *bus,
                                  uint8_t dst_id,
                                  RS485_MsgType_t msg_type,
                                  const uint8_t *payload,
                                  uint8_t len,
                                  uint32_t now_ms)
{
  if ((bus == NULL) || (bus->huart == NULL) || (len > RS485_MAX_PAYLOAD))
  {
    return HAL_ERROR;
  }
  if ((len > 0U) && (payload == NULL))
  {
    return HAL_ERROR;
  }
  if ((bus->tx_active != 0U) || (bus->huart->gState != HAL_UART_STATE_READY))
  {
    bus->stats.tx_busy_drops++;
    return HAL_BUSY;
  }

  uint16_t total_len = rs485_encode_raw(bus->tx_buf,
                                        bus->local_id,
                                        dst_id,
                                        (uint8_t)msg_type,
                                        bus->tx_seq++,
                                        payload,
                                        len);
  return rs485_transmit_raw(bus, total_len, now_ms);
}

bool RS485_ScheduleReply(RS485_Bus_t *bus,
                         uint8_t dst_id,
                         RS485_MsgType_t msg_type,
                         const uint8_t *payload,
                         uint8_t len,
                         uint32_t now_ms)
{
  if ((bus == NULL) || (len > RS485_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
  {
    return false;
  }

  uint16_t total_len = rs485_encode_raw(bus->pending_reply_buf,
                                        bus->local_id,
                                        dst_id,
                                        (uint8_t)msg_type,
                                        bus->tx_seq++,
                                        payload,
                                        len);
  bus->pending_reply_len = (uint8_t)total_len;
  bus->pending_reply_rx_ms = now_ms;
  bus->pending_reply_valid = 1U;
  return true;
}

HAL_StatusTypeDef RS485_SendMultiCmd(RS485_Bus_t *bus,
                                     const RS485_MultiCmd_t *cmd,
                                     uint32_t now_ms)
{
  uint8_t buf[RS485_PAYLOAD_MULTI_CMD_LEN];
  if (cmd == NULL)
  {
    return HAL_ERROR;
  }

  uint8_t nw = cmd->num_wheels;
  if (nw > RS485_MAX_WHEELS)
  {
    nw = RS485_MAX_WHEELS;
  }

  buf[0] = nw;
  buf[1] = cmd->poll_id;
  buf[2] = cmd->flags;
  buf[3] = cmd->reserved;

  for (uint8_t w = 0U; w < RS485_MAX_WHEELS; w++)
  {
    uint8_t base = (uint8_t)(4U + (w * 3U));
    buf[base + 0U] = (w < nw) ? cmd->wheels[w].mode : (uint8_t)RS485_WHEEL_DISABLED;
    rs485_write_u16_le(&buf[base + 1U], (w < nw) ? (uint16_t)cmd->wheels[w].target : 0U);
  }

  return RS485_SendFrame(bus, RS485_ADDR_BCAST, RS485_MSG_MULTI_CMD, buf, RS485_PAYLOAD_MULTI_CMD_LEN, now_ms);
}

HAL_StatusTypeDef RS485_SendWheelCmd(RS485_Bus_t *bus,
                                     uint8_t dst_id,
                                     const RS485_WheelCmd_t *cmd,
                                     uint32_t now_ms)
{
  uint8_t buf[RS485_PAYLOAD_WHEEL_CMD_LEN];
  if (cmd == NULL)
  {
    return HAL_ERROR;
  }

  buf[0] = cmd->mode;
  buf[1] = cmd->flags;
  rs485_write_u32_le(&buf[2], (uint32_t)cmd->target);

  return RS485_SendFrame(bus, dst_id, RS485_MSG_WHEEL_CMD, buf, RS485_PAYLOAD_WHEEL_CMD_LEN, now_ms);
}

HAL_StatusTypeDef RS485_SendCfgCmd(RS485_Bus_t *bus,
                                   uint8_t dst_id,
                                   const RS485_CfgCmd_t *cfg,
                                   uint32_t now_ms)
{
  uint8_t buf[RS485_PAYLOAD_CFG_CMD_LEN];
  if (cfg == NULL)
  {
    return HAL_ERROR;
  }

  buf[0] = cfg->subcmd;
  buf[1] = cfg->param_u8;
  rs485_write_u32_le(&buf[2], (uint32_t)cfg->value);

  return RS485_SendFrame(bus, dst_id, RS485_MSG_CFG_CMD, buf, RS485_PAYLOAD_CFG_CMD_LEN, now_ms);
}

bool RS485_ScheduleTelemReply(RS485_Bus_t *bus,
                              uint8_t dst_id,
                              const RS485_Telem_t *telem,
                              uint32_t now_ms)
{
  uint8_t buf[RS485_PAYLOAD_TELEM_LEN];
  if (telem == NULL)
  {
    return false;
  }

  buf[0] = telem->node_id;
  buf[1] = telem->mode;
  buf[2] = telem->status_flags;
  rs485_write_u16_le(&buf[3], (uint16_t)telem->actual_rpm_x10);
  rs485_write_u16_le(&buf[5], (uint16_t)telem->duty_permille);
  rs485_write_u32_le(&buf[7], (uint32_t)telem->enc_count);

  return RS485_ScheduleReply(bus, dst_id, RS485_MSG_TELEM, buf, RS485_PAYLOAD_TELEM_LEN, now_ms);
}

bool RS485_ScheduleCfgAckReply(RS485_Bus_t *bus,
                               uint8_t dst_id,
                               const RS485_CfgAck_t *ack,
                               uint32_t now_ms)
{
  uint8_t buf[RS485_PAYLOAD_CFG_ACK_LEN];
  if (ack == NULL)
  {
    return false;
  }

  buf[0] = ack->node_id;
  buf[1] = ack->subcmd;
  buf[2] = ack->status;
  buf[3] = ack->reserved;

  return RS485_ScheduleReply(bus, dst_id, RS485_MSG_CFG_ACK, buf, RS485_PAYLOAD_CFG_ACK_LEN, now_ms);
}

bool RS485_DecodeMultiCmd(const RS485_Frame_t *frame, RS485_MultiCmd_t *out_cmd)
{
  if ((frame == NULL) || (out_cmd == NULL) ||
      (frame->msg_type != (uint8_t)RS485_MSG_MULTI_CMD) ||
      (frame->len != RS485_PAYLOAD_MULTI_CMD_LEN))
  {
    return false;
  }

  out_cmd->num_wheels = frame->payload[0];
  if (out_cmd->num_wheels > RS485_MAX_WHEELS)
  {
    out_cmd->num_wheels = RS485_MAX_WHEELS;
  }
  out_cmd->poll_id  = frame->payload[1];
  out_cmd->flags    = frame->payload[2];
  out_cmd->reserved = frame->payload[3];

  for (uint8_t w = 0U; w < RS485_MAX_WHEELS; w++)
  {
    uint8_t base = (uint8_t)(4U + (w * 3U));
    out_cmd->wheels[w].mode   = frame->payload[base + 0U];
    out_cmd->wheels[w].target = (int16_t)rs485_read_u16_le(&frame->payload[base + 1U]);
  }
  return true;
}

bool RS485_DecodeWheelCmd(const RS485_Frame_t *frame, RS485_WheelCmd_t *out_cmd)
{
  if ((frame == NULL) || (out_cmd == NULL) ||
      (frame->msg_type != (uint8_t)RS485_MSG_WHEEL_CMD) ||
      (frame->len != RS485_PAYLOAD_WHEEL_CMD_LEN))
  {
    return false;
  }

  out_cmd->mode   = frame->payload[0];
  out_cmd->flags  = frame->payload[1];
  out_cmd->target = (int32_t)rs485_read_u32_le(&frame->payload[2]);
  return true;
}

bool RS485_DecodeTelem(const RS485_Frame_t *frame, RS485_Telem_t *out_telem)
{
  if ((frame == NULL) || (out_telem == NULL) ||
      (frame->msg_type != (uint8_t)RS485_MSG_TELEM) ||
      (frame->len != RS485_PAYLOAD_TELEM_LEN))
  {
    return false;
  }

  out_telem->node_id        = frame->payload[0];
  out_telem->mode           = frame->payload[1];
  out_telem->status_flags   = frame->payload[2];
  out_telem->actual_rpm_x10 = (int16_t)rs485_read_u16_le(&frame->payload[3]);
  out_telem->duty_permille  = (int16_t)rs485_read_u16_le(&frame->payload[5]);
  out_telem->enc_count      = (int32_t)rs485_read_u32_le(&frame->payload[7]);
  return true;
}

bool RS485_DecodeCfgCmd(const RS485_Frame_t *frame, RS485_CfgCmd_t *out_cfg)
{
  if ((frame == NULL) || (out_cfg == NULL) ||
      (frame->msg_type != (uint8_t)RS485_MSG_CFG_CMD) ||
      (frame->len != RS485_PAYLOAD_CFG_CMD_LEN))
  {
    return false;
  }

  out_cfg->subcmd   = frame->payload[0];
  out_cfg->param_u8 = frame->payload[1];
  out_cfg->value    = (int32_t)rs485_read_u32_le(&frame->payload[2]);
  return true;
}

bool RS485_DecodeCfgAck(const RS485_Frame_t *frame, RS485_CfgAck_t *out_ack)
{
  if ((frame == NULL) || (out_ack == NULL) ||
      (frame->msg_type != (uint8_t)RS485_MSG_CFG_ACK) ||
      (frame->len != RS485_PAYLOAD_CFG_ACK_LEN))
  {
    return false;
  }

  out_ack->node_id  = frame->payload[0];
  out_ack->subcmd   = frame->payload[1];
  out_ack->status   = frame->payload[2];
  out_ack->reserved = frame->payload[3];
  return true;
}
