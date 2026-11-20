/**
  ******************************************************************************
  * @file    rf_hc12.c
  * @brief   Portable framed HC-12 RF protocol, auto-baud AT setup, circular DMA
  *          RX stream parser, UART error recovery, and strict source-ID filter.
  ******************************************************************************
  */

#include "rf_hc12.h"
#include <string.h>

#define RF_HC12_SET_LOW_DELAY_MS    50U
#define RF_HC12_SET_EXIT_DELAY_MS   100U
#define RF_HC12_AT_TX_TIMEOUT_MS    100U
#define RF_HC12_AT_RX_BYTE_MS       50U
#define RF_HC12_BAUD_REBOOT_MS      200U
#define RF_HC12_DEFAULT_POWER       8U

static void rf_uart_flush_rx(UART_HandleTypeDef *huart)
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

static uint16_t rf_strnlen(const char *s, uint16_t max_len)
{
  uint16_t n = 0U;
  if (s == NULL)
  {
    return 0U;
  }
  while ((n < max_len) && (s[n] != '\0'))
  {
    n++;
  }
  return n;
}

static void rf_format_baud_cmd(char *out, uint32_t baud)
{
  /* Writes "AT+B<baud>\r\n" without pulling stdio snprintf into G030 32KB flash */
  char rev[12];
  uint8_t idx = 0U;
  uint8_t p = 0U;

  out[p++] = 'A';
  out[p++] = 'T';
  out[p++] = '+';
  out[p++] = 'B';

  if (baud == 0U)
  {
    rev[idx++] = '0';
  }
  else
  {
    while ((baud > 0U) && (idx < (uint8_t)sizeof(rev)))
    {
      rev[idx++] = (char)('0' + (baud % 10U));
      baud /= 10U;
    }
  }

  while (idx > 0U)
  {
    idx--;
    out[p++] = rev[idx];
  }

  out[p++] = '\r';
  out[p++] = '\n';
  out[p] = '\0';
}

uint16_t RF_HC12_Crc16Ccitt(const uint8_t *data, uint16_t len)
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

uint8_t RF_HC12_Crc8(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0x00U;
  if (data == NULL)
  {
    return crc;
  }

  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0U; b < 8U; b++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0x07U);
      }
      else
      {
        crc = (uint8_t)(crc << 1U);
      }
    }
  }
  return crc;
}

static void rf_dispatch_valid_frame(RF_HC12_t *rf, const uint8_t *buf, uint32_t now_ms)
{
  uint8_t version  = buf[2];
  uint8_t src_id   = buf[3];
  uint8_t dst_id   = buf[4];
  uint8_t msg_type = buf[5];
  uint8_t seq      = buf[6];
  uint8_t len      = buf[7];

  if ((version != RF_HC12_PROTO_VERSION) || (len > RF_HC12_MAX_PAYLOAD))
  {
    return;
  }

  /* Strict source-ID whitelist enforced BEFORE destination check */
  bool src_allowed = (src_id < 32U) && ((rf->allowed_src_mask & RF_SRC_MASK(src_id)) != 0UL);
  if (!src_allowed)
  {
    rf->stats.rejected_src_frames++;
    rf->stats.last_rejected_src = src_id;
    if (src_id == (uint8_t)RF_NODE_TDK)
    {
      rf->stats.rejected_tdk_frames++;
    }
    return;
  }

  /* Destination filter */
  if ((dst_id != rf->local_id) && (dst_id != (uint8_t)RF_NODE_BCAST))
  {
    rf->stats.rejected_dst_frames++;
    return;
  }

  rf->last_rx.version  = version;
  rf->last_rx.src_id   = src_id;
  rf->last_rx.dst_id   = dst_id;
  rf->last_rx.msg_type = msg_type;
  rf->last_rx.seq      = seq;
  rf->last_rx.len      = len;
  if (len > 0U)
  {
    memcpy(rf->last_rx.payload, &buf[RF_HC12_HEADER_SIZE], len);
  }
  if (len < RF_HC12_MAX_PAYLOAD)
  {
    memset(&rf->last_rx.payload[len], 0, RF_HC12_MAX_PAYLOAD - len);
  }

  rf->rx_updated = 1U;
  rf->has_ever_rx = 1U;
  rf->last_rx_tick_ms = now_ms;
  rf->stats.rx_ok_frames++;

  if (rf->rx_callback != NULL)
  {
    rf->rx_callback(rf, &rf->last_rx, rf->rx_callback_ctx);
  }
}

static void rf_dispatch_6byte_frame(RF_HC12_t *rf, const uint8_t *buf, uint32_t now_ms)
{
  /* buf format (6 bytes, proven in RB2026_TX_PS2_NEW & RBC2026_MAINBOARD_R1):
   * [0] = 255 (RF_6BYTE_SOF / 0xFF)
   * [1] = JoyY (0..200, neutral = 100)
   * [2] = JoyX (0..200, neutral = 100)
   * [3] = Btn1 (du.button[1]: L2, R2, L1, R1, △, ◯, ✕, □)
   * [4] = Btn0 (du.button[0]: Select, L3, R3, Start, D-Pad)
   * [5] = Cnt  (rolling counter 0..249)
   */
  uint8_t src_id = (uint8_t)RF_NODE_TDK;

  /* Strict source-ID whitelist enforced BEFORE destination check */
  bool src_allowed = (src_id < 32U) && ((rf->allowed_src_mask & RF_SRC_MASK(src_id)) != 0UL);
  if (!src_allowed)
  {
    rf->stats.rejected_src_frames++;
    rf->stats.last_rejected_src = src_id;
    rf->stats.rejected_tdk_frames++;
    return;
  }

  rf->last_rx.version  = RF_HC12_PROTO_VERSION;
  rf->last_rx.src_id   = src_id;
  rf->last_rx.dst_id   = rf->local_id;
  rf->last_rx.msg_type = (uint8_t)RF_MSG_TDK_CMD;
  rf->last_rx.seq      = buf[5]; /* Cnt */
  rf->last_rx.len      = RF_PAYLOAD_TDK_LEN;

  rf->last_rx.payload[0] = 255U;   /* ps2_addr (analog mode) */
  rf->last_rx.payload[1] = buf[1]; /* joy_sum[0] (JoyY) */
  rf->last_rx.payload[2] = buf[2]; /* joy_sum[1] (JoyX) */
  rf->last_rx.payload[3] = buf[1]; /* joy_left[0] */
  rf->last_rx.payload[4] = buf[2]; /* joy_left[1] */
  rf->last_rx.payload[5] = buf[4]; /* ps2_btn[0] (du.button[0]) */
  rf->last_rx.payload[6] = buf[3]; /* ps2_btn[1] (du.button[1]) */
  rf->last_rx.payload[7] = RF_TDK_PS2_VALID_BIT | RF_TDK_PS2_ANALOG_BIT;
  rf->last_rx.payload[8] = buf[5]; /* aux (cnt) */

  rf->rx_updated = 1U;
  rf->has_ever_rx = 1U;
  rf->last_rx_tick_ms = now_ms;
  rf->stats.rx_ok_frames++;

  if (rf->rx_callback != NULL)
  {
    rf->rx_callback(rf, &rf->last_rx, rf->rx_callback_ctx);
  }
}

static void rf_dispatch_compact_frame(RF_HC12_t *rf, const uint8_t *buf, uint32_t now_ms)
{
  /* buf format (11 bytes for TDK):
   * [0] = 0x55 (RF_COMPACT_SOF0)
   * [1] = 0xAA (RF_COMPACT_SOF1)
   * [2] = src_id (RF_NODE_TDK = 0x01)
   * [3] = joy_left[0] (LX)
   * [4] = joy_left[1] (LY)
   * [5] = joy_sum[0]  (RX)
   * [6] = joy_sum[1]  (RY)
   * [7] = ps2_btn[0]
   * [8] = ps2_btn[1]
   * [9] = ext_sw
   * [10] = CRC-8 of bytes 2..9 (8 bytes)
   */
  uint8_t src_id = buf[2];

  /* Strict source-ID whitelist enforced BEFORE destination check */
  bool src_allowed = (src_id < 32U) && ((rf->allowed_src_mask & RF_SRC_MASK(src_id)) != 0UL);
  if (!src_allowed)
  {
    rf->stats.rejected_src_frames++;
    rf->stats.last_rejected_src = src_id;
    if (src_id == (uint8_t)RF_NODE_TDK)
    {
      rf->stats.rejected_tdk_frames++;
    }
    return;
  }

  if (src_id == (uint8_t)RF_NODE_TDK)
  {
    rf->last_rx.version  = RF_HC12_PROTO_VERSION;
    rf->last_rx.src_id   = src_id;
    rf->last_rx.dst_id   = rf->local_id;
    rf->last_rx.msg_type = (uint8_t)RF_MSG_TDK_CMD;
    rf->last_rx.seq      = 0U;
    rf->last_rx.len      = RF_PAYLOAD_TDK_LEN;

    rf->last_rx.payload[0] = 255U;   /* ps2_addr (analog mode) */
    rf->last_rx.payload[1] = buf[5]; /* joy_sum[0] (RX) */
    rf->last_rx.payload[2] = buf[6]; /* joy_sum[1] (RY) */
    rf->last_rx.payload[3] = buf[3]; /* joy_left[0] (LX) */
    rf->last_rx.payload[4] = buf[4]; /* joy_left[1] (LY) */
    rf->last_rx.payload[5] = buf[7]; /* ps2_btn[0] */
    rf->last_rx.payload[6] = buf[8]; /* ps2_btn[1] */
    rf->last_rx.payload[7] = buf[9]; /* ext_sw */
    rf->last_rx.payload[8] = 0U;     /* aux */

    rf->rx_updated = 1U;
    rf->has_ever_rx = 1U;
    rf->last_rx_tick_ms = now_ms;
    rf->stats.rx_ok_frames++;

    if (rf->rx_callback != NULL)
    {
      rf->rx_callback(rf, &rf->last_rx, rf->rx_callback_ctx);
    }
  }
}

static void rf_resync_from_frame_buf(RF_HC12_t *rf, uint32_t now_ms)
{
  uint8_t saved[RF_HC12_MAX_FRAME_SIZE];
  uint8_t avail = rf->frame_pos;
  uint8_t s = 1U;

  if (avail <= 1U)
  {
    rf->frame_pos = 0U;
    return;
  }

  memcpy(saved, rf->frame_buf, avail);
  rf->frame_pos = 0U;

  while (s < avail)
  {
    /* Check for 6-byte competition frame SOF: 255 */
    if (saved[s] == RF_6BYTE_SOF)
    {
      uint8_t rem = (uint8_t)(avail - s);
      if (rem < RF_6BYTE_LEN)
      {
        memcpy(rf->frame_buf, &saved[s], rem);
        rf->frame_pos = rem;
        return;
      }
      rf_dispatch_6byte_frame(rf, &saved[s], now_ms);
      s = (uint8_t)(s + RF_6BYTE_LEN);
      continue;
    }

    /* Check for compact SOF: 0x55, 0xAA */
    if (saved[s] == RF_COMPACT_SOF0)
    {
      uint8_t rem = (uint8_t)(avail - s);
      if (rem == 1U)
      {
        rf->frame_buf[0] = RF_COMPACT_SOF0;
        rf->frame_pos = 1U;
        return;
      }
      if (saved[s + 1U] == RF_COMPACT_SOF1)
      {
        if (rem == 2U)
        {
          rf->frame_buf[0] = RF_COMPACT_SOF0;
          rf->frame_buf[1] = RF_COMPACT_SOF1;
          rf->frame_pos = 2U;
          return;
        }
        uint8_t node = saved[s + 2U];
        if (node == (uint8_t)RF_NODE_TDK)
        {
          if (rem < RF_COMPACT_TDK_LEN)
          {
            memcpy(rf->frame_buf, &saved[s], rem);
            rf->frame_pos = rem;
            return;
          }
          uint8_t crc_calc = RF_HC12_Crc8(&saved[s + 2U], 8U);
          uint8_t crc_rx   = saved[s + 10U];
          if (crc_calc == crc_rx)
          {
            rf_dispatch_compact_frame(rf, &saved[s], now_ms);
            s = (uint8_t)(s + RF_COMPACT_TDK_LEN);
            continue;
          }
          else
          {
            rf->stats.crc_err_frames++;
            s++;
            continue;
          }
        }
      }
    }

    /* Check for legacy SOF: 0xAA, 0x55 */
    if (saved[s] == RF_HC12_SOF0)
    {
      uint8_t rem = (uint8_t)(avail - s);
      if (rem == 1U)
      {
        rf->frame_buf[0] = RF_HC12_SOF0;
        rf->frame_pos = 1U;
        return;
      }

      if (saved[s + 1U] == RF_HC12_SOF1)
      {
        if (rem == 2U)
        {
          rf->frame_buf[0] = RF_HC12_SOF0;
          rf->frame_buf[1] = RF_HC12_SOF1;
          rf->frame_pos = 2U;
          return;
        }

        if (saved[s + 2U] != RF_HC12_PROTO_VERSION)
        {
          s++;
          continue;
        }

        if (rem < RF_HC12_HEADER_SIZE)
        {
          memcpy(rf->frame_buf, &saved[s], rem);
          rf->frame_pos = rem;
          return;
        }

        uint8_t plen = saved[s + 7U];
        if (plen > RF_HC12_MAX_PAYLOAD)
        {
          rf->stats.len_err_frames++;
          s++;
          continue;
        }

        uint8_t total = (uint8_t)(RF_HC12_HEADER_SIZE + plen + RF_HC12_CRC_SIZE);
        if (rem < total)
        {
          memcpy(rf->frame_buf, &saved[s], rem);
          rf->frame_pos = rem;
          return;
        }

        uint16_t crc_calc = RF_HC12_Crc16Ccitt(&saved[s + 2U], (uint16_t)(6U + plen));
        uint16_t crc_rx = ((uint16_t)saved[s + 8U + plen] << 8U) |
                          (uint16_t)saved[s + 9U + plen];

        if (crc_calc == crc_rx)
        {
          rf_dispatch_valid_frame(rf, &saved[s], now_ms);
          s = (uint8_t)(s + total);
          continue;
        }
        else
        {
          rf->stats.crc_err_frames++;
          s++;
          continue;
        }
      }
    }

    s++;
  }
}

static void rf_feed_byte(RF_HC12_t *rf, uint8_t byte, uint32_t now_ms)
{
  uint8_t pos = rf->frame_pos;

  if (pos == 0U)
  {
    if ((byte == RF_6BYTE_SOF) || (byte == RF_HC12_SOF0) || (byte == RF_COMPACT_SOF0))
    {
      rf->frame_buf[0] = byte;
      rf->frame_pos = 1U;
    }
    return;
  }

  /* 6-byte competition frame stream: [255, JoyY, JoyX, Btn1, Btn0, Cnt] */
  if (rf->frame_buf[0] == RF_6BYTE_SOF)
  {
    if (byte == RF_6BYTE_SOF)
    {
      /* Re-synchronize on SOF */
      rf->frame_pos = 1U;
      return;
    }

    rf->frame_buf[pos] = byte;
    rf->frame_pos = (uint8_t)(pos + 1U);

    if (rf->frame_pos == RF_6BYTE_LEN)
    {
      rf->frame_pos = 0U;
      rf_dispatch_6byte_frame(rf, rf->frame_buf, now_ms);
    }
    return;
  }

  if (pos == 1U)
  {
    if (byte == RF_6BYTE_SOF)
    {
      rf->frame_buf[0] = RF_6BYTE_SOF;
      rf->frame_pos = 1U;
      return;
    }

    if (rf->frame_buf[0] == RF_HC12_SOF0)
    {
      if (byte == RF_HC12_SOF1)
      {
        rf->frame_buf[1] = byte;
        rf->frame_pos = 2U;
      }
      else if (byte == RF_HC12_SOF0)
      {
        rf->frame_pos = 1U;
      }
      else
      {
        rf->frame_pos = (byte == RF_COMPACT_SOF0) ? 1U : 0U;
        if (rf->frame_pos == 1U)
        {
          rf->frame_buf[0] = byte;
        }
      }
      return;
    }
    else /* rf->frame_buf[0] == RF_COMPACT_SOF0 */
    {
      if (byte == RF_COMPACT_SOF1)
      {
        rf->frame_buf[1] = byte;
        rf->frame_pos = 2U;
      }
      else if (byte == RF_COMPACT_SOF0)
      {
        rf->frame_pos = 1U;
      }
      else
      {
        rf->frame_pos = (byte == RF_HC12_SOF0) ? 1U : 0U;
        if (rf->frame_pos == 1U)
        {
          rf->frame_buf[0] = byte;
        }
      }
      return;
    }
  }

  if (pos >= RF_HC12_MAX_FRAME_SIZE)
  {
    rf->frame_pos = 0U;
    return;
  }

  rf->frame_buf[pos] = byte;
  rf->frame_pos = (uint8_t)(pos + 1U);

  /* Compact 11-byte frame parsing */
  if (rf->frame_buf[0] == RF_COMPACT_SOF0)
  {
    if (pos == 2U)
    {
      if ((byte != (uint8_t)RF_NODE_TDK) && (byte != (uint8_t)RF_NODE_MANUAL))
      {
        rf_resync_from_frame_buf(rf, now_ms);
      }
      return;
    }

    if (rf->frame_buf[2] == (uint8_t)RF_NODE_TDK)
    {
      if (rf->frame_pos == RF_COMPACT_TDK_LEN)
      {
        uint8_t crc_calc = RF_HC12_Crc8(&rf->frame_buf[2], 8U);
        uint8_t crc_rx   = rf->frame_buf[10];

        if (crc_calc == crc_rx)
        {
          rf->frame_pos = 0U;
          rf_dispatch_compact_frame(rf, rf->frame_buf, now_ms);
        }
        else
        {
          rf->stats.crc_err_frames++;
          rf_resync_from_frame_buf(rf, now_ms);
        }
      }
    }
    return;
  }

  /* Legacy framed parsing */
  if (pos == 2U)
  {
    if (byte != RF_HC12_PROTO_VERSION)
    {
      rf_resync_from_frame_buf(rf, now_ms);
    }
    return;
  }

  if (pos == 7U)
  {
    if (byte > RF_HC12_MAX_PAYLOAD)
    {
      rf->stats.len_err_frames++;
      rf_resync_from_frame_buf(rf, now_ms);
    }
    return;
  }

  if (rf->frame_pos >= RF_HC12_HEADER_SIZE)
  {
    uint8_t plen = rf->frame_buf[7];
    uint8_t total = (uint8_t)(RF_HC12_HEADER_SIZE + plen + RF_HC12_CRC_SIZE);
    if (rf->frame_pos == total)
    {
      uint16_t crc_calc = RF_HC12_Crc16Ccitt(&rf->frame_buf[2], (uint16_t)(6U + plen));
      uint16_t crc_rx = ((uint16_t)rf->frame_buf[8U + plen] << 8U) |
                        (uint16_t)rf->frame_buf[9U + plen];

      if (crc_calc == crc_rx)
      {
        rf->frame_pos = 0U;
        rf_dispatch_valid_frame(rf, rf->frame_buf, now_ms);
      }
      else
      {
        rf->stats.crc_err_frames++;
        rf_resync_from_frame_buf(rf, now_ms);
      }
    }
  }
}

void RF_HC12_Init(RF_HC12_t *rf,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *set_port,
                  uint16_t set_pin,
                  RF_NodeId_t local_id,
                  uint32_t allowed_src_mask)
{
  if (rf == NULL)
  {
    return;
  }

  memset(rf, 0, sizeof(*rf));
  rf->huart = huart;
  rf->set_port = set_port;
  rf->set_pin = set_pin;
  rf->local_id = (uint8_t)local_id;
  rf->allowed_src_mask = allowed_src_mask;

  if (rf->set_port != NULL)
  {
    HAL_GPIO_WritePin(rf->set_port, rf->set_pin, GPIO_PIN_SET);
  }
}

void RF_HC12_SetRxCallback(RF_HC12_t *rf, RF_HC12_RxCallback_t cb, void *ctx)
{
  if (rf == NULL)
  {
    return;
  }
  rf->rx_callback = cb;
  rf->rx_callback_ctx = ctx;
}

HAL_StatusTypeDef RF_HC12_SendAT(RF_HC12_t *rf,
                                 const char *cmd,
                                 const char *expected,
                                 uint32_t timeout_ms)
{
  uint8_t rx_tmp[32];
  uint8_t rx_idx = 0U;
  uint8_t byte = 0U;

  if ((rf == NULL) || (rf->huart == NULL) || (cmd == NULL) || (expected == NULL))
  {
    return HAL_ERROR;
  }

  uint16_t cmd_len = rf_strnlen(cmd, (uint16_t)(sizeof(rx_tmp) - 1U));
  if (cmd_len == 0U)
  {
    return HAL_ERROR;
  }

  memset(rx_tmp, 0, sizeof(rx_tmp));

  (void)HAL_UART_Abort(rf->huart);
  rf_uart_flush_rx(rf->huart);

  if (HAL_UART_Transmit(rf->huart, (uint8_t *)cmd, cmd_len, RF_HC12_AT_TX_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Receive immediately without delay */
  uint32_t start_ms = HAL_GetTick();
  while (((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms) &&
         (rx_idx < (uint8_t)(sizeof(rx_tmp) - 1U)))
  {
    if (HAL_UART_Receive(rf->huart, &byte, 1U, RF_HC12_AT_RX_BYTE_MS) == HAL_OK)
    {
      rx_tmp[rx_idx++] = byte;
      rx_tmp[rx_idx] = '\0';
      if (strstr((const char *)rx_tmp, expected) != NULL)
      {
        return HAL_OK;
      }
    }
  }

  /* Fallback for strict modules: if cmd had \r\n and timed out, retry bare command without \r\n */
  if ((cmd_len > 2U) && (cmd[cmd_len - 2U] == '\r') && (cmd[cmd_len - 1U] == '\n'))
  {
    uint16_t bare_len = cmd_len - 2U;
    rx_idx = 0U;
    memset(rx_tmp, 0, sizeof(rx_tmp));
    (void)HAL_UART_Abort(rf->huart);
    rf_uart_flush_rx(rf->huart);

    if (HAL_UART_Transmit(rf->huart, (uint8_t *)cmd, bare_len, RF_HC12_AT_TX_TIMEOUT_MS) == HAL_OK)
    {
      start_ms = HAL_GetTick();
      while (((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms) &&
             (rx_idx < (uint8_t)(sizeof(rx_tmp) - 1U)))
      {
        if (HAL_UART_Receive(rf->huart, &byte, 1U, RF_HC12_AT_RX_BYTE_MS) == HAL_OK)
        {
          rx_tmp[rx_idx++] = byte;
          rx_tmp[rx_idx] = '\0';
          if (strstr((const char *)rx_tmp, expected) != NULL)
          {
            return HAL_OK;
          }
        }
      }
    }
  }

  return HAL_ERROR;
}

bool RF_HC12_SetAddress(RF_HC12_t *rf, uint8_t channel, uint8_t power, uint32_t target_baud)
{
  static const uint32_t k_probe_bauds[6] = {9600U, 115200U, 19200U, 38400U, 4800U, 2400U};
  char ch_cmd[12];
  char pwr_cmd[8];
  char baud_cmd[20];
  bool found = false;
  bool configured = false;

  if ((rf == NULL) || (rf->huart == NULL))
  {
    return false;
  }

  if ((channel < 1U) || (channel > 100U))
  {
    channel = RF_HC12_DEFAULT_CHANNEL;
  }
  if ((power < 1U) || (power > 8U))
  {
    power = RF_HC12_DEFAULT_POWER;
  }
  if (target_baud == 0U)
  {
    target_baud = RF_HC12_DEFAULT_BAUD;
  }

  rf->at_found = 0U;
  rf->at_configured = 0U;
  rf->at_detected_baud = 0U;
  memset(rf->at_resp, 0, sizeof(rf->at_resp));

  if (rf->set_port != NULL)
  {
    HAL_GPIO_WritePin(rf->set_port, rf->set_pin, GPIO_PIN_RESET);
    HAL_Delay(RF_HC12_SET_LOW_DELAY_MS);
  }

  for (uint8_t i = 0U; i < 6U; i++)
  {
    (void)HAL_UART_Abort(rf->huart);
    rf->huart->Init.BaudRate = k_probe_bauds[i];
    if (HAL_UART_Init(rf->huart) != HAL_OK)
    {
      continue;
    }
    HAL_Delay(50U);
    rf_uart_flush_rx(rf->huart);

    if (RF_HC12_SendAT(rf, "AT\r\n", "OK", 500U) == HAL_OK)
    {
      found = true;
      rf->at_found = 1U;
      rf->at_detected_baud = k_probe_bauds[i];
      break;
    }
  }

  if (found)
  {
    /* CRITICAL: HC-12 in AT command mode ALWAYS communicates at the probed baud rate (usually 9600).
     * Therefore, we MUST send Channel, Power, and Mode commands at this speed BEFORE changing baud!
     */

    /* 1. Format and send Channel: AT+Cxxx\r\n */
    ch_cmd[0] = 'A';
    ch_cmd[1] = 'T';
    ch_cmd[2] = '+';
    ch_cmd[3] = 'C';
    ch_cmd[4] = (char)('0' + (channel / 100U));
    ch_cmd[5] = (char)('0' + ((channel / 10U) % 10U));
    ch_cmd[6] = (char)('0' + (channel % 10U));
    ch_cmd[7] = '\r';
    ch_cmd[8] = '\n';
    ch_cmd[9] = '\0';

    bool ch_ok = (RF_HC12_SendAT(rf, ch_cmd, "OK", 500U) == HAL_OK);
    HAL_Delay(100U);

    /* 2. Format and send Power: AT+Px\r\n */
    pwr_cmd[0] = 'A';
    pwr_cmd[1] = 'T';
    pwr_cmd[2] = '+';
    pwr_cmd[3] = 'P';
    pwr_cmd[4] = (char)('0' + power);
    pwr_cmd[5] = '\r';
    pwr_cmd[6] = '\n';
    pwr_cmd[7] = '\0';

    bool pwr_ok = (RF_HC12_SendAT(rf, pwr_cmd, "OK", 500U) == HAL_OK);
    HAL_Delay(100U);

    /* 3. Transmission mode: AT+FU3\r\n */
    bool fu_ok = (RF_HC12_SendAT(rf, "AT+FU3\r\n", "OK", 500U) == HAL_OK);
    HAL_Delay(100U);

    /* 4. Format and send Baud rate: AT+Bxxxxx\r\n (SENT LAST) */
    rf_format_baud_cmd(baud_cmd, target_baud);
    bool baud_ok = (RF_HC12_SendAT(rf, baud_cmd, "OK", 500U) == HAL_OK);
    HAL_Delay(RF_HC12_BAUD_REBOOT_MS);

    /* 5. Query and capture actual stored parameters into rf->at_resp */
    (void)HAL_UART_Abort(rf->huart);
    rf_uart_flush_rx(rf->huart);
    if (HAL_UART_Transmit(rf->huart, (uint8_t *)"AT+RX\r\n", 7U, 100U) == HAL_OK)
    {
      uint8_t rx_idx = 0U;
      uint32_t rx_start = HAL_GetTick();
      while (((uint32_t)(HAL_GetTick() - rx_start) < 600U) &&
             (rx_idx < (sizeof(rf->at_resp) - 1U)))
      {
        uint8_t b;
        if (HAL_UART_Receive(rf->huart, &b, 1U, 20U) == HAL_OK)
        {
          rf->at_resp[rx_idx++] = (char)b;
        }
      }
    }

    /* 6. Require ALL essential parameters to succeed (no false-positives) */
    if (ch_ok && pwr_ok && fu_ok && baud_ok)
    {
      configured = true;
      rf->at_configured = 1U;
    }
  }

  if (rf->set_port != NULL)
  {
    HAL_GPIO_WritePin(rf->set_port, rf->set_pin, GPIO_PIN_SET);
    HAL_Delay(RF_HC12_SET_EXIT_DELAY_MS);
  }

  /* ALWAYS leave UART at target_baud (115200), even if AT had no response (/SET open) */
  (void)HAL_UART_Abort(rf->huart);
  rf->huart->Init.BaudRate = target_baud;
  (void)HAL_UART_Init(rf->huart);
  rf_uart_flush_rx(rf->huart);

  return configured;
}

bool RF_HC12_SetupRoutine(RF_HC12_t *rf, uint8_t channel, uint32_t target_baud)
{
  return RF_HC12_SetAddress(rf, channel, RF_HC12_DEFAULT_POWER, target_baud);
}

HAL_StatusTypeDef RF_HC12_StartRx(RF_HC12_t *rf)
{
  if ((rf == NULL) || (rf->huart == NULL))
  {
    return HAL_ERROR;
  }

  rf->rx_read_pos = 0U;
  rf->frame_pos = 0U;
  memset(rf->rx_dma_buf, 0, sizeof(rf->rx_dma_buf));

  if (rf->huart->hdmarx == NULL)
  {
    return HAL_OK;
  }

  (void)HAL_UART_AbortReceive(rf->huart);
  rf_uart_flush_rx(rf->huart);
  return HAL_UART_Receive_DMA(rf->huart, rf->rx_dma_buf, RF_HC12_RX_BUF_SIZE);
}

void RF_HC12_FeedBytes(RF_HC12_t *rf, const uint8_t *data, uint16_t len, uint32_t now_ms)
{
  if ((rf == NULL) || (data == NULL))
  {
    return;
  }
  for (uint16_t i = 0U; i < len; i++)
  {
    rf_feed_byte(rf, data[i], now_ms);
  }
}

void RF_HC12_ProcessRx(RF_HC12_t *rf, uint32_t now_ms)
{
  if ((rf == NULL) || (rf->huart == NULL) || (rf->huart->hdmarx == NULL))
  {
    return;
  }

  /* Self-heal if HAL UART RX stopped due to an unhandled noise/framing transient */
  if (rf->huart->RxState == HAL_UART_STATE_READY)
  {
    (void)RF_HC12_StartRx(rf);
    return;
  }

  uint32_t remaining = __HAL_DMA_GET_COUNTER(rf->huart->hdmarx);
  if (remaining > RF_HC12_RX_BUF_SIZE)
  {
    remaining = RF_HC12_RX_BUF_SIZE;
  }

  uint16_t write_pos = (uint16_t)(RF_HC12_RX_BUF_SIZE - remaining);
  if (write_pos >= RF_HC12_RX_BUF_SIZE)
  {
    write_pos = 0U;
  }

  while (rf->rx_read_pos != write_pos)
  {
    uint8_t byte = rf->rx_dma_buf[rf->rx_read_pos];
    rf->rx_read_pos++;
    if (rf->rx_read_pos >= RF_HC12_RX_BUF_SIZE)
    {
      rf->rx_read_pos = 0U;
    }
    rf_feed_byte(rf, byte, now_ms);
  }
}

void RF_HC12_UartErrorCallback(RF_HC12_t *rf, UART_HandleTypeDef *huart)
{
  if ((rf == NULL) || (rf->huart == NULL) || (huart == NULL) ||
      (huart->Instance != rf->huart->Instance))
  {
    return;
  }

  rf->stats.uart_err_recoveries++;
  rf_uart_flush_rx(rf->huart);
  if (rf->huart->hdmarx != NULL)
  {
    (void)RF_HC12_StartRx(rf);
  }
}

bool RF_HC12_PopLatestFrame(RF_HC12_t *rf, RF_Frame_t *out_frame)
{
  if ((rf == NULL) || (out_frame == NULL) || (rf->rx_updated == 0U))
  {
    return false;
  }
  *out_frame = rf->last_rx;
  rf->rx_updated = 0U;
  return true;
}

bool RF_HC12_IsLinkAlive(const RF_HC12_t *rf, uint32_t now_ms, uint32_t timeout_ms)
{
  if ((rf == NULL) || (rf->has_ever_rx == 0U))
  {
    return false;
  }
  return ((uint32_t)(now_ms - rf->last_rx_tick_ms) <= timeout_ms);
}

HAL_StatusTypeDef RF_HC12_SendFrame(RF_HC12_t *rf,
                                    RF_NodeId_t dst_id,
                                    RF_MsgType_t msg_type,
                                    const uint8_t *payload,
                                    uint8_t len)
{
  if ((rf == NULL) || (rf->huart == NULL) || (len > RF_HC12_MAX_PAYLOAD))
  {
    return HAL_ERROR;
  }
  if ((len > 0U) && (payload == NULL))
  {
    return HAL_ERROR;
  }
  if (rf->huart->gState != HAL_UART_STATE_READY)
  {
    rf->stats.tx_busy_drops++;
    return HAL_BUSY;
  }

  rf->tx_buf[0] = RF_HC12_SOF0;
  rf->tx_buf[1] = RF_HC12_SOF1;
  rf->tx_buf[2] = RF_HC12_PROTO_VERSION;
  rf->tx_buf[3] = rf->local_id;
  rf->tx_buf[4] = (uint8_t)dst_id;
  rf->tx_buf[5] = (uint8_t)msg_type;
  rf->tx_buf[6] = rf->tx_seq++;
  rf->tx_buf[7] = len;

  if (len > 0U)
  {
    memcpy(&rf->tx_buf[RF_HC12_HEADER_SIZE], payload, len);
  }

  uint16_t crc = RF_HC12_Crc16Ccitt(&rf->tx_buf[2], (uint16_t)(6U + len));
  rf->tx_buf[8U + len] = (uint8_t)(crc >> 8U);
  rf->tx_buf[9U + len] = (uint8_t)(crc & 0xFFU);

  uint16_t total_len = (uint16_t)(RF_HC12_HEADER_SIZE + len + RF_HC12_CRC_SIZE);
  HAL_StatusTypeDef st;
  if (rf->huart->hdmatx != NULL)
  {
    st = HAL_UART_Transmit_DMA(rf->huart, rf->tx_buf, total_len);
  }
  else
  {
    st = HAL_UART_Transmit(rf->huart, rf->tx_buf, total_len, 25U);
  }

  if (st == HAL_OK)
  {
    rf->stats.tx_ok_frames++;
  }
  else if (st == HAL_BUSY)
  {
    rf->stats.tx_busy_drops++;
  }
  return st;
}

HAL_StatusTypeDef RF_HC12_SendTdkCmd(RF_HC12_t *rf,
                                     RF_NodeId_t dst_id,
                                     const RF_PayloadTdk_t *cmd)
{
  (void)dst_id;
  if ((rf == NULL) || (rf->huart == NULL) || (cmd == NULL))
  {
    return HAL_ERROR;
  }
  if (rf->huart->gState != HAL_UART_STATE_READY)
  {
    rf->stats.tx_busy_drops++;
    return HAL_BUSY;
  }

  /* 6-byte ultra-fast competition frame (proven in RB2026_TX_PS2_NEW):
   * [0] = 255 (RF_6BYTE_SOF / 0xFF)
   * [1] = JoyY (cmd->joy_sum[0])
   * [2] = JoyX (cmd->joy_sum[1])
   * [3] = Btn1 (cmd->ps2_btn[1])
   * [4] = Btn0 (cmd->ps2_btn[0])
   * [5] = Cnt  (rf->tx_seq: 0..249)
   */
  rf->tx_buf[0] = RF_6BYTE_SOF;
  rf->tx_buf[1] = cmd->joy_sum[0];
  rf->tx_buf[2] = cmd->joy_sum[1];
  rf->tx_buf[3] = cmd->ps2_btn[1];
  rf->tx_buf[4] = cmd->ps2_btn[0];
  if (rf->tx_seq >= 250U)
  {
    rf->tx_seq = 0U;
  }
  rf->tx_buf[5] = rf->tx_seq++;

  HAL_StatusTypeDef st;
  if (rf->huart->hdmatx != NULL)
  {
    st = HAL_UART_Transmit_DMA(rf->huart, rf->tx_buf, RF_6BYTE_LEN);
  }
  else
  {
    st = HAL_UART_Transmit(rf->huart, rf->tx_buf, RF_6BYTE_LEN, 25U);
  }

  if (st == HAL_OK)
  {
    rf->stats.tx_ok_frames++;
  }
  else if (st == HAL_BUSY)
  {
    rf->stats.tx_busy_drops++;
  }
  return st;
}

HAL_StatusTypeDef RF_HC12_SendManualToAuto(RF_HC12_t *rf,
                                           RF_NodeId_t dst_id,
                                           const RF_PayloadManualToAuto_t *msg)
{
  uint8_t buf[RF_PAYLOAD_MANUAL_AUTO_LEN];
  if (msg == NULL)
  {
    return HAL_ERROR;
  }

  buf[0]  = msg->cmd;
  buf[1]  = msg->mode;
  buf[2]  = msg->flags;
  buf[3]  = msg->reserved;
  buf[4]  = (uint8_t)(msg->limit_mask & 0xFFU);
  buf[5]  = (uint8_t)((msg->limit_mask >> 8U) & 0xFFU);
  buf[6]  = (uint8_t)((uint16_t)msg->arg0 & 0xFFU);
  buf[7]  = (uint8_t)(((uint16_t)msg->arg0 >> 8U) & 0xFFU);
  buf[8]  = (uint8_t)((uint16_t)msg->arg1 & 0xFFU);
  buf[9]  = (uint8_t)(((uint16_t)msg->arg1 >> 8U) & 0xFFU);
  buf[10] = (uint8_t)((uint16_t)msg->arg2 & 0xFFU);
  buf[11] = (uint8_t)(((uint16_t)msg->arg2 >> 8U) & 0xFFU);
  buf[12] = (uint8_t)((uint16_t)msg->arg3 & 0xFFU);
  buf[13] = (uint8_t)(((uint16_t)msg->arg3 >> 8U) & 0xFFU);

  return RF_HC12_SendFrame(rf, dst_id, RF_MSG_MANUAL_TO_AUTO, buf, RF_PAYLOAD_MANUAL_AUTO_LEN);
}

bool RF_HC12_DecodeTdkCmd(const RF_Frame_t *frame, RF_PayloadTdk_t *out_cmd)
{
  if ((frame == NULL) || (out_cmd == NULL) ||
      (frame->msg_type != (uint8_t)RF_MSG_TDK_CMD) ||
      (frame->len != RF_PAYLOAD_TDK_LEN))
  {
    return false;
  }

  out_cmd->ps2_addr    = frame->payload[0];
  out_cmd->joy_sum[0]  = frame->payload[1];
  out_cmd->joy_sum[1]  = frame->payload[2];
  out_cmd->joy_left[0] = frame->payload[3];
  out_cmd->joy_left[1] = frame->payload[4];
  out_cmd->ps2_btn[0]  = frame->payload[5];
  out_cmd->ps2_btn[1]  = frame->payload[6];
  out_cmd->ext_sw      = frame->payload[7];
  out_cmd->aux         = frame->payload[8];
  return true;
}

bool RF_HC12_DecodeManualToAuto(const RF_Frame_t *frame, RF_PayloadManualToAuto_t *out_msg)
{
  if ((frame == NULL) || (out_msg == NULL) ||
      (frame->msg_type != (uint8_t)RF_MSG_MANUAL_TO_AUTO) ||
      (frame->len != RF_PAYLOAD_MANUAL_AUTO_LEN))
  {
    return false;
  }

  out_msg->cmd        = frame->payload[0];
  out_msg->mode       = frame->payload[1];
  out_msg->flags      = frame->payload[2];
  out_msg->reserved   = frame->payload[3];
  out_msg->limit_mask = (uint16_t)frame->payload[4] | ((uint16_t)frame->payload[5] << 8U);
  out_msg->arg0       = (int16_t)((uint16_t)frame->payload[6]  | ((uint16_t)frame->payload[7]  << 8U));
  out_msg->arg1       = (int16_t)((uint16_t)frame->payload[8]  | ((uint16_t)frame->payload[9]  << 8U));
  out_msg->arg2       = (int16_t)((uint16_t)frame->payload[10] | ((uint16_t)frame->payload[11] << 8U));
  out_msg->arg3       = (int16_t)((uint16_t)frame->payload[12] | ((uint16_t)frame->payload[13] << 8U));
  return true;
}
