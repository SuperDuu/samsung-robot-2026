/**
  ******************************************************************************
  * @file    rf_hc12.h
  * @brief   Portable framed HC-12 RF protocol, auto-baud AT setup, circular DMA
  *          RX stream parser, UART error recovery, and strict source-ID filter.
  ******************************************************************************
  */

#ifndef INC_RF_HC12_H_
#define INC_RF_HC12_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#define RF_HC12_SOF0               0xAAU
#define RF_HC12_SOF1               0x55U
#define RF_HC12_PROTO_VERSION      0x01U

#define RF_HC12_HEADER_SIZE        8U
#define RF_HC12_CRC_SIZE           2U
#define RF_HC12_MAX_PAYLOAD        16U
#define RF_HC12_MAX_FRAME_SIZE     (RF_HC12_HEADER_SIZE + RF_HC12_MAX_PAYLOAD + RF_HC12_CRC_SIZE) /* 26B */
#define RF_HC12_RX_BUF_SIZE        128U

#define RF_HC12_DEFAULT_CHANNEL    59U
#define RF_HC12_DEFAULT_BAUD       115200U
#define RF_HC12_DEFAULT_TIMEOUT_MS 250U

typedef enum
{
  RF_NODE_UNKNOWN = 0x00U,
  RF_NODE_TDK     = 0x01U, /* Handheld PS2 transmitter (tay_dk) */
  RF_NODE_MANUAL  = 0x02U, /* Manual robot (MainSamsungRB) */
  RF_NODE_AUTO    = 0x03U, /* Autonomous robot (MainRB2) */
  RF_NODE_BCAST   = 0xFFU  /* Broadcast destination */
} RF_NodeId_t;

#define RF_SRC_MASK(node_id)       (1UL << ((uint32_t)(node_id) & 0x1FU))
#define RF_SRC_MASK_ALL            (0xFFFFFFFFUL)

typedef enum
{
  RF_MSG_HEARTBEAT      = 0x01U,
  RF_MSG_TDK_CMD        = 0x10U,
  RF_MSG_MANUAL_TO_AUTO = 0x20U
} RF_MsgType_t;

/* Bits carried in RF_PayloadTdk_t.ext_sw */
#define RF_TDK_EXT_SW_BIT          (1U << 0) /* PA8 SW pressed */
#define RF_TDK_EXT_BUTTON_BIT      (1U << 1) /* PB3 BUTTON pressed */
#define RF_TDK_PS2_VALID_BIT       (1U << 2) /* PS2 responded (0x41 or 0x73) */
#define RF_TDK_PS2_ANALOG_BIT      (1U << 3) /* PS2 in full analog mode (0x73) */

/* Bits carried in RF_PayloadManualToAuto_t.flags */
#define RF_MANUAL_FLAG_TDK_LINK    (1U << 0) /* Manual robot has live tdk RF link */
#define RF_MANUAL_FLAG_SW1         (1U << 1) /* MainSamsungRB SW1 pressed */
#define RF_MANUAL_FLAG_SW2         (1U << 2) /* MainSamsungRB SW2 pressed */

#define RF_PAYLOAD_TDK_LEN         9U
#define RF_PAYLOAD_MANUAL_AUTO_LEN 14U

typedef struct
{
  uint8_t ps2_addr;
  uint8_t joy_sum[2];
  uint8_t joy_left[2];
  uint8_t ps2_btn[2];
  uint8_t ext_sw;
  uint8_t aux;
} RF_PayloadTdk_t;

typedef struct
{
  uint8_t  cmd;
  uint8_t  mode;
  uint8_t  flags;
  uint8_t  reserved;
  uint16_t limit_mask;
  int16_t  arg0;
  int16_t  arg1;
  int16_t  arg2;
  int16_t  arg3;
} RF_PayloadManualToAuto_t;

typedef struct
{
  uint8_t version;
  uint8_t src_id;
  uint8_t dst_id;
  uint8_t msg_type;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[RF_HC12_MAX_PAYLOAD];
} RF_Frame_t;

typedef struct
{
  uint32_t tx_ok_frames;
  uint32_t tx_busy_drops;
  uint32_t rx_ok_frames;
  uint32_t crc_err_frames;
  uint32_t len_err_frames;
  uint32_t rejected_src_frames;
  uint32_t rejected_tdk_frames;
  uint32_t rejected_dst_frames;
  uint32_t uart_err_recoveries;
  uint8_t  last_rejected_src;
} RF_HC12_Stats_t;

struct RF_HC12_s;
typedef void (*RF_HC12_RxCallback_t)(struct RF_HC12_s *rf, const RF_Frame_t *frame, void *ctx);

typedef struct RF_HC12_s
{
  UART_HandleTypeDef  *huart;
  GPIO_TypeDef        *set_port;
  uint16_t             set_pin;
  uint8_t              local_id;
  uint32_t             allowed_src_mask;
  uint8_t              tx_seq;
  uint8_t              tx_buf[RF_HC12_MAX_FRAME_SIZE];
  uint8_t              rx_dma_buf[RF_HC12_RX_BUF_SIZE];
  uint16_t             rx_read_pos;
  uint8_t              frame_buf[RF_HC12_MAX_FRAME_SIZE];
  uint8_t              frame_pos;
  uint8_t              at_found;
  uint8_t              at_configured;
  uint32_t             at_detected_baud;
  uint32_t             last_rx_tick_ms;
  uint8_t              has_ever_rx;
  uint8_t              rx_updated;
  RF_Frame_t           last_rx;
  RF_HC12_Stats_t      stats;
  RF_HC12_RxCallback_t rx_callback;
  void                *rx_callback_ctx;
} RF_HC12_t;

void RF_HC12_Init(RF_HC12_t *rf,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *set_port,
                  uint16_t set_pin,
                  RF_NodeId_t local_id,
                  uint32_t allowed_src_mask);

void RF_HC12_SetRxCallback(RF_HC12_t *rf, RF_HC12_RxCallback_t cb, void *ctx);

HAL_StatusTypeDef RF_HC12_SendAT(RF_HC12_t *rf,
                                 const char *cmd,
                                 const char *expected,
                                 uint32_t timeout_ms);

/* Scans common baud rates (9600 first), configures channel + P8 + target_baud if /SET is LOW,
 * and ALWAYS restores UART to target_baud (default 115200U) before returning. */
bool RF_HC12_SetupRoutine(RF_HC12_t *rf, uint8_t channel, uint32_t target_baud);

HAL_StatusTypeDef RF_HC12_StartRx(RF_HC12_t *rf);
void RF_HC12_ProcessRx(RF_HC12_t *rf, uint32_t now_ms);
void RF_HC12_FeedBytes(RF_HC12_t *rf, const uint8_t *data, uint16_t len, uint32_t now_ms);
void RF_HC12_UartErrorCallback(RF_HC12_t *rf, UART_HandleTypeDef *huart);

bool RF_HC12_PopLatestFrame(RF_HC12_t *rf, RF_Frame_t *out_frame);
bool RF_HC12_IsLinkAlive(const RF_HC12_t *rf, uint32_t now_ms, uint32_t timeout_ms);

HAL_StatusTypeDef RF_HC12_SendFrame(RF_HC12_t *rf,
                                    RF_NodeId_t dst_id,
                                    RF_MsgType_t msg_type,
                                    const uint8_t *payload,
                                    uint8_t len);

HAL_StatusTypeDef RF_HC12_SendTdkCmd(RF_HC12_t *rf,
                                     RF_NodeId_t dst_id,
                                     const RF_PayloadTdk_t *cmd);

HAL_StatusTypeDef RF_HC12_SendManualToAuto(RF_HC12_t *rf,
                                           RF_NodeId_t dst_id,
                                           const RF_PayloadManualToAuto_t *msg);

bool RF_HC12_DecodeTdkCmd(const RF_Frame_t *frame, RF_PayloadTdk_t *out_cmd);
bool RF_HC12_DecodeManualToAuto(const RF_Frame_t *frame, RF_PayloadManualToAuto_t *out_msg);

uint16_t RF_HC12_Crc16Ccitt(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_RF_HC12_H_ */
