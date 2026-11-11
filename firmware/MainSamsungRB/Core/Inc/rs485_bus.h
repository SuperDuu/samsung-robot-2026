/**
  ******************************************************************************
  * @file    rs485_bus.h
  * @brief   Portable half-duplex RS-485 framed binary protocol, circular DMA RX
  *          parser, TC-gated RE/DE transceiver direction control, UART error
  *          recovery, and multi-wheel command / telemetry TDMA helpers.
  *
  * Shared across MainSamsungRB (Master, 4 wheels), MainRB2 (Master, 2 wheels),
  * and PidBoard (Slaves 1..4 on HB_RBC).
  ******************************************************************************
  */

#ifndef INC_RS485_BUS_H_
#define INC_RS485_BUS_H_

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

#define RS485_SOF0                  0xAAU
#define RS485_SOF1                  0x55U
#define RS485_PROTO_VERSION         0x01U

#define RS485_HEADER_SIZE           8U
#define RS485_CRC_SIZE              2U
#define RS485_MAX_PAYLOAD           20U
#define RS485_MAX_FRAME_SIZE        (RS485_HEADER_SIZE + RS485_MAX_PAYLOAD + RS485_CRC_SIZE) /* 30B */
#define RS485_RX_BUF_SIZE           128U

#define RS485_ADDR_MASTER           0x00U
#define RS485_ADDR_SLAVE_MIN        0x01U
#define RS485_ADDR_SLAVE_MAX        0x04U
#define RS485_ADDR_BCAST            0xFFU

#define RS485_MAX_WHEELS            4U
#define RS485_CMD_WATCHDOG_MS       250U
#define RS485_SLAVE_TURNAROUND_MS   2U   /* >= 1.0 ms real bus-idle gap despite 1 ms SysTick quantization */
#define RS485_TX_WATCHDOG_MS        20U

#define RS485_PAYLOAD_MULTI_CMD_LEN 16U
#define RS485_PAYLOAD_WHEEL_CMD_LEN 6U
#define RS485_PAYLOAD_TELEM_LEN     11U
#define RS485_PAYLOAD_CFG_CMD_LEN   6U
#define RS485_PAYLOAD_CFG_ACK_LEN   4U

typedef enum
{
  RS485_MSG_MULTI_CMD = 0x10U, /* Master -> 0xFF/node: 4-wheel targets + 1 poll_id */
  RS485_MSG_WHEEL_CMD = 0x11U, /* Master -> node: single-wheel target (32-bit)    */
  RS485_MSG_TELEM     = 0x20U, /* Slave -> Master: RPM x10, duty, fault, encoder   */
  RS485_MSG_CFG_CMD   = 0x30U, /* Master -> node: PING / ID / PID / CPR / FLASH    */
  RS485_MSG_CFG_ACK   = 0x31U  /* Slave -> Master: config command acknowledge      */
} RS485_MsgType_t;

typedef enum
{
  RS485_WHEEL_DISABLED = 0U, /* IR2184 ~SD = LOW (Hi-Z coast), integral cleared  */
  RS485_WHEEL_BRAKE    = 1U, /* IR2184 ~SD = HIGH, IN1=IN2=0 (active low brake)  */
  RS485_WHEEL_DUTY     = 2U, /* Direct PWM duty command (-975 .. +975 permille)  */
  RS485_WHEEL_VELOCITY = 3U, /* Closed-loop speed command in 0.1 RPM (deci-RPM)  */
  RS485_WHEEL_POSITION = 4U  /* Cascaded position command in 0.1 deg (or counts) */
} RS485_WheelMode_t;

typedef enum
{
  RS485_CFG_PING          = 0U,
  RS485_CFG_SET_NODE_ID   = 1U,
  RS485_CFG_SET_VEL_KP_M  = 2U,  /* value in milli-units (e.g. 250 = 0.250)  */
  RS485_CFG_SET_VEL_KI_M  = 3U,  /* value in milli-units (e.g. 7000 = 7.000) */
  RS485_CFG_SET_VEL_KD_M  = 4U,  /* value in milli-units (e.g. 1 = 0.001)    */
  RS485_CFG_SET_POS_KP_M  = 5U,  /* value in milli-units (e.g. 750 = 0.750)  */
  RS485_CFG_SET_POS_KD_M  = 6U,  /* value in milli-units (e.g. 1 = 0.001)    */
  RS485_CFG_SET_CPR       = 7U,  /* encoder counts per shaft revolution      */
  RS485_CFG_SET_INVERT    = 8U,  /* param_u8: 0 = normal, 1 = invert motor+enc */
  RS485_CFG_ZERO_ENC      = 9U,  /* zero cumulative encoder count & pos ref  */
  RS485_CFG_SAVE_FLASH    = 10U, /* persist config to Flash Page 15          */
  RS485_CFG_LOAD_DEFAULTS = 11U  /* restore compile-time default config      */
} RS485_CfgSubcmd_t;

#define RS485_CMD_FLAG_ESTOP        (1U << 0)
#define RS485_CMD_FLAG_CLEAR_FAULTS (1U << 1)

#define RS485_STAT_SATURATED        (1U << 0)
#define RS485_STAT_WATCHDOG         (1U << 1)
#define RS485_STAT_FLASH_VALID      (1U << 2)
#define RS485_STAT_GATE_ENABLED     (1U << 3)
#define RS485_STAT_OVER_ERR         (1U << 4)

typedef struct
{
  uint8_t mode;
  int16_t target;
} RS485_WheelSlot_t;

typedef struct
{
  uint8_t           num_wheels;
  uint8_t           poll_id;
  uint8_t           flags;
  uint8_t           reserved;
  RS485_WheelSlot_t wheels[RS485_MAX_WHEELS];
} RS485_MultiCmd_t;

typedef struct
{
  uint8_t mode;
  uint8_t flags;
  int32_t target;
} RS485_WheelCmd_t;

typedef struct
{
  uint8_t node_id;
  uint8_t mode;
  uint8_t status_flags;
  int16_t actual_rpm_x10;
  int16_t duty_permille;
  int32_t enc_count;
} RS485_Telem_t;

typedef struct
{
  uint8_t subcmd;
  uint8_t param_u8;
  int32_t value;
} RS485_CfgCmd_t;

typedef struct
{
  uint8_t node_id;
  uint8_t subcmd;
  uint8_t status;
  uint8_t reserved;
} RS485_CfgAck_t;

typedef struct
{
  uint8_t version;
  uint8_t src_id;
  uint8_t dst_id;
  uint8_t msg_type;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[RS485_MAX_PAYLOAD];
} RS485_Frame_t;

typedef struct
{
  uint32_t tx_ok_frames;
  uint32_t tx_busy_drops;
  uint32_t rx_ok_frames;
  uint32_t crc_err_frames;
  uint32_t len_err_frames;
  uint32_t rejected_src_frames;
  uint32_t rejected_dst_frames;
  uint32_t uart_err_recoveries;
  uint32_t tx_wdog_recoveries;
} RS485_Stats_t;

struct RS485_Bus_s;
typedef void (*RS485_RxCallback_t)(struct RS485_Bus_s *bus, const RS485_Frame_t *frame, void *ctx);

typedef struct RS485_Bus_s
{
  UART_HandleTypeDef  *huart;
  GPIO_TypeDef        *re_de_port;
  uint16_t             re_de_pin;
  uint8_t              local_id;
  uint8_t              tx_seq;
  volatile uint8_t     tx_active;
  uint32_t             tx_start_ms;
  uint8_t              tx_buf[RS485_MAX_FRAME_SIZE];
  uint8_t              rx_dma_buf[RS485_RX_BUF_SIZE];
  uint16_t             rx_read_pos;
  uint8_t              frame_buf[RS485_MAX_FRAME_SIZE];
  uint8_t              frame_pos;
  uint32_t             last_rx_tick_ms;
  uint8_t              has_ever_rx;
  uint8_t              rx_updated;
  RS485_Frame_t        last_rx;
  uint8_t              pending_reply_valid;
  uint8_t              pending_reply_len;
  uint32_t             pending_reply_rx_ms;
  uint8_t              pending_reply_buf[RS485_MAX_FRAME_SIZE];
  RS485_Stats_t        stats;
  RS485_RxCallback_t   rx_callback;
  void                *rx_callback_ctx;
} RS485_Bus_t;

void RS485_Init(RS485_Bus_t *bus,
                UART_HandleTypeDef *huart,
                GPIO_TypeDef *re_de_port,
                uint16_t re_de_pin,
                uint8_t local_id);

void RS485_SetLocalId(RS485_Bus_t *bus, uint8_t local_id);
void RS485_SetRxCallback(RS485_Bus_t *bus, RS485_RxCallback_t cb, void *ctx);

HAL_StatusTypeDef RS485_StartRx(RS485_Bus_t *bus);
void RS485_ProcessRx(RS485_Bus_t *bus, uint32_t now_ms);
void RS485_FeedBytes(RS485_Bus_t *bus, const uint8_t *data, uint16_t len, uint32_t now_ms);
void RS485_TickMs(RS485_Bus_t *bus, uint32_t now_ms);

void RS485_TxCpltCallback(RS485_Bus_t *bus, UART_HandleTypeDef *huart);
void RS485_UartErrorCallback(RS485_Bus_t *bus, UART_HandleTypeDef *huart);

bool RS485_PopLatestFrame(RS485_Bus_t *bus, RS485_Frame_t *out_frame);
bool RS485_IsLinkAlive(const RS485_Bus_t *bus, uint32_t now_ms, uint32_t timeout_ms);

HAL_StatusTypeDef RS485_SendFrame(RS485_Bus_t *bus,
                                  uint8_t dst_id,
                                  RS485_MsgType_t msg_type,
                                  const uint8_t *payload,
                                  uint8_t len,
                                  uint32_t now_ms);

bool RS485_ScheduleReply(RS485_Bus_t *bus,
                         uint8_t dst_id,
                         RS485_MsgType_t msg_type,
                         const uint8_t *payload,
                         uint8_t len,
                         uint32_t now_ms);

HAL_StatusTypeDef RS485_SendMultiCmd(RS485_Bus_t *bus,
                                     const RS485_MultiCmd_t *cmd,
                                     uint32_t now_ms);

HAL_StatusTypeDef RS485_SendWheelCmd(RS485_Bus_t *bus,
                                     uint8_t dst_id,
                                     const RS485_WheelCmd_t *cmd,
                                     uint32_t now_ms);

HAL_StatusTypeDef RS485_SendCfgCmd(RS485_Bus_t *bus,
                                   uint8_t dst_id,
                                   const RS485_CfgCmd_t *cfg,
                                   uint32_t now_ms);

bool RS485_ScheduleTelemReply(RS485_Bus_t *bus,
                              uint8_t dst_id,
                              const RS485_Telem_t *telem,
                              uint32_t now_ms);

bool RS485_ScheduleCfgAckReply(RS485_Bus_t *bus,
                               uint8_t dst_id,
                               const RS485_CfgAck_t *ack,
                               uint32_t now_ms);

bool RS485_DecodeMultiCmd(const RS485_Frame_t *frame, RS485_MultiCmd_t *out_cmd);
bool RS485_DecodeWheelCmd(const RS485_Frame_t *frame, RS485_WheelCmd_t *out_cmd);
bool RS485_DecodeTelem(const RS485_Frame_t *frame, RS485_Telem_t *out_telem);
bool RS485_DecodeCfgCmd(const RS485_Frame_t *frame, RS485_CfgCmd_t *out_cfg);
bool RS485_DecodeCfgAck(const RS485_Frame_t *frame, RS485_CfgAck_t *out_ack);

uint16_t RS485_Crc16Ccitt(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_RS485_BUS_H_ */
