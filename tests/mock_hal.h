/**
  ******************************************************************************
  * @file    mock_hal.h
  * @brief   Host-side STM32 HAL mock & virtual sensor/bus simulator header
  *          for compiling MainRB2/MainSamsungRB/PidBoard C modules under GCC
  *          with AddressSanitizer & UBSan.
  ******************************************************************************
  */

#ifndef TESTS_MOCK_HAL_H_
#define TESTS_MOCK_HAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  HAL_OK      = 0x00U,
  HAL_ERROR   = 0x01U,
  HAL_BUSY    = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET   = 1U
} GPIO_PinState;

#define GPIO_PIN_0   ((uint16_t)0x0001U)
#define GPIO_PIN_1   ((uint16_t)0x0002U)
#define GPIO_PIN_2   ((uint16_t)0x0004U)
#define GPIO_PIN_3   ((uint16_t)0x0008U)
#define GPIO_PIN_4   ((uint16_t)0x0010U)
#define GPIO_PIN_5   ((uint16_t)0x0020U)
#define GPIO_PIN_6   ((uint16_t)0x0040U)
#define GPIO_PIN_7   ((uint16_t)0x0080U)
#define GPIO_PIN_8   ((uint16_t)0x0100U)
#define GPIO_PIN_9   ((uint16_t)0x0200U)
#define GPIO_PIN_10  ((uint16_t)0x0400U)
#define GPIO_PIN_11  ((uint16_t)0x0800U)
#define GPIO_PIN_12  ((uint16_t)0x1000U)
#define GPIO_PIN_13  ((uint16_t)0x2000U)
#define GPIO_PIN_14  ((uint16_t)0x4000U)
#define GPIO_PIN_15  ((uint16_t)0x8000U)

typedef struct
{
  uint32_t BSRR;
  uint32_t ODR;
  uint32_t IDR;
} GPIO_TypeDef;

typedef struct
{
  uint32_t SR;
  uint32_t DR;
} USART_TypeDef;

typedef struct
{
  uint32_t CR1;
} I2C_TypeDef;

typedef struct
{
  uint32_t CR2;
} ADC_TypeDef;

typedef struct
{
  uint32_t CNT;
  uint32_t ARR;
  uint32_t CCR1;
  uint32_t CCR2;
  uint32_t CCR3;
  uint32_t CCR4;
} TIM_TypeDef;

typedef struct
{
  uint32_t State;
  uint32_t cndtr;
} DMA_HandleTypeDef;

#define HAL_DMA_STATE_READY      0x01U

#define HAL_UART_STATE_RESET     0x00U
#define HAL_UART_STATE_READY     0x20U
#define HAL_UART_STATE_BUSY_TX   0x21U
#define HAL_UART_STATE_BUSY_RX   0x22U

#define HAL_UART_ERROR_NONE      0x00000000U
#define HAL_UART_ERROR_PE        0x00000001U
#define HAL_UART_ERROR_NE        0x00000002U
#define HAL_UART_ERROR_FE        0x00000004U
#define HAL_UART_ERROR_ORE       0x00000008U

#define HAL_ADC_STATE_READY      0x00000001U
#define HAL_ADC_STATE_REG_BUSY   0x00000100U

#define HAL_I2C_STATE_READY      0x20U
#define HAL_I2C_ERROR_NONE       0x00000000U
#define HAL_I2C_ERROR_BERR       0x00000001U
#define HAL_I2C_ERROR_AF         0x00000004U
#define HAL_I2C_ERROR_TIMEOUT    0x00000020U

#define I2C_MEMADD_SIZE_8BIT     0x00000001U
#define I2C_MEMADD_SIZE_16BIT    0x00000002U

#define TIM_CHANNEL_1            0x00000000U
#define TIM_CHANNEL_2            0x00000004U
#define TIM_CHANNEL_3            0x00000008U
#define TIM_CHANNEL_4            0x0000000CU

typedef struct
{
  uint32_t BaudRate;
} UART_InitTypeDef;

typedef struct
{
  USART_TypeDef     *Instance;
  UART_InitTypeDef   Init;
  DMA_HandleTypeDef *hdmarx;
  DMA_HandleTypeDef *hdmatx;
  uint32_t           gState;
  uint32_t           RxState;
  uint32_t           ErrorCode;
  uint8_t           *pRxBuffPtr;
  uint16_t           RxXferSize;
  uint8_t            last_tx_buf[64];
  uint16_t           last_tx_len;
  uint32_t           tx_calls;
} UART_HandleTypeDef;

typedef struct
{
  I2C_TypeDef *Instance;
  uint32_t     State;
  uint32_t     ErrorCode;
} I2C_HandleTypeDef;

typedef struct
{
  ADC_TypeDef       *Instance;
  uint32_t           State;
  DMA_HandleTypeDef *DMA_Handle;
  uint32_t          *dma_dest;
} ADC_HandleTypeDef;

typedef struct
{
  TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

extern GPIO_TypeDef g_mock_gpioa;
extern GPIO_TypeDef g_mock_gpiob;
extern GPIO_TypeDef g_mock_gpioc;
extern GPIO_TypeDef g_mock_gpiod;
extern USART_TypeDef g_mock_usart1;
extern USART_TypeDef g_mock_usart2;
extern USART_TypeDef g_mock_uart4;
extern I2C_TypeDef g_mock_i2c2;
extern I2C_TypeDef g_mock_i2c3;
extern ADC_TypeDef g_mock_adc1;
extern TIM_TypeDef g_mock_tim3;
extern TIM_TypeDef g_mock_tim4;
extern TIM_TypeDef g_mock_tim7;
extern TIM_TypeDef g_mock_tim12;

#define GPIOA  (&g_mock_gpioa)
#define GPIOB  (&g_mock_gpiob)
#define GPIOC  (&g_mock_gpioc)
#define GPIOD  (&g_mock_gpiod)
#define USART1 (&g_mock_usart1)
#define USART2 (&g_mock_usart2)
#define UART4  (&g_mock_uart4)
#define I2C2   (&g_mock_i2c2)
#define I2C3   (&g_mock_i2c3)
#define ADC1   (&g_mock_adc1)
#define TIM3   (&g_mock_tim3)
#define TIM4   (&g_mock_tim4)
#define TIM7   (&g_mock_tim7)
#define TIM12  (&g_mock_tim12)

/* MainRB PCB Pin Map */
#define L3_Pin            GPIO_PIN_13
#define L3_GPIO_Port      GPIOC
#define L2_Pin            GPIO_PIN_14
#define L2_GPIO_Port      GPIOC
#define L1_Pin            GPIO_PIN_15
#define L1_GPIO_Port      GPIOC
#define s3_Pin            GPIO_PIN_0
#define s3_GPIO_Port      GPIOC
#define s2_Pin            GPIO_PIN_1
#define s2_GPIO_Port      GPIOC
#define s1_Pin            GPIO_PIN_2
#define s1_GPIO_Port      GPIOC
#define s0_Pin            GPIO_PIN_3
#define s0_GPIO_Port      GPIOC
#define RE_DE_Pin         GPIO_PIN_1
#define RE_DE_GPIO_Port   GPIOA
#define L11_Pin           GPIO_PIN_2
#define L11_GPIO_Port     GPIOB
#define SW1_Pin           GPIO_PIN_12
#define SW1_GPIO_Port     GPIOB
#define LS4_Pin           GPIO_PIN_13
#define LS4_GPIO_Port     GPIOB
#define LS3_Pin           GPIO_PIN_6
#define LS3_GPIO_Port     GPIOC
#define LS2_Pin           GPIO_PIN_7
#define LS2_GPIO_Port     GPIOC
#define LS1_Pin           GPIO_PIN_8
#define LS1_GPIO_Port     GPIOC
#define SW2_Pin           GPIO_PIN_15
#define SW2_GPIO_Port     GPIOA
#define L10_Pin           GPIO_PIN_12
#define L10_GPIO_Port     GPIOC
#define L9_Pin            GPIO_PIN_2
#define L9_GPIO_Port      GPIOD
#define L8_Pin            GPIO_PIN_3
#define L8_GPIO_Port      GPIOB
#define L7_Pin            GPIO_PIN_4
#define L7_GPIO_Port      GPIOB
#define L6_Pin            GPIO_PIN_5
#define L6_GPIO_Port      GPIOB
#define L5_Pin            GPIO_PIN_6
#define L5_GPIO_Port      GPIOB
#define L4_Pin            GPIO_PIN_7
#define L4_GPIO_Port      GPIOB

#define HAL_IS_BIT_SET(REG, BIT)       (((REG) & (BIT)) == (BIT))
#define __HAL_DMA_GET_COUNTER(hdma)    ((hdma)->cndtr)
#define __HAL_TIM_SET_COUNTER(htim, v) ((void)((htim)->Instance->CNT = (v)))
#define __HAL_TIM_SET_COMPARE(htim, c, v) \
  do { \
    if ((c) == TIM_CHANNEL_1) (htim)->Instance->CCR1 = (v); \
    else if ((c) == TIM_CHANNEL_2) (htim)->Instance->CCR2 = (v); \
    else if ((c) == TIM_CHANNEL_3) (htim)->Instance->CCR3 = (v); \
    else (htim)->Instance->CCR4 = (v); \
  } while (0)

#define __HAL_UART_FLUSH_DRREGISTER(huart) ((void)((huart)->Instance->DR))
#define __HAL_UART_CLEAR_OREFLAG(huart)    ((void)((huart)->Instance->SR &= ~HAL_UART_ERROR_ORE))
#define __HAL_UART_CLEAR_FEFLAG(huart)     ((void)((huart)->Instance->SR &= ~HAL_UART_ERROR_FE))
#define __HAL_UART_CLEAR_NEFLAG(huart)     ((void)((huart)->Instance->SR &= ~HAL_UART_ERROR_NE))
#define __HAL_UART_CLEAR_PEFLAG(huart)     ((void)((huart)->Instance->SR &= ~HAL_UART_ERROR_PE))

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint32_t HAL_I2C_GetError(const I2C_HandleTypeDef *hi2c);

/* ---- Virtual Bus & Sensor Simulation Controls for Tests ---- */
#define MOCK_VL53_MAX_SENSORS 4U

typedef struct
{
  bool          present;
  bool          stuck_addr52;
  bool          hang_boot;
  bool          hang_ranging;
  bool          force_nack;
  GPIO_TypeDef *xshut_port;
  uint16_t      xshut_pin;
  uint8_t       addr_8bit;
  bool          prev_xshut;
  uint8_t       regs[0x0120];
  uint8_t       init_cfg_snapshot[91];
  bool          init_cfg_captured;
  bool          ranging_active;
  bool          data_ready;
  uint8_t       range_status;
  uint16_t      dist_mm;
  uint32_t      cfg_writes;
  uint32_t      fw_cfg_writes;
} Mock_VL53L1X_t;

typedef struct
{
  bool     present;
  uint8_t  addr_8bit;
  uint8_t  page;
  uint8_t  opr_mode;
  uint8_t  calib_stat;
  float    yaw_deg;
  float    roll_deg;
  float    pitch_deg;
  uint32_t i2c_tx_count;
  uint32_t i2c_rx_count;
  uint32_t nack_remaining;
  bool     bus_stuck;
  uint8_t  last_reg;
} Mock_BNO055_t;

extern Mock_VL53L1X_t g_mock_vl53[MOCK_VL53_MAX_SENSORS];
extern Mock_BNO055_t  g_mock_bno;
extern uint32_t       g_mock_i2c_recover_calls;
extern uint32_t       g_mock_uart_abort_rx_calls;

void MockHal_ResetAll(void);
void MockHal_SetTick(uint32_t ms);
void MockHal_AdvanceTick(uint32_t ms);
bool MockHal_GetGpioOut(GPIO_TypeDef *port, uint16_t pin);

void MockHal_VL53SetupDefault(uint8_t idx, GPIO_TypeDef *port, uint16_t pin, uint16_t dist_mm);
void MockHal_VL53SetDistance(uint8_t idx, uint16_t dist_mm, uint8_t range_status);

void MockHal_UartInjectDmaBytes(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_MOCK_HAL_H_ */
