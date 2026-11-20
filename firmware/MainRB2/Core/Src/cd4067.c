/**
 ******************************************************************************
 * @file    cd4067.c
 * @brief   16-channel CD74HC4067M analog/digital multiplexer driver with
 *          2-point White/Black sweep calibration, 0..1000 normalization, and
 *          continuous weighted centroid [-700..+700].
 *
 * Hardware mapping on MainRB PCB:
 *   - MUX U8 (CD74HC4067M): ~E tied to GND via R30 (10k, always enabled).
 *     - s0 (U8.10) = PC3  (Channel bit 0)
 *     - s1 (U8.11) = PC2  (Channel bit 1)
 *     - s2 (U8.14) = PC1  (Channel bit 2)
 *     - s3 (U8.13) = PC0  (Channel bit 3)
 *   - Signal path: U8.COM -> R42 (1k) -> U6A (LMV358 follower, VDDA) -> R43
 * (100R) -> PA0 (ADC1_IN0).
 *   - LINE_11 mode: I0..I10 (J11..J21) mirrored 1:1 to L1..L11; I11..I15
 * available via raw[].
 *   - LIMIT_SWITCH mode: I0..I15 scanned as 16 limit switches with Schmitt
 * hysteresis + 8 ms debounce.
 *   - LINE_2X8 mode: Front row I8..I15, Rear row I0..I7, encoded onto 11
 * adjacent LEDs L1..L11.
 *
 * Timing budget (@ APB1Tim = 84 MHz, APB2 = 84 MHz):
 *   - TIM7: Prescaler = 83, Period = 99 -> 100 us/channel (10 kHz tick, 1.6 ms
 * full 16-ch sweep = 625 Hz).
 *   - ADC1: PCLK2/4 = 21 MHz, SamplingTime = 84 cycles (4.00 us) + 12 SAR
 * cycles (0.57 us) = 4.57 us.
 *   - DMA2_Stream0 (DMA_NORMAL, Prio Very High): triggers ADC_ConvCpltCallback
 * at ~5 us; switches MUX atomically via GPIOC->BSRR, leaving ~95 us passive RC
 * + LMV358 op-amp settling before next tick.
 ******************************************************************************
 */

#include "cd4067.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} CD4067_LedPin_t;

#define CD4067_MUX_SET_MASK(ch)                                                \
  ((((uint32_t)(ch) & 0x01U) ? (uint32_t)s0_Pin : 0U) |                        \
   (((uint32_t)(ch) & 0x02U) ? (uint32_t)s1_Pin : 0U) |                        \
   (((uint32_t)(ch) & 0x04U) ? (uint32_t)s2_Pin : 0U) |                        \
   (((uint32_t)(ch) & 0x08U) ? (uint32_t)s3_Pin : 0U))

#define CD4067_MUX_ALL_PINS ((uint32_t)(s0_Pin | s1_Pin | s2_Pin | s3_Pin))

#define CD4067_MUX_BSRR_ENTRY(ch)                                              \
  (CD4067_MUX_SET_MASK(ch) |                                                   \
   ((CD4067_MUX_ALL_PINS & ~CD4067_MUX_SET_MASK(ch)) << 16U))

static const uint32_t cd4067_mux_bsrr_lut[CD4067_NUM_CHANNELS] = {
    CD4067_MUX_BSRR_ENTRY(0U),  CD4067_MUX_BSRR_ENTRY(1U),
    CD4067_MUX_BSRR_ENTRY(2U),  CD4067_MUX_BSRR_ENTRY(3U),
    CD4067_MUX_BSRR_ENTRY(4U),  CD4067_MUX_BSRR_ENTRY(5U),
    CD4067_MUX_BSRR_ENTRY(6U),  CD4067_MUX_BSRR_ENTRY(7U),
    CD4067_MUX_BSRR_ENTRY(8U),  CD4067_MUX_BSRR_ENTRY(9U),
    CD4067_MUX_BSRR_ENTRY(10U), CD4067_MUX_BSRR_ENTRY(11U),
    CD4067_MUX_BSRR_ENTRY(12U), CD4067_MUX_BSRR_ENTRY(13U),
    CD4067_MUX_BSRR_ENTRY(14U), CD4067_MUX_BSRR_ENTRY(15U)};

/* Active-LOW line LEDs L1..L11 on MainRB PCB */
static const CD4067_LedPin_t cd4067_line_leds[CD4067_NUM_LINE_SENSORS] = {
    {L1_GPIO_Port, L1_Pin},   {L2_GPIO_Port, L2_Pin},  {L3_GPIO_Port, L3_Pin},
    {L4_GPIO_Port, L4_Pin},   {L5_GPIO_Port, L5_Pin},  {L6_GPIO_Port, L6_Pin},
    {L7_GPIO_Port, L7_Pin},   {L8_GPIO_Port, L8_Pin},  {L9_GPIO_Port, L9_Pin},
    {L10_GPIO_Port, L10_Pin}, {L11_GPIO_Port, L11_Pin}};

/* Signed weights in tenths for 11-sensor linear array (I0..I10 -> -1000..+1000)
 */
static const int16_t cd4067_line_weights_x10[CD4067_NUM_LINE_SENSORS] = {
    -1000, -800, -600, -400, -200, 0, 200, 400, 600, 800, 1000};

/* Left-to-right weights for either 8-sensor row (-700..+700). */
static const int16_t cd4067_row_weights_x10[CD4067_NUM_ROW_SENSORS] = {
    -700, -500, -300, -100, 100, 300, 500, 700};

CD4067_Data_t cd4067_data = {0};

static ADC_HandleTypeDef *cd4067_hadc = NULL;
static TIM_HandleTypeDef *cd4067_htim = NULL;
static uint32_t cd4067_dma_word = 0U;
static volatile uint8_t cd4067_active_ch = 0U;
static volatile uint8_t cd4067_next_ch = 0U;
static volatile uint8_t cd4067_cal_stage =
    0U; /* 0 = warmup, 1 = avg tare, 2 = run, 3 = 2-pt record */
static uint32_t cd4067_offset_accum[CD4067_NUM_CHANNELS] = {0};
static uint64_t cd4067_cal_sum[CD4067_NUM_CHANNELS] = {0};
static uint32_t cd4067_cal_samples[CD4067_NUM_CHANNELS] = {0};
static uint8_t cd4067_limit_raw_level[CD4067_NUM_CHANNELS] = {0};
static uint8_t cd4067_limit_stable_level[CD4067_NUM_CHANNELS] = {0};
static uint8_t cd4067_limit_streak[CD4067_NUM_CHANNELS] = {0};

static void cd4067_reset_limit_debounce(uint16_t active_low_mask) {
  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    uint8_t idle_level =
        ((active_low_mask & (uint16_t)(1U << i)) != 0U) ? 1U : 0U;
    cd4067_limit_raw_level[i] = idle_level;
    cd4067_limit_stable_level[i] = idle_level;
    cd4067_limit_streak[i] = 0U;
  }
}

static void cd4067_clear_runtime_outputs(void) {
  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    cd4067_data.norm[i] = 0U;
    cd4067_data.sensor_active[i] = 0U;
  }
  cd4067_data.active_mask = 0U;
  cd4067_data.active_count = 0U;
  cd4067_data.line_error = 0;
  cd4067_data.limit_mask = 0U;
  cd4067_data.front_mask = 0U;
  cd4067_data.rear_mask = 0U;
  cd4067_data.front_count = 0U;
  cd4067_data.rear_count = 0U;
  cd4067_data.front_error = 0;
  cd4067_data.rear_error = 0;
  cd4067_data.front_centroid_x10 = 0;
  cd4067_data.rear_centroid_x10 = 0;
}

static bool cd4067_is_channel_used(CD4067_Mode_t mode, uint8_t ch) {
  if (mode == CD4067_MODE_LINE_2X8) {
    return (ch < CD4067_NUM_CHANNELS);
  }
  if (mode == CD4067_MODE_LINE_11) {
    return (ch < CD4067_NUM_LINE_SENSORS);
  }
  return false;
}

static bool cd4067_validate_min_max(CD4067_Mode_t mode,
                                    const uint16_t min_arr[CD4067_NUM_CHANNELS],
                                    const uint16_t max_arr[CD4067_NUM_CHANNELS],
                                    uint16_t *out_low_mask) {
  uint16_t bad = 0U;
  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    if (!cd4067_is_channel_used(mode, i)) {
      continue;
    }
    if ((max_arr[i] <= min_arr[i]) ||
        ((uint16_t)(max_arr[i] - min_arr[i]) < CD4067_MIN_CONTRAST_ADC) ||
        (max_arr[i] > 4095U)) {
      bad |= (uint16_t)(1U << i);
    }
  }
  if (out_low_mask != NULL) {
    *out_low_mask = bad;
  }
  return (bad == 0U);
}

void CD4067_SelectChannel(uint8_t channel) {
  GPIOC->BSRR = cd4067_mux_bsrr_lut[channel & 0x0FU];
}

void CD4067_SetLineLed(uint8_t index, bool on) {
  if (index >= CD4067_NUM_LINE_SENSORS) {
    return;
  }
  HAL_GPIO_WritePin(cd4067_line_leds[index].port, cd4067_line_leds[index].pin,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void CD4067_SetAllLineLeds(bool on) {
  for (uint8_t i = 0U; i < CD4067_NUM_LINE_SENSORS; i++) {
    CD4067_SetLineLed(i, on);
  }
}

void CD4067_StartCalibration(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    cd4067_offset_accum[i] = 0U;
    cd4067_data.offset[i] = 0U;
  }
  cd4067_clear_runtime_outputs();
  cd4067_data.sweep_count = 0U;
  cd4067_data.two_point_active = 0U;
  cd4067_data.two_point_recording = 0U;
  cd4067_data.low_contrast_mask = 0U;

  if (cd4067_data.mode == CD4067_MODE_LIMIT_SWITCH) {
    cd4067_reset_limit_debounce(cd4067_data.active_low_mask);
    cd4067_data.cal_done = 1U;
    cd4067_cal_stage = 2U;
  } else {
    cd4067_data.cal_done = 0U;
    cd4067_cal_stage = 0U;
  }

  if (primask == 0U) {
    __enable_irq();
  }

  CD4067_SetAllLineLeds(false);
}

void CD4067_BeginTwoPointCal(void) {
  if (cd4067_data.mode == CD4067_MODE_LIMIT_SWITCH) {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    cd4067_data.cal_min[i] = 4095U;
    cd4067_data.cal_max[i] = 0U;
    cd4067_cal_sum[i] = 0U;
    cd4067_cal_samples[i] = 0U;
  }
  cd4067_clear_runtime_outputs();
  cd4067_data.sweep_count = 0U;
  cd4067_data.low_contrast_mask = 0U;
  cd4067_data.cal_done = 0U;
  cd4067_data.two_point_recording = 1U;
  cd4067_cal_stage = 3U;

  if (primask == 0U) {
    __enable_irq();
  }
}

bool CD4067_FinishTwoPointCal(CD4067_Polarity_t polarity) {
  if (cd4067_data.mode == CD4067_MODE_LIMIT_SWITCH) {
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint16_t min_snap[CD4067_NUM_CHANNELS];
  uint16_t max_snap[CD4067_NUM_CHANNELS];
  uint32_t high_votes = 0U;
  uint32_t low_votes = 0U;

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    min_snap[i] = cd4067_data.cal_min[i];
    max_snap[i] = cd4067_data.cal_max[i];

    if (cd4067_is_channel_used(cd4067_data.mode, i) &&
        (cd4067_cal_samples[i] > 0U) && (max_snap[i] > min_snap[i])) {
      uint32_t mean_val = (uint32_t)(cd4067_cal_sum[i] / cd4067_cal_samples[i]);
      uint32_t mid_val = ((uint32_t)min_snap[i] + (uint32_t)max_snap[i]) / 2U;
      /* Line is narrow vs floor: average stays closer to background, so if mean
       * < mid, line is HIGH */
      if (mean_val <= mid_val) {
        high_votes++;
      } else {
        low_votes++;
      }
    }
  }

  uint16_t bad_mask = 0U;
  bool ok =
      cd4067_validate_min_max(cd4067_data.mode, min_snap, max_snap, &bad_mask);
  cd4067_data.low_contrast_mask = bad_mask;
  cd4067_data.two_point_recording = 0U;

  if (ok) {
    if (polarity == CD4067_POLARITY_LINE_HIGH) {
      cd4067_data.line_on_high = 1U;
    } else if (polarity == CD4067_POLARITY_LINE_LOW) {
      cd4067_data.line_on_high = 0U;
    } else {
      cd4067_data.line_on_high = (high_votes >= low_votes) ? 1U : 0U;
    }

    cd4067_data.two_point_active = 1U;
    cd4067_data.cal_done = 1U;
    cd4067_cal_stage = 2U;
  } else {
    /* Abort back to uncalibrated until retried or valid Flash cal re-applied */
    cd4067_data.two_point_active = 0U;
    cd4067_data.cal_done = 0U;
    cd4067_cal_stage = 0U;
  }

  if (primask == 0U) {
    __enable_irq();
  }

  return ok;
}

bool CD4067_ApplyTwoPointCal(const uint16_t min_arr[CD4067_NUM_CHANNELS],
                             const uint16_t max_arr[CD4067_NUM_CHANNELS],
                             uint8_t line_on_high) {
  if ((min_arr == NULL) || (max_arr == NULL) || (line_on_high > 1U)) {
    return false;
  }

  uint16_t bad_mask = 0U;
  CD4067_Mode_t mode = cd4067_data.mode;
  if (mode == CD4067_MODE_LIMIT_SWITCH) {
    return false;
  }
  if (!cd4067_validate_min_max(mode, min_arr, max_arr, &bad_mask)) {
    cd4067_data.low_contrast_mask = bad_mask;
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    cd4067_data.cal_min[i] = min_arr[i];
    cd4067_data.cal_max[i] = max_arr[i];
    cd4067_data.offset[i] = line_on_high ? min_arr[i] : max_arr[i];
  }
  cd4067_clear_runtime_outputs();
  cd4067_data.line_on_high = line_on_high;
  cd4067_data.low_contrast_mask = 0U;
  cd4067_data.two_point_recording = 0U;
  cd4067_data.two_point_active = 1U;
  cd4067_data.cal_done = 1U;
  cd4067_cal_stage = 2U;

  if (primask == 0U) {
    __enable_irq();
  }

  return true;
}

void CD4067_ExportTwoPointCal(uint16_t min_arr[CD4067_NUM_CHANNELS],
                              uint16_t max_arr[CD4067_NUM_CHANNELS],
                              uint8_t *line_on_high) {
  if ((min_arr == NULL) || (max_arr == NULL)) {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    min_arr[i] = cd4067_data.cal_min[i];
    max_arr[i] = cd4067_data.cal_max[i];
  }
  if (line_on_high != NULL) {
    *line_on_high = cd4067_data.line_on_high;
  }

  if (primask == 0U) {
    __enable_irq();
  }
}

bool CD4067_IsTwoPointCalRecording(void) {
  return (cd4067_data.two_point_recording != 0U);
}

bool CD4067_IsTwoPointActive(void) {
  return (cd4067_data.two_point_active != 0U);
}

void CD4067_SetThreshold(uint16_t threshold) {
  cd4067_data.threshold = threshold;
}

void CD4067_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim) {
  cd4067_hadc = hadc;
  cd4067_htim = htim;
  cd4067_dma_word = 0U;
  cd4067_active_ch = 0U;
  cd4067_next_ch = 0U;
  cd4067_data.current_channel = 0U;
  cd4067_data.threshold = CD4067_DEFAULT_THRESHOLD;
  cd4067_data.mode = CD4067_MODE_LINE_11;
  cd4067_data.led_view = CD4067_LED_VIEW_LOW11;
  cd4067_data.active_low_mask = 0xFFFFU;
  cd4067_data.line_on_high = 1U;
  cd4067_data.sweep_seq = 0U;

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    cd4067_data.raw[i] = 0U;
    cd4067_data.cal_min[i] = 0U;
    cd4067_data.cal_max[i] = 4095U;
  }

  CD4067_SelectChannel(0U);
  CD4067_StartCalibration();
}

static HAL_StatusTypeDef cd4067_start_sweep_timer(void) {
  if ((cd4067_hadc == NULL) || (cd4067_htim == NULL)) {
    return HAL_ERROR;
  }

  cd4067_active_ch = 0U;
  cd4067_next_ch = 0U;
  cd4067_data.current_channel = 0U;
  CD4067_SelectChannel(0U);

  __HAL_TIM_SET_COUNTER(cd4067_htim, 0U);
  return HAL_TIM_Base_Start_IT(cd4067_htim);
}

HAL_StatusTypeDef CD4067_Start(void) {
  cd4067_data.mode = CD4067_MODE_LINE_11;
  cd4067_data.led_view = CD4067_LED_VIEW_LOW11;
  CD4067_StartCalibration();
  return cd4067_start_sweep_timer();
}

HAL_StatusTypeDef CD4067_StartLimitSwitches(uint16_t active_low_mask) {
  if ((cd4067_hadc == NULL) || (cd4067_htim == NULL)) {
    return HAL_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  cd4067_data.mode = CD4067_MODE_LIMIT_SWITCH;
  cd4067_data.led_view = CD4067_LED_VIEW_LOW11;
  cd4067_data.active_low_mask = active_low_mask;

  if (primask == 0U) {
    __enable_irq();
  }

  CD4067_StartCalibration();
  return cd4067_start_sweep_timer();
}

HAL_StatusTypeDef CD4067_StartLine2x8(uint16_t threshold) {
  if ((cd4067_hadc == NULL) || (cd4067_htim == NULL)) {
    return HAL_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  cd4067_data.mode = CD4067_MODE_LINE_2X8;
  cd4067_data.led_view = CD4067_LED_VIEW_AUTO_ALT;
  cd4067_data.threshold =
      (threshold == 0U) ? CD4067_DEFAULT_THRESHOLD : threshold;

  if (primask == 0U) {
    __enable_irq();
  }

  /* Keep preloaded Flash 2-point calibration if already active; else start boot
   * tare */
  if (cd4067_data.two_point_active == 0U) {
    CD4067_StartCalibration();
  }
  return cd4067_start_sweep_timer();
}

void CD4067_Stop(void) {
  if (cd4067_htim != NULL) {
    (void)HAL_TIM_Base_Stop_IT(cd4067_htim);
  }
  if (cd4067_hadc != NULL) {
    (void)HAL_ADC_Stop_DMA(cd4067_hadc);
  }
}

void CD4067_SetLedViewMode(CD4067_LedView_t view) {
  cd4067_data.led_view = view;
}

CD4067_LedView_t CD4067_GetLedViewMode(void) { return cd4067_data.led_view; }

CD4067_LedView_t CD4067_CycleLedViewMode(void) {
  CD4067_LedView_t cur = cd4067_data.led_view;
  CD4067_LedView_t next = cur;

  if (cd4067_data.mode == CD4067_MODE_LINE_2X8) {
    switch (cur) {
    case CD4067_LED_VIEW_AUTO_ALT:
      next = CD4067_LED_VIEW_FRONT_LOCK;
      break;
    case CD4067_LED_VIEW_FRONT_LOCK:
      next = CD4067_LED_VIEW_REAR_LOCK;
      break;
    case CD4067_LED_VIEW_REAR_LOCK:
      next = CD4067_LED_VIEW_DUAL_5_1_5;
      break;
    default:
      next = CD4067_LED_VIEW_AUTO_ALT;
      break;
    }
  } else if (cd4067_data.mode == CD4067_MODE_LIMIT_SWITCH) {
    switch (cur) {
    case CD4067_LED_VIEW_LOW11:
      next = CD4067_LED_VIEW_AUTO_ALT;
      break;
    case CD4067_LED_VIEW_AUTO_ALT:
      next = CD4067_LED_VIEW_FRONT_LOCK;
      break;
    case CD4067_LED_VIEW_FRONT_LOCK:
      next = CD4067_LED_VIEW_REAR_LOCK;
      break;
    case CD4067_LED_VIEW_REAR_LOCK:
      next = CD4067_LED_VIEW_DUAL_5_1_5;
      break;
    default:
      next = CD4067_LED_VIEW_LOW11;
      break;
    }
  }

  cd4067_data.led_view = next;
  return next;
}

void CD4067_UpdateLeds(uint32_t now_ms) {
  (void)now_ms;

  /* L1..L11 reflect the 11 line sensors (channels 0..10) directly and cleanly
   * 1:1: Sensor sees line -> LED ON (active-HIGH) Sensor sees floor -> LED OFF
   * No walker animation, no contrast-error blinking, no 600ms alternating view.
   */
  uint16_t all_mask = cd4067_data.active_mask;
  for (uint8_t i = 0U; i < CD4067_NUM_LINE_SENSORS; i++) {
    bool is_on = ((all_mask & (uint16_t)(1U << i)) != 0U);
    CD4067_SetLineLed(i, is_on);
  }
}

uint16_t CD4067_GetRaw(uint8_t channel) {
  if (channel >= CD4067_NUM_CHANNELS) {
    return 0U;
  }
  return cd4067_data.raw[channel];
}

uint16_t CD4067_GetNorm(uint8_t channel) {
  if (channel >= CD4067_NUM_CHANNELS) {
    return 0U;
  }
  return cd4067_data.norm[channel];
}

int16_t CD4067_GetLineError(void) { return cd4067_data.line_error; }

uint16_t CD4067_GetActiveMask(void) { return cd4067_data.active_mask; }

bool CD4067_IsCalibrated(void) { return (cd4067_data.cal_done != 0U); }

uint16_t CD4067_GetLimitMask(void) { return cd4067_data.limit_mask; }

bool CD4067_IsLimitPressed(uint8_t channel) {
  if (channel >= CD4067_NUM_CHANNELS) {
    return false;
  }
  return ((cd4067_data.limit_mask & (uint16_t)(1U << channel)) != 0U);
}

uint8_t CD4067_GetFrontMask(void) { return cd4067_data.front_mask; }

uint8_t CD4067_GetRearMask(void) { return cd4067_data.rear_mask; }

int16_t CD4067_GetFrontError(void) { return cd4067_data.front_error; }

int16_t CD4067_GetRearError(void) { return cd4067_data.rear_error; }

int16_t CD4067_GetFrontCentroidX10(void) {
  return cd4067_data.front_centroid_x10;
}

int16_t CD4067_GetRearCentroidX10(void) {
  return cd4067_data.rear_centroid_x10;
}

void CD4067_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if ((cd4067_htim == NULL) || (htim == NULL) ||
      (htim->Instance != cd4067_htim->Instance)) {
    return;
  }
  if ((cd4067_hadc == NULL) || (cd4067_hadc->DMA_Handle == NULL)) {
    return;
  }
  if (HAL_IS_BIT_SET(cd4067_hadc->State, HAL_ADC_STATE_REG_BUSY) ||
      (cd4067_hadc->DMA_Handle->State != HAL_DMA_STATE_READY)) {
    return;
  }

  cd4067_active_ch = cd4067_next_ch;
  (void)HAL_ADC_Start_DMA(cd4067_hadc, &cd4067_dma_word, 1U);
}

static void cd4067_process_limit_sample(uint8_t ch, uint16_t sample) {
  if (sample <= CD4067_LIMIT_LOW_MAX) {
    cd4067_limit_raw_level[ch] = 0U;
  } else if (sample >= CD4067_LIMIT_HIGH_MIN) {
    cd4067_limit_raw_level[ch] = 1U;
  }

  uint8_t raw_lvl = cd4067_limit_raw_level[ch];
  if (raw_lvl != cd4067_limit_stable_level[ch]) {
    cd4067_limit_streak[ch]++;
    if (cd4067_limit_streak[ch] >= CD4067_LIMIT_DEBOUNCE_SWEEPS) {
      cd4067_limit_stable_level[ch] = raw_lvl;
      cd4067_limit_streak[ch] = 0U;
    }
  } else {
    cd4067_limit_streak[ch] = 0U;
  }

  bool is_active_low =
      ((cd4067_data.active_low_mask & (uint16_t)(1U << ch)) != 0U);
  bool pressed = is_active_low ? (cd4067_limit_stable_level[ch] == 0U)
                               : (cd4067_limit_stable_level[ch] != 0U);
  cd4067_data.sensor_active[ch] = pressed ? 1U : 0U;
}

static uint16_t cd4067_normalize_sample(uint8_t ch, uint16_t sample) {
  uint16_t mn = cd4067_data.cal_min[ch];
  uint16_t mx = cd4067_data.cal_max[ch];
  if (mx <= mn) {
    return 0U;
  }

  uint16_t clamped = sample;
  if (clamped < mn) {
    clamped = mn;
  } else if (clamped > mx) {
    clamped = mx;
  }

  uint32_t span = (uint32_t)mx - (uint32_t)mn;
  uint32_t norm = (((uint32_t)(clamped - mn)) * 1000UL + (span / 2UL)) / span;
  if (norm > 1000UL) {
    norm = 1000UL;
  }

  return (uint16_t)(cd4067_data.line_on_high ? norm : (1000UL - norm));
}

static void cd4067_commit_limit_sweep(void) {
  uint16_t mask = 0U;
  uint8_t cnt = 0U;

  for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
    if (cd4067_data.sensor_active[i] != 0U) {
      mask |= (uint16_t)(1U << i);
      cnt++;
    }
  }

  cd4067_data.limit_mask = mask;
  cd4067_data.active_mask = mask;
  cd4067_data.active_count = cnt;
  cd4067_data.front_mask = (uint8_t)(mask & 0xFFU);
  cd4067_data.rear_mask = (uint8_t)((mask >> 8U) & 0xFFU);
  cd4067_data.sweep_seq++;
}

static void cd4067_commit_line11_sweep(void) {
  int32_t w_sum = 0;
  uint32_t w_den = 0U;
  int32_t bin_sum = 0;
  uint8_t sum_cnt = 0U;
  uint16_t mask = 0U;
  bool use_2pt = (cd4067_data.two_point_active != 0U);

  for (uint8_t i = 0U; i < CD4067_NUM_LINE_SENSORS; i++) {
    if (cd4067_data.sensor_active[i] != 0U) {
      bin_sum += cd4067_line_weights_x10[i];
      sum_cnt++;
      mask |= (uint16_t)(1U << i);
    }
    if (use_2pt && (cd4067_data.norm[i] > CD4067_NORM_NOISE_FLOOR)) {
      uint32_t eff = (uint32_t)cd4067_data.norm[i] - CD4067_NORM_NOISE_FLOOR;
      w_sum += (int32_t)eff * (int32_t)cd4067_line_weights_x10[i];
      w_den += eff;
    }
  }

  cd4067_data.active_mask = mask;
  cd4067_data.active_count = sum_cnt;
  if (use_2pt && (w_den > 0U) && (sum_cnt > 0U)) {
    int16_t cx10 = (int16_t)(w_sum / (int32_t)w_den);
    cd4067_data.front_centroid_x10 = cx10;
    cd4067_data.line_error = (int16_t)(cx10 / 10);
  } else if (sum_cnt > 0U) {
    int16_t cx10 = (int16_t)(bin_sum / (int32_t)sum_cnt);
    cd4067_data.front_centroid_x10 = cx10;
    cd4067_data.line_error = (int16_t)(cx10 / 10);
  }
  cd4067_data.sweep_seq++;
}

static void cd4067_commit_line2x8_sweep(void) {
  int32_t f_bin_sum = 0;
  int32_t r_bin_sum = 0;
  int32_t f_w_sum = 0;
  int32_t r_w_sum = 0;
  uint32_t f_w_den = 0U;
  uint32_t r_w_den = 0U;
  uint8_t f_cnt = 0U;
  uint8_t r_cnt = 0U;
  uint8_t f_mask = 0U;
  uint8_t r_mask = 0U;
  bool use_2pt = (cd4067_data.two_point_active != 0U);

  for (uint8_t i = 0U; i < CD4067_NUM_ROW_SENSORS; i++) {
    uint8_t fi = (uint8_t)(CD4067_FRONT_FIRST_CH + i);
    uint8_t ri = (uint8_t)(CD4067_REAR_FIRST_CH + i);
    int16_t front_weight = cd4067_row_weights_x10[CD4067_NUM_ROW_SENSORS - 1U - i];
    if (cd4067_data.sensor_active[fi] != 0U) {
      f_bin_sum += front_weight;
      f_cnt++;
      f_mask |= (uint8_t)(1U << i);
    }
    if (use_2pt && (cd4067_data.norm[fi] > CD4067_NORM_NOISE_FLOOR)) {
      uint32_t eff = (uint32_t)cd4067_data.norm[fi] - CD4067_NORM_NOISE_FLOOR;
      f_w_sum += (int32_t)eff * (int32_t)front_weight;
      f_w_den += eff;
    }

    if (cd4067_data.sensor_active[ri] != 0U) {
      r_bin_sum += cd4067_row_weights_x10[i];
      r_cnt++;
      r_mask |= (uint8_t)(1U << i);
    }
    if (use_2pt && (cd4067_data.norm[ri] > CD4067_NORM_NOISE_FLOOR)) {
      uint32_t eff = (uint32_t)cd4067_data.norm[ri] - CD4067_NORM_NOISE_FLOOR;
      r_w_sum += (int32_t)eff * (int32_t)cd4067_row_weights_x10[i];
      r_w_den += eff;
    }
  }

  cd4067_data.front_mask = f_mask;
  cd4067_data.rear_mask = r_mask;
  cd4067_data.front_count = f_cnt;
  cd4067_data.rear_count = r_cnt;
  cd4067_data.active_mask = ((uint16_t)f_mask << CD4067_FRONT_FIRST_CH) |
                            ((uint16_t)r_mask << CD4067_REAR_FIRST_CH);
  cd4067_data.active_count = (uint8_t)(f_cnt + r_cnt);

  if (use_2pt && (f_w_den > 0U) && (f_cnt > 0U)) {
    int16_t cx10 = (int16_t)(f_w_sum / (int32_t)f_w_den);
    cd4067_data.front_centroid_x10 = cx10;
    cd4067_data.front_error = (int16_t)(cx10 / 10);
    cd4067_data.line_error = cd4067_data.front_error;
  } else if (f_cnt > 0U) {
    int16_t cx10 = (int16_t)(f_bin_sum / (int32_t)f_cnt);
    cd4067_data.front_centroid_x10 = cx10;
    cd4067_data.front_error = (int16_t)(cx10 / 10);
    cd4067_data.line_error = cd4067_data.front_error;
  }

  if (use_2pt && (r_w_den > 0U) && (r_cnt > 0U)) {
    int16_t cx10 = (int16_t)(r_w_sum / (int32_t)r_w_den);
    cd4067_data.rear_centroid_x10 = cx10;
    cd4067_data.rear_error = (int16_t)(cx10 / 10);
  } else if (r_cnt > 0U) {
    int16_t cx10 = (int16_t)(r_bin_sum / (int32_t)r_cnt);
    cd4067_data.rear_centroid_x10 = cx10;
    cd4067_data.rear_error = (int16_t)(cx10 / 10);
  }

  cd4067_data.sweep_seq++;
}

void CD4067_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if ((cd4067_hadc == NULL) || (hadc == NULL) ||
      (hadc->Instance != cd4067_hadc->Instance)) {
    return;
  }

  uint8_t ch = cd4067_active_ch & 0x0FU;
  uint16_t sample =
      (uint16_t)(*(const volatile uint32_t *)&cd4067_dma_word & 0x0FFFU);
  cd4067_data.raw[ch] = sample;
  cd4067_data.current_channel = ch;

  /* Immediately switch MUX to next channel so ~95 us passive settling begins
   * now */
  uint8_t next_ch = (uint8_t)((ch + 1U) & 0x0FU);
  cd4067_next_ch = next_ch;
  CD4067_SelectChannel(next_ch);

  CD4067_Mode_t mode = cd4067_data.mode;

  if (mode == CD4067_MODE_LIMIT_SWITCH) {
    cd4067_process_limit_sample(ch, sample);
    if (ch == (CD4067_NUM_CHANNELS - 1U)) {
      cd4067_commit_limit_sweep();
    }
    return;
  }

  if (cd4067_cal_stage == 1U) {
    cd4067_offset_accum[ch] += sample;
  } else if (cd4067_cal_stage == 3U) {
    if (sample < cd4067_data.cal_min[ch]) {
      cd4067_data.cal_min[ch] = sample;
    }
    if (sample > cd4067_data.cal_max[ch]) {
      cd4067_data.cal_max[ch] = sample;
    }
    cd4067_cal_sum[ch] += sample;
    cd4067_cal_samples[ch]++;
  } else if (cd4067_cal_stage == 2U) {
    if (cd4067_is_channel_used(mode, ch)) {
      bool active = false;
      if (cd4067_data.two_point_active != 0U) {
        uint16_t norm = cd4067_normalize_sample(ch, sample);
        cd4067_data.norm[ch] = norm;
        active = (norm >= CD4067_NORM_ACTIVE_THRESH);
      } else {
        int32_t diff = (int32_t)sample - (int32_t)cd4067_data.offset[ch];
        if (diff < 0) {
          diff = -diff;
        }
        cd4067_data.norm[ch] =
            (diff >= (int32_t)cd4067_data.threshold) ? 1000U : 0U;
        active = (diff > (int32_t)cd4067_data.threshold);
      }
      cd4067_data.sensor_active[ch] = active ? 1U : 0U;
      if ((mode == CD4067_MODE_LINE_11) && (ch < CD4067_NUM_LINE_SENSORS)) {
        CD4067_SetLineLed(ch, active);
      }
    }
  }

  /* End-of-sweep processing on channel 15 */
  if (ch == (CD4067_NUM_CHANNELS - 1U)) {
    uint16_t sweeps = cd4067_data.sweep_count;

    if (cd4067_cal_stage == 0U) {
      sweeps++;
      cd4067_data.sweep_count = sweeps;
      if (sweeps >= CD4067_CALIB_WARMUP_SWEEPS) {
        cd4067_data.sweep_count = 0U;
        cd4067_cal_stage = 1U;
      }
    } else if (cd4067_cal_stage == 1U) {
      sweeps++;
      cd4067_data.sweep_count = sweeps;
      if (sweeps >= CD4067_CALIB_AVG_SWEEPS) {
        for (uint8_t i = 0U; i < CD4067_NUM_CHANNELS; i++) {
          cd4067_data.offset[i] = (uint16_t)(cd4067_offset_accum[i] >> 8U);
        }
        cd4067_cal_stage = 2U;
        cd4067_data.cal_done = 1U;
      }
    } else if (cd4067_cal_stage == 3U) {
      cd4067_data.sweep_count++;
      cd4067_data.sweep_seq++;
    } else if (mode == CD4067_MODE_LINE_2X8) {
      cd4067_commit_line2x8_sweep();
    } else {
      cd4067_commit_line11_sweep();
    }
  }
}
