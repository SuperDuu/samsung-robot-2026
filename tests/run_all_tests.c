/**
 ******************************************************************************
 * @file    run_all_tests.c
 * @brief   Host unit + falsification test suite compiling real firmware C
 *          modules against virtual hardware simulators with ASan + UBSan.
 ******************************************************************************
 */

#include "VL53L1X_api.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "cd4067.h"
#include "drive_kinematics.h"
#include "flash_cal.h"
#include "itc2026_auto.h"
#include "mock_hal.h"
#include "rf_hc12.h"
#include "rs485_bus.h"
#include "servo.h"
#include "tof_vl53l1x.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t VL51L1X_DEFAULT_CONFIGURATION[];

static void sim_adc_sweep(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim,
                          const uint16_t ch_vals[16]) {
  for (uint8_t ch = 0U; ch < 16U; ch++) {
    CD4067_TIM_PeriodElapsedCallback(htim);
    assert(hadc->dma_dest != NULL);
    *(hadc->dma_dest) = ch_vals[ch];
    CD4067_ADC_ConvCpltCallback(hadc);
  }
}

static void test_vl53l1x_uld_boot_and_poll(void) {
  printf("[1/6] Running VL53L1X ULD firmware config & ToF manager tests...\n");
  MockHal_ResetAll();

  MockHal_VL53SetupDefault(0U, LS1_GPIO_Port, LS1_Pin, 900U);
  MockHal_VL53SetupDefault(1U, LS2_GPIO_Port, LS2_Pin, 1200U);
  MockHal_VL53SetupDefault(2U, LS3_GPIO_Port, LS3_Pin, 1500U);
  MockHal_VL53SetupDefault(3U, LS4_GPIO_Port, LS4_Pin, 1800U);

  I2C_HandleTypeDef hi2c2 = {.Instance = I2C2};
  ToF_Manager_t mgr;
  uint8_t online = ToF_Init(&mgr, &hi2c2, NULL);
  assert(online == 4U);
  assert(mgr.online_mask == 0x0FU);
  assert(!mgr.addr52_conflict);

  const uint8_t expected_addrs[4] = {0x54U, 0x56U, 0x58U, 0x5AU};
  for (uint8_t i = 0U; i < 4U; i++) {
    assert(mgr.sensors[i].fw_loaded);
    assert(mgr.sensors[i].online);
    assert(g_mock_vl53[i].addr_8bit == expected_addrs[i]);
    /* Verify ALL 91 bytes (0x2D..0x87) of official ST
     * VL51L1X_DEFAULT_CONFIGURATION landed verbatim */
    assert(g_mock_vl53[i].init_cfg_captured);
    assert(memcmp(g_mock_vl53[i].init_cfg_snapshot,
                  VL51L1X_DEFAULT_CONFIGURATION, VL53L1X_DEFAULT_CFG_LEN) == 0);
    assert((g_mock_vl53[i].regs[0x2EU] & 0x01U) ==
           0x01U); /* 2.8V EXTSUP applied */
    assert(g_mock_vl53[i].regs[0x4BU] ==
           0x14U); /* Short distance mode 1: PHASECAL_TIMEOUT_MACROP */
    assert(g_mock_vl53[i].regs[0x60U] ==
           0x07U); /* Short distance mode 1: VCSEL_PERIOD_A */
    assert(g_mock_vl53[i].ranging_active);
  }

  /* 3-sample median filter outlier rejection test on Front sensor (slot 0) */
  uint32_t t = HAL_GetTick() + 10U;
  MockHal_VL53SetDistance(0U, 1200U, 9U);
  for (int k = 0; k < 12; k++) {
    t += 6U;
    MockHal_SetTick(t);
    ToF_Poll(&mgr, t);
  }
  assert(ToF_GetDistanceMm(&mgr, TOF_SENSOR_FRONT, t) == 1200U);

  MockHal_VL53SetDistance(0U, 180U, 9U); /* Single optical glitch spike */
  for (int k = 0; k < 4; k++) {
    t += 6U;
    MockHal_SetTick(t);
    ToF_Poll(&mgr, t);
  }
  assert(ToF_GetDistanceMm(&mgr, TOF_SENSOR_FRONT, t) ==
         1200U); /* Rejected by median! */

  MockHal_VL53SetDistance(0U, 250U,
                          9U); /* Two confirming samples establish new median */
  for (int k = 0; k < 8; k++) {
    t += 6U;
    MockHal_SetTick(t);
    ToF_Poll(&mgr, t);
  }
  assert(ToF_GetDistanceMm(&mgr, TOF_SENSOR_FRONT, t) == 250U);

  uint16_t min_mm = 0U;
  assert(ToF_IsDirectionBlocked(&mgr, true, 350U, t, &min_mm) == true);
  assert(min_mm == 250U);

  /* Falsification: NACK storm triggers offline + backoff + recovery without
   * hanging */
  g_mock_vl53[0].force_nack = true;
  for (int k = 0; k < 30; k++) {
    t += 6U;
    MockHal_SetTick(t);
    ToF_Poll(&mgr, t);
  }
  assert(!mgr.sensors[0].online);
  assert(mgr.i2c_recoveries >= 1U);

  g_mock_vl53[0].force_nack = false;
  t += TOF_RECOVER_BACKOFF_MS + 20U;
  MockHal_SetTick(t);
  for (int k = 0; k < 8; k++) {
    t += 6U;
    MockHal_SetTick(t);
    ToF_Poll(&mgr, t);
  }
  assert(mgr.sensors[0].online);
  assert(mgr.sensor_reinits >= 1U);

  /* Falsification: stuck 0x52 conflict & hung boot timeout */
  MockHal_ResetAll();
  MockHal_VL53SetupDefault(0U, LS1_GPIO_Port, LS1_Pin, 500U);
  g_mock_vl53[0].stuck_addr52 = true;
  g_mock_vl53[0].hang_boot = true;
  online = ToF_Init(&mgr, &hi2c2, NULL);
  assert(mgr.addr52_conflict);
  assert(online == 0U);
  assert(!mgr.sensors[0].fw_loaded);
  printf("      PASS.\n");
}

static uint8_t s_rx_frames_ok = 0U;
static RS485_Telem_t s_last_telem;
static void test_rs485_cb(RS485_Bus_t *bus, const RS485_Frame_t *frame,
                          void *ctx) {
  (void)bus;
  (void)ctx;
  if (RS485_DecodeTelem(frame, &s_last_telem)) {
    s_rx_frames_ok++;
  }
}

static void test_rs485_non_destructive_error_recovery(void) {
  printf(
      "[2/6] Running RS-485 mid-frame UART error & CRC-16 recovery tests...\n");
  MockHal_ResetAll();
  s_rx_frames_ok = 0U;
  memset(&s_last_telem, 0, sizeof(s_last_telem));

  DMA_HandleTypeDef hdma_rx = {.State = HAL_DMA_STATE_READY};
  DMA_HandleTypeDef hdma_tx = {.State = HAL_DMA_STATE_READY};
  UART_HandleTypeDef huart2 = {.Instance = USART2,
                               .hdmarx = &hdma_rx,
                               .hdmatx = &hdma_tx,
                               .gState = HAL_UART_STATE_READY,
                               .RxState = HAL_UART_STATE_READY};

  RS485_Bus_t master;
  RS485_Init(&master, &huart2, RE_DE_GPIO_Port, RE_DE_Pin, RS485_ADDR_MASTER);
  RS485_SetRxCallback(&master, test_rs485_cb, NULL);
  assert(RS485_StartRx(&master) == HAL_OK);
  assert(!MockHal_GetGpioOut(RE_DE_GPIO_Port, RE_DE_Pin)); /* RX LOW at idle */

  /* Build a valid telemetry wire frame via slave deferred turnaround
   * (RS485_SLAVE_TURNAROUND_MS=2) */
  UART_HandleTypeDef huart_slave = {.Instance = USART1,
                                    .gState = HAL_UART_STATE_READY,
                                    .RxState = HAL_UART_STATE_READY};
  RS485_Bus_t slave;
  RS485_Init(&slave, &huart_slave, NULL, 0U, 2U);
  RS485_Telem_t tx_telem = {.node_id = 2U,
                            .mode = 3U,
                            .status_flags = 0U,
                            .actual_rpm_x10 = -1450,
                            .duty_permille = -420,
                            .enc_count = 12345};
  assert(RS485_ScheduleTelemReply(&slave, RS485_ADDR_MASTER, &tx_telem, 10U));
  RS485_TickMs(&slave, 11U);
  assert(huart_slave.tx_calls ==
         0U); /* Must NOT fire before 2 ms bus-idle gap */
  RS485_TickMs(&slave, 13U);
  assert(huart_slave.tx_calls == 1U);
  uint16_t wire_len = huart_slave.last_tx_len;
  uint8_t wire_bytes[64];
  memcpy(wire_bytes, huart_slave.last_tx_buf, wire_len);

  /* Split delivery: feed first 9 bytes, process into frame_buf */
  MockHal_UartInjectDmaBytes(&huart2, wire_bytes, 9U);
  RS485_ProcessRx(&master, 20U);
  assert(master.frame_pos == 9U);
  assert(s_rx_frames_ok == 0U);

  /* Inject remaining bytes into DMA ring, then fire ORE|FE UART Error IRQ! */
  uint32_t aborts_before = g_mock_uart_abort_rx_calls;
  MockHal_UartInjectDmaBytes(&huart2, &wire_bytes[9],
                             (uint16_t)(wire_len - 9U));
  huart2.ErrorCode = HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE;
  RS485_UartErrorCallback(&master, &huart2);

  /* Must preserve buffered DMA + frame_buf and complete frame with ZERO abort
   */
  assert(master.stats.uart_ore_errors == 1U);
  assert(master.stats.uart_fe_errors == 1U);
  assert(master.stats.uart_err_recoveries == 1U);
  assert(g_mock_uart_abort_rx_calls == aborts_before);
  assert(s_rx_frames_ok == 1U);
  assert(s_last_telem.actual_rpm_x10 == -1450);
  assert(s_last_telem.enc_count == 12345);

  /* Falsification: flipping 1 bit in CRC must drop frame and increment
   * crc_err_frames */
  wire_bytes[wire_len - 1U] ^= 0x01U;
  RS485_FeedBytes(&master, wire_bytes, wire_len, 30U);
  assert(master.stats.crc_err_frames == 1U);
  assert(s_rx_frames_ok == 1U);

  /* Verify MultiCmd + RE/DE TX watchdog */
  RS485_MultiCmd_t mcmd = {
      .num_wheels = 4U, .poll_id = 1U, .flags = RS485_CMD_FLAG_CLEAR_FAULTS};
  mcmd.wheels[0].mode = RS485_WHEEL_VELOCITY;
  mcmd.wheels[0].target = 2200;
  assert(RS485_SendMultiCmd(&master, &mcmd, 40U) == HAL_OK);
  assert(master.tx_active == 1U);
  assert(MockHal_GetGpioOut(RE_DE_GPIO_Port,
                            RE_DE_Pin)); /* HIGH while TX active */
  RS485_TickMs(&master,
               65U); /* Missed TC IRQ -> TX watchdog triggers at >= 20ms */
  assert(master.tx_active == 0U);
  assert(master.stats.tx_wdog_recoveries == 1U);
  assert(!MockHal_GetGpioOut(RE_DE_GPIO_Port,
                             RE_DE_Pin)); /* Released back to RX LOW */
  printf("      PASS.\n");
}

static void test_bno055_hardening_and_backoff(void) {
  printf("[3/6] Running BNO055 page cache, backoff recovery & yaw unwrap "
         "tests...\n");
  MockHal_ResetAll();
  g_mock_bno.present = true;
  g_mock_bno.yaw_deg = 350.0f;

  I2C_HandleTypeDef hi2c3 = {.Instance = I2C3};
  BNO055_IMU_t imu;
  assert(BNO055_IMU_Init(&imu, &hi2c3));
  assert(imu.healthy);

  /* Page caching check: 19 Euler polls (< CALIB_POLL_DIV=20) must perform ZERO
   * extra TX page writes */
  uint32_t tx_before = g_mock_bno.i2c_tx_count;
  g_mock_bno.yaw_deg = 359.0f;
  for (int i = 0; i < 10; i++) {
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&imu));
  }
  assert(g_mock_bno.i2c_tx_count == tx_before);

  /* Yaw wrap continuity: 350 -> 359 -> 2 must advance relative yaw by +12 deg
   * (not -348 deg) */
  g_mock_bno.yaw_deg = 2.0f;
  MockHal_AdvanceTick(10U);
  assert(BNO055_IMU_Update(&imu));
  assert(fabsf(imu.yaw - 12.0f) < 0.25f);

  /* Fault injection A: 3 NACKs force unhealthy; backoff avoids blocking for
   * 250ms */
  g_mock_bno.nack_remaining = 3U;
  for (int i = 0; i < 3; i++) {
    MockHal_AdvanceTick(10U);
    assert(!BNO055_IMU_Update(&imu));
  }
  assert(!imu.healthy);
  assert(g_mock_bno.nack_remaining == 0U);

  uint32_t tx_idle = g_mock_bno.i2c_tx_count;
  uint32_t rx_idle = g_mock_bno.i2c_rx_count;
  MockHal_AdvanceTick(100U);
  assert(!BNO055_IMU_Update(&imu));
  assert(g_mock_bno.i2c_tx_count == tx_idle);
  assert(g_mock_bno.i2c_rx_count == rx_idle);

  /* Advance past 250ms backoff -> hot NDOF restore */
  MockHal_AdvanceTick(200U);
  assert(BNO055_IMU_Update(&imu));
  assert(imu.healthy);
  assert(imu.recoveries == 1U);

  /* Fault injection B: stuck SDA/SCL triggers 250ms backoff then I2C3 bus
   * recovery + NDOF restore */
  g_mock_bno.bus_stuck = true;
  for (int i = 0; i < 3; i++) {
    MockHal_AdvanceTick(10U);
    assert(!BNO055_IMU_Update(&imu));
  }
  assert(!imu.healthy);
  tx_idle = g_mock_bno.i2c_tx_count;
  rx_idle = g_mock_bno.i2c_rx_count;
  MockHal_AdvanceTick(100U);
  assert(!BNO055_IMU_Update(&imu));
  assert(g_mock_bno.i2c_tx_count == tx_idle);
  assert(g_mock_bno.i2c_rx_count == rx_idle);

  MockHal_AdvanceTick(200U);
  assert(BNO055_IMU_Update(&imu));
  assert(imu.healthy);
  assert(imu.recoveries == 2U);
  assert(g_mock_i2c_recover_calls >= 2U);

  /* (a) Default 6-DoF IMU (0x08) mode & silent brown-out (OPR_MODE -> 0x00, yaw
   * -> 0) zero-jump recovery */
  {
    MockHal_ResetAll();
    g_mock_bno.present = true;
    I2C_HandleTypeDef hi2c3 = {.Instance = I2C3};
    BNO055_IMU_t bo_imu;
    g_mock_bno.yaw_deg = 0.0f;
    assert(BNO055_IMU_Init(&bo_imu, &hi2c3));
    assert(bo_imu.op_mode == BNO055_OPERATION_MODE_IMU);
    assert((g_mock_bno.opr_mode & 0x0FU) == (uint8_t)BNO055_OPERATION_MODE_IMU);

    /* Sub-check (a1): silent brown-out while at 0 deg caught by 5 Hz OPR_MODE
     * check + 1-tick resync */
    g_mock_bno.opr_mode = 0x00U;
    for (int k = 0; k < 21; k++) {
      MockHal_AdvanceTick(10U);
      assert(BNO055_IMU_Update(&bo_imu));
    }
    assert(bo_imu.mode_resets == 1U);
    assert((g_mock_bno.opr_mode & 0x0FU) == (uint8_t)BNO055_OPERATION_MODE_IMU);
    assert(!bo_imu.resync_yaw_prev);

    g_mock_bno.yaw_deg = 90.0f;
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&bo_imu));
    assert(fabsf(bo_imu.yaw - 90.0f) < 0.1f);

    /* Sub-check (a2): brown-out while rotated: OPR_MODE resets to CONFIG
     * (0x00), Euler drops to {0,0,0} with HAL_OK */
    g_mock_bno.opr_mode = 0x00U;
    g_mock_bno.yaw_deg = 0.0f;
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&bo_imu));
    assert(bo_imu.mode_resets == 2U);
    assert((g_mock_bno.opr_mode & 0x0FU) == (uint8_t)BNO055_OPERATION_MODE_IMU);
    assert(fabsf(bo_imu.yaw - 90.0f) < 0.1f);

    /* Post-restore resync + +15 deg relative turn */
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&bo_imu));
    assert(fabsf(bo_imu.yaw - 90.0f) < 0.1f);
    g_mock_bno.yaw_deg = 15.0f;
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&bo_imu));
    assert(fabsf(bo_imu.yaw - 105.0f) < 0.1f);

    /* Sub-check (a3): 180-deg turn and 2-second stationary hold must NOT flip or jump yaw */
    g_mock_bno.yaw_deg = 180.0f;
    MockHal_AdvanceTick(10U);
    assert(BNO055_IMU_Update(&bo_imu));
    float yaw_at_180 = bo_imu.yaw;
    for (int k = 0; k < 200; k++) {
      MockHal_AdvanceTick(10U);
      assert(BNO055_IMU_Update(&bo_imu));
    }
    assert(fabsf(bo_imu.yaw - yaw_at_180) < 0.25f);
  }

  /* (b) Synthetic ZRO drifts (+0.25, -0.31, +0.19 deg/s) learned during 1.5s
   * stillness cancel 60s drift (< 0.8 deg) */
  {
    const double test_drifts[3] = {0.25, -0.31, 0.19};
    I2C_HandleTypeDef hi2c3 = {.Instance = I2C3};
    BNO055_IMU_t drift_imu;

    for (size_t d = 0U; d < 3U; d++) {
      MockHal_ResetAll();
      g_mock_bno.present = true;
      g_mock_bno.yaw_deg = 0.0f;
      g_mock_bno.calib_stat = 0xFFU;
      assert(BNO055_IMU_Init(&drift_imu, &hi2c3));

      BNO055_IMU_SetStationaryHint(&drift_imu, true);
      double sim_drift_deg = 0.0;
      for (int k = 1; k <= 150; k++) {
        MockHal_AdvanceTick(10U);
        sim_drift_deg += test_drifts[d] * 0.01;
        double raw = fmod(sim_drift_deg, 360.0);
        if (raw < 0.0) {
          raw += 360.0;
        }
        g_mock_bno.yaw_deg = (float)raw;
        assert(BNO055_IMU_Update(&drift_imu));
      }
      assert(drift_imu.bias_updates >= 1U);
      assert(fabsf(drift_imu.yaw_bias_dps - (float)test_drifts[d]) < 0.07f);

      /* Drive +/-45 deg sinusoid for 60s with ongoing synthetic drift */
      BNO055_IMU_SetStationaryHint(&drift_imu, false);
      for (int k = 1; k <= 6000; k++) {
        MockHal_AdvanceTick(10U);
        sim_drift_deg += test_drifts[d] * 0.01;
        double motion =
            45.0 * sin(2.0 * 3.141592653589793 * ((double)k / 1000.0));
        double raw = fmod(sim_drift_deg + motion, 360.0);
        if (raw < 0.0) {
          raw += 360.0;
        }
        g_mock_bno.yaw_deg = (float)raw;
        assert(BNO055_IMU_Update(&drift_imu));
      }
      assert(fabsf(drift_imu.yaw) < 0.8f);
    }

    /* (c) Falsification: 2.0 deg/s rotation during stationary_hint must NOT
     * corrupt yaw_bias_dps */
    float bias_before = drift_imu.yaw_bias_dps;
    uint32_t updates_before = drift_imu.bias_updates;
    BNO055_IMU_SetStationaryHint(&drift_imu, true);
    double false_yaw = (double)g_mock_bno.yaw_deg;
    for (int k = 1; k <= 200; k++) {
      MockHal_AdvanceTick(10U);
      false_yaw = fmod(false_yaw + (2.0 * 0.01), 360.0);
      g_mock_bno.yaw_deg = (float)false_yaw;
      assert(BNO055_IMU_Update(&drift_imu));
    }
    assert(drift_imu.bias_updates == updates_before);
    assert(fabsf(drift_imu.yaw_bias_dps - bias_before) < 0.001f);
  }
  printf("      PASS.\n");
}

static void test_drive_kinematics_custom_field_centric_and_curvature(void) {
  printf("[4/6] Running custom field-centric, snap/rotate/gears & curvature-FF "
         "tests...\n");
  Drive_OmniCtrl_t ctrl;
  Drive_OmniCtrlInit(&ctrl, true);

  Drive_Ps2Input_t in = {.joy_sum = {252U, 100U},
                         .valid = true}; /* 252 underflow = full forward */
  uint8_t modes[4];
  int16_t targets[4];
  Drive_OmniCtrlStep(&ctrl, &in, false, 0.0f, true, 4000, modes, targets);
  /* Full forward (vx=0, vy=-100) -> Front Left/Right < 0, Rear > 0, no spin */
  assert(modes[0] == DRIVE_WHEEL_MODE_VELOCITY);
  assert((targets[0] < 0) && (targets[1] < 0) && (targets[2] > 0) &&
         (targets[3] > 0));

  /* Falsification: 359 deg -> pressing TRIANGLE (0 deg) must use shortest +1
   * deg error, NOT -359 spin! */
  memset(&in, 0, sizeof(in));
  in.valid = true;
  in.joy_sum[0] = 100U;
  in.joy_sum[1] = 100U;
  in.ps2_btn[1] = DRIVE_PS2_B1_TRIANGLE;
  Drive_OmniCtrlStep(&ctrl, &in, false, 359.0f, true, 4000, modes, targets);
  assert(ctrl.omega_fb_pct > 0.0f && ctrl.omega_fb_pct <= 5.0f);

  /* Custom axis snaps: SQUARE (128) -> target=+90 (quay trai), CIRCLE (32) -> target=-90 (quay phai) */
  in.ps2_btn[1] = DRIVE_PS2_B1_SQUARE;
  Drive_OmniCtrlStep(&ctrl, &in, false, 90.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - 90.0f) < 0.01f);
  assert(fabsf(ctrl.field_axis_offset_deg - 90.0f) < 0.01f);

  Drive_OmniCtrlSetBlueSide(&ctrl, false);
  in.ps2_btn[1] = DRIVE_PS2_B1_CIRCLE;
  Drive_OmniCtrlStep(&ctrl, &in, false, -90.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - (-90.0f)) < 0.01f);
  assert(fabsf(ctrl.field_axis_offset_deg - (-90.0f)) < 0.01f);

  /* R3 toggles +/-45 deg diagonal snap; L1/R1 rotates with lead-cap and holds
   * on release */
  in.ps2_btn[1] = DRIVE_PS2_B1_TRIANGLE;
  Drive_OmniCtrlStep(&ctrl, &in, false, 0.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg) < 0.01f && !ctrl.diagonal_45);
  in.ps2_btn[1] = 0U;
  in.ps2_btn[0] = DRIVE_PS2_B0_R3;
  Drive_OmniCtrlStep(&ctrl, &in, false, 0.0f, true, 4000, modes, targets);
  assert(ctrl.diagonal_45 && fabsf(ctrl.target_heading_deg - (-45.0f)) < 0.01f);
  in.ps2_btn[0] = 0U;
  Drive_OmniCtrlStep(&ctrl, &in, false, 0.0f, true, 4000, modes, targets);
  in.ps2_btn[0] = DRIVE_PS2_B0_R3;
  Drive_OmniCtrlStep(&ctrl, &in, false, 0.0f, true, 4000, modes, targets);
  assert(!ctrl.diagonal_45 && fabsf(ctrl.target_heading_deg) < 0.01f);

  /* Falsification: Standalone R1 is dedicated to clamp toggle, so it MUST NOT
   * rotate the robot or change target_heading! */
  in.ps2_btn[0] = 0U;
  in.ps2_btn[1] = DRIVE_PS2_B1_R1;
  float pre_r1_hdg = ctrl.target_heading_deg;
  for (int step = 0; step < 30; step++) {
    Drive_OmniCtrlStep(&ctrl, &in, false, 20.0f, true, 4000, modes, targets);
  }
  assert(fabsf(ctrl.target_heading_deg - pre_r1_hdg) < 0.01f);
  in.ps2_btn[1] = 0U;
  Drive_OmniCtrlStep(&ctrl, &in, false, 20.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - pre_r1_hdg) < 0.01f);

  /* Fine yaw trim with L2 (left) and R2 (right) at 20 RPM (200 rpm_x10) */
  in.ps2_btn[1] = DRIVE_PS2_B1_L2;
  Drive_OmniCtrlStep(&ctrl, &in, false, 15.0f, true, 4000, modes, targets);
  assert(fabsf(fabsf(ctrl.omega_fb_pct) - DRIVE_OMNI_TRIM_ROT_PCT) < 0.01f);
  assert(abs(targets[0]) == 200); /* 20.0 RPM per wheel */

  /* Release L2: target_heading_deg remains at setpoint (0 deg), while heading_trim_deg absorbs offset */
  in.ps2_btn[1] = 0U;
  Drive_OmniCtrlStep(&ctrl, &in, false, 18.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - 0.0f) < 0.01f);
  assert(fabsf(ctrl.heading_trim_deg - (-18.0f)) < 0.01f);

  /* Pad snap test: pressing Triangle (0 deg) MUST NOT snap back to drifted raw 0 deg! */
  in.ps2_btn[1] = DRIVE_PS2_B1_TRIANGLE;
  Drive_OmniCtrlStep(&ctrl, &in, false, 18.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - 0.0f) < 0.01f);
  assert(targets[0] == 0 && targets[1] == 0 && targets[2] == 0 && targets[3] == 0);

  /* R2 rotates in opposite direction at 20 RPM */
  in.ps2_btn[1] = DRIVE_PS2_B1_R2;
  Drive_OmniCtrlStep(&ctrl, &in, false, 18.0f, true, 4000, modes, targets);
  assert(fabsf(fabsf(ctrl.omega_fb_pct) - DRIVE_OMNI_TRIM_ROT_PCT) < 0.01f);
  assert(abs(targets[0]) == 200); /* 20.0 RPM per wheel */
  in.ps2_btn[1] = 0U;

  /* Large turn test: Rotating 180 deg with L2 must update target_heading to 180 deg and PRESERVE field-centric! */
  ctrl.heading_trim_deg = 0.0f;
  ctrl.target_heading_deg = 0.0f;
  in.ps2_btn[1] = DRIVE_PS2_B1_L2;
  for (int step = 0; step < 50; step++) {
    Drive_OmniCtrlStep(&ctrl, &in, false, 180.0f, true, 4000, modes, targets);
  }
  in.ps2_btn[1] = 0U; /* Release */
  Drive_OmniCtrlStep(&ctrl, &in, false, 180.0f, true, 4000, modes, targets);
  assert(fabsf(ctrl.target_heading_deg - 180.0f) < 0.1f);
  assert(fabsf(ctrl.slew_heading_deg - 180.0f) < 0.1f);

  /* Falsification: Heading loop gain MUST be invariant across all speed gears!
   * Same 10 deg error must yield identical wheel RPM at Gear 0 (40%) and Gear 3 (100%). */
  {
    Drive_OmniCtrl_t c_lo, c_hi;
    Drive_OmniCtrlInit(&c_lo, true);
    Drive_OmniCtrlInit(&c_hi, true);
    c_lo.gear = 0U; /* 40% */
    c_hi.gear = 3U; /* 100% */
    Drive_Ps2Input_t hold_in = {.joy_sum = {100U, 100U}, .valid = true};
    uint8_t m_lo[4], m_hi[4];
    int16_t t_lo[4], t_hi[4];
    /* Prime at 0 deg, then apply steady 10 deg offset (yaw = -10 deg -> +10 deg error) */
    Drive_OmniCtrlStep(&c_lo, &hold_in, false, 0.0f, true, 4000, m_lo, t_lo);
    Drive_OmniCtrlStep(&c_hi, &hold_in, false, 0.0f, true, 4000, m_hi, t_hi);
    for (int k = 0; k < 20; k++) {
      Drive_OmniCtrlStep(&c_lo, &hold_in, false, -10.0f, true, 4000, m_lo, t_lo);
      Drive_OmniCtrlStep(&c_hi, &hold_in, false, -10.0f, true, 4000, m_hi, t_hi);
    }
    assert(t_lo[0] == t_hi[0] && t_lo[0] > 380 && t_lo[0] < 420); /* ~40.0 RPM = 400 rpm_x10 */

    /* Falsification: +/-0.25 deg IMU noise at rest must stay silent in BRAKE (zero chatter) */
    Drive_OmniCtrl_t c_noise;
    Drive_OmniCtrlInit(&c_noise, true);
    Drive_OmniCtrlStep(&c_noise, &hold_in, false, 0.0f, true, 4000, m_lo, t_lo);
    for (int k = 0; k < 40; k++) {
      float noisy_yaw = (k % 2 == 0) ? 0.25f : -0.25f;
      Drive_OmniCtrlStep(&c_noise, &hold_in, false, noisy_yaw, true, 4000, m_lo, t_lo);
      assert(m_lo[0] == DRIVE_WHEEL_MODE_BRAKE && t_lo[0] == 0);
    }
    /* Schmitt-trigger hysteresis: 1.2 deg drift stays locked (no stick-slip twitch),
     * 1.6 deg drift wakes up cleanly */
    Drive_OmniCtrlStep(&c_noise, &hold_in, false, 1.2f, true, 4000, m_lo, t_lo);
    assert(c_noise.heading_settled && m_lo[0] == DRIVE_WHEEL_MODE_BRAKE && t_lo[0] == 0);
    Drive_OmniCtrlStep(&c_noise, &hold_in, false, 1.6f, true, 4000, m_lo, t_lo);
    assert(!c_noise.heading_settled && m_lo[0] == DRIVE_WHEEL_MODE_VELOCITY && t_lo[0] != 0);

    /* True Field-Centric (pure_field_centric = true): pressing CIRCLE (90 deg)
     * must keep field_axis_offset_deg == 0 so pushing forward at yaw=90 deg
     * moves robot-left (field-forward) instead of robot-forward! */
    Drive_OmniCtrl_t c_fc;
    Drive_OmniCtrlInit(&c_fc, true);
    c_fc.pure_field_centric = true;
    Drive_Ps2Input_t fc_in = {.joy_sum = {100U, 100U}, .valid = true};
    fc_in.ps2_btn[1] = DRIVE_PS2_B1_SQUARE; /* 128: snap to 90 deg (quay trai) */
    Drive_OmniCtrlStep(&c_fc, &fc_in, false, 90.0f, true, 4000, m_lo, t_lo);
    assert(fabsf(c_fc.target_heading_deg - 90.0f) < 0.01f);
    assert(fabsf(c_fc.field_axis_offset_deg) < 0.01f);
    fc_in.ps2_btn[1] = 0U;
    fc_in.joy_sum[0] = 0U; /* Full forward (vy = -100) */
    Drive_OmniCtrlStep(&c_fc, &fc_in, false, 90.0f, true, 4000, m_lo, t_lo);
    assert(t_lo[0] < 0 && t_lo[1] > 0 && t_lo[2] > 0 && t_lo[3] < 0);

    /* Instant hard BRAKE pulse on joystick release (wipes PidBoard vel_int_err_ms)
     * even when a 2.5 deg heading drift is present */
    fc_in.joy_sum[0] = 100U; /* Stick released to neutral */
    Drive_OmniCtrlStep(&c_fc, &fc_in, false, 92.5f, true, 4000, m_lo, t_lo);
    assert(m_lo[0] == DRIVE_WHEEL_MODE_BRAKE && t_lo[0] == 0 && t_lo[1] == 0 &&
           t_lo[2] == 0 && t_lo[3] == 0);
  }

  /* Rigid-body curvature feedforward ratio for R=2050, W=320: inside wheel
   * slows in BOTH directions */
  Drive_LineFollowState_t lstate;
  Drive_LineFollowReset(&lstate);
  Drive_LineGains_t gains = {18,
                             8,
                             200U,
                             4000,
                             2050,
                             320,
                             +1,
                             DRIVE_LINE_YAW_KP_DEFAULT,
                             DRIVE_LINE_NARROW_MAX_CNT,
                             +1};
  uint8_t dmodes[2];
  int16_t dtargets[2];
  Drive_LineFollowCentroidStep(&lstate, &gains, true, 2050, 0x18U, 0, 100U,
                               false, dmodes, dtargets);
  assert(dtargets[0] == 2210 && dtargets[1] == 1890); /* L=R+W/2, R=R-W/2 */
  Drive_LineFollowCentroidStep(&lstate, &gains, true, -2050, 0x18U, 0, 110U,
                               false, dmodes, dtargets);
  assert(dtargets[0] == -2210 &&
         dtargets[1] == -1890); /* Reverse CW still faster on outside Left! */

  /* Dual-Bar heading alignment: pure chassis skew (e_lat=0, e_yaw=+400)
   * generates CW torque in BOTH fwd & rev */
  Drive_LineGains_t straight_gains = {18, 8, 200U, 4000, 0, 320, 0, 14, 3U, +1};
  Drive_LineFollowReset(&lstate);
  Drive_LineFollowDualBarStep(&lstate, &straight_gains, true, 1800, 0x20U, 1U,
                              200, 0x04U, 1U, -200, 10U, false, dmodes,
                              dtargets);
  assert(dtargets[0] == 2360 &&
         dtargets[1] == 1240); /* u_yaw = -560 -> CW rotation */
  Drive_LineFollowDualBarStep(&lstate, &straight_gains, true, -1800, 0x20U, 1U,
                              200, 0x04U, 1U, -200, 20U, false, dmodes,
                              dtargets);
  assert(dtargets[0] == -1240 &&
         dtargets[1] ==
             -2360); /* Reverse: |right| > |left| also rotates nose CW */

  /* rear_polarity_sign = -1 (180-deg rotated Rear PCB): same sign reading means
   * mirrored chassis ends -> flips rear */
  Drive_LineGains_t mirrored_gains = straight_gains;
  mirrored_gains.rear_polarity_sign = -1;
  Drive_LineFollowReset(&lstate);
  Drive_LineFollowDualBarStep(&lstate, &mirrored_gains, true, 1800, 0x20U, 1U,
                              200, 0x20U, 1U, 200, 30U, false, dmodes,
                              dtargets);
  assert(dtargets[0] == 2360 && dtargets[1] == 1240);

  /* Wide-marker guard: asymmetric 4-sensor tape spike (+350) must NOT jerk
   * steering on approach to Load Port */
  Drive_LineFollowReset(&lstate);
  Drive_LineFollowDualBarStep(&lstate, &gains, true, 2050, 0x18U, 2U, 0, 0x18U,
                              2U, 0, 40U, false, dmodes, dtargets);
  assert(dtargets[0] == 2210 && dtargets[1] == 1890);
  Drive_LineFollowDualBarStep(&lstate, &gains, true, 2050, 0x3CU, 4U, 350,
                              0x18U, 2U, 0, 50U, false, dmodes, dtargets);
  assert(dtargets[0] == 2210 &&
         dtargets[1] ==
             1890); /* Clean curvature FF held without +350 wide-tape kick */

  /* Closed-loop rigid-body planar convergence comparison: Front aligned (e_f=0)
   * but 5 deg skew vs Dual-Bar */
  {
    const float L_bar = 300.0f, d_f = 150.0f, d_r = 150.0f, W_mm = 320.0f;
    const float v_per_rpmx10 = 0.42f, mm_per_unit = 0.075f, dt = 0.01f;
    Drive_LineGains_t sim_single = {18, 4, 200U, 4000, 0, 320, 0, 0, 3U, +1};
    Drive_LineGains_t sim_dual = {18, 4, 200U, 4000, 0, 320, 0, 14, 3U, +1};
    float y_s = -d_f * sinf(5.0f * 0.01745329f), psi_s = 5.0f * 0.01745329f;
    float y_d = y_s, psi_d = psi_s;
    Drive_LineFollowState_t st_s, st_d;
    Drive_LineFollowReset(&st_s);
    Drive_LineFollowReset(&st_d);
    for (int k = 0; k < 60; k++) {
      int16_t ef_s = (int16_t)((y_s + d_f * sinf(psi_s)) / mm_per_unit);
      int16_t ef_d = (int16_t)((y_d + d_f * sinf(psi_d)) / mm_per_unit);
      int16_t er_d = (int16_t)((y_d - d_r * sinf(psi_d)) / mm_per_unit);
      uint8_t m_s[2], m_d[2];
      int16_t w_s[2], w_d[2];
      Drive_LineFollowCentroidStep(&st_s, &sim_single, true, 1600, 0x18U, ef_s,
                                   (uint32_t)(k * 10), false, m_s, w_s);
      Drive_LineFollowDualBarStep(&st_d, &sim_dual, true, 1600, 0x18U, 2U, ef_d,
                                  0x18U, 2U, er_d, (uint32_t)(k * 10), false,
                                  m_d, w_d);
      /* Simulate 4% L/R drive asymmetry disturbance */
      float vl_s = w_s[0] * v_per_rpmx10 * 0.96f, vr_s = w_s[1] * v_per_rpmx10;
      float vl_d = w_d[0] * v_per_rpmx10 * 0.96f, vr_d = w_d[1] * v_per_rpmx10;
      psi_s += ((vr_s - vl_s) / W_mm) * dt;
      y_s += 0.5f * (vl_s + vr_s) * sinf(psi_s) * dt;
      psi_d += ((vr_d - vl_d) / W_mm) * dt;
      y_d += 0.5f * (vl_d + vr_d) * sinf(psi_d) * dt;
    }
    (void)L_bar;
    assert(
        fabsf(psi_d) <
        0.015f); /* < 0.86 deg residual yaw skew even under 4% motor mismatch */
    assert(fabsf(y_d) < 5.0f);
    assert((fabsf(psi_d) + fabsf(y_d) / 100.0f) <=
           (fabsf(psi_s) + fabsf(y_s) / 100.0f));
  }

  /* (d) SELECT + L1/R1 adjusts heading_trim_deg by +/-0.5 deg per edge without
   * unlocking open/step spin */
  {
    Drive_OmniCtrl_t trim_ctrl;
    Drive_OmniCtrlInit(&trim_ctrl, true);
    Drive_Ps2Input_t tin = {.joy_sum = {100U, 100U}, .valid = true};
    uint8_t tmodes[4];
    int16_t ttargets[4];

    /* Settle heading_initialized at 0 deg */
    tin.ps2_btn[0] = DRIVE_PS2_B0_SELECT;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    assert(fabsf(trim_ctrl.heading_trim_deg) < 0.001f);

    /* Rising edge SELECT + R1: trim = -0.5 deg, target_heading unchanged,
     * positive omega correction */
    tin.ps2_btn[1] = DRIVE_PS2_B1_R1;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    assert(fabsf(trim_ctrl.heading_trim_deg -
                 (-DRIVE_OMNI_HEADING_TRIM_STEP_DEG)) < 0.001f);
    assert(fabsf(trim_ctrl.target_heading_deg) < 0.001f);
    assert(trim_ctrl.omega_fb_pct > 0.0f);

    /* Holding SELECT + R1 must not auto-repeat or rotate target_heading */
    for (int k = 0; k < 10; k++) {
      Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                         ttargets);
    }
    assert(fabsf(trim_ctrl.heading_trim_deg -
                 (-DRIVE_OMNI_HEADING_TRIM_STEP_DEG)) < 0.001f);
    assert(fabsf(trim_ctrl.target_heading_deg) < 0.001f);

    /* Release R1, press SELECT + L1: trim returns to 0.0 deg */
    tin.ps2_btn[1] = 0U;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    tin.ps2_btn[1] = DRIVE_PS2_B1_L1;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    assert(fabsf(trim_ctrl.heading_trim_deg) < 0.001f);

    /* Falsification: toggling START off (open-loop) + SELECT + R1 updates trim
     * without open spin */
    tin.ps2_btn[0] = (uint8_t)(DRIVE_PS2_B0_SELECT | DRIVE_PS2_B0_START);
    tin.ps2_btn[1] = 0U;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    assert(!trim_ctrl.field_centric && !trim_ctrl.heading_hold);

    tin.ps2_btn[0] = DRIVE_PS2_B0_SELECT;
    tin.ps2_btn[1] = DRIVE_PS2_B1_R1;
    Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                       ttargets);
    assert(fabsf(trim_ctrl.heading_trim_deg -
                 (-DRIVE_OMNI_HEADING_TRIM_STEP_DEG)) < 0.001f);
    /* D-pad micro-adjustment test (16=UP, 32=RIGHT, 64=DOWN, 128=LEFT):
     * Must move in precision crawl mode (DRIVE_OMNI_CRAWL_PCT = 15% = 60 RPM / 0.13 m/s) */
    tin.joy_sum[0] = 100U;
    tin.joy_sum[1] = 100U;
    tin.ps2_btn[0] = DRIVE_PS2_B0_UP; /* 16 */
    tin.ps2_btn[1] = 0U;
    for (int step = 0; step < 10; step++) {
      Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                         ttargets);
    }
    assert(trim_ctrl.active_speed_pct == DRIVE_OMNI_CRAWL_PCT);
    /* Target RPM must be around 42 RPM (424 rpm_x10) for precision crawl (15% speed) */
    for (uint8_t w = 0U; w < 4U; w++) {
      assert(abs(ttargets[w]) == 424); /* ~42.4 RPM */
    }

    /* Servo code guard: values 200..203 must NOT trigger D-pad or movement */
    tin.ps2_btn[0] = 200U; /* Servo 1 forward code */
    for (int step = 0; step < 10; step++) {
      Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, true, 4000, tmodes,
                         ttargets);
    }
    assert(ttargets[0] == 0 && ttargets[1] == 0 && ttargets[2] == 0 &&
           ttargets[3] == 0);

    /* L1 3x Speed Boost test (tx_buf[3] = 4):
     * Pushing joystick forward with L1 must triple speed compared to normal,
     * and L1 must NOT produce CCW rotation (omega == 0)! */
    Drive_OmniCtrlInit(&trim_ctrl, false);
    trim_ctrl.heading_hold = false; /* Disable closed-loop heading for pure velocity comparison */
    tin.ps2_btn[0] = 0U;
    tin.joy_sum[0] = 70U; /* Tilted forward: target_y = -30% */
    tin.joy_sum[1] = 100U;
    tin.ps2_btn[1] = 0U; /* Base speed (default Gear 0 = 33%) */
    for (int step = 0; step < 20; step++) {
      Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, false, 4000, tmodes,
                         ttargets);
    }
    int16_t base_rpm = abs(ttargets[0]);

    Drive_OmniCtrlInit(&trim_ctrl, false);
    trim_ctrl.heading_hold = false;
    tin.ps2_btn[1] = DRIVE_PS2_B1_L1; /* 3x Boost */
    for (int step = 0; step < 20; step++) {
      Drive_OmniCtrlStep(&trim_ctrl, &tin, false, 0.0f, false, 4000, tmodes,
                         ttargets);
    }
    int16_t boosted_rpm = abs(ttargets[0]);
    assert(boosted_rpm >= (int16_t)(base_rpm * 2.8f)); /* Tripled speed */
    assert(fabsf(trim_ctrl.slew.omega_pct) < 0.01f);    /* L1 does NOT rotate CCW! */
  }
  printf("      PASS.\n");
}

static void test_cd4067_and_flash_cal(void) {
  printf("[5/6] Running CD4067 2-pt cal, centroid & Flash CRC-32 tests...\n");
  MockHal_ResetAll();
  FlashCal_MockReset();

  DMA_HandleTypeDef hadc_dma = {.State = HAL_DMA_STATE_READY};
  ADC_HandleTypeDef hadc1 = {
      .Instance = ADC1, .State = HAL_ADC_STATE_READY, .DMA_Handle = &hadc_dma};
  TIM_HandleTypeDef htim7 = {.Instance = TIM7};
  CD4067_Init(&hadc1, &htim7);
  assert(CD4067_StartLine2x8(CD4067_DEFAULT_THRESHOLD) == HAL_OK);

  /* Uneven sensor gains across ch 0..15 + narrow line over floor */
  uint16_t floor_vals[16];
  uint16_t line_vals[16];
  for (uint8_t i = 0U; i < 16U; i++) {
    floor_vals[i] = (uint16_t)(300U + i * 60U);
    line_vals[i] = (uint16_t)(floor_vals[i] + 900U + i * 50U);
  }

  /* Falsification: low-contrast sweep (< 400) must be REJECTED */
  CD4067_BeginTwoPointCal();
  uint16_t weak_line[16];
  memcpy(weak_line, line_vals, sizeof(weak_line));
  weak_line[5] = (uint16_t)(floor_vals[5] + 200U); /* Contrast only 200 */
  sim_adc_sweep(&hadc1, &htim7, floor_vals);
  sim_adc_sweep(&hadc1, &htim7, weak_line);
  assert(!CD4067_FinishTwoPointCal(CD4067_POLARITY_AUTO));
  assert((cd4067_data.low_contrast_mask & (1U << 5U)) != 0U);
  assert(!CD4067_IsCalibrated());

  /* Valid 2-point sweep (3x floor vs 1x line so AUTO polarity cleanly infers
   * line_on_high=1) */
  CD4067_BeginTwoPointCal();
  sim_adc_sweep(&hadc1, &htim7, floor_vals);
  sim_adc_sweep(&hadc1, &htim7, floor_vals);
  sim_adc_sweep(&hadc1, &htim7, floor_vals);
  sim_adc_sweep(&hadc1, &htim7, line_vals);
  assert(CD4067_FinishTwoPointCal(CD4067_POLARITY_AUTO));
  assert(cd4067_data.line_on_high == 1U);
  assert(CD4067_IsCalibrated());

  /* Sub-sensor centroid accuracy: weight between sensor 3 (-100) and sensor 4
   * (+100) */
  uint16_t probe[16];
  memcpy(probe, floor_vals, sizeof(probe));
  probe[3] = (uint16_t)((floor_vals[3] + line_vals[3]) / 2U); /* norm 500 */
  probe[4] = line_vals[4];                                    /* norm 1000 */
  sim_adc_sweep(&hadc1, &htim7, probe);
  /* eff_norm[3] = 350 @ -100, eff_norm[4] = 850 @ +100 -> (-35000 + 85000)/1200
   * = +41 */
  int16_t c = CD4067_GetFrontCentroidX10();
  assert(c >= 38 && c <= 44);

  /* Save to Flash Sector 11 -> reload -> falsify via 1-bit flip */
  FlashCal_Record_t rec;
  FlashCal_InitDefaults(&rec);
  rec.seq = 7U;
  rec.flags = FLASH_CAL_FLAG_LINE_VALID | FLASH_CAL_FLAG_IMU_VALID;
  rec.imu_field_offset_deg = 12.5f;
  CD4067_ExportTwoPointCal(rec.cal_min, rec.cal_max, &rec.line_on_high);
  assert(FlashCal_Save(&rec));

  FlashCal_Record_t loaded;
  assert(FlashCal_Load(&loaded));
  assert(loaded.seq == 7U);
  assert(fabsf(loaded.imu_field_offset_deg - 12.5f) < 0.001f);

  FlashCal_MockStoragePtr()[20] ^= 0x04U; /* Flip 1 bit */
  assert(!FlashCal_Load(&loaded));        /* Must reject on CRC-32 mismatch! */

  /* Inverted floor/line polarity test: bright floor (~3200), dark black line
   * (~800) */
  uint16_t bright_floor[16];
  uint16_t black_line[16];
  for (uint8_t i = 0U; i < 16U; i++) {
    bright_floor[i] = 3200U;
    black_line[i] = 800U;
  }
  CD4067_BeginTwoPointCal();
  sim_adc_sweep(&hadc1, &htim7, bright_floor);
  sim_adc_sweep(&hadc1, &htim7, bright_floor);
  sim_adc_sweep(&hadc1, &htim7, bright_floor);
  sim_adc_sweep(&hadc1, &htim7, black_line);
  assert(CD4067_FinishTwoPointCal(CD4067_POLARITY_AUTO));
  assert(cd4067_data.line_on_high == 0U);
  memcpy(probe, bright_floor, sizeof(probe));
  probe[3] = 800U;
  probe[4] = 800U;
  sim_adc_sweep(&hadc1, &htim7, probe);
  assert(cd4067_data.front_mask == 0x18U);
  assert(abs(CD4067_GetFrontCentroidX10()) <= 5);
  printf("      PASS.\n");
}

static void test_itc2026_auto_fsm_and_rf_security(void) {
  printf("[6/6] Running ITC 2026 autonomous FSM & RF TDK-rejection tests...\n");
  MockHal_ResetAll();

  /* RF strict whitelist test: RF_NODE_AUTO accepts RF_NODE_MANUAL, rejects
   * RF_NODE_TDK */
  UART_HandleTypeDef huart_tx_tdk = {.Instance = USART1,
                                     .gState = HAL_UART_STATE_READY};
  UART_HandleTypeDef huart_tx_man = {.Instance = USART2,
                                     .gState = HAL_UART_STATE_READY};
  UART_HandleTypeDef huart_rx_auto = {.Instance = UART4,
                                      .gState = HAL_UART_STATE_READY};

  RF_HC12_t rf_tdk, rf_man, rf_auto;
  RF_HC12_Init(&rf_tdk, &huart_tx_tdk, NULL, 0U, RF_NODE_TDK, RF_SRC_MASK_ALL);
  RF_HC12_Init(&rf_man, &huart_tx_man, NULL, 0U, RF_NODE_MANUAL,
               RF_SRC_MASK(RF_NODE_TDK));
  RF_HC12_Init(&rf_auto, &huart_rx_auto, NULL, 0U, RF_NODE_AUTO,
               RF_SRC_MASK(RF_NODE_MANUAL));

  RF_PayloadTdk_t tdk_msg = {.ext_sw = RF_TDK_PS2_VALID_BIT,
                             .joy_left = {120, 130},
                             .joy_sum = {200, 210},
                             .ps2_btn = {0x11, 0x22}};
  assert(RF_HC12_SendTdkCmd(&rf_tdk, RF_NODE_BCAST, &tdk_msg) == HAL_OK);
  assert(huart_tx_tdk.last_tx_len ==
         RF_6BYTE_LEN); /* Must be exactly 6 bytes */
  assert(huart_tx_tdk.last_tx_buf[0] == RF_6BYTE_SOF);
  assert(huart_tx_tdk.last_tx_buf[1] == 200);
  assert(huart_tx_tdk.last_tx_buf[2] == 210);
  assert(huart_tx_tdk.last_tx_buf[3] == 0x22);
  assert(huart_tx_tdk.last_tx_buf[4] == 0x11);
  assert(huart_tx_tdk.last_tx_buf[5] == 0U);

  /* rf_auto rejects RF_NODE_TDK */
  RF_HC12_FeedBytes(&rf_auto, huart_tx_tdk.last_tx_buf,
                    huart_tx_tdk.last_tx_len, 10U);
  assert(rf_auto.stats.rejected_tdk_frames == 1U);
  assert(rf_auto.stats.rx_ok_frames == 0U);

  /* rf_man accepts RF_NODE_TDK 6-byte competition frame and decodes payload
   * perfectly */
  RF_HC12_FeedBytes(&rf_man, huart_tx_tdk.last_tx_buf, huart_tx_tdk.last_tx_len,
                    10U);
  assert(rf_man.stats.rx_ok_frames == 1U);
  RF_Frame_t pop_frame;
  RF_PayloadTdk_t decoded = {0};
  assert(RF_HC12_PopLatestFrame(&rf_man, &pop_frame) == true);
  assert(RF_HC12_DecodeTdkCmd(&pop_frame, &decoded) == true);
  assert(decoded.joy_sum[0] == 200 && decoded.joy_sum[1] == 210);
  assert(decoded.ps2_btn[0] == 0x11 && decoded.ps2_btn[1] == 0x22);

  /* Continuous sequence counter verification */
  assert(RF_HC12_SendTdkCmd(&rf_tdk, RF_NODE_BCAST, &tdk_msg) == HAL_OK);
  assert(huart_tx_tdk.last_tx_buf[5] == 1U); /* Seq count increments 0 -> 1 */
  RF_HC12_FeedBytes(&rf_man, huart_tx_tdk.last_tx_buf, huart_tx_tdk.last_tx_len,
                    15U);
  assert(rf_man.stats.rx_ok_frames == 2U);

  RF_PayloadManualToAuto_t man_msg = {.cmd = 1U};
  assert(RF_HC12_SendManualToAuto(&rf_man, RF_NODE_AUTO, &man_msg) == HAL_OK);
  RF_HC12_FeedBytes(&rf_auto, huart_tx_man.last_tx_buf,
                    huart_tx_man.last_tx_len, 20U);
  assert(rf_auto.stats.rx_ok_frames == 1U);

  /* Autonomous ITC 2026 FSM full cycle */
  ITC_Auto_t fsm;
  ITC_AutoInit(&fsm, ITC_BASE_RUN_RPMX10, 0.0f);
  ITC_AutoInput_t in = {0};
  ITC_AutoOutput_t out = {0};

  in.cal_done = true;
  in.front_dist_mm = 2000U;
  in.rear_dist_mm = 2000U;
  in.now_ms = 100U;
  in.start_toggle_edge = true;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_EXIT_START_BOX);
  assert(out.wheel_targets[0] > 0 && out.wheel_targets[1] > 0);

  /* Reach track line -> transition into TRACK_RUN */
  in.start_toggle_edge = false;
  in.front_mask = 0x18U;
  in.front_count = 2U;
  in.now_ms = 500U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_TRACK_RUN);
  assert(out.dir_sign == +1);

  /* Hit Load Port wide tape marker -> brake + extend SERVO_CH_1 to 135 deg for
   * 2500ms */
  in.front_mask = 0x7EU;
  in.front_count = 6U;
  in.now_ms = 800U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_LOAD_PORT_HOLD);
  assert(fabsf(out.arm_deg - ITC_SERVO_ARM_TRIGGER_DEG) < 0.1f);
  assert(out.wheel_targets[0] == 0 && out.wheel_targets[1] == 0);
  assert(out.ports_triggered == 1U);

  in.now_ms = 3400U;
  /* Departure over wide Load Port tape (front_count=6) must still drive cleanly
   * off the port! */
  in.rear_mask = 0x18U;
  in.rear_count = 2U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_TRACK_RUN);
  assert(fabsf(out.arm_deg - ITC_SERVO_ARM_STOW_DEG) < 0.1f);
  assert(out.wheel_targets[0] > 0 && out.wheel_targets[1] > 0);

  /* Clean narrow track acquisition + skew-aware speed moderation check */
  in.now_ms = 3450U;
  in.front_mask = 0x18U;
  in.front_count = 2U;
  in.front_centroid_x10 = 250;
  in.rear_centroid_x10 = -250; /* |e_yaw| = 500 -> throttles linear speed to 70%
                                  while squaring up */
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_TRACK_RUN);
  assert((out.wheel_targets[0] + out.wheel_targets[1]) / 2 <
         ITC_BASE_RUN_RPMX10);
  in.front_centroid_x10 = 0;
  in.rear_centroid_x10 = 0;

  /* Opponent blocks front track -> OBSTACLE_STOP -> wait >= 1200ms -> reverse
   * CCW */
  in.front_dist_mm = 280U;
  in.front_blocked = true;
  in.now_ms = 4000U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_OBSTACLE_STOP);
  assert(out.wheel_targets[0] == 0 && out.wheel_targets[1] == 0);

  in.now_ms =
      5300U; /* >= 1200ms persisted block, rear_dist_mm = 2000mm clear */
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_TRACK_RUN);
  assert(out.dir_sign == -1);
  assert(out.reversals == 1U);
  assert(out.wheel_targets[0] < 0 && out.wheel_targets[1] < 0);

  /* Sandwich falsification: now REAR blocks while FRONT is STILL blocked ->
   * stay stopped! */
  in.rear_dist_mm = 250U;
  in.rear_blocked = true;
  in.now_ms = 5400U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_OBSTACLE_STOP);
  in.now_ms = 7500U;
  ITC_AutoStep(&fsm, &in, &out);
  assert(out.state == ITC_STATE_OBSTACLE_STOP);
  assert(out.wheel_targets[0] == 0 && out.wheel_targets[1] == 0);
  printf("      PASS.\n");
}

static void test_servo_180_and_clamp_toggle(void) {
  printf("[7/7] Running 180-deg servo calibration & R1 clamp toggle tests...\n");
  MockHal_ResetAll();

  TIM_HandleTypeDef htim3 = {.Instance = TIM3};
  TIM_HandleTypeDef htim4 = {.Instance = TIM4};
  TIM_HandleTypeDef htim12 = {.Instance = TIM12};

  assert(Servo_Init(&htim3, &htim4, &htim12) == HAL_OK);

  /* Configure SERVO_CLAMP_CH (CH6 / TIM4_CH4 / PB9) as 180-deg positional clamp servo */
  assert(Servo_Config180(SERVO_CLAMP_CH, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false) == HAL_OK);
  assert(Servo_Config180(SERVO_CH_1, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false) == HAL_OK);
  assert(Servo_Config180(SERVO_CH_2, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false) == HAL_OK);
  /* Configure SERVO_360_CH2 (CH5 / TIM4_CH3 / PB8) as continuous 360 */
  assert(Servo_Config360(SERVO_360_CH2, SERVO_PWM_NEUTRAL_US, 500U, 2.0f, false) == HAL_OK);

  /* 1. Default initial state on TIM4_CH4 (SERVO_CLAMP_CH): 0.0 deg (closed per user spec) */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, SERVO_CLAMP_DEFAULT_CLOSE_DEG) == HAL_OK);
  /* Pulse = 500 + 0 = 500 us */
  assert(g_mock_tim4.CCR4 == 500U);

  Servo_ChannelState_t st;
  assert(Servo_GetState(SERVO_CLAMP_CH, &st));
  assert(st.mode == SERVO_MODE_ANGLE_180);
  assert(st.current_us == 500U);
  assert(fabsf(st.last_cmd - 0.0f) < 0.01f);

  /* 2. 1st press (toggle to open): 30.0 deg */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, SERVO_CLAMP_DEFAULT_OPEN_DEG) == HAL_OK);
  /* Pulse = 500 + (30 / 180) * 2000 = 833 us */
  assert(g_mock_tim4.CCR4 == 833U);
  assert(Servo_GetState(SERVO_CLAMP_CH, &st));
  assert(st.current_us == 833U);
  assert(fabsf(st.last_cmd - 30.0f) < 0.01f);

  /* 3. 2nd press (toggle to closed): 0.0 deg */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, SERVO_CLAMP_DEFAULT_CLOSE_DEG) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 500U);

  /* 4. Full range & clamping boundary checks on TIM4_CH4 */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, 90.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 1500U);

  assert(Servo_SetAngle180(SERVO_CLAMP_CH, 180.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 2500U);

  /* Clamp negative angles to 0 deg */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, -10.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 500U);

  /* Clamp excessive angles to 180 deg */
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, 200.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 2500U);

  /* 5. Continuous 360 #2 on TIM4_CH3 (SERVO_360_CH2 / PB8): verify independent speed */
  assert(Servo_SetSpeed360(SERVO_360_CH2, 70.0f) == HAL_OK);
  /* 70% of 500us = 350us -> 1500 + 350 = 1850 us */
  assert(g_mock_tim4.CCR3 == 1850U);
  assert(Servo_Stop(SERVO_360_CH2) == HAL_OK);
  assert(g_mock_tim4.CCR3 == 1500U);

  /* 6. Inverted mode test on SERVO_CLAMP_CH */
  assert(Servo_Config180(SERVO_CLAMP_CH, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, true) == HAL_OK);
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, 0.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 2500U);
  assert(Servo_SetAngle180(SERVO_CLAMP_CH, 180.0f) == HAL_OK);
  assert(g_mock_tim4.CCR4 == 500U);
  /* Restore normal polarity */
  assert(Servo_Config180(SERVO_CLAMP_CH, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false) == HAL_OK);

  /* 7. Simulate R1 button toggle state machine */
  bool clamp_opened = false;
  uint8_t prev_r1 = 0U;
  float angles[6];

  uint8_t r1_inputs[6] = {0U, 1U, 1U, 0U, 1U, 0U};
  /* Step 0: unpressed -> initial (0 deg)
   * Step 1: press R1 (rise) -> open (40 deg)
   * Step 2: hold R1 -> stays open (40 deg)
   * Step 3: release R1 -> stays open (40 deg)
   * Step 4: press R1 (rise) -> closed (0 deg)
   * Step 5: release R1 -> stays closed (0 deg)
   */
  for (int i = 0; i < 6; i++) {
    uint8_t r1_now = r1_inputs[i];
    if ((r1_now != 0U) && (prev_r1 == 0U)) {
      clamp_opened = !clamp_opened;
    }
    angles[i] = clamp_opened ? SERVO_CLAMP_DEFAULT_OPEN_DEG
                             : SERVO_CLAMP_DEFAULT_CLOSE_DEG;
    prev_r1 = r1_now;
  }
  assert(fabsf(angles[0] - 0.0f) < 0.01f);
  assert(fabsf(angles[1] - 30.0f) < 0.01f);
  assert(fabsf(angles[2] - 30.0f) < 0.01f);
  assert(fabsf(angles[3] - 30.0f) < 0.01f);
  assert(fabsf(angles[4] - 0.0f) < 0.01f);
  assert(fabsf(angles[5] - 0.0f) < 0.01f);

  printf("      PASS.\n");
}

int main(void) {
  test_vl53l1x_uld_boot_and_poll();
  test_rs485_non_destructive_error_recovery();
  test_bno055_hardening_and_backoff();
  test_drive_kinematics_custom_field_centric_and_curvature();
  test_cd4067_and_flash_cal();
  test_itc2026_auto_fsm_and_rf_security();
  test_servo_180_and_clamp_toggle();
  printf("ALL 7 HOST UNIT & FALSIFICATION TEST SUITES PASSED!\n");
  return 0;
}
