/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************
*/

/**
 * @file  VL53L1X_api.h
 * @brief Official STMicroelectronics VL53L1X Ultra Lite Driver (ULD) v3.5.2 API
 */

#ifndef _API_H_
#define _API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53l1_platform.h"

#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       5
#define VL53L1X_IMPLEMENTATION_VER_SUB         2
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

typedef int8_t VL53L1X_ERROR;

#define SOFT_RESET                                          0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                    0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS      0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS  0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS  0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                  0x001E
#define MM_CONFIG__INNER_OFFSET_MM                          0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                          0x0022
#define PAD_I2C_HV__EXTSUP_CONFIG                           0x002E
#define GPIO_HV_MUX__CTRL                                   0x0030
#define GPIO__TIO_HV_STATUS                                 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                       0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                     0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                   0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                        0x0060
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                   0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                   0x0062
#define RANGE_CONFIG__VCSEL_PERIOD_B                        0x0063
#define RANGE_CONFIG__SIGMA_THRESH                          0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS         0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                      0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD              0x006C
#define SYSTEM__THRESH_HIGH                                 0x0072
#define SYSTEM__THRESH_LOW                                  0x0074
#define SD_CONFIG__WOI_SD0                                  0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                        0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                    0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE       0x0080
#define SYSTEM__SEQUENCE_CONFIG                             0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD               0x0082
#define SYSTEM__INTERRUPT_CLEAR                             0x0086
#define SYSTEM__MODE_START                                  0x0087
#define VL53L1_RESULT__RANGE_STATUS                         0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0       0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                  0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0               0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0  0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                    0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD             0x013E

#define VL53L1X_DEFAULT_CFG_ADDR_START                      0x2DU
#define VL53L1X_DEFAULT_CFG_ADDR_END                        0x87U
#define VL53L1X_DEFAULT_CFG_LEN                             91U
#define VL53L1X_INIT_DATA_READY_TIMEOUT_ITERS               250U

typedef struct {
  uint8_t  major;
  uint8_t  minor;
  uint8_t  build;
  uint32_t revision;
} VL53L1X_Version_t;

typedef struct {
  uint8_t  Status;
  uint16_t Distance;
  uint16_t Ambient;
  uint16_t SigPerSPAD;
  uint16_t NumSPADs;
} VL53L1X_Result_t;

extern const uint8_t VL51L1X_DEFAULT_CONFIGURATION[VL53L1X_DEFAULT_CFG_LEN];

VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion);
VL53L1X_ERROR VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address);
VL53L1X_ERROR VL53L1X_SensorInit(uint16_t dev);
VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t dev);
VL53L1X_ERROR VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t IntPol);
VL53L1X_ERROR VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t *pIntPol);
VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev);
VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev);
VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady);
VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs);
VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t *pTimingBudgetInMs);
VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DistanceMode);
VL53L1X_ERROR VL53L1X_GetDistanceMode(uint16_t dev, uint16_t *pDistanceMode);
VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t InterMeasurementInMs);
VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t *pIM);
VL53L1X_ERROR VL53L1X_BootState(uint16_t dev, uint8_t *state);
VL53L1X_ERROR VL53L1X_GetSensorId(uint16_t dev, uint16_t *id);
VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance);
VL53L1X_ERROR VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t *signalPerSp);
VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t *amb);
VL53L1X_ERROR VL53L1X_GetSignalRate(uint16_t dev, uint16_t *signalRate);
VL53L1X_ERROR VL53L1X_GetSpadNb(uint16_t dev, uint16_t *spNb);
VL53L1X_ERROR VL53L1X_GetAmbientRate(uint16_t dev, uint16_t *ambRate);
VL53L1X_ERROR VL53L1X_GetRangeStatus(uint16_t dev, uint8_t *rangeStatus);
VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult);
VL53L1X_ERROR VL53L1X_SetOffset(uint16_t dev, int16_t OffsetValue);
VL53L1X_ERROR VL53L1X_GetOffset(uint16_t dev, int16_t *Offset);
VL53L1X_ERROR VL53L1X_SetXtalk(uint16_t dev, uint16_t XtalkValue);
VL53L1X_ERROR VL53L1X_GetXtalk(uint16_t dev, uint16_t *Xtalk);
VL53L1X_ERROR VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t ThreshLow,
                                         uint16_t ThreshHigh, uint8_t Window,
                                         uint8_t IntOnNoTarget);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t *window);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high);
VL53L1X_ERROR VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y);
VL53L1X_ERROR VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y);
VL53L1X_ERROR VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter);
VL53L1X_ERROR VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter);
VL53L1X_ERROR VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal);
VL53L1X_ERROR VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t *signal);
VL53L1X_ERROR VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma);
VL53L1X_ERROR VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t *signal);
VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(uint16_t dev);

#ifdef __cplusplus
}
#endif

#endif /* _API_H_ */

