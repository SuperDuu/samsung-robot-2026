/*
 Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Core and is dual licensed,
 either 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

/**
 * @file  VL53L1X_api.c
 * @brief Official STMicroelectronics VL53L1X Ultra Lite Driver (ULD) v3.5.2
 *        with timeout guards on SensorInit / StartTemperatureUpdate loops.
 */

#include "VL53L1X_api.h"
#include <string.h>

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[VL53L1X_DEFAULT_CFG_LEN] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xcc, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetDistanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetDistanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

static const uint8_t status_rtn[24] = {
  255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
  255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
  255, 255, 11, 12
};

VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion)
{
  if (pVersion == NULL)
  {
    return 1;
  }
  pVersion->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
  pVersion->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
  pVersion->build = VL53L1X_IMPLEMENTATION_VER_SUB;
  pVersion->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
  return 0;
}

VL53L1X_ERROR VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address)
{
  return VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, (uint8_t)(new_address >> 1U));
}

VL53L1X_ERROR VL53L1X_SensorInit(uint16_t dev)
{
  VL53L1X_ERROR status = 0;
  uint8_t Addr = 0x00U;
  uint8_t tmp = 0U;

  for (Addr = VL53L1X_DEFAULT_CFG_ADDR_START; Addr <= VL53L1X_DEFAULT_CFG_ADDR_END; Addr++)
  {
    status |= VL53L1_WrByte(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - VL53L1X_DEFAULT_CFG_ADDR_START]);
    if (status != 0)
    {
      return status;
    }
  }

  status |= VL53L1X_StartRanging(dev);
  if (status != 0)
  {
    return status;
  }

  uint16_t wait_iter = 0U;
  while ((tmp == 0U) && (status == 0) && (wait_iter < VL53L1X_INIT_DATA_READY_TIMEOUT_ITERS))
  {
    status |= VL53L1X_CheckForDataReady(dev, &tmp);
    if ((tmp == 0U) && (status == 0))
    {
      (void)VL53L1_WaitMs(dev, 2);
    }
    wait_iter++;
  }

  if (status != 0)
  {
    (void)VL53L1X_StopRanging(dev);
    return status;
  }
  if (tmp == 0U)
  {
    (void)VL53L1X_StopRanging(dev);
    return -1;
  }

  status |= VL53L1X_ClearInterrupt(dev);
  status |= VL53L1X_StopRanging(dev);
  status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09U); /* two bounds VHV */
  status |= VL53L1_WrByte(dev, 0x000BU, 0x00U); /* start VHV from previous temperature */
  return status;
}

VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t dev)
{
  return VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01U);
}

VL53L1X_ERROR VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t NewPolarity)
{
  uint8_t Temp = 0U;
  VL53L1X_ERROR status = 0;

  status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
  if (status != 0)
  {
    return status;
  }
  Temp = (uint8_t)(Temp & 0xEFU);
  status |= VL53L1_WrByte(dev, GPIO_HV_MUX__CTRL, (uint8_t)(Temp | (uint8_t)((!(NewPolarity & 1U)) << 4U)));
  return status;
}

VL53L1X_ERROR VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t *pInterruptPolarity)
{
  uint8_t Temp = 0U;
  VL53L1X_ERROR status = 0;

  if (pInterruptPolarity == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
  Temp = (uint8_t)(Temp & 0x10U);
  *pInterruptPolarity = (uint8_t)(!(Temp >> 4U));
  return status;
}

VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev)
{
  return VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40U);
}

VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev)
{
  return VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00U);
}

VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady)
{
  uint8_t Temp = 0U;
  uint8_t IntPol = 0U;
  VL53L1X_ERROR status = 0;

  if (isDataReady == NULL)
  {
    return 1;
  }
  *isDataReady = 0U;
  status |= VL53L1X_GetInterruptPolarity(dev, &IntPol);
  status |= VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
  if (status == 0)
  {
    *isDataReady = ((Temp & 1U) == IntPol) ? 1U : 0U;
  }
  return status;
}

VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs)
{
  uint16_t DM = 0U;
  VL53L1X_ERROR status = 0;

  status |= VL53L1X_GetDistanceMode(dev, &DM);
  if (status != 0)
  {
    return status;
  }
  if (DM == 0U)
  {
    return 1;
  }
  else if (DM == 1U) /* Short DistanceMode */
  {
    switch (TimingBudgetInMs)
    {
      case 15U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001DU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027U);
        break;
      case 20U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006EU);
        break;
      case 33U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006EU);
        break;
      case 50U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01AEU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8U);
        break;
      case 100U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388U);
        break;
      case 200U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496U);
        break;
      case 500U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1U);
        break;
      default:
        status = 1;
        break;
    }
  }
  else /* Long DistanceMode (2) */
  {
    switch (TimingBudgetInMs)
    {
      case 20U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001EU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022U);
        break;
      case 33U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006EU);
        break;
      case 50U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00ADU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6U);
        break;
      case 100U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CCU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EAU);
        break;
      case 200U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9U);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8U);
        break;
      case 500U:
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048FU);
        status |= VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4U);
        break;
      default:
        status = 1;
        break;
    }
  }
  return status;
}

VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t *pTimingBudget)
{
  uint16_t Temp = 0U;
  VL53L1X_ERROR status = 0;

  if (pTimingBudget == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
  switch (Temp)
  {
    case 0x001DU:
      *pTimingBudget = 15U;
      break;
    case 0x0051U:
    case 0x001EU:
      *pTimingBudget = 20U;
      break;
    case 0x00D6U:
    case 0x0060U:
      *pTimingBudget = 33U;
      break;
    case 0x01AEU:
    case 0x00ADU:
      *pTimingBudget = 50U;
      break;
    case 0x02E1U:
    case 0x01CCU:
      *pTimingBudget = 100U;
      break;
    case 0x03E1U:
    case 0x02D9U:
      *pTimingBudget = 200U;
      break;
    case 0x0591U:
    case 0x048FU:
      *pTimingBudget = 500U;
      break;
    default:
      status |= 1;
      *pTimingBudget = 0U;
      break;
  }
  return status;
}

VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DM)
{
  uint16_t TB = 0U;
  VL53L1X_ERROR status = 0;

  status |= VL53L1X_GetTimingBudgetInMs(dev, &TB);
  if (status != 0)
  {
    return status;
  }
  switch (DM)
  {
    case 1U:
      status |= VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14U);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07U);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05U);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38U);
      status |= VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0705U);
      status |= VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606U);
      break;
    case 2U:
      status |= VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0AU);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0FU);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0DU);
      status |= VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8U);
      status |= VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0F0DU);
      status |= VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0EU);
      break;
    default:
      status = 1;
      break;
  }

  if (status == 0)
  {
    status |= VL53L1X_SetTimingBudgetInMs(dev, TB);
  }
  return status;
}

VL53L1X_ERROR VL53L1X_GetDistanceMode(uint16_t dev, uint16_t *DM)
{
  uint8_t TempDM = 0U;
  VL53L1X_ERROR status = 0;

  if (DM == NULL)
  {
    return 1;
  }
  *DM = 0U;
  status |= VL53L1_RdByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
  if (TempDM == 0x14U)
  {
    *DM = 1U;
  }
  else if (TempDM == 0x0AU)
  {
    *DM = 2U;
  }
  return status;
}

VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t InterMeasMs)
{
  uint16_t ClockPLL = 0U;
  VL53L1X_ERROR status = 0;

  status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
  if (status != 0)
  {
    return status;
  }
  ClockPLL = (uint16_t)(ClockPLL & 0x3FFU);
  status |= VL53L1_WrDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
                           (uint32_t)((float)ClockPLL * (float)InterMeasMs * 1.075f));
  return status;
}

VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t *pIM)
{
  uint16_t ClockPLL = 0U;
  VL53L1X_ERROR status = 0;
  uint32_t tmp = 0U;

  if (pIM == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
  ClockPLL = (uint16_t)(ClockPLL & 0x3FFU);
  if ((status == 0) && (ClockPLL > 0U))
  {
    *pIM = (uint16_t)((float)tmp / ((float)ClockPLL * 1.065f));
  }
  else
  {
    *pIM = 0U;
  }
  return status;
}

VL53L1X_ERROR VL53L1X_BootState(uint16_t dev, uint8_t *state)
{
  VL53L1X_ERROR status = 0;
  uint8_t tmp = 0U;

  if (state == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
  *state = tmp;
  return status;
}

VL53L1X_ERROR VL53L1X_GetSensorId(uint16_t dev, uint16_t *sensorId)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (sensorId == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
  *sensorId = tmp;
  return status;
}

VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (distance == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);
  *distance = tmp;
  return status;
}

VL53L1X_ERROR VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t *signalRate)
{
  VL53L1X_ERROR status = 0;
  uint16_t SpNb = 1U;
  uint16_t signal = 0U;

  if (signalRate == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
  *signalRate = (SpNb > 0U) ? (uint16_t)((200.0f * (float)signal) / (float)SpNb) : 0U;
  return status;
}

VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t *ambPerSp)
{
  VL53L1X_ERROR status = 0;
  uint16_t AmbientRate = 0U;
  uint16_t SpNb = 1U;

  if (ambPerSp == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
  *ambPerSp = (SpNb > 0U) ? (uint16_t)((200.0f * (float)AmbientRate) / (float)SpNb) : 0U;
  return status;
}

VL53L1X_ERROR VL53L1X_GetSignalRate(uint16_t dev, uint16_t *signal)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (signal == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
  *signal = (uint16_t)(tmp * 8U);
  return status;
}

VL53L1X_ERROR VL53L1X_GetSpadNb(uint16_t dev, uint16_t *spNb)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (spNb == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
  *spNb = (uint16_t)(tmp >> 8U);
  return status;
}

VL53L1X_ERROR VL53L1X_GetAmbientRate(uint16_t dev, uint16_t *ambRate)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (ambRate == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
  *ambRate = (uint16_t)(tmp * 8U);
  return status;
}

VL53L1X_ERROR VL53L1X_GetRangeStatus(uint16_t dev, uint8_t *rangeStatus)
{
  VL53L1X_ERROR status = 0;
  uint8_t RgSt = 0U;

  if (rangeStatus == NULL)
  {
    return 1;
  }
  *rangeStatus = 255U;
  status |= VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
  RgSt = (uint8_t)(RgSt & 0x1FU);
  if (RgSt < 24U)
  {
    *rangeStatus = status_rtn[RgSt];
  }
  return status;
}

VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult)
{
  VL53L1X_ERROR status = 0;
  uint8_t Temp[17] = {0};
  uint8_t RgSt = 255U;

  if (pResult == NULL)
  {
    return 1;
  }
  status |= VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, Temp, 17U);
  RgSt = (uint8_t)(Temp[0] & 0x1FU);
  if (RgSt < 24U)
  {
    RgSt = status_rtn[RgSt];
  }
  pResult->Status = RgSt;
  pResult->Ambient = (uint16_t)(((uint16_t)Temp[7] << 8U | (uint16_t)Temp[8]) * 8U);
  pResult->NumSPADs = Temp[3];
  pResult->SigPerSPAD = (uint16_t)(((uint16_t)Temp[15] << 8U | (uint16_t)Temp[16]) * 8U);
  pResult->Distance = (uint16_t)(((uint16_t)Temp[13] << 8U) | (uint16_t)Temp[14]);
  return status;
}

VL53L1X_ERROR VL53L1X_SetOffset(uint16_t dev, int16_t OffsetValue)
{
  VL53L1X_ERROR status = 0;
  int16_t Temp = (int16_t)(OffsetValue * 4);

  status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)Temp);
  status |= VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0000U);
  status |= VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0000U);
  return status;
}

VL53L1X_ERROR VL53L1X_GetOffset(uint16_t dev, int16_t *offset)
{
  VL53L1X_ERROR status = 0;
  uint16_t Temp = 0U;

  if (offset == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
  Temp = (uint16_t)((Temp << 3U) >> 5U);
  *offset = (int16_t)Temp;
  if (*offset > 1024)
  {
    *offset = (int16_t)(*offset - 2048);
  }
  return status;
}

VL53L1X_ERROR VL53L1X_SetXtalk(uint16_t dev, uint16_t XtalkValue)
{
  VL53L1X_ERROR status = 0;

  status |= VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000U);
  status |= VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000U);
  status |= VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
                          (uint16_t)(((uint32_t)XtalkValue << 9U) / 1000U));
  return status;
}

VL53L1X_ERROR VL53L1X_GetXtalk(uint16_t dev, uint16_t *xtalk)
{
  VL53L1X_ERROR status = 0;
  uint16_t raw = 0U;

  if (xtalk == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &raw);
  *xtalk = (uint16_t)(((uint32_t)raw * 1000U) >> 9U);
  return status;
}

VL53L1X_ERROR VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t ThreshLow,
                                         uint16_t ThreshHigh, uint8_t Window,
                                         uint8_t IntOnNoTarget)
{
  VL53L1X_ERROR status = 0;
  uint8_t Temp = 0U;

  status |= VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
  Temp = (uint8_t)(Temp & 0x47U);
  if (IntOnNoTarget == 0U)
  {
    status |= VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (uint8_t)(Temp | (Window & 0x07U)));
  }
  else
  {
    status |= VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (uint8_t)((Temp | (Window & 0x07U)) | 0x40U));
  }
  status |= VL53L1_WrWord(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
  status |= VL53L1_WrWord(dev, SYSTEM__THRESH_LOW, ThreshLow);
  return status;
}

VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t *window)
{
  VL53L1X_ERROR status = 0;
  uint8_t tmp = 0U;

  if (window == NULL)
  {
    return 1;
  }
  status |= VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
  *window = (uint16_t)(tmp & 0x07U);
  return status;
}

VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low)
{
  if (low == NULL)
  {
    return 1;
  }
  return VL53L1_RdWord(dev, SYSTEM__THRESH_LOW, low);
}

VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high)
{
  if (high == NULL)
  {
    return 1;
  }
  return VL53L1_RdWord(dev, SYSTEM__THRESH_HIGH, high);
}

VL53L1X_ERROR VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter)
{
  return VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
}

VL53L1X_ERROR VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter)
{
  if (ROICenter == NULL)
  {
    return 1;
  }
  return VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
}

VL53L1X_ERROR VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y)
{
  uint8_t OpticalCenter = 199U;
  VL53L1X_ERROR status = 0;

  if (X < 4U)
  {
    X = 4U;
  }
  if (Y < 4U)
  {
    Y = 4U;
  }
  if (X > 16U)
  {
    X = 16U;
  }
  if (Y > 16U)
  {
    Y = 16U;
  }
  status |= VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
  if ((X > 10U) || (Y > 10U))
  {
    OpticalCenter = 199U;
  }
  status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
  status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                          (uint8_t)(((Y - 1U) << 4U) | (X - 1U)));
  return status;
}

VL53L1X_ERROR VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y)
{
  VL53L1X_ERROR status = 0;
  uint8_t tmp = 0U;

  if ((ROI_X == NULL) || (ROI_Y == NULL))
  {
    return 1;
  }
  status = VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
  *ROI_X = (uint16_t)(((uint16_t)tmp & 0x0FU) + 1U);
  *ROI_Y = (uint16_t)((((uint16_t)tmp & 0xF0U) >> 4U) + 1U);
  return status;
}

VL53L1X_ERROR VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t Signal)
{
  return VL53L1_WrWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, (uint16_t)(Signal >> 3U));
}

VL53L1X_ERROR VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t *signal)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (signal == NULL)
  {
    return 1;
  }
  status = VL53L1_RdWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *signal = (uint16_t)(tmp << 3U);
  return status;
}

VL53L1X_ERROR VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t Sigma)
{
  if (Sigma > (0xFFFFU >> 2U))
  {
    return 1;
  }
  return VL53L1_WrWord(dev, RANGE_CONFIG__SIGMA_THRESH, (uint16_t)(Sigma << 2U));
}

VL53L1X_ERROR VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t *sigma)
{
  VL53L1X_ERROR status = 0;
  uint16_t tmp = 0U;

  if (sigma == NULL)
  {
    return 1;
  }
  status = VL53L1_RdWord(dev, RANGE_CONFIG__SIGMA_THRESH, &tmp);
  *sigma = (uint16_t)(tmp >> 2U);
  return status;
}

VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(uint16_t dev)
{
  VL53L1X_ERROR status = 0;
  uint8_t tmp = 0U;

  status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81U); /* full VHV */
  status |= VL53L1_WrByte(dev, 0x000BU, 0x92U);
  status |= VL53L1X_StartRanging(dev);

  uint16_t wait_iter = 0U;
  while ((tmp == 0U) && (status == 0) && (wait_iter < VL53L1X_INIT_DATA_READY_TIMEOUT_ITERS))
  {
    status |= VL53L1X_CheckForDataReady(dev, &tmp);
    if ((tmp == 0U) && (status == 0))
    {
      (void)VL53L1_WaitMs(dev, 2);
    }
    wait_iter++;
  }

  status |= VL53L1X_ClearInterrupt(dev);
  status |= VL53L1X_StopRanging(dev);
  status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09U); /* two bounds VHV */
  status |= VL53L1_WrByte(dev, 0x000BU, 0x00U); /* start VHV from previous temperature */
  if (tmp == 0U)
  {
    return (status != 0) ? status : -1;
  }
  return status;
}

