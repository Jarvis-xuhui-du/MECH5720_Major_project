/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_H
#define TMF8829_H

#include "tmf8829_shim.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TMF8829_DRIVER_MAJOR_VERSION  1
#define TMF8829_DRIVER_MINOR_VERSION  4

#define TMF8829_SLAVE_ADDR         0x41

#define SPI_WR_CMD                 0x02
#define SPI_RD_CMD                 0x03

#define  TMF8829_FIFO_SIZE         (8192)

#define CAP_DISCHARGE_TIME_MS      3
#define ENABLE_TIME_MS             3
#define CPU_READY_TIME_MS          3

#define BL_SUCCESS_OK              0
#define BL_ERROR_CMD              -1
#define BL_ERROR_TIMEOUT          -2

#define APP_SUCCESS_OK            (BL_SUCCESS_OK)
#define APP_ERROR_CMD             (BL_ERROR_CMD)
#define APP_ERROR_TIMEOUT         (BL_ERROR_TIMEOUT)
#define APP_ERROR_PARAM           -3
#define APP_ERROR_NO_RESULT       -4

#define TMF8829_APP_INT_RESULTS                 0x01
#define TMF8829_APP_INT_MOTION                  0x02
#define TMF8829_APP_INT_PROXIMITY               0x04
#define TMF8829_APP_INT_HISTOGRAMS              0x08

#define TMF8829_APP_INT_ALL       ( TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_MOTION  | TMF8829_APP_INT_PROXIMITY | TMF8829_APP_INT_HISTOGRAMS )

#define TMF8829_APP_ID                          0x01
#define TMF8829_MAJOR                           0x02
#define TMF8829_MINOR                           0x04
#define TMF8829_MEASURE_STATUS                  0x05
#define TMF8829_SERIAL_NUMBER_0                 0x1c
#define TMF8829_CID_RID                         0x20

#define TMF8829_CFG_PERIOD_MS_LSB               0x22
#define TMF8829_CFG_PERIOD_MS_MSB               0x23
#define TMF8829_CFG_KILO_ITERATIONS_LSB         0x24
#define TMF8829_CFG_KILO_ITERATIONS_MSB         0x25
#define TMF8829_CFG_FP_MODE                     0x26
#define TMF8829_CFG_SPAD_DEADTIME               0x29
#define TMF8829_CFG_RESULT_FORMAT               0x2A
#define TMF8829_CFG_DUMP_HISTOGRAMS             0x2B
#define TMF8829_CFG_VCSEL_ON                    0x30
#define TMF8829_CFG_VCDRV_2                     0x33
#define TMF8829_CFG_VCDRV_3                     0x34
#define TMF8829_CFG_ALG_DISTANCE                0x52
#define TMF8829_CFG_INT_THRESHOLD_LOW_LSB       0x68
#define TMF8829_CFG_INT_THRESHOLD_LOW_MSB       0x69
#define TMF8829_CFG_INT_THRESHOLD_HIGH_LSB      0x6A
#define TMF8829_CFG_INT_THRESHOLD_HIGH_MSB      0x6B
#define TMF8829_CFG_INT_PERSISTANCE             0x6C
#define TMF8829_CFG_LAST_AVAILABLE              0xDF
#define TMF8829_CFG_PAGE_SIZE                   (TMF8829_CFG_LAST_AVAILABLE - TMF8829_CFG_PERIOD_MS_LSB + 1)

#define TMF8829_FID_RESULTS         0x10
#define TMF8829_FID_HISTOGRAMS      0x20

#define TMF8829_FID_MASK            0xF0
#define TMF8829_FPM_MASK            0x0F

#define TMF8829_CFG_FP_MODE_8x8A    0
#define TMF8829_CFG_FP_MODE_8x8B    1
#define TMF8829_CFG_FP_MODE_16x16   2
#define TMF8829_CFG_FP_MODE_32x32   3
#define TMF8829_CFG_FP_MODE_32x32s  4
#define TMF8829_CFG_FP_MODE_48x32   5

#define TMF8829_PRE_HEADER_SIZE      5
#define TMF8829_FRAME_HEADER_SIZE   16
#define TMF8829_FRAME_FOOTER_SIZE   12
#define TMF8829_FRAME_EOF_SIZE       2

#define TMF8829_FRAME_HEADER_OFFSET  4
#define TMF8829_FRAME_EOF            0xE0F7

#define TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK         0x07
#define TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK  0x08
#define TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK   0x10
#define TMF8829_CFG_RESULT_FORMAT_XTALK_MASK            0x20

#define TMF8829_RESULT_FRAME_SUBIDX_MASK                0x40

#define TMF8829_COM_FIFOSTATUS             0xFA
#define TMF8829_COM_SYS_TICK_0             0xFB

#define RESULT_REG( regName )             ( (TMF8829_COM_##regName) - (TMF8829_COM_FIFOSTATUS) )

#define CLK_CORRECTION_PAIRS               4

#define TMF8829_COM_REG_APP_ID              0x00
#define TMF8829_COM_REG_CMD_STAT            0x08

#define TMF8829_COM_APP_ID__application     0x01
#define TMF8829_COM_APP_ID__bootloader      0x80

#define TMF8829_CMD_STAT__cmd_stat__CMD_MEASURE                       0x10
#define TMF8829_CMD_STAT__cmd_stat__CMD_CLEAR_STATUS                  0x11
#define TMF8829_CMD_STAT__cmd_stat__CMD_WRITE_PAGE                    0x15
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CONFIG_PAGE              0x16
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8                  0x40
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_LONG_RANGE       0x41
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_HIGH_ACCURACY    0x42
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16                0x43
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16_HIGH_ACCURACY  0x44
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32                0x45
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32_HIGH_ACCURACY  0x46
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32                0x47
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY  0x48
#define TMF8829_CMD_STAT__cmd_stat__CMD_STOP                          0xff

#define TMF8829_CMD_STAT__cmd_stat__STAT_OK               0x00
#define TMF8829_CMD_STAT__cmd_stat__STAT_ACCEPTED         0x01

#define TMF8829_COM_RESULT__measurement_res_frame  0xAA
#define TMF8829_COM_RESULT__measurement_hist_frame 0xBB
#define TMF8829_COM_RESULT__measurement_header     0xFD
#define TMF8829_COM_RESULT__error_frame            0xFE
#define TMF8829_COM_RESULT__no_frame               0xFF

#define TMF8829_COM_ERROR_eof                      0x01
#define TMF8829_COM_ERROR_unkown                   0x00

#define TMF8829_LOG_LEVEL_NONE              0x00
#define TMF8829_LOG_LEVEL_ERROR             0x01
#define TMF8829_LOG_LEVEL_RESULTS_HEADER    0x02
#define TMF8829_LOG_LEVEL_RESULTS           0x04
#define TMF8829_LOG_LEVEL_CLK_CORRECTION    0x08
#define TMF8829_LOG_LEVEL_INFO              0x10
#define TMF8829_LOG_LEVEL_VERBOSE           0x20
#define TMF8829_LOG_LEVEL_I2C               0x80
#define TMF8829_LOG_LEVEL_DEBUG             0xFF

typedef struct _tmf8829DriverInfo
{
  uint8_t version[2];
} tmf8829DriverInfo;

typedef struct _tmf8829DeviceInfo
{
  uint32_t deviceSerialNumber;
  uint8_t appVersion[4];
  uint8_t chipVersion[2];
} tmf8829DeviceInfo;

typedef struct _tmf8829Driver
{
  tmf8829DeviceInfo device;
  tmf8829DriverInfo info;
  uint32_t hostTicks[ CLK_CORRECTION_PAIRS ];
  uint32_t tmf8829Ticks[ CLK_CORRECTION_PAIRS ];
  uint16_t clkCorrRatioUQ;
  uint8_t clkCorrectionIdx;
  uint8_t i2cSlaveAddress;
  uint8_t clkCorrectionEnable;
  uint8_t cyclicRunning;
  uint8_t logLevel;
  uint8_t config[TMF8829_CFG_PAGE_SIZE];
  uint8_t dataBuffer[ DATA_BUFFER_SIZE ];
} tmf8829Driver;

void tmf8829Initialise( tmf8829Driver * driver );

void tmf8829SetLogLevel( tmf8829Driver * driver, uint8_t level );

void tmf8829ClkCorrection( tmf8829Driver * driver, uint8_t enable );

void tmf8829Reset( tmf8829Driver * driver );

void tmf8829Enable( tmf8829Driver * driver );

void tmf8829Disable( tmf8829Driver * driver );

void tmf8829Standby( tmf8829Driver * driver );

void tmf8829PowerUp( tmf8829Driver * driver );

int tmf8829isDeviceWakeup( tmf8829Driver * driver );

void tmf8829Wakeup( tmf8829Driver * driver );

int8_t tmf8829IsCpuReady( tmf8829Driver * driver, uint8_t waitInMs );

int8_t tmf8829BootloaderCmdSpiOff( tmf8829Driver * driver );

int8_t tmf8829BootloaderCmdI2cOff( tmf8829Driver * driver );

int8_t tmf8829BootloaderSetRamAddr ( tmf8829Driver * driver, uint32_t addr );

int8_t tmf8829BootloaderWriteRamBoth ( tmf8829Driver * driver, uint8_t len );

uint32_t tmf8829GetUint32( const uint8_t * data );

uint16_t tmf8829GetUint16( const uint8_t * data );

void tmf8829SetUint16( uint16_t value, uint8_t * data );

int8_t tmf8829BootloaderStartRamApp ( tmf8829Driver * driver );

int8_t tmf8829BootloaderStartRamAppAndPowerOn ( tmf8829Driver * driver );

int8_t tmf8829DownloadFirmware( tmf8829Driver * driver, uint32_t imageStartAddress, const uint8_t * image, int32_t imageSizeInBytes, uint8_t useFifo );

int8_t tmf8829ReadDeviceInfo( tmf8829Driver * driver );

int8_t tmf8829Command( tmf8829Driver * driver, uint8_t cmd);

int8_t tmf8829CmdLoadConfigPage( tmf8829Driver * driver );

int8_t tmf8829CmdWritePage( tmf8829Driver * driver );

int8_t tmf8829GetConfiguration(tmf8829Driver * driver);

int8_t tmf8829SetConfiguration(tmf8829Driver * driver);

int8_t tmf8829StartMeasurement( tmf8829Driver * driver );

int8_t tmf8829StopMeasurement( tmf8829Driver * driver );

uint8_t tmf8829GetAndClrInterrupts( tmf8829Driver * driver, uint8_t mask );

void tmf8829ClrAndEnableInterrupts( tmf8829Driver * driver, uint8_t mask );

void tmf8829DisableInterrupts( tmf8829Driver * driver, uint8_t mask );

int8_t tmf8829ReadResults( tmf8829Driver * driver );

int8_t tmf8829ReadHistogram( tmf8829Driver * driver );

void tmf8829CorrectDistanceDataSegment( tmf8829Driver * driver, uint16_t size, uint8_t layout );

uint16_t tmf8829CorrectDistance( tmf8829Driver * driver, uint16_t distance);

uint8_t tmf8829GetPixelSize(uint8_t layout);

#ifdef __cplusplus
}
#endif

#endif
