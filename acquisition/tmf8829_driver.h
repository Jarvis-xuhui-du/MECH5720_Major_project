/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_DRIVER_H
#define TMF8829_DRIVER_H

#include "tmf8829.h"
#include "tmf8829_shim.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif

#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

#define BUS_I2C               1
#define BUS_SPI               2

typedef struct _tmf8829_chip
{
    tmf8829Driver tof_core;
    tmf8829FrameParser_t frameParser;
#ifdef ENABLE_JSON_LOGGING
    tmf8829_json_logger_t jsonLogger;
    uint8_t json_enabled;
#endif
#ifdef ENABLE_KEYSTONE
    keystoneContext_t keystoneCtx;
    uint8_t keystoneEnabled;
#endif
    int gpiod_enable;
    uint8_t bustype;
} tmf8829_chip;

void calculatePreconfigurationAndDualMode(int mode, int *preConfiguration, int *dualMode);
void tmf8829_set_busType(tmf8829_chip *chip, int isUseI2C);
void tmf8829_reprogramSlaveI2C(tmf8829_chip *chip, int newAddress);
int tmf8829SettingConfiguration(tmf8829_chip *chip, tmf8829_cfg_t *cfg);
int tmf8829ConfigMode(tmf8829_chip *chip, int cmd);

int tmf8829_app_process_irq(tmf8829_chip *chip);
void tmf8829_dump_registers(tmf8829_chip *chip);

void tmf8829_cleanup(tmf8829_chip *chip);

int tmf8829_probe(tmf8829_chip *chip);

#endif
