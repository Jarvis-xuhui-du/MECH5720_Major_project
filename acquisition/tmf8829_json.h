/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_JSON_H
#define TMF8829_JSON_H

#include <stdio.h>
#include <stdint.h>
#include <zlib.h>
#include <pthread.h>
#include <semaphore.h>
#include "tmf8829_frameparser.h"

#define JSON_MAX_FILE_PATH      256
#define JSON_LOGGER_VERSION     "1"
#define JSON_MAX_HISTO_BINS     256
#define JSON_MAX_MP_COUNT       1536
#define JSON_QUEUE_SIZE         64

typedef struct json_frame_data
{
    tmf8829FrameData_t frame;
    tmf8829PixelResult_t *pixelResults;
#if ENABLE_HISTOGRAM
    int histogram_ready;
    int histogram_ha_ready;
    tmf8829Histogram_t *histograms;
    tmf8829Histogram_t *histogramsHA;
#endif
    uint8_t resultFormat;
} json_frame_data_t;

typedef struct tmf8829_json_logger
{
    char file_path[JSON_MAX_FILE_PATH];
    gzFile file;
    int is_open;
    int frame_count;
    uint32_t serial_number;
    uint8_t fw_version[4];
    tmf8829_cfg_t cfg;

#if ENABLE_HISTOGRAM

    int histogram_ready;
    int histogram_ha_ready;
    int histogram_subframes_received;
    int histogram_subframes_received_ha;
    int histogram_expected_subframes;
    tmf8829Histogram_t *histograms;
    tmf8829Histogram_t *histogramsHA;
    int dualMode;
    int dualCurrentPhase;

    int histogram_subframe0_ready;
    int histogram_subframe1_ready;
    int histogram_subframe0_ready_ha;
    int histogram_subframe1_ready_ha;
#endif

    pthread_t writer_thread;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;
    pthread_cond_t queue_full_cond;
    json_frame_data_t frame_queue[JSON_QUEUE_SIZE];
    int queue_head;
    int queue_tail;
    int queue_count;
    volatile int thread_running;
    volatile int stop_requested;
} tmf8829_json_logger_t;

int tmf8829_json_init(tmf8829_json_logger_t *logger, const char *prefix,
                      uint32_t serial_number, const uint8_t fw_version[4]);

void tmf8829_json_close(tmf8829_json_logger_t *logger);

void tmf8829_json_save_config(tmf8829_json_logger_t *logger, tmf8829_cfg_t *cfg);

void tmf8829_json_write_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat);

int tmf8829_json_queue_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat);

#if ENABLE_HISTOGRAM

void tmf8829_json_cache_histogram(tmf8829_json_logger_t *logger,
                                   tmf8829FrameParser_t *parser,
                                   int subFrame);
#endif

void tmf8829_json_flush(tmf8829_json_logger_t *logger);

void tmf8829_json_get_timestamp(char *buffer, int size);

#endif
