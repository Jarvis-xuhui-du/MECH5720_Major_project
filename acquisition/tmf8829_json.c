/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include "tmf8829_json.h"
#include "tmf8829.h"
#include "tmf8829_shim.h"

static int tmf8829_json_write_frame_internal(tmf8829_json_logger_t *logger,
                                               json_frame_data_t *frame_data);

static void* json_writer_thread(void *arg)
{
    tmf8829_json_logger_t *logger = (tmf8829_json_logger_t *)arg;
    json_frame_data_t frame_data;
    json_frame_data_t *queue_slot;
    int frame_num = 0;
    int have_frame = 0;
    int queue_tail_idx;

    PRINT_INFO("JSON writer thread started\n");

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (1)
    {

        pthread_mutex_lock(&logger->queue_mutex);

        while (logger->queue_count == 0 && !logger->stop_requested)
        {
            pthread_cond_wait(&logger->queue_cond, &logger->queue_mutex);
        }

        if (logger->stop_requested && logger->queue_count == 0)
        {
            pthread_mutex_unlock(&logger->queue_mutex);

            break;
        }

        have_frame = 0;
        if (logger->queue_count > 0)
        {

            queue_tail_idx = logger->queue_tail;

            queue_slot = &logger->frame_queue[queue_tail_idx];
            frame_data = *queue_slot;
#if ENABLE_HISTOGRAM
            PRINT_DEBUG("Dequeued frame %d, pixelResults=%p, histograms=%p, histogramsHA=%p\n",
                       frame_data.frame.frameNumber, (void*)frame_data.pixelResults,
                       (void*)frame_data.histograms, (void*)frame_data.histogramsHA);
#else
            PRINT_DEBUG("Dequeued frame %d, pixelResults=%p\n",
                       frame_data.frame.frameNumber, (void*)frame_data.pixelResults);
#endif
            logger->queue_tail = (logger->queue_tail + 1) % JSON_QUEUE_SIZE;
            logger->queue_count--;
            have_frame = 1;

            pthread_cond_signal(&logger->queue_full_cond);
        }

        pthread_mutex_unlock(&logger->queue_mutex);

        if (have_frame)
        {
            frame_num++;

            if (frame_data.pixelResults == NULL)
            {
                PRINT_DEBUG("ERROR: frame_data.pixelResults is NULL for frame %d\n", frame_data.frame.frameNumber);
                continue;
            }

            uint32_t start = getSysTick();
            int kbWritten = tmf8829_json_write_frame_internal(logger, &frame_data);
            uint32_t end = getSysTick();

            if (frame_data.pixelResults != NULL)
            {
                free(frame_data.pixelResults);
            }
#if ENABLE_HISTOGRAM
            if (frame_data.histograms != NULL)
            {
                free(frame_data.histograms);
            }
            if (frame_data.histogramsHA != NULL)
            {
                free(frame_data.histogramsHA);
            }
#endif

            pthread_mutex_lock(&logger->queue_mutex);
            queue_slot->pixelResults = NULL;
#if ENABLE_HISTOGRAM
            queue_slot->histograms = NULL;
            queue_slot->histogramsHA = NULL;
#endif
            pthread_mutex_unlock(&logger->queue_mutex);

            {
                uint32_t writeTimeMs = (end - start) / 1000.0;
                uint32_t bps = writeTimeMs > 0 ? kbWritten / writeTimeMs : 0.0;
                PRINT_INFO("Frame #%d saved, %dkB/%dms=%dkB/ms, queue size %d\n",
                       frame_data.frame.frameNumber, kbWritten, writeTimeMs, bps, logger->queue_count);
            }
        }
        else if (logger->stop_requested)
        {

            break;
        }
    }

    return NULL;
}

static int tmf8829_json_write_frame_internal(tmf8829_json_logger_t *logger,
                                               json_frame_data_t *frame_data)
{
    int row, col, i;
    tmf8829PixelResult_t *pixel;
    tmf8829FrameData_t *frame;
    int numPeaks;
    int useNoise, useXtalk, useSignal;
#if ENABLE_HISTOGRAM
    int bin;
    tmf8829Histogram_t *histogram = NULL;
    int binSize;
#endif
    z_off_t startPos, endPos;
    int kbWritten;

    if (logger == NULL || !logger->is_open || frame_data == NULL)
    {
        return 0;
    }

    startPos = gztell(logger->file);

    frame = &frame_data->frame;

    numPeaks = frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeaks == 0) numPeaks = 1;
    useNoise = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    useXtalk = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;
    useSignal = (frame_data->resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;

    gzprintf(logger->file,
             "        %s{\n"
             "            \"info\": {\n"
             "                \"frame_number\": %d,\n"
             "                \"read_time\": %u,\n"
             "                \"systick_t0\": %u,\n"
             "                \"systick_t1\": %u,\n"
             "                \"temperature\": %d,\n"
             "                \"warnings\": %d\n"
             "            },\n",
             logger->frame_count > 0 ? "," : "",
             frame->frameNumber, frame->systick, frame->t0Integration,
             frame->t1Integration, frame->temperature, frame->warnings);

#if ENABLE_HISTOGRAM

    if (frame->numRows == 8 && frame->numCols == 8)
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    }
    else
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;
    }

    if (frame_data->histogram_ready)
    {
        gzprintf(logger->file, "            \"mp_histo\": [\n");

        for (row = 0; row < frame->numRows; row++)
        {
            gzprintf(logger->file, "                [\n");
            for (col = 0; col < frame->numCols; col++)
            {
                if (frame_data->histograms != NULL)
                {
                    histogram = &frame_data->histograms[row * frame->numCols + col];
                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": [");
                    for (bin = 0; bin < binSize; bin++)
                    {
                        gzprintf(logger->file, "%u%s", histogram->bin[bin],
                                 bin < binSize - 1 ? "," : "");
                    }
                    gzprintf(logger->file,
                             "]\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
                else
                {
                    PRINT_DEBUG("ERROR: histograms is NULL at row=%d, col=%d\n", row, col);

                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": []\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
            }
            gzprintf(logger->file,
                     "                ]%s\n",
                     row < frame->numRows - 1 ? "," : "");
        }
        gzprintf(logger->file, "            ],\n");

        if (frame_data->histogram_ha_ready)
        {
            gzprintf(logger->file, "            \"mp_histo_HA\": [\n");

            for (row = 0; row < frame->numRows; row++)
            {
                gzprintf(logger->file, "                [\n");
                for (col = 0; col < frame->numCols; col++)
                {
                    histogram = &frame_data->histogramsHA[row * frame->numCols + col];
                    gzprintf(logger->file,
                             "                    {\n"
                             "                        \"bin\": [");
                    for (bin = 0; bin < binSize; bin++)
                    {
                        gzprintf(logger->file, "%u%s", histogram->bin[bin],
                                 bin < binSize - 1 ? "," : "");
                    }
                    gzprintf(logger->file,
                             "]\n"
                             "                    }%s\n",
                             col < frame->numCols - 1 ? "," : "");
                }
                gzprintf(logger->file,
                         "                ]%s\n",
                         row < frame->numRows - 1 ? "," : "");
            }
            gzprintf(logger->file, "            ],\n");
        }
        else
        {

            gzprintf(logger->file,
                     "            \"mp_histo_HA\": [\n"
                     "            ],\n");
        }
    }
    else
    {

        gzprintf(logger->file,
                 "            \"mp_histo\": [\n"
                 "            ],\n"
                 "            \"mp_histo_HA\": [\n"
                 "            ],\n");
    }
#else

    gzprintf(logger->file,
             "            \"mp_histo\": [\n"
             "            ],\n");
#endif

    gzprintf(logger->file,
             "            \"ref_histo\": [\n"
             "            ],\n"
             "            \"results\": [\n");

    for (row = 0; row < frame->numRows; row++)
    {
        gzprintf(logger->file, "                [\n");

        for (col = 0; col < frame->numCols; col++)
        {
            pixel = &frame_data->pixelResults[row * frame->numCols + col];

            gzprintf(logger->file,
                     "                    {\n"
                     "                        \"noise\": %d,\n"
                     "                        \"peaks\": [\n",
                     useNoise ? pixel->noise : 0);

            for (i = 0; i < numPeaks; i++)
            {
                gzprintf(logger->file,
                         "                            {\n"
                         "                                \"distance\": %d,\n"
                         "                                \"signal\": %d,\n"
                         "                                \"snr\": %d,\n"
                         "                                \"x\": \"%.2f\",\n"
                         "                                \"y\": \"%.2f\",\n"
                         "                                \"z\": \"%.2f\"\n"
                         "                        }%s\n",
                         pixel->peaks[i].distance,
                         useSignal ? pixel->peaks[i].signal : 0,
                         pixel->peaks[i].snr,
                         pixel->peaks[i].x, pixel->peaks[i].y, pixel->peaks[i].z,
                         i < numPeaks - 1 ? "," : "");
            }

            gzprintf(logger->file,
                     "                        ],\n"
                     "                        \"xtalk\": %d\n"
                     "                    }%s\n",
                     useXtalk ? pixel->xtalk : 0,
                     col < frame->numCols - 1 ? "," : "");
        }

        gzprintf(logger->file,
                 "                ]%s\n",
                 row < frame->numRows - 1 ? "," : "");
    }

    gzprintf(logger->file,
             "            ]\n"
             "        }\n");

    endPos = gztell(logger->file);
    kbWritten = (int)((endPos - startPos) >> 10);

    logger->frame_count++;

    return kbWritten;
}

void tmf8829_json_get_timestamp(char *buffer, int size)
{
    time_t now;
    struct tm *tm_info;

    time(&now);
    tm_info = localtime(&now);

    snprintf(buffer, size, "%04d-%02d-%02d-%02d-%02d-%02d",
             tm_info->tm_year + 1900,
             tm_info->tm_mon + 1,
             tm_info->tm_mday,
             tm_info->tm_hour,
             tm_info->tm_min,
             tm_info->tm_sec);
}

int tmf8829_json_init(tmf8829_json_logger_t *logger, const char *prefix,
                      uint32_t serial_number, const uint8_t fw_version[4])
{
    char timestamp[64];
    int ret;

    if (logger == NULL)
    {
        return -1;
    }

    memset(logger, 0, sizeof(tmf8829_json_logger_t));
    logger->serial_number = serial_number;
    memcpy(logger->fw_version, fw_version, 4);
    logger->frame_count = 0;
    logger->thread_running = 0;
    logger->stop_requested = 0;
    logger->queue_head = 0;
    logger->queue_tail = 0;
    logger->queue_count = 0;

    int i;
    for (i = 0; i < JSON_QUEUE_SIZE; i++)
    {
        logger->frame_queue[i].pixelResults = NULL;
#if ENABLE_HISTOGRAM
        logger->frame_queue[i].histograms = NULL;
        logger->frame_queue[i].histogramsHA = NULL;
        logger->frame_queue[i].histogram_ready = 0;
        logger->frame_queue[i].histogram_ha_ready = 0;
#endif
    }

    if (pthread_mutex_init(&logger->queue_mutex, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue mutex\n");
        return -1;
    }

    if (pthread_cond_init(&logger->queue_cond, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue condition variable\n");
        pthread_mutex_destroy(&logger->queue_mutex);
        return -1;
    }

    if (pthread_cond_init(&logger->queue_full_cond, NULL) != 0)
    {
        PRINT_INFO("Error: Failed to initialize queue full condition variable\n");
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        return -1;
    }

    tmf8829_json_get_timestamp(timestamp, sizeof(timestamp));

    snprintf(logger->file_path, JSON_MAX_FILE_PATH,
             "%s_UID%u-%s.json.gz",
             prefix,
             serial_number,
             timestamp);

    logger->file = gzopen(logger->file_path, "wb1");
    if (logger->file == NULL)
    {
        PRINT_INFO("Cannot create JSON file: %s\n", strerror(errno));
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        pthread_cond_destroy(&logger->queue_full_cond);
        return -1;
    }

    logger->is_open = 1;

    gzprintf(logger->file, "{\n");
    gzprintf(logger->file, "    \"Result_Set\": [\n");

    logger->thread_running = 1;
    logger->stop_requested = 0;
    ret = pthread_create(&logger->writer_thread, NULL, json_writer_thread, logger);
    if (ret != 0)
    {
        PRINT_INFO("Failed to create JSON writer thread: %d\n", ret);
        gzclose(logger->file);
        logger->is_open = 0;
        pthread_mutex_destroy(&logger->queue_mutex);
        pthread_cond_destroy(&logger->queue_cond);
        pthread_cond_destroy(&logger->queue_full_cond);
        return -1;
    }

    PRINT_INFO("JSON logger initialized: %s (async mode)\n", logger->file_path);

    return 0;
}

void tmf8829_json_close(tmf8829_json_logger_t *logger)
{
    int i;

    if (logger == NULL || !logger->is_open)
    {
        return;
    }

    gzflush(logger->file, Z_FINISH);

    gzclose(logger->file);
    logger->is_open = 0;

    for (i = 0; i < JSON_QUEUE_SIZE; i++)
    {
        if (logger->frame_queue[i].pixelResults != NULL)
        {
            free(logger->frame_queue[i].pixelResults);
            logger->frame_queue[i].pixelResults = NULL;
        }
#if ENABLE_HISTOGRAM
        if (logger->frame_queue[i].histograms != NULL)
        {
            free(logger->frame_queue[i].histograms);
            logger->frame_queue[i].histograms = NULL;
        }
        if (logger->frame_queue[i].histogramsHA != NULL)
        {
            free(logger->frame_queue[i].histogramsHA);
            logger->frame_queue[i].histogramsHA = NULL;
        }
#endif
    }

#if ENABLE_HISTOGRAM
    if (logger->histograms != NULL)
    {
        free(logger->histograms);
        logger->histograms = NULL;
    }
    if (logger->histogramsHA != NULL)
    {
        free(logger->histogramsHA);
        logger->histogramsHA = NULL;
    }
#endif

    pthread_mutex_destroy(&logger->queue_mutex);
    pthread_cond_destroy(&logger->queue_cond);
    pthread_cond_destroy(&logger->queue_full_cond);

    PRINT_INFO("JSON logger closed: %s (%d frames saved)\n",
           logger->file_path, logger->frame_count);
}

void tmf8829_json_save_config(tmf8829_json_logger_t *logger, tmf8829_cfg_t *cfg)
{
    if (logger == NULL || cfg == NULL)
    {
        return;
    }
    memcpy(&logger->cfg, cfg, sizeof(tmf8829_cfg_t));
}

void tmf8829_json_flush(tmf8829_json_logger_t *logger)
{
    if (logger == NULL || !logger->is_open)
    {
        return;
    }

    if (logger->thread_running)
    {
        PRINT_DEBUG("Waiting for JSON writer thread to finish before flush...\n");

        pthread_mutex_lock(&logger->queue_mutex);
        logger->stop_requested = 1;
        pthread_cond_signal(&logger->queue_cond);
        pthread_mutex_unlock(&logger->queue_mutex);

        usleep(500000);

        int ret = pthread_join(logger->writer_thread, NULL);
        if (ret != 0)
        {
            PRINT_INFO("Warning: pthread_join failed during flush (%d)\n", ret);
        }
        logger->thread_running = 0;

        PRINT_DEBUG("Writer thread stopped, remaining frames in queue: %d\n", logger->queue_count);
    }

    gzprintf(logger->file, "    ],\n");

    gzprintf(logger->file, "    \"configuration\": {\n");
    gzprintf(logger->file, "        \"confidence_threshold\": %d,\n", logger->cfg.conf_threshold);
    gzprintf(logger->file, "        \"dead_time\": %d,\n", logger->cfg.deadtime);
    gzprintf(logger->file, "        \"dual_mode\": %d,\n", logger->cfg.dualMode);
    gzprintf(logger->file, "        \"fp_mode\": %d,\n", logger->cfg.fpMode);
    gzprintf(logger->file, "        \"high_accuracy_iterations\": %d,\n", logger->cfg.shortIteration);
    gzprintf(logger->file, "        \"histograms\": %d,\n", logger->cfg.histogram_dump);
    gzprintf(logger->file, "        \"iterations\": %d,\n", logger->cfg.iteration);
    gzprintf(logger->file, "        \"nr_peaks\": %d,\n", logger->cfg.resultFormat & 0x07);
    gzprintf(logger->file, "        \"period\": %d,\n", logger->cfg.period);
    gzprintf(logger->file, "        \"noise_strength\": %d,\n", (logger->cfg.resultFormat & 0x10) ? 1 : 0);
    gzprintf(logger->file, "        \"signal_strength\": %d,\n", (logger->cfg.resultFormat & 0x08) ? 1 : 0);
    gzprintf(logger->file, "        \"xtalk\": %d\n", (logger->cfg.resultFormat & 0x20) ? 1 : 0);
    gzprintf(logger->file, "    },\n");

    gzprintf(logger->file, "    \"info\": [\n");
    gzprintf(logger->file, "        {\n");
    gzprintf(logger->file, "            \"fw version\": [\n");
    gzprintf(logger->file, "                %d,\n", logger->fw_version[0]);
    gzprintf(logger->file, "                %d,\n", logger->fw_version[1]);
    gzprintf(logger->file, "                %d,\n", logger->fw_version[2]);
    gzprintf(logger->file, "                %d\n", logger->fw_version[3]);
    gzprintf(logger->file, "            ],\n");
    gzprintf(logger->file, "            \"logger version\": \"%s\",\n", JSON_LOGGER_VERSION);
    gzprintf(logger->file, "            \"serial number\": %u\n", logger->serial_number);
    gzprintf(logger->file, "        }\n");
    gzprintf(logger->file, "    ]\n");
    gzprintf(logger->file, "}\n");

    gzflush(logger->file, Z_SYNC_FLUSH);
}

int tmf8829_json_queue_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat)
{
    int next_head;
    json_frame_data_t *frame_data;

    if (logger == NULL || !logger->is_open || parser == NULL)
    {
        return -1;
    }

    pthread_mutex_lock(&logger->queue_mutex);

    next_head = (logger->queue_head + 1) % JSON_QUEUE_SIZE;
    if (logger->queue_count >= JSON_QUEUE_SIZE)
    {
        PRINT_INFO("Warning: JSON queue full, dropping frame %d\n", parser->frame.frameNumber);
        pthread_mutex_unlock(&logger->queue_mutex);
        return -1;
    }

    frame_data = &logger->frame_queue[logger->queue_head];

#if ENABLE_HISTOGRAM
    PRINT_DEBUG("Queue slot %d: pixelResults=%p, histograms=%p, histogramsHA=%p (before alloc)\n",
           logger->queue_head, (void*)frame_data->pixelResults,
           (void*)frame_data->histograms, (void*)frame_data->histogramsHA);
#else
    PRINT_DEBUG("Queue slot %d: pixelResults=%p (before alloc)\n",
           logger->queue_head, (void*)frame_data->pixelResults);
#endif

    int numPixels = parser->frame.numRows * parser->frame.numCols;

    if (frame_data->pixelResults == NULL)
    {
        PRINT_DEBUG("Allocating pixelResults for frame %d at slot %d\n",
                   parser->frame.frameNumber, logger->queue_head);
        frame_data->pixelResults = (tmf8829PixelResult_t *)malloc(
            sizeof(tmf8829PixelResult_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->pixelResults == NULL)
        {
            PRINT_INFO("Error: Failed to allocate pixel results for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
        PRINT_DEBUG("Allocated pixelResults at %p for frame %d\n",
                   (void*)frame_data->pixelResults, parser->frame.frameNumber);
    }

#if ENABLE_HISTOGRAM

    if (frame_data->histograms == NULL)
    {
        frame_data->histograms = (tmf8829Histogram_t *)malloc(
            sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->histograms == NULL)
        {
            PRINT_INFO("Error: Failed to allocate histograms for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
    }

    if (frame_data->histogramsHA == NULL)
    {
        frame_data->histogramsHA = (tmf8829Histogram_t *)malloc(
            sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (frame_data->histogramsHA == NULL)
        {
            PRINT_INFO("Error: Failed to allocate HA histograms for frame %d\n",
                   parser->frame.frameNumber);
            pthread_mutex_unlock(&logger->queue_mutex);
            return -1;
        }
    }
#endif

    frame_data->frame = parser->frame;
    frame_data->resultFormat = resultFormat;

    frame_data->frame.pixelResults = frame_data->pixelResults;

    memcpy(frame_data->pixelResults, parser->pixelResults,
           sizeof(tmf8829PixelResult_t) * numPixels);

#if ENABLE_HISTOGRAM

    frame_data->histogram_ready = logger->histogram_ready;
    frame_data->histogram_ha_ready = logger->histogram_ha_ready;

    int histogram_ready_copy = logger->histogram_ready;
    if (histogram_ready_copy && logger->histograms != NULL)
    {
        memcpy(frame_data->histograms, logger->histograms,
               sizeof(tmf8829Histogram_t) * numPixels);
    }

    int histogram_ha_ready_copy = logger->histogram_ha_ready;
    if (histogram_ha_ready_copy && logger->histogramsHA != NULL)
    {
        memcpy(frame_data->histogramsHA, logger->histogramsHA,
               sizeof(tmf8829Histogram_t) * numPixels);
    }

    if (histogram_ha_ready_copy)
    {
        frame_data->frame.histograms = frame_data->histogramsHA;
    }
    else if (histogram_ready_copy)
    {
        frame_data->frame.histograms = frame_data->histograms;
    }
    else
    {

        frame_data->frame.histograms = NULL;
    }

    if (histogram_ready_copy)
    {
        logger->histogram_ready = 0;
    }
    if (histogram_ha_ready_copy)
    {
        logger->histogram_ha_ready = 0;
    }
#endif

    logger->queue_head = next_head;
    logger->queue_count++;

    pthread_cond_signal(&logger->queue_cond);

    pthread_mutex_unlock(&logger->queue_mutex);

    return 0;
}

void tmf8829_json_write_frame(tmf8829_json_logger_t *logger,
                               tmf8829FrameParser_t *parser,
                               uint8_t resultFormat)
{

    tmf8829_json_queue_frame(logger, parser, resultFormat);
}

#if ENABLE_HISTOGRAM
void tmf8829_json_cache_histogram(tmf8829_json_logger_t *logger,
                                   tmf8829FrameParser_t *parser,
                                   int subFrame)
{
    int row, col, bin;
    int numRows, numCols;
    int fovRows, fovCols;
    int pixelRowsPerMp, pixelColsPerMp;
    int leftFovOffset, pixelRowOffset, pixelColumnOffset;
    int targetRow, targetCol;
    int numFramesNeeded;
    tmf8829FrameData_t *frame;
    int binSize;
    int isHAHistogram = 0;

    if (logger == NULL || !logger->is_open || parser == NULL)
    {
        return;
    }

    PRINT_DEBUG("JSON cache_histogram: subFrame=%d, histograms=%p, histogramsHA=%p\n",
           subFrame, (void*)logger->histograms, (void*)logger->histogramsHA);

    logger->dualMode = parser->dualMode;

    if (parser->dualMode != 0)
    {
        isHAHistogram = parser->isHAHistogram;
        PRINT_DEBUG("JSON histogram: subFrame=%d, dualMode=%d, isHAHistogram=%d, dualHistoCount=%d\n",
               subFrame, parser->dualMode, isHAHistogram, parser->dualHistoCount);
    }

    frame = &parser->frame;
    numRows = frame->numRows;
    numCols = frame->numCols;

    if (numRows == 8 && numCols == 8)
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    else
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;

    if (logger->histograms == NULL)
    {
        logger->histograms = (tmf8829Histogram_t *)malloc(
                sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (logger->histograms == NULL)
        {
            PRINT_INFO("Failed to allocate histogram buffer\n");
            return;
        }
        memset(logger->histograms, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
    }

    if (logger->histogramsHA == NULL && parser->dualMode != 0)
    {
        logger->histogramsHA = (tmf8829Histogram_t *)malloc(
                sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        if (logger->histogramsHA == NULL)
        {
            PRINT_INFO("Failed to allocate HA histogram buffer\n");
            return;
        }
        memset(logger->histogramsHA, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
    }

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {

        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            numFramesNeeded = 6;
        else
            numFramesNeeded = 4;
    }
    else
    {

        numFramesNeeded = 2;
    }

    if (parser->dualMode != 0)
    {

    }

    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_8x8B)
    {
        fovRows = 8;
        fovCols = 8;
    }
    else
    {
        fovRows = 16;
        fovCols = 16;
    }

    pixelRowsPerMp = numRows / fovRows;
    pixelColsPerMp = numCols / fovCols;

    int frameCols = fovCols / 2;

    leftFovOffset = 0;
    pixelRowOffset = 0;
    pixelColumnOffset = 0;

    if (subFrame % 2 != 0)
    {
        leftFovOffset = numCols / 2;
    }

    if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
    {

        if (subFrame == 2 || subFrame == 3 || subFrame == 8 || subFrame == 9)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame == 4 || subFrame == 5 || subFrame == 10 || subFrame == 11)
        {
            pixelColumnOffset = 2;
        }
        if (subFrame > 5)
        {
            pixelRowOffset = 1;
        }
    }
    else if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {

        if (subFrame == 2 || subFrame == 3 || subFrame == 6 || subFrame == 7)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame > 3)
        {
            pixelRowOffset = 1;
        }
    }

    int startNewCycle = 0;

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {

        int subFramesPerCycle;
        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            subFramesPerCycle = 6;
        else
            subFramesPerCycle = 4;

        int isFirstSubframeGroup = (subFrame < subFramesPerCycle);

        if (isFirstSubframeGroup)
        {

            if (parser->dualMode != 0)
            {

                if (logger->histogram_ready && logger->histogram_ha_ready)
                {

                    startNewCycle = 1;
                }
                else if (isHAHistogram && (logger->histogram_subframes_received_ha & 0x01) == 0)
                {

                    startNewCycle = 1;
                }

            }
            else
            {

                if (logger->histogram_subframe0_ready)
                {

                    startNewCycle = 1;
                }
                else if ((logger->histogram_subframes_received & 0x01) == 0)
                {

                    startNewCycle = 1;
                }
            }
        }
        else
        {

            startNewCycle = 0;
        }
    }
    else
    {

        if (parser->dualMode != 0)
        {

            if (logger->histogram_ready && logger->histogram_ha_ready)
            {

                startNewCycle = 1;
            }
            else if (subFrame == 0 && isHAHistogram && (logger->histogram_subframes_received_ha & 0x01) == 0)
            {

                startNewCycle = 1;
            }
        }
        else
        {

            if (logger->histogram_ready)
            {

                startNewCycle = 1;
            }
            else if (subFrame == 0 && (logger->histogram_subframes_received & 0x01) == 0)
            {

                startNewCycle = 1;
            }
        }
    }

    if (startNewCycle)
    {
        PRINT_DEBUG("JSON histogram: starting new cycle (subFrame=%d, isHA=%d)\n", subFrame, isHAHistogram);
        PRINT_DEBUG("JSON histogram: histograms=%p, histogramsHA=%p\n",
               (void*)logger->histograms, (void*)logger->histogramsHA);
        if (logger->histograms != NULL)
        {
            PRINT_DEBUG("JSON histogram: clearing histograms buffer\n");
            memset(logger->histograms, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        }
        if (logger->histogramsHA != NULL)
        {
            PRINT_DEBUG("JSON histogram: clearing histogramsHA buffer\n");
            memset(logger->histogramsHA, 0,
                   sizeof(tmf8829Histogram_t) * TMF8829_MAX_ROWS * TMF8829_MAX_COLS);
        }
        logger->histogram_subframes_received = 0;
        logger->histogram_subframes_received_ha = 0;
        logger->histogram_ready = 0;
        logger->histogram_ha_ready = 0;
        logger->histogram_subframe0_ready = 0;
        logger->histogram_subframe1_ready = 0;
        logger->histogram_subframe0_ready_ha = 0;
        logger->histogram_subframe1_ready_ha = 0;
        logger->dualCurrentPhase = 0;

        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
        {

            if (subFrame < numFramesNeeded)
            {

                logger->histogram_expected_subframes = (1 << numFramesNeeded) - 1;
            }
            else
            {

                logger->histogram_expected_subframes = ((1 << numFramesNeeded) - 1) << numFramesNeeded;
            }
            PRINT_DEBUG("JSON histogram: set histogram_expected_subframes=0x%x for subFrame=%d\n",
                   logger->histogram_expected_subframes, subFrame);
        }
        else
        {

            logger->histogram_expected_subframes = (1 << numFramesNeeded) - 1;
        }
        PRINT_DEBUG("JSON histogram: new measurement cycle starting\n");
    }

    if ((isHAHistogram && logger->histogram_ha_ready) ||
        (!isHAHistogram && logger->histogram_ready))
    {
        PRINT_DEBUG("JSON histogram: skipping subFrame=%d (data already complete for this cycle)\n", subFrame);
        return;
    }

        PRINT_DEBUG("JSON histogram: before update, ha_mask=0x%x, def_mask=0x%x, numFramesNeeded=%d\n",
               logger->histogram_subframes_received_ha, logger->histogram_subframes_received, numFramesNeeded);

        PRINT_DEBUG("JSON histogram: subFrame=%d, leftFov=%d, rowOff=%d, colOff=%d, isHA=%d, ready=%d, ha_ready=%d\n",
               subFrame, leftFovOffset, pixelRowOffset, pixelColumnOffset, isHAHistogram,
               logger->histogram_ready, logger->histogram_ha_ready);

    int pixelsCopied = 0;
    for (row = 0; row < fovRows; row++)
    {
        for (col = 0; col < frameCols; col++)
        {

            targetRow = row * pixelRowsPerMp + pixelRowOffset;
            targetCol = col * pixelColsPerMp + leftFovOffset + pixelColumnOffset;

            if (targetRow >= numRows || targetCol >= numCols)
            {
                continue;
            }

            tmf8829Histogram_t *srcHist = parser->frame.histograms;
            if (srcHist == NULL)
            {
                continue;
            }
            srcHist = &srcHist[targetRow * numCols + targetCol];

            tmf8829Histogram_t *dstHist;
            if (isHAHistogram && parser->dualMode != 0)
            {
                if (logger->histogramsHA == NULL)
                {
                    PRINT_INFO("JSON histogram: WARNING - histogramsHA is NULL, skipping pixel\n");
                    continue;
                }
                dstHist = &logger->histogramsHA[targetRow * numCols + targetCol];
            }
            else
            {
                if (logger->histograms == NULL)
                {
                    PRINT_INFO("JSON histogram: WARNING - histograms is NULL, skipping pixel\n");
                    continue;
                }
                dstHist = &logger->histograms[targetRow * numCols + targetCol];
            }

            for (bin = 0; bin < binSize; bin++)
            {
                dstHist->bin[bin] = srcHist->bin[bin];
            }
            pixelsCopied++;
        }
    }

        PRINT_DEBUG("JSON histogram: copied %d pixels\n", pixelsCopied);

    if (isHAHistogram && parser->dualMode != 0)
    {
        logger->histogram_subframes_received_ha |= (1 << subFrame);
        PRINT_DEBUG("JSON histogram: HA subFrame=%d, mask=0x%x\n", subFrame, logger->histogram_subframes_received_ha);
    }
    else
    {
        logger->histogram_subframes_received |= (1 << subFrame);
        PRINT_DEBUG("JSON histogram: Default subFrame=%d, mask=0x%x\n", subFrame, logger->histogram_subframes_received);
    }

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
    {

        int framesPerPhase;
        if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
            framesPerPhase = 6;
        else
            framesPerPhase = 4;

        int allFramesMaskPhase = (1 << framesPerPhase) - 1;

        if (isHAHistogram &&
            (logger->histogram_subframes_received_ha & allFramesMaskPhase) == allFramesMaskPhase)
        {

            logger->dualCurrentPhase = 1;
        }
        else if (!isHAHistogram &&
                 (logger->histogram_subframes_received & allFramesMaskPhase) == allFramesMaskPhase)
        {

            logger->dualCurrentPhase = 0;
        }
    }
    else if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16 && parser->dualMode != 0)
    {

    }

    uint32_t allFramesMask;

    if (parser->dualMode != 0 && parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {

        int subFramesPerCycle = (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32) ? 6 : 4;
        uint32_t subframe0Mask = (1 << subFramesPerCycle) - 1;
        uint32_t subframe1Mask = subframe0Mask << subFramesPerCycle;

        PRINT_DEBUG("JSON histogram: checking complete (high-res dual), ha_mask=0x%x, def_mask=0x%x, sf0_mask=0x%x, sf1_mask=0x%x\n",
               logger->histogram_subframes_received_ha,
               logger->histogram_subframes_received,
               subframe0Mask, subframe1Mask);

        if ((logger->histogram_subframes_received_ha & subframe0Mask) == subframe0Mask &&
            !logger->histogram_subframe0_ready_ha)
        {
            logger->histogram_subframe0_ready_ha = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 0 HA histogram ready\n");

            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        if ((logger->histogram_subframes_received_ha & subframe1Mask) == subframe1Mask &&
            !logger->histogram_subframe1_ready_ha)
        {
            logger->histogram_subframe1_ready_ha = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 1 HA histogram ready\n");

            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        if ((logger->histogram_subframes_received & subframe0Mask) == subframe0Mask &&
            !logger->histogram_subframe0_ready)
        {
            logger->histogram_subframe0_ready = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 0 Default histogram ready\n");

            PRINT_DEBUG("JSON histogram: checking COMPLETE (sf0_ha=%d, sf1_ha=%d, sf0_def=%d, sf1_def=%d\n)",
                   logger->histogram_subframe0_ready_ha, logger->histogram_subframe1_ready_ha,
                   logger->histogram_subframe0_ready, logger->histogram_subframe1_ready);
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }

        if ((logger->histogram_subframes_received & subframe1Mask) == subframe1Mask &&
            !logger->histogram_subframe1_ready)
        {
            logger->histogram_subframe1_ready = 1;
            PRINT_DEBUG("JSON histogram: Sub-frame 1 Default histogram ready\n");

            PRINT_DEBUG("JSON histogram: checking COMPLETE (sf0_ha=%d, sf1_ha=%d, sf0_def=%d, sf1_def=%d\n)",
                   logger->histogram_subframe0_ready_ha, logger->histogram_subframe1_ready_ha,
                   logger->histogram_subframe0_ready, logger->histogram_subframe1_ready);
            if (logger->histogram_subframe0_ready_ha && logger->histogram_subframe1_ready_ha &&
                logger->histogram_subframe0_ready && logger->histogram_subframe1_ready)
            {
                logger->histogram_ready = 1;
                logger->histogram_ha_ready = 1;
                PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames HA and Default histograms ready\n");
            }
        }
    }
    else if (parser->dualMode != 0)
    {

        allFramesMask = logger->histogram_expected_subframes;

        PRINT_DEBUG("JSON histogram: checking complete (low-res dual), ha_mask=0x%x(0x%x), def_mask=0x%x(0x%x), allMask=0x%x\n",
               logger->histogram_subframes_received_ha, allFramesMask,
               logger->histogram_subframes_received, allFramesMask, allFramesMask);

        if ((logger->histogram_subframes_received_ha & allFramesMask) == allFramesMask &&
            (logger->histogram_subframes_received & allFramesMask) == allFramesMask)
        {
            logger->histogram_ready = 1;
            logger->histogram_ha_ready = 1;
            PRINT_DEBUG("JSON histogram: COMPLETE! Both HA and Default histograms ready\n");
        }
    }
    else
    {

        allFramesMask = (1 << numFramesNeeded) - 1;

        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {

            int subframe0Mask = allFramesMask;
            int subFramesPerCycle = (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32) ? 6 : 4;
            int subframe1Mask = subframe0Mask << subFramesPerCycle;

            if ((logger->histogram_subframes_received & subframe0Mask) == subframe0Mask &&
                !logger->histogram_subframe0_ready)
            {

                logger->histogram_subframe0_ready = 1;
                PRINT_DEBUG("JSON histogram: Sub-frame 0 Default histogram ready\n");

                if (logger->histogram_subframe1_ready)
                {
                    logger->histogram_ready = 1;
                    PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames Default histograms ready\n");
                }
            }

            if ((logger->histogram_subframes_received & subframe1Mask) == subframe1Mask &&
                !logger->histogram_subframe1_ready)
            {

                logger->histogram_subframe1_ready = 1;
                PRINT_DEBUG("JSON histogram: Sub-frame 1 Default histogram ready\n");

                if (logger->histogram_subframe0_ready)
                {
                    logger->histogram_ready = 1;
                    PRINT_DEBUG("JSON histogram: COMPLETE! Both sub-frames Default histograms ready\n");
                }
            }
        }
        else
        {

            if ((logger->histogram_subframes_received & allFramesMask) == allFramesMask && !logger->histogram_ready)
            {
                logger->histogram_ready = 1;
                PRINT_DEBUG("JSON histogram: complete (all %d frames received, mask=0x%x)\n",
                       numFramesNeeded, allFramesMask);
            }
        }
    }
}
#endif
