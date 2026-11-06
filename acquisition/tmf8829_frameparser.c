/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>
#include "tmf8829.h"
#include "tmf8829_driver.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif
#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

#define PARSER_STATE_IDLE           0
#define PARSER_STATE_HEADER         1
#define PARSER_STATE_DATA           2
#define PARSER_STATE_COMPLETE       3

void tmf8829FrameParserInit(tmf8829FrameParser_t *parser)
{
    memset(parser, 0, sizeof(tmf8829FrameParser_t));
    parser->state = PARSER_STATE_IDLE;
    parser->frame.pixelResults = parser->pixelResults;
    parser->lastFrameNumber = 0;
    parser->dualMode = 0;
    parser->dualCurrentPhase = 0;
    parser->dualHistoCount = 0;

    parser->fpsFirstSystick = 0;
    parser->fpsLastSystick = 0;
    parser->fpsFrameCount = 0;

    int maxResultSize = TMF8829_MAX_ROWS * TMF8829_MAX_COLS * 15 + TMF8829_FRAME_FOOTER_SIZE;
    parser->resultBuffer = (uint8_t *)malloc(maxResultSize);
    if (parser->resultBuffer != NULL)
    {
        parser->resultBufferAllocated = maxResultSize;
    }

#if ENABLE_HISTOGRAM
    parser->histograms = NULL;
    parser->histogramsHA = NULL;
    parser->histogramsAllocated = 0;
    parser->histogramsHAAllocated = 0;
    parser->histogramBuffer = NULL;
    parser->histogramBufferAllocated = 0;
    parser->histogramDataLen = 0;
#endif
}

void tmf8829FrameParserCleanup(tmf8829FrameParser_t *parser)
{

    if (parser->resultBuffer != NULL)
    {
        free(parser->resultBuffer);
        parser->resultBuffer = NULL;
        parser->resultBufferAllocated = 0;
    }

#if ENABLE_HISTOGRAM
    if (parser->histograms != NULL)
    {
        free(parser->histograms);
        parser->histograms = NULL;
        parser->histogramsAllocated = 0;
    }
    if (parser->histogramsHA != NULL)
    {
        free(parser->histogramsHA);
        parser->histogramsHA = NULL;
        parser->histogramsHAAllocated = 0;
    }
    if (parser->histogramBuffer != NULL)
    {
        free(parser->histogramBuffer);
        parser->histogramBuffer = NULL;
        parser->histogramBufferAllocated = 0;
    }
#endif
}

#if ENABLE_HISTOGRAM
static int tmf8829AllocateHistograms(tmf8829FrameParser_t *parser, int count)
{
    if (parser->histograms != NULL && parser->histogramsAllocated >= count)
    {
        return 0;
    }

    if (parser->histograms != NULL)
    {
        free(parser->histograms);
    }

    parser->histograms = (tmf8829Histogram_t *)malloc(
        sizeof(tmf8829Histogram_t) * count);
    if (parser->histograms == NULL)
    {
        PRINT_INFO("Error: Failed to allocate histogram storage\n");
        return -1;
    }

    memset(parser->histograms, 0, sizeof(tmf8829Histogram_t) * count);
    parser->histogramsAllocated = count;
    return 0;
}

static int tmf8829AllocateHistogramsHA(tmf8829FrameParser_t *parser, int count)
{
    if (parser->histogramsHA != NULL && parser->histogramsHAAllocated >= count)
    {
        return 0;
    }

    if (parser->histogramsHA != NULL)
    {
        free(parser->histogramsHA);
    }

    parser->histogramsHA = (tmf8829Histogram_t *)malloc(
        sizeof(tmf8829Histogram_t) * count);
    if (parser->histogramsHA == NULL)
    {
        PRINT_INFO("Failed to allocate HA histogram storage\n");
        return -1;
    }

    memset(parser->histogramsHA, 0, sizeof(tmf8829Histogram_t) * count);
    parser->histogramsHAAllocated = count;
    return 0;
}
#endif

void tmf8829GetResolution(int fpMode, int *rows, int *cols)
{
    switch (fpMode)
    {
        case TMF8829_CFG_FP_MODE_48x32:
            *rows = 32;
            *cols = 48;
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            *rows = 32;
            *cols = 32;
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            *rows = 16;
            *cols = 16;
            break;
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            *rows = 8;
            *cols = 8;
            break;
    }
}

int tmf8829GetHistogramsPerResult(int fpMode, int dualMode)
{
    int baseHistograms;

    switch (fpMode)
    {
        case TMF8829_CFG_FP_MODE_48x32:
            baseHistograms = 6;
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            baseHistograms = 4;
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            baseHistograms = 2;
            break;
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            baseHistograms = 2;
            break;
    }

    if (dualMode == 1 || dualMode == 2)
    {
        if (fpMode > TMF8829_CFG_FP_MODE_16x16)
        {

            return baseHistograms * 2;
        }
        else
        {

            return baseHistograms * 2;
        }
    }

    return baseHistograms;
}

int tmf8829GetPixelDataSize(uint8_t resultFormat)
{
    int numPeak = resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeak == 0) numPeak = 1;

    int useSignal = (resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    int useNoise = (resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    int useXtalk = (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;

    int size = (numPeak * (3 + 2 * useSignal)) + (2 * useNoise) + (2 * useXtalk);
    return size;
}

static void tmf8829CalcPixelXYZ(int col, int row, int fp_mode, float distance,
                                uint8_t fovCorrection, float focal_length,
                                float *outX, float *outY, float *outZ)
{
    float X, Y;
    float spanX, spanY;
    float xNorm, yNorm, depth, depthFactor;
    float fovCorrX, fovCorrY;
    float FOCAL_LENGTH_NOM = 560.0f;

    uint8_t fovCorrectionX = fovCorrection & 0x03;
    uint8_t fovCorrectionY = (fovCorrection >> 2) & 0x03;

    fovCorrX = ((float)fovCorrectionX - 1.5f) / 2.5f / 16.0f;
    fovCorrY = ((float)fovCorrectionY - 1.5f) / 2.5f / 16.0f;

    switch (fp_mode)
    {
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            X = 8;
            Y = 8;
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            X = 16;
            Y = 16;
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            X = 32;
            Y = 32;
            break;
        case TMF8829_CFG_FP_MODE_48x32:
            X = 48;
            Y = 32;
            break;
    }

    spanX = X * 3.0f / 4.0f;
    spanY = Y;

    xNorm = ((float)col - X / 2.0f + 0.5f) / spanX - fovCorrX;
    yNorm = ((float)row - Y / 2.0f + 0.5f) / spanY - fovCorrY;

    if (focal_length != FOCAL_LENGTH_NOM) {
        float scale = FOCAL_LENGTH_NOM / focal_length;
        xNorm *= scale;
        yNorm *= scale;
    }

    depthFactor = sqrtf(1.0f + xNorm * xNorm + yNorm * yNorm);
    depth = distance / depthFactor;

    *outX = depth * xNorm;
    *outY = depth * yNorm;
    *outZ = depth;
}

int tmf8829ParseFrameHeader(tmf8829FrameParser_t *parser, uint8_t *data)
{
    tmf8829FrameHeader_t *header;

    parser->frame.systick = data[1] | (data[2] << 8) | (data[3] << 16) | ((uint32_t)data[4] << 24);

    header = (tmf8829FrameHeader_t *)(data + TMF8829_PRE_HEADER_SIZE);

    parser->frame.frameType = header->id & TMF8829_FID_MASK;
    parser->frame.fpMode = header->id & TMF8829_FPM_MASK;
    parser->frame.layout = header->layout;
    parser->frame.frameNumber = header->fNumber;
    parser->frame.temperature = header->temperature[2];

    tmf8829GetResolution(parser->frame.fpMode, &parser->frame.numRows, &parser->frame.numCols);

#if ENABLE_HISTOGRAM

    parser->histogramDataLen = 0;
#endif
    parser->dataLen = 0;

    parser->state = PARSER_STATE_HEADER;

    return 0;
}

int tmf8829ParseFrameFooter(tmf8829FrameParser_t *parser, uint8_t *data, int totalLen)
{

    int footerOffset = totalLen - TMF8829_FRAME_FOOTER_SIZE;

    if (footerOffset < TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE)
    {
        return -1;
    }

    tmf8829FrameFooter_t *footer = (tmf8829FrameFooter_t *)(data + footerOffset);

    parser->frame.t0Integration = footer->t0Integration;
    parser->frame.t1Integration = footer->t1Integration;
    parser->frame.frameStatus = footer->frameStatus;

    parser->frame.warnings = footer->frameStatus & ~TMF8829_FRAME_VALID;

    parser->state = PARSER_STATE_COMPLETE;

    tmf8829PrintFrameInfo(parser);

    return 0;
}

int tmf8829ParseResultData(tmf8829FrameParser_t *parser, uint8_t *data, uint8_t resultFormat)
{
    int pixelDataSize;
    int numPeaks;
    int useSignal, useNoise, useXtalk;
    int row, col, i;
    int offset;
    uint8_t *pixelPtr;
    tmf8829PixelResult_t *pixel;
    float distance;
    int subFrameIdx = 0;
    int actualRows;
    int targetRow;
    uint16_t maxXtalk = 0;

    numPeaks = resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeaks == 0) numPeaks = 1;

    useSignal = (resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    useNoise = (resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    useXtalk = (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;

    pixelDataSize = tmf8829GetPixelDataSize(resultFormat);

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
        actualRows = parser->frame.numRows / 2;
    }
    else
    {
        actualRows = parser->frame.numRows;
        subFrameIdx = 0;
    }

    offset = TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE;

    for (row = 0; row < actualRows; row++)
    {

        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            targetRow = row * 2 + subFrameIdx;
        }
        else
        {
            targetRow = row;
        }

        for (col = 0; col < parser->frame.numCols; col++)
        {
            pixelPtr = data + offset;
            pixel = &parser->pixelResults[targetRow * parser->frame.numCols + col];

            pixel->noise = 0;
            pixel->xtalk = 0;
            pixel->numPeaks = numPeaks;

            int byteOffset = 0;

            if (useNoise)
            {
                pixel->noise = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                byteOffset += 2;
            }

            if (useXtalk)
            {
                pixel->xtalk = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                byteOffset += 2;
            }

            for (i = 0; i < numPeaks; i++)
            {

                uint16_t distQuarter = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                distance = distQuarter / 4.0f;
                pixel->peaks[i].distance = (int)distance;
                pixel->peaks[i].snr = pixelPtr[byteOffset + 2];
                byteOffset += 3;

                if (useSignal)
                {
                    pixel->peaks[i].signal = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                    byteOffset += 2;
                }
                else
                {
                    pixel->peaks[i].signal = 0;
                }

                tmf8829CalcPixelXYZ(col, targetRow, parser->frame.fpMode, distance,
                                    parser->fovCorrection, 560,
                                    &pixel->peaks[i].x, &pixel->peaks[i].y, &pixel->peaks[i].z);
            }

            offset += pixelDataSize;

            if (useXtalk && pixel->xtalk > maxXtalk)
            {
                maxXtalk = pixel->xtalk;
            }
        }
    }

    parser->maxXtalk = maxXtalk;

    return 0;
}

void handleReceivedFrameHeaderData(void *dptr, uint8_t *data)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;

    tmf8829ParseFrameHeader(parser, data);

    parser->dataLen = 0;

    if (parser->resultBuffer != NULL)
    {
        memcpy(parser->resultBuffer, data, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE);
        parser->dataLen = TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE;
    }
}

void handleReceivedResultData(void *dptr, uint8_t *data, uint16_t size)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;

    if (parser->resultBuffer != NULL &&
        parser->dataLen + size <= parser->resultBufferAllocated)
    {
        memcpy(parser->resultBuffer + parser->dataLen, data, size);
        parser->dataLen += size;
    }
    else
    {
        PRINT_INFO("Warning: Result buffer overflow or not allocated, truncating\n");
    }

    parser->state = PARSER_STATE_DATA;
}

#if ENABLE_HISTOGRAM
int tmf8829ParseHistogramData(tmf8829FrameParser_t *parser, uint8_t *data, int subFrame)
{
    int row, col, bin;
    int offset;
    int numRows, numCols;
    int binSize;
    int numHistograms;
    tmf8829Histogram_t *histogram;
    int fovRows, fovCols;
    int pixelRowsPerMp, pixelColsPerMp;
    int leftFovOffset, pixelRowOffset, pixelColumnOffset;
    int targetRow, targetCol;

    tmf8829GetResolution(parser->frame.fpMode, &numRows, &numCols);

    numHistograms = numRows * numCols;
    if (parser->isHAHistogram)
    {
        if (tmf8829AllocateHistogramsHA(parser, numHistograms) != 0)
        {
            return -1;
        }
    }
    else
    {
        if (tmf8829AllocateHistograms(parser, numHistograms) != 0)
        {
            return -1;
        }
    }

    if (numRows == 8 && numCols == 8)
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    }
    else
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;
    }

    offset = 0;

    offset += 4 * 192;

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

    for (row = 0; row < fovRows; row++)
    {
        for (col = 0; col < frameCols; col++)
        {

            targetRow = row * pixelRowsPerMp + pixelRowOffset;
            targetCol = col * pixelColsPerMp + leftFovOffset + pixelColumnOffset;

            if (targetRow >= numRows || targetCol >= numCols)
            {
                PRINT_INFO("Warning: histogram index out of bounds (%d,%d)\n", targetRow, targetCol);

                offset += binSize * 3;
                continue;
            }

            if (parser->isHAHistogram && parser->dualMode != 0)
            {
                histogram = &parser->histogramsHA[targetRow * numCols + targetCol];
            }
            else
            {
                histogram = &parser->histograms[targetRow * numCols + targetCol];
            }

            for (bin = 0; bin < binSize; bin++)
            {
                histogram->bin[bin] = data[offset] |
                                      (data[offset + 1] << 8) |
                                      ((uint32_t)data[offset + 2] << 16);
                offset += 3;
            }
        }
    }

    parser->frame.hasHistogram = 1;
    parser->frame.histogramSubFrame = subFrame;

    if (parser->isHAHistogram && parser->dualMode != 0)
    {
        parser->frame.histograms = parser->histogramsHA;
    }
    else
    {
        parser->frame.histograms = parser->histograms;
    }

    return 0;
}
#endif

#if ENABLE_HISTOGRAM
void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;

        if (parser->histogramBuffer == NULL)
        {
            parser->histogramBuffer = (uint8_t *)malloc(TMF8829_HISTOGRAM_BUFFER_SIZE);
            if (parser->histogramBuffer == NULL)
            {
                PRINT_INFO("Failed to allocate histogram buffer\n");
                return;
            }
            parser->histogramBufferAllocated = TMF8829_HISTOGRAM_BUFFER_SIZE;
        }

    if (parser->histogramDataLen + size <= parser->histogramBufferAllocated)
    {
        memcpy(parser->histogramBuffer + parser->histogramDataLen, data, size);
        parser->histogramDataLen += size;
    }
    else
    {
        PRINT_INFO("Warning: Histogram buffer overflow, truncating\n");
    }

    parser->dataLen += size;
    parser->state = PARSER_STATE_DATA;
}
#else
void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size)
{

    (void)dptr;
    (void)data;
    (void)size;
}
#endif

void handleReceivedResultDataEnd(void *dptr)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    tmf8829Driver *driver = &chip->tof_core;
    uint8_t *data = parser->resultBuffer;
    uint8_t resultFormat = driver->config[TMF8829_CFG_RESULT_FORMAT - TMF8829_CFG_PERIOD_MS_LSB];
    uint8_t dumpHistograms = driver->config[TMF8829_CFG_DUMP_HISTOGRAMS - TMF8829_CFG_PERIOD_MS_LSB];
    int subFrameIdx = 0;
    int expectedFrameDiff = 0;

    tmf8829ParseFrameFooter(parser, data, parser->dataLen);

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;

        if (subFrameIdx == 0)
        {
            memset(parser->pixelResults, 0, sizeof(parser->pixelResults));
        }
    }

    tmf8829ParseResultData(parser, data, resultFormat);

    parser->xtalkPercentage = 0.0f;
    if (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK)
    {
        int iteration = driver->config[TMF8829_CFG_KILO_ITERATIONS_LSB - TMF8829_CFG_PERIOD_MS_LSB] |
                        driver->config[TMF8829_CFG_KILO_ITERATIONS_MSB - TMF8829_CFG_PERIOD_MS_LSB] << 8;
        int iterationsDivider = 2;

        if (parser->frame.fpMode >= TMF8829_CFG_FP_MODE_32x32)
        {
            iterationsDivider = 8;
        }

        if (parser->frame.fpMode >= TMF8829_CFG_FP_MODE_48x32)
        {
            iterationsDivider = 12;
        }

        if (iterationsDivider > 0)
        {
            int xtalkIteration = (iteration / iterationsDivider) * 1024;
            if (xtalkIteration > 0)
            {
                parser->xtalkPercentage = 100.0f * parser->maxXtalk / xtalkIteration;
            }
        }
    }

    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;

        if (dumpHistograms)
        {

            int histogramsPerSubframe = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            expectedFrameDiff = histogramsPerSubframe + 1;
        }
        else
        {

            expectedFrameDiff = 1;
        }

        int frameDiff = parser->frame.frameNumber - parser->lastFrameNumber;

        int isFirstFrame = (parser->lastFrameNumber == 0);

        if (subFrameIdx == 0)
        {

            parser->resultSubFrameReceived = (1 << subFrameIdx);

            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {

                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (fpMode=%d, expected frameDiff=%d, actual=%d)\n",
                       lostFrames, parser->frame.fpMode, expectedFrameDiff, frameDiff);
            }
        }
        else
        {
            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {

                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (expected frameDiff=%d, actual=%d)\n",
                       lostFrames, expectedFrameDiff, frameDiff);
            }
            else
            {
                parser->resultSubFrameReceived |= (1 << subFrameIdx);

                if (parser->resultSubFrameReceived == 0x3)
                {

                    parser->resultReady = 1;

                    if (parser->fpsFirstSystick == 0)
                    {

                        parser->fpsFirstSystick = parser->frame.systick;
                    }
                    parser->fpsLastSystick = parser->frame.systick;
                    parser->fpsFrameCount++;
                }
            }
        }

        parser->lastFrameNumber = parser->frame.frameNumber;
    }
    else
    {

        parser->resultReady = 1;

        if (parser->fpsFirstSystick == 0)
        {

            parser->fpsFirstSystick = parser->frame.systick;
        }
        parser->fpsLastSystick = parser->frame.systick;
        parser->fpsFrameCount++;

        if (dumpHistograms)
        {

            int frameDiff = parser->frame.frameNumber - parser->lastFrameNumber;
            int isFirstFrame = (parser->lastFrameNumber == 0);
            int histogramsPerResult = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            int expectedFrameDiff = histogramsPerResult + 1;

            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {

                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (expected frameDiff=%d, actual=%d)\n",
                       lostFrames, expectedFrameDiff, frameDiff);
            }
        }
    }

#ifdef ENABLE_JSON_LOGGING

#if ENABLE_HISTOGRAM

    int shouldWrite = 0;
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {

        int histogram_dump_enabled = chip->tof_core.config[TMF8829_CFG_DUMP_HISTOGRAMS - TMF8829_CFG_PERIOD_MS_LSB];

        if (histogram_dump_enabled)
        {

            shouldWrite = (chip->jsonLogger.is_open && parser->resultReady && chip->jsonLogger.histogram_ready);
        }
        else
        {

            shouldWrite = (chip->jsonLogger.is_open && parser->resultReady);
        }
    }
    else
    {

        shouldWrite = (chip->jsonLogger.is_open && parser->resultReady);
    }

    if (shouldWrite)
#else
    if (chip->jsonLogger.is_open && parser->resultReady)
#endif
    {
        tmf8829_json_write_frame(&chip->jsonLogger, parser, resultFormat);

#if ENABLE_HISTOGRAM

        chip->jsonLogger.histogram_subframes_received = 0;
        chip->jsonLogger.histogram_subframes_received_ha = 0;
        chip->jsonLogger.histogram_ready = 0;
        chip->jsonLogger.histogram_ha_ready = 0;
        chip->jsonLogger.histogram_subframe0_ready = 0;
        chip->jsonLogger.histogram_subframe1_ready = 0;
        chip->jsonLogger.histogram_subframe0_ready_ha = 0;
        chip->jsonLogger.histogram_subframe1_ready_ha = 0;
#endif
    }
#endif

#ifdef ENABLE_KEYSTONE

    if (parser->keystoneEnabled && parser->resultReady) {

        tmf8829_chip *chip = (tmf8829_chip *)((char*)parser - offsetof(tmf8829_chip, frameParser));

        float xData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        float yData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        float zData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        int row, col;

        for (row = 0; row < parser->frame.numRows; row++) {
            for (col = 0; col < parser->frame.numCols; col++) {
                int index = row * parser->frame.numCols + col;
                tmf8829Peak_t *peak = &parser->pixelResults[index].peaks[0];
                xData[index] = peak->x;
                yData[index] = peak->y;
                zData[index] = peak->z;
            }
        }

        keystoneProcessFrame(&chip->keystoneCtx,
                             parser->frame.numRows,
                             parser->frame.numCols,
                             xData, yData, zData, NULL);

        keystoneGetAngles(&chip->keystoneCtx,
                         &parser->keystoneAngleX,
                         &parser->keystoneAngleY,
                         &parser->keystoneAngleZ);
    }
#endif

    tmf8829PrintFrameResults(parser);

    int wasResultReady = parser->resultReady;
    parser->resultReady = 0;

    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16 || wasResultReady)
    {
        parser->resultSubFrameReceived = 0;
    }

    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16)
    {
        parser->dualHistoCount = 0;
    }

}

#if ENABLE_HISTOGRAM
void handleReceivedHistogramDataEnd(void *dptr)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    uint8_t *data = parser->histogramBuffer;
    int subFrame;

    subFrame = parser->frame.layout & 0x0F;

    if (parser->dualMode != 0)
    {

        if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16)
        {

            int totalHistosPerResult = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            int histosPerMode = totalHistosPerResult / 2;

            if (parser->dualHistoCount < histosPerMode)
            {
                parser->isHAHistogram = 1;
            }
            else
            {
                parser->isHAHistogram = 0;
            }

            parser->dualHistoCount++;
        }
        else
        {
            int histosPerCycle = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);

            parser->isHAHistogram = (parser->dualHistoCount < histosPerCycle / 2);

            parser->dualHistoCount++;

            if (parser->dualHistoCount >= histosPerCycle)
            {
                parser->dualHistoCount = 0;
            }
        }
    }
    else
    {
        parser->isHAHistogram = 0;
    }

    tmf8829ParseFrameFooter(parser, data, parser->histogramDataLen);

    tmf8829ParseHistogramData(parser, data, subFrame);

#ifdef ENABLE_JSON_LOGGING

    if (chip->jsonLogger.is_open)
    {
        tmf8829_json_cache_histogram(&chip->jsonLogger, parser, subFrame);
    }
#endif

    parser->histogramDataLen = 0;
}
#else
void handleReceivedHistogramDataEnd(void *dptr)
{

    (void)dptr;
}
#endif

void tmf8829PrintFrameInfo(tmf8829FrameParser_t *parser)
{
    if (parser->frame.warnings)
    {
        PRINT_INFO("Frame #%d has warnings: 0x%02x\n", parser->frame.frameNumber, parser->frame.warnings);
    }

    if (parser->frame.frameType == TMF8829_FID_RESULTS)
    {
        int fullRows, fullCols;
        tmf8829GetResolution(parser->frame.fpMode, &fullRows, &fullCols);

        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            int subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
            PRINT_INFO("Result Frame #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d)\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrameIdx, fullRows / parser->frame.numRows);
        }
        else
        {
             PRINT_INFO("Result Frame #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                    parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick);
        }
    }

    if (parser->frame.frameType == TMF8829_FID_HISTOGRAMS)
    {
        int subFrame = parser->frame.layout & 0x0F;
        int totalSubFrames = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);

        if (parser->dualMode != 0)
        {
            const char *haType = parser->isHAHistogram ? "HA" : "Default";
            PRINT_INFO("Histogram #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d), %s\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrame, totalSubFrames, haType);
        }
        else
        {
            PRINT_INFO("Histogram #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d)\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrame, totalSubFrames);
        }
    }
}

void tmf8829PrintFrameResults(tmf8829FrameParser_t *parser)
{
    int row, col;
    tmf8829PixelResult_t *pixel;

    PRINT_DEBUG("Frame #%d (res=%dx%d):\n",
           parser->frame.frameNumber,
           parser->frame.numRows, parser->frame.numCols);

    for (row = 0; row < parser->frame.numRows; row++)
    {
        for (col = 0; col < parser->frame.numCols; col++)
        {
            pixel = &parser->pixelResults[row * parser->frame.numCols + col];
            PRINT_DEBUG("%4d ", pixel->peaks[0].distance);
        }
        PRINT_DEBUG("\n");
    }

    if (parser->xtalkPercentage > 0)
    {
        PRINT_DEBUG("maxXtalk:%d, %.3f%%\n", parser->maxXtalk, parser->xtalkPercentage);
    }

#ifdef ENABLE_KEYSTONE
    if (parser->keystoneEnabled && parser->resultReady)
    {

        time_t now;
        struct tm *tm_info;
        char timestamp[32];

        time(&now);
        tm_info = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

        PRINT_INFO("[%s][F=#%d][sys=%u] Keystone angles: X=%.2f deg, Y=%.2f deg, Z=%.2f deg\n",
               timestamp,
               parser->frame.frameNumber,
               parser->frame.systick,
               parser->keystoneAngleX,
               parser->keystoneAngleY,
               parser->keystoneAngleZ);
    }
#endif
}

void tmf8829PrintFpsStats(tmf8829FrameParser_t *parser)
{
    if (parser->fpsFrameCount == 0)
    {
        PRINT_INFO("FPS Statistics: No complete frames received\n");
        return;
    }

    uint32_t timeDiff = parser->fpsLastSystick - parser->fpsFirstSystick;

    if (timeDiff == 0)
    {
        PRINT_INFO("FPS Statistics: %d frames in <8us (too fast to measure)\n",
               parser->fpsFrameCount);
        return;
    }

    double timeInSeconds = (double)timeDiff / 125000.0;

    double fps = (double)parser->fpsFrameCount / timeInSeconds;

    PRINT_INFO("FPS Statistics: %d frames in %.3f seconds = %.2f FPS\n",
           parser->fpsFrameCount, timeInSeconds, fps);
}
