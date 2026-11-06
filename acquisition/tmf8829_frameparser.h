/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef FRAMEPARSER_H
#define FRAMEPARSER_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/types.h>
#include <stdint.h>

#ifdef ENABLE_HISTOGRAM

#else
  #define ENABLE_HISTOGRAM 0
#endif

#define TMF8829_MAX_PEAKS           4
#define TMF8829_MAX_ROWS            32
#define TMF8829_MAX_COLS            48
#if ENABLE_HISTOGRAM
#define TMF8829_HISTOGRAM_BIN_SIZE_8x8    256
#define TMF8829_HISTOGRAM_BIN_SIZE_OTHER  64
#endif

#define TMF8829_FRAME_VALID                     0x01
#define TMF8829_FRAME_ABORTED                   0xC0
#define TMF8829_FRAME_WARNING_HV_CP_OVERLOAD    0x08
#define TMF8829_FRAME_WARNING_VCDRV_OVERLOAD    0x10
#define TMF8829_FRAME_WARNING_VCDRV_BURST_EXCEEDED 0x20

typedef struct __attribute__ ((packed)) tmf8829FrameHeader
{
    uint8_t  id;
    uint8_t  layout;
    uint16_t payload;
    uint32_t fNumber;
    int8_t   temperature[3];
    uint8_t  bdv;
    uint16_t refPos0;
    uint16_t refPos1;
} tmf8829FrameHeader_t;

typedef struct __attribute__ ((packed)) tmf8829FrameFooter
{
    uint32_t t0Integration;
    uint32_t t1Integration;
    uint8_t  frameStatus;
    uint8_t  reserved;
    uint16_t eof;
} tmf8829FrameFooter_t;

typedef struct tmf8829Peak
{
    uint16_t distance;
    uint8_t  snr;
    uint16_t signal;
    float    x;
    float    y;
    float    z;
} tmf8829Peak_t;

typedef struct tmf8829PixelResult
{
    uint16_t noise;
    uint16_t xtalk;
    uint8_t  numPeaks;
    tmf8829Peak_t peaks[TMF8829_MAX_PEAKS];
} tmf8829PixelResult_t;

#if ENABLE_HISTOGRAM
typedef struct tmf8829Histogram
{
    uint32_t bin[TMF8829_HISTOGRAM_BIN_SIZE_8x8];
} tmf8829Histogram_t;
#endif

typedef struct tmf8829FrameData
{

    uint8_t  frameType;
    uint8_t  fpMode;
    uint8_t  layout;

    uint32_t frameNumber;
    uint32_t systick;
    int8_t   temperature;

    uint32_t t0Integration;
    uint32_t t1Integration;
    uint8_t  frameStatus;
    uint8_t  warnings;

    int      numRows;
    int      numCols;

    tmf8829PixelResult_t *pixelResults;

#if ENABLE_HISTOGRAM

    int      hasHistogram;
    int      histogramSubFrame;
    tmf8829Histogram_t *histograms;
#endif
} tmf8829FrameData_t;

#if ENABLE_HISTOGRAM
#define TMF8829_HISTOGRAM_BUFFER_SIZE  310000
#endif

typedef struct tmf8829FrameParser
{

    int      state;
    int      dataLen;

    int      expectedSubFrames;
    int      currentSubFrame;
    uint8_t  subFrameReceived;

    int      dualMode;
    int      dualCurrentPhase;
    int      dualHistoCount;
    int      isHAHistogram;

    tmf8829FrameData_t frame;

    tmf8829PixelResult_t pixelResults[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];

    uint8_t *resultBuffer;
    int      resultBufferAllocated;

    int      resultSubFrameReceived;
    int      resultReady;
    uint32_t lastFrameNumber;

    uint32_t fpsFirstSystick;
    uint32_t fpsLastSystick;
    int      fpsFrameCount;

    uint16_t maxXtalk;
    float    xtalkPercentage;

    uint8_t  fovCorrection;

#ifdef ENABLE_KEYSTONE

    int      keystoneEnabled;
    double   keystoneAngleX;
    double   keystoneAngleY;
    double   keystoneAngleZ;
#endif

#if ENABLE_HISTOGRAM

    tmf8829Histogram_t *histograms;
    tmf8829Histogram_t *histogramsHA;
    int      histogramsAllocated;
    int      histogramsHAAllocated;

    uint8_t *histogramBuffer;
    int      histogramBufferAllocated;
    int      histogramDataLen;
#endif
} tmf8829FrameParser_t;

typedef struct tmf8829_cfg
{
    int conf_threshold;
    int fpMode;
    int deadtime;
    int period;
    int resultFormat;
    int iteration;
    int shortIteration;
    int histogram_dump;
    int dualMode;
} tmf8829_cfg_t;

void tmf8829FrameParserInit(tmf8829FrameParser_t *parser);

void tmf8829FrameParserCleanup(tmf8829FrameParser_t *parser);

int tmf8829ParseFrameHeader(tmf8829FrameParser_t *parser, uint8_t *data);

int tmf8829ParseFrameFooter(tmf8829FrameParser_t *parser, uint8_t *data, int totalLen);

int tmf8829ParseResultData(tmf8829FrameParser_t *parser, uint8_t *data, uint8_t resultFormat);

#if ENABLE_HISTOGRAM

int tmf8829ParseHistogramData(tmf8829FrameParser_t *parser, uint8_t *data, int subFrame);
#endif

void tmf8829GetResolution(int fpMode, int *rows, int *cols);

int tmf8829GetPixelDataSize(uint8_t resultFormat);

int tmf8829GetHistogramsPerResult(int fpMode, int dualMode);

void handleReceivedFrameHeaderData(void *dptr, uint8_t *data);

void handleReceivedResultData(void *dptr, uint8_t *data, uint16_t size);

void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size);

void handleReceivedResultDataEnd(void *dptr);

void handleReceivedHistogramDataEnd(void *dptr);

void tmf8829PrintFrameResults(tmf8829FrameParser_t *parser);

void tmf8829PrintFrameInfo(tmf8829FrameParser_t *parser);

void tmf8829PrintFpsStats(tmf8829FrameParser_t *parser);

#endif
