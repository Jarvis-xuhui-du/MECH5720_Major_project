/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_KEYSTONE_H
#define TMF8829_KEYSTONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define KEYSTONE_MAX_ZONE_NUMBER        (48*32)
#define KEYSTONE_LINEAR_NUMBER          2
#define KEYSTONE_PI                     (3.14159265)
#define KEYSTONE_DENOISE_AVERAGE_COUNT  5
#define KEYSTONE_DENOISE_PEAK_COUNT     1
#define KEYSTONE_MAX_FIT_DATA           (48*32)

typedef struct keystonePixelInfo {
    int channel;
    int confidence;
    int distance_mm;
    float x;
    float y;
    float z;
} keystonePixelInfo_t;

typedef struct keystoneContext {

    int enableDenoise;
    int snrThreshold;

    int rows;
    int cols;
    int frameSize;

    double fitRawDataX[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    double fitRawDataY[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    double fitRawDataZ[KEYSTONE_MAX_FIT_DATA][KEYSTONE_DENOISE_AVERAGE_COUNT];
    int denoiseIndex;

    int useZoneBits;

    const int *pixelArray8x8;
    const int *pixelArray16x16;
    const int *pixelArray32x32;
    const int *pixelArray48x32;

    double angles[3];
} keystoneContext_t;

int keystoneInit(keystoneContext_t* ctx);

void keystoneCleanup(keystoneContext_t* ctx);

int keystoneSetZonePattern(keystoneContext_t* ctx, bool useZonePattern);

int keystoneProcessFrame(keystoneContext_t* ctx, int rows, int cols,
                         float* xData, float* yData, float* zData,
                         int* confidenceData);

void keystoneGetAngles(keystoneContext_t* ctx, double* angleX, double* angleY, double* angleZ);

void keystoneSetDenoise(keystoneContext_t* ctx, int enable);

#ifdef __cplusplus
}
#endif

#endif
