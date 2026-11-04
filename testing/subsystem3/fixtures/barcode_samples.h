/** @file barcode_samples.h
 * @brief Width fixtures measured from the supplied PDF vector rectangles.
 * See source_manifest.json; time scale is synthetic, patterns are from PDF.
 */
#ifndef BARCODE_SAMPLES_H
#define BARCODE_SAMPLES_H

static uint16_t const g_sample_a[LINE_BARCODE_FRAME_RUNS] =
{
    60u, 180u, 60u, 60u, 180u, 60u, 180u, 60u, 60u, 60u, 180u, 60u, 60u, 60u, 60u,
    180u, 60u, 60u, 180u, 60u, 60u, 180u, 60u, 60u, 180u, 60u, 180u, 60u, 60u
};

static uint16_t const g_sample_z[LINE_BARCODE_FRAME_RUNS] =
{
    60u, 180u, 60u, 60u, 180u, 60u, 180u, 60u, 60u, 60u, 60u, 180u, 180u, 60u, 180u,
    60u, 60u, 60u, 60u, 60u, 60u, 180u, 60u, 60u, 180u, 60u, 180u, 60u, 60u
};

#endif /* BARCODE_SAMPLES_H */
