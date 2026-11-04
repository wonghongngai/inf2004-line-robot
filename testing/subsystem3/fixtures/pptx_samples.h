/* Width ratios extracted from BARCODE.pptx slides 2,6,10,13,14,15.
 * EMF bar rectangles/opaque text rectangles; Y PNG sampled at y=80
 * above the licence overlay. Outer white margins are excluded. */
static const struct { char letter; unsigned char widths[29]; } pptx_samples[] = {
    {'A', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 3, 1, 1, 1, 1, 3, 1, 1, 3, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
    {'F', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 1, 1, 3, 1, 3, 3, 1, 1, 1, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
    {'Z', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 1, 3, 3, 1, 3, 1, 1, 1, 1, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
    {'W', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
    {'X', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 1, 3, 1, 1, 3, 1, 1, 1, 3, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
    {'Y', {1, 3, 1, 1, 3, 1, 3, 1, 1, 1, 3, 3, 1, 1, 3, 1, 1, 1, 1, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1}},
};
