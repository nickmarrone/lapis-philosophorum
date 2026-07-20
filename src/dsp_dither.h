/**
 * dsp_dither.h — Header-only TPDF (triangular) dither generator built from
 * two independent xorshift32 draws.
 */

#pragma once
#include <cstdint>
#include "dsp_common.h"

namespace mastering_dsp {

struct TpdfDither {
    explicit TpdfDither(uint32_t seed) : state(seed) {}   // seed must be nonzero

    float Sample()
    {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        const float u1 = (float)(state >> 8) * (1.f/16777216.f);
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        const float u2 = (float)(state >> 8) * (1.f/16777216.f);
        return (u1 - u2) * amp;
    }

    float amp = 0.f;      // set at control rate = dither_lsb * kLsb24
    uint32_t state;
};

} // namespace mastering_dsp
