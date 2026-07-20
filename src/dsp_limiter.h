/**
 * dsp_limiter.h — Header-only stereo-linked brickwall limiter: instantaneous
 * attack (gain only ever drops immediately to satisfy the peak), exponential
 * release, with a safety clamp applied after gain application.
 */

#pragma once
#include <cmath>
#include "dsp_common.h"

namespace mastering_dsp {

struct Limiter {
    void Configure(float ceiling_db, float release_ms, float fs)
    {
        ceiling_lin = DbToLin(ceiling_db);
        rel_coef    = MsToCoef(release_ms, fs);
    }

    void ProcessSample(float& l, float& r)
    {
        const float peak = fmaxf(fabsf(l), fabsf(r));
        const float gd   = (peak > ceiling_lin) ? ceiling_lin / peak : 1.f;
        if (gd < gain) gain = gd;
        else           gain += rel_coef * (gd - gain);
        l *= gain; r *= gain;
        l = Clampf(l, -ceiling_lin, ceiling_lin);
        r = Clampf(r, -ceiling_lin, ceiling_lin);
    }

    float gain = 1.f;
    float ceiling_lin = 1.f, rel_coef = 0.01f;
};

} // namespace mastering_dsp
