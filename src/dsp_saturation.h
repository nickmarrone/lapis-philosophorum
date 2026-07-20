/**
 * dsp_saturation.h — Header-only waveshaper (cubic soft clip / hard clip)
 * with pre-shape DC offset (asymmetry) and a per-channel one-pole (~5 Hz)
 * DC blocker on the output.
 */

#pragma once
#include <cmath>
#include <cstdint>
#include "mastering_dsp.h"
#include "dsp_common.h"

namespace mastering_dsp {

struct Saturator {
    void Configure(const OutParams& p, float fs)
    {
        drive_lin     = DbToLin(p.drive_db);
        offset        = p.asym;
        sat_type      = p.sat_type;
        offset_shaped = Shape(offset, sat_type);
        dc_r          = 1.f - (2.f * 3.14159265f * 5.f) / fs;
        bypass        = p.sat_bypass;
    }

    static float Shape(float x, uint8_t type)
    {
        if (type == 1) {
            return Clampf(x, -1.f, 1.f);
        }
        return (fabsf(x) >= 1.f) ? copysignf(1.f, x) : 1.5f * (x - x*x*x/3.f);
    }

    float ProcessSample(float in, int ch)
    {
        if (bypass) return in;
        const float x   = Clampf(in * drive_lin + offset, -kSatClamp, kSatClamp);
        const float y   = Shape(x, sat_type) - offset_shaped;
        const float out = y - dc_x1[ch] + dc_r * dc_y1[ch];
        dc_x1[ch] = y;
        dc_y1[ch] = out;
        return out;
    }

    float drive_lin = 1.f, offset = 0.f, offset_shaped = 0.f, dc_r = 0.999f;
    uint8_t sat_type = 0; bool bypass = false;
    float dc_x1[2] = {0.f, 0.f}, dc_y1[2] = {0.f, 0.f};
};

} // namespace mastering_dsp
