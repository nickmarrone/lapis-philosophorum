/**
 * dsp_saturation.h — Header-only waveshaper (cubic soft clip / hard clip)
 * with pre-shape DC offset (asymmetry) and a per-channel one-pole (~5 Hz)
 * DC blocker on the output.
 *
 * Waveshaping uses 1st-order Antiderivative Anti-Aliasing (ADAA): instead of
 * evaluating f(x[n]) directly, the output is the integral-mean of f over the
 * interval [x[n-1], x[n]]:
 *
 *   y[n] = (F(x[n]) − F(x[n-1])) / (x[n] − x[n-1])
 *
 * where F is the closed-form antiderivative of f, derived to be continuous at
 * the piecewise boundaries (±1).  This suppresses alias energy by ~20 dB with
 * zero additional latency and no oversampling.  When the input barely moves
 * (|dx| < 1e-7), the formula is ill-conditioned and falls back to f(midpoint).
 *
 * Reference: Parker et al., DAFx-16.
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
        for (int ch = 0; ch < 2; ++ch) {
            adaa_x1[ch] = 0.f;
            adaa_F1[ch] = Antideriv(0.f, sat_type);
        }
    }

    static float Shape(float x, uint8_t type)
    {
        if (type == 1) {
            return Clampf(x, -1.f, 1.f);
        }
        return (fabsf(x) >= 1.f) ? copysignf(1.f, x) : 1.5f * (x - x*x*x/3.f);
    }

    // Antiderivative of Shape(), continuous at x = ±1.
    // Soft clip: F(x) = 0.75x² − x⁴/8  (|x| ≤ 1), ±x ∓ 0.375 outside.
    // Hard clip: F(x) = 0.5x²           (|x| ≤ 1), ±x ∓ 0.5   outside.
    static float Antideriv(float x, uint8_t type)
    {
        if (type == 1) {
            if (x < -1.f) return -x - 0.5f;
            if (x >  1.f) return  x - 0.5f;
            return 0.5f * x * x;
        }
        if (x < -1.f) return -x - 0.375f;
        if (x >  1.f) return  x - 0.375f;
        const float x2 = x * x;
        return 0.75f * x2 - 0.125f * x2 * x2;
    }

    float ProcessSample(float in, int ch)
    {
        if (bypass) return in;
        const float x  = Clampf(in * drive_lin + offset, -kSatClamp, kSatClamp);
        const float Fx = Antideriv(x, sat_type);
        const float dx = x - adaa_x1[ch];
        float y;
        if (fabsf(dx) < 1e-7f) {
            y = Shape((x + adaa_x1[ch]) * 0.5f, sat_type);
        } else {
            y = (Fx - adaa_F1[ch]) / dx;
        }
        adaa_x1[ch] = x;
        adaa_F1[ch] = Fx;
        y -= offset_shaped;
        const float out = y - dc_x1[ch] + dc_r * dc_y1[ch];
        dc_x1[ch] = y;
        dc_y1[ch] = out;
        return out;
    }

    float drive_lin = 1.f, offset = 0.f, offset_shaped = 0.f, dc_r = 0.999f;
    uint8_t sat_type = 0; bool bypass = false;
    float dc_x1[2] = {0.f, 0.f}, dc_y1[2] = {0.f, 0.f};
    float adaa_x1[2] = {0.f, 0.f}, adaa_F1[2] = {0.f, 0.f};
};

} // namespace mastering_dsp
