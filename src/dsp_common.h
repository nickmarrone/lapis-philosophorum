/**
 * dsp_common.h — Shared constants + tiny helpers used by every DSP stage
 * header. Kept namespace-scope constexpr/inline out of the per-stage headers
 * so multiple stage headers can be included in one translation unit without
 * redefinition collisions.
 */

#pragma once
#include <cmath>

namespace mastering_dsp {

constexpr float kLog10Scale = 8.6858896f;    // 20/ln(10)
constexpr float kDbToLn     = 0.11512925f;   // ln(10)/20
constexpr float kMinDetLin  = 3.1623e-5f;    // -90 dBFS
constexpr float kKneeDb     = 6.f;
constexpr float kLsb24      = 1.1920929e-7f; // 2^-23
constexpr float kSatClamp   = 1.25f;

inline float DbToLin(float db) { return expf(kDbToLn * db); }

inline float MsToCoef(float ms, float fs) { return 1.f - expf(-1000.f / (ms * fs)); }

inline float Clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

} // namespace mastering_dsp
