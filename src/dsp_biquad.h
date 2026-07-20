/**
 * dsp_biquad.h — Header-only RBJ biquad (Direct Form I), ported verbatim
 * from the template's prior EQ-only DSP file (LowShelf/HighShelf/Peaking
 * coefficient math, already a0-normalized).
 */

#pragma once
#include <cmath>

namespace mastering_dsp {

struct BiquadCoeffs { float b0=1.f, b1=0.f, b2=0.f, a1=0.f, a2=0.f; };
struct BiquadState  { float x1=0.f, x2=0.f, y1=0.f, y2=0.f; };

inline float BiquadProcess(BiquadState& s, const BiquadCoeffs& c, float x)
{
    const float y = c.b0*x + c.b1*s.x1 + c.b2*s.x2 - c.a1*s.y1 - c.a2*s.y2;
    s.x2 = s.x1;  s.x1 = x;
    s.y2 = s.y1;  s.y1 = y;
    return y;
}

inline BiquadCoeffs MakeLowShelf(float f_hz, float gain_db, float fs)
{
    constexpr float kPi = 3.14159265358979323846f;
    const float A     = powf(10.f, gain_db / 40.f);
    const float w0    = 2.f * kPi * f_hz / fs;
    const float cw    = cosf(w0);
    const float sqA   = sqrtf(A);
    const float alpha = sinf(w0) * 0.7071068f;
    const float a0inv = 1.f / ((A+1.f) + (A-1.f)*cw + 2.f*sqA*alpha);
    return {
          A * ((A+1.f) - (A-1.f)*cw + 2.f*sqA*alpha) * a0inv,
        2.f*A * ((A-1.f) - (A+1.f)*cw              ) * a0inv,
          A * ((A+1.f) - (A-1.f)*cw - 2.f*sqA*alpha) * a0inv,
        -2.f * ((A-1.f) + (A+1.f)*cw              ) * a0inv,
               ((A+1.f) + (A-1.f)*cw - 2.f*sqA*alpha) * a0inv,
    };
}

inline BiquadCoeffs MakePeaking(float f_hz, float gain_db, float q, float fs)
{
    constexpr float kPi = 3.14159265358979323846f;
    const float A     = powf(10.f, gain_db / 40.f);
    const float w0    = 2.f * kPi * f_hz / fs;
    const float cw    = cosf(w0);
    const float alpha = sinf(w0) / (2.f * q);
    const float a0inv = 1.f / (1.f + alpha / A);
    return {
        (1.f + alpha * A) * a0inv,
        (-2.f * cw      ) * a0inv,
        (1.f - alpha * A) * a0inv,
        (-2.f * cw      ) * a0inv,
        (1.f - alpha / A) * a0inv,
    };
}

inline BiquadCoeffs MakeHighShelf(float f_hz, float gain_db, float fs)
{
    constexpr float kPi = 3.14159265358979323846f;
    const float A     = powf(10.f, gain_db / 40.f);
    const float w0    = 2.f * kPi * f_hz / fs;
    const float cw    = cosf(w0);
    const float sqA   = sqrtf(A);
    const float alpha = sinf(w0) * 0.7071068f;
    const float a0inv = 1.f / ((A+1.f) - (A-1.f)*cw + 2.f*sqA*alpha);
    return {
          A * ((A+1.f) + (A-1.f)*cw + 2.f*sqA*alpha) * a0inv,
       -2.f*A * ((A-1.f) + (A+1.f)*cw              ) * a0inv,
          A * ((A+1.f) + (A-1.f)*cw - 2.f*sqA*alpha) * a0inv,
        2.f * ((A-1.f) - (A+1.f)*cw               ) * a0inv,
              ((A+1.f) - (A-1.f)*cw - 2.f*sqA*alpha) * a0inv,
    };
}

} // namespace mastering_dsp
