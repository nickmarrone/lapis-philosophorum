/**
 * lim_test_common.h — measurement support for the brickwall limiter.
 *
 * What a limiter has to be measured on is narrower than a compressor and much
 * narrower than a saturator, because it makes only one promise: nothing leaves
 * above the ceiling, and nothing is clipped on the way to keeping that promise.
 * Both halves matter. A limiter that meets its ceiling by hard-clipping into
 * the safety clamp is meeting it the way a wall meets a car.
 *
 * So the primary instrument here reads the signal *before* the safety Clampf.
 * That is the number the old one-pole attack got wrong by 12 dB while the
 * post-clamp output looked perfect — measuring the output alone cannot
 * distinguish "limited to the ceiling" from "clipped at the ceiling", which is
 * precisely the distinction under test.
 *
 * Reading it needs no hook in the DSP. `Limiter` is a plain struct, the read
 * index is `(write + 1) % (kLookahead + 1)`, and for kLookahead > 0 that slot
 * is never the one written this call — so capturing `buf_l[rd]` before
 * ProcessSample and `gain` after reproduces the pre-clamp product exactly.
 *
 * The programs are chosen for the failure mode, not for realism: what broke the
 * one-pole was transients *shorter than the window*, so an impulse train and a
 * stab-laden mix carry more weight here than a sine ever could. The steady sine
 * and square cases are kept as negative controls — they passed before the fix
 * too, and a harness that only ran them would have shipped the bug.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "test_report.h"
#include "dsp_common.h"
#include "dsp_limiter.h"

namespace limtest {

using testrep::Fmt;
using testrep::Report;

constexpr double kFs = 48000.0;
constexpr double kPi = 3.14159265358979323846;

/* ── Programs ────────────────────────────────────────────────────────────── */

enum Program {
    kSineStabs,     // steady 1 kHz with 20-sample +6 dB stabs — the mixed case
    kImpulses,      // isolated one-sample peaks — the pure lookahead case
    kWhiteNoise,    // dense, every sample a new peak candidate
    kSquare,        // steady-state negative control; the one-pole passed this
    kDenseMix,      // three tones plus periodic stabs
    kLevelStep,     // quiet, then a hard jump to +18 dB
    kProgramCount
};

inline const char* ProgramName(int p)
{
    switch (p)
    {
        case kSineStabs:  return "1 kHz + 20-sample stabs";
        case kImpulses:   return "isolated impulses";
        case kWhiteNoise: return "white noise +12 dB";
        case kSquare:     return "square 100 Hz +10 dB";
        case kDenseMix:   return "dense mix + stabs";
        default:          return "level step 0 -> +18 dB";
    }
}

/** Deterministic across platforms — std::rand's sequence is not guaranteed. */
struct Lcg {
    uint32_t s = 22695477u;
    float    Bipolar() { s = s * 1103515245u + 12345u; return (float)(s >> 8) * (1.f / 8388608.f) - 1.f; }
};

/** One sample of the given program. `rng` is only touched by the noise case. */
inline float ProgramSample(int prog, int n, Lcg& rng)
{
    const double t = (double)n / kFs;
    switch (prog)
    {
        case kSineStabs:
            return (float)(std::sin(2 * kPi * 1000 * t) * ((n % 4800 < 20) ? 2.0 : 1.0));
        case kImpulses:
            return (n % 977 == 0) ? 6.f : 0.f;
        case kWhiteNoise:
            return 4.f * rng.Bipolar();
        case kSquare:
            return (std::sin(2 * kPi * 100 * t) > 0 ? 3.16f : -3.16f);
        case kDenseMix: {
            float s = (float)(1.5 * std::sin(2 * kPi * 55 * t)
                              + 1.2 * std::sin(2 * kPi * 440 * t + 1.0)
                              + 0.9 * std::sin(2 * kPi * 3300 * t + 2.0));
            return (n % 2400 < 8) ? s * 3.f : s;
        }
        default:
            return (float)((n > 96000 ? 8.0 : 0.2) * std::sin(2 * kPi * 300 * t));
    }
}

/* ── Instruments ─────────────────────────────────────────────────────────── */

struct Overshoot {
    double worst_db   = -1e9;  // peak excess over the ceiling, pre-clamp
    long   clipped    = 0;     // samples the safety Clampf had to cut
    double clip_energy = 0.0;
};

/**
 * Relative slack below which an "overshoot" is float rounding rather than a
 * limiter that let something through.
 *
 * Zero overshoot is exact in real arithmetic — the boxcar mean of B running
 * minima is <= the peak's own target by construction. In float it is exact to
 * within the rounding of that mean and of the `delayed * gain` product, so the
 * product can sit an ulp or two above the ceiling. That residual is what the
 * safety Clampf is actually for, and 1e-6 relative (8.7e-6 dB) is about four
 * ulps at these magnitudes — tight enough that the 12.35 dB failure this
 * harness exists to catch could not hide under it by six orders of magnitude.
 */
constexpr double kFloatSlack = 1e-6;

/**
 * Run a program through a configured limiter and report what the safety clamp
 * was asked to absorb. `skip` discards the startup transient — the windows boot
 * full of 1.f, which is correct but is not steady state.
 */
inline Overshoot MeasureOvershoot(mastering_dsp::Limiter& lim,
                                  int                     prog,
                                  int                     samples,
                                  int                     skip = 4000)
{
    using namespace mastering_dsp;
    Overshoot  o;
    Lcg        rng;
    const float ceil = lim.ceiling_lin;

    for (int n = 0; n < samples; n++)
    {
        const float s  = ProgramSample(prog, n, rng);
        const int   rd = (lim.write + 1) % (kLookahead + 1);
        const float dl = lim.buf_l[rd];      // what this call will emit, pre-gain

        float l = s, r = s;
        lim.ProcessSample(l, r);

        if (n < skip) continue;
        const double pre = std::fabs(dl * lim.gain);
        if (pre > ceil * (1.0 + kFloatSlack))
        {
            o.clipped++;
            o.clip_energy += (pre - ceil) * (pre - ceil);
        }
        if (pre > 0.0) o.worst_db = std::fmax(o.worst_db, 20.0 * std::log10(pre / ceil));
    }
    return o;
}

/** Peak of the limiter's actual output, in dBFS. */
inline double OutputPeakDb(mastering_dsp::Limiter& lim, int prog, int samples, int skip = 4000)
{
    Lcg    rng;
    double pk = 0.0;
    for (int n = 0; n < samples; n++)
    {
        const float s = ProgramSample(prog, n, rng);
        float       l = s, r = s;
        lim.ProcessSample(l, r);
        if (n >= skip) pk = std::fmax(pk, std::fabs(l));
    }
    return 20.0 * std::log10(pk + 1e-30);
}

/**
 * 4x true-peak meter for the limiter's *output*.
 *
 * Deliberately built from the same TruePeak4x the DSP uses. That makes this a
 * measurement of the limiter, not of the interpolator — if the half-band were
 * wrong, a meter built on a different filter would disagree and the test would
 * blame the limiter. `sat_response_test` already asserts the half-band's
 * stopband and round-trip against tools/halfband_design.py, so its correctness
 * is established elsewhere and can be leaned on here.
 *
 * The consequence worth stating: this cannot detect an error *common* to the
 * detector and the meter. It is a check that the limiter uses its detector
 * correctly, not an independent audit of BS.1770 conformance.
 */
struct TruePeakMeter {
    mastering_dsp::TruePeak4x tp;
    double                    peak = 0.0;
    void Push(float x) { peak = std::fmax(peak, tp.Process(x)); }
};

/** Highest true peak the limiter emits on a program, in dBTP. */
inline double OutputTruePeakDb(mastering_dsp::Limiter& lim, int prog, int samples, int skip = 4000)
{
    Lcg           rng;
    TruePeakMeter m;
    for (int n = 0; n < samples; n++)
    {
        const float s = ProgramSample(prog, n, rng);
        float       l = s, r = s;
        lim.ProcessSample(l, r);
        if (n >= skip) m.Push(l);
    }
    return 20.0 * std::log10(m.peak + 1e-30);
}

/** HF-dense program — the case that separates sample peak from true peak. */
inline float IspSample(int n)
{
    const double t = (double)n / kFs;
    return (float)(1.8 * (0.6 * std::sin(2 * kPi * 11000 * t)
                          + 0.6 * std::sin(2 * kPi * 13700 * t + 0.7)
                          + 0.5 * std::sin(2 * kPi * 19000 * t + 2.1)));
}

inline mastering_dsp::Limiter MakeLimiter(float ceiling_db, float release_ms, bool byp = false)
{
    mastering_dsp::Limiter l;
    l.Configure(ceiling_db, release_ms, byp, (float)kFs);
    return l;
}

} // namespace limtest
