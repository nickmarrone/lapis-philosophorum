/**
 * sat_test_common.h — measurement support for the tape saturator.
 *
 * The instrument set here follows from what a saturator actually is:
 *
 *   1. A thing that makes harmonics on purpose. So the primary probe reads
 *      individual harmonic bins, not a lumped THD number — "how fast does the
 *      series decay" is the question that separates a tape curve from a
 *      clipper, and a single THD figure cannot answer it.
 *   2. A thing that makes aliases by accident. Those land at *known*
 *      frequencies — |k*f0 - m*fs| for the k-th harmonic — so they can be read
 *      with the same single-bin probe. No FFT is needed to measure a spectrum
 *      you can predict the support of.
 *   3. A thing that is supposed to be inaudible when it is not driven. That is
 *      a magnitude-response measurement, which means driving the real stage and
 *      comparing against the analytic filter response.
 *
 * Single-bin evaluation rather than an FFT is a deliberate carry-over of
 * eq_response_test's naive-DFT idiom and of invariant 11's spirit: the tests
 * tree compiles with g++ and nothing else. CMSIS-DSP is vendored in libDaisy
 * but it is the ARM target's library and the host harnesses do not link it.
 *
 * A Hann window is applied before every bin read. Without it the spectral
 * leakage from a non-integer number of periods swamps a -90 dB alias bin, and
 * the alias frequencies are not ours to choose — they fall where the folding
 * puts them.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "test_report.h"
#include "mastering_dsp.h"
#include "dsp_saturation.h"

namespace sattest {

using testrep::Fmt;
using testrep::Report;

constexpr double kFs = 48000.0;
constexpr double kPi = 3.14159265358979323846;

/* ── Parameter construction ──────────────────────────────────────────────── */

/** Neutral params: unity drive, full wet, no emphasis, no bump, 15 ips. */
inline mastering_dsp::SatParams MakeParams()
{
    mastering_dsp::SatParams p{};
    p.drive_db  = 0.f;
    p.mix       = 1.f;
    p.emphasis  = 0.f;
    p.asym      = 0.f;
    p.bump      = 0.f;
    p.character = mastering_dsp::kSat15Ips;
    p.bypass    = false;
    return p;
}

/**
 * A saturator configured and settled, with no state left from any previous
 * case. Every probe below builds one of these rather than reusing a shared
 * instance — the stage carries filter, ADAA, oversampler and dry-delay state,
 * and a leaked envelope is exactly the kind of thing that makes a harness lie.
 *
 * The settle loop matters for a second reason: the audio side eases drive, mix
 * and asym at 5 ms, so a measurement taken immediately after Configure() reads
 * the ramp rather than the setting.
 */
struct Rig {
    mastering_dsp::Saturator sat;

    explicit Rig(const mastering_dsp::SatParams& p, int settle_ms = 120)
    {
        sat.Init(float(kFs));
        // Configure at the real 16 ms control cadence so the control-side glide
        // reaches its target the way it does in the firmware.
        const int frames = settle_ms / 16 + 2;
        for (int f = 0; f < frames; f++) sat.Configure(p, float(kFs));
        sat.BeginBlock();
        // Push silence through so the dry delay and filter states are primed.
        for (int n = 0; n < int(kFs * settle_ms / 1000.0); n++) {
            float l = 0.f, r = 0.f;
            sat.ProcessSample(l, r);
        }
    }

    /** Run a block, keeping BeginBlock() on the real 24-sample cadence. */
    void Run(const float* in, float* out, int n)
    {
        for (int i = 0; i < n; i++) {
            if (i % 24 == 0) sat.BeginBlock();
            float l = in[i], r = in[i];
            sat.ProcessSample(l, r);
            out[i] = l;
        }
    }
};

/* ── Generators ──────────────────────────────────────────────────────────── */

inline std::vector<float> Sine(double f_hz, double amp, int n, double phase = 0.0)
{
    std::vector<float> v(static_cast<size_t>(n));
    for (int i = 0; i < n; i++)
        v[size_t(i)] = float(amp * std::sin(2.0 * kPi * f_hz * i / kFs + phase));
    return v;
}

/** Deterministic white noise, xorshift32 — same generator the compressor
 *  harness uses, so "noise" means the same signal across harnesses. */
inline std::vector<float> Noise(double amp, int n, uint32_t seed = 0x13579BDFu)
{
    std::vector<float> v(static_cast<size_t>(n));
    uint32_t s = seed;
    for (int i = 0; i < n; i++) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        v[size_t(i)] = float(amp * (2.0 * (double(s) / 4294967296.0) - 1.0));
    }
    return v;
}

/* ── Spectral probes ─────────────────────────────────────────────────────── */

/**
 * Amplitude at one frequency, via a Hann-windowed single-bin DFT.
 *
 * Returned as a linear amplitude referred to full scale: a full-amplitude sine
 * at f reads 1.0. The 2.0 accounts for the negative-frequency image and the
 * 0.5 for the Hann window's coherent gain.
 */
inline double BinAmp(const std::vector<float>& x, double f_hz, int skip = 0)
{
    const int n = int(x.size()) - skip;
    if (n <= 0) return 0.0;
    const double w = 2.0 * kPi * f_hz / kFs;
    double re = 0.0, im = 0.0, wsum = 0.0;
    for (int i = 0; i < n; i++) {
        const double win = 0.5 - 0.5 * std::cos(2.0 * kPi * i / (n - 1));
        const double v   = double(x[size_t(i + skip)]) * win;
        re += v * std::cos(w * i);
        im -= v * std::sin(w * i);
        wsum += win;
    }
    return 2.0 * std::hypot(re, im) / wsum;
}

inline double BinDb(const std::vector<float>& x, double f_hz, int skip = 0)
{
    return 20.0 * std::log10(BinAmp(x, f_hz, skip) + 1e-30);
}

/**
 * Where the k-th harmonic of f0 actually lands after sampling.
 *
 * Above Nyquist it folds: reduce modulo fs into [0, fs), then reflect anything
 * above fs/2. This is the whole reason no FFT is needed — the alias support is
 * computable in closed form.
 */
inline double FoldFreq(double f0, int k)
{
    double f = std::fmod(f0 * k, kFs);
    if (f > kFs / 2.0) f = kFs - f;
    return f;
}

/** True when harmonic k of f0 is above Nyquist, i.e. it can only appear at all
 *  by aliasing. These are the bins that measure the anti-aliasing. */
inline bool IsAlias(double f0, int k) { return f0 * k > kFs / 2.0; }

/** Harmonic levels H1..Hk, in dB relative to the fundamental. */
inline std::vector<double> Harmonics(const std::vector<float>& y, double f0,
                                     int kmax, int skip = 0)
{
    std::vector<double> h;
    const double h1 = BinAmp(y, f0, skip);
    for (int k = 1; k <= kmax; k++)
        h.push_back(20.0 * std::log10(BinAmp(y, FoldFreq(f0, k), skip) / (h1 + 1e-30)
                                      + 1e-30));
    return h;
}

/**
 * Total alias energy from driving a tone at f0, in dB below the fundamental.
 *
 * Sums only the bins where a harmonic has folded — a harmonic that lands below
 * Nyquist honestly is signal the stage is meant to produce, not an artifact.
 */
inline double AliasDb(const std::vector<float>& y, double f0, int kmax, int skip = 0)
{
    const double h1 = BinAmp(y, f0, skip);
    double p = 0.0;
    for (int k = 2; k <= kmax; k++) {
        if (!IsAlias(f0, k)) continue;
        const double f = FoldFreq(f0, k);
        if (f < 20.0) continue;                 // DC-adjacent; the blocker owns it
        const double a = BinAmp(y, f, skip);
        p += a * a;
    }
    return 10.0 * std::log10(p / (h1 * h1 + 1e-30) + 1e-30);
}

/* ── Level probes ────────────────────────────────────────────────────────── */

inline double RmsDb(const std::vector<float>& x, int skip = 0)
{
    double s = 0.0;
    const int n = int(x.size()) - skip;
    for (int i = 0; i < n; i++) s += double(x[size_t(i + skip)]) * x[size_t(i + skip)];
    return 10.0 * std::log10(s / n + 1e-30);
}

/** Mean value — the DC the asymmetry is supposed to not leave behind. */
inline double MeanDb(const std::vector<float>& x, int skip = 0)
{
    double s = 0.0;
    const int n = int(x.size()) - skip;
    for (int i = 0; i < n; i++) s += double(x[size_t(i + skip)]);
    return 20.0 * std::log10(std::fabs(s / n) + 1e-30);
}

/**
 * Gain of the stage at a given frequency and level, in dB.
 *
 * Measured at the bin rather than by RMS so harmonics do not contaminate it —
 * at high drive the RMS ratio would read the distortion as gain.
 */
inline double GainDb(const mastering_dsp::SatParams& p, double f_hz, double amp,
                     int n = 24000)
{
    Rig rig(p);
    const auto in = Sine(f_hz, amp, n);
    std::vector<float> out(static_cast<size_t>(n));
    rig.Run(in.data(), out.data(), n);
    const int skip = 2000;
    return 20.0 * std::log10(BinAmp(out, f_hz, skip) / (amp + 1e-30) + 1e-30);
}

} // namespace sattest
