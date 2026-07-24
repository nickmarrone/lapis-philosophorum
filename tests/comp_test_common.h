/**
 * comp_test_common.h — host-side measurement support for the compressor.
 *
 * A compressor is not measured the way a filter is. There is no transfer
 * function to sweep; the things that decide whether it sounds right are its
 * steady-state input/output curve, the *shape* of its gain trajectory in time,
 * and how both of those move with the program material. So what lives here is:
 *
 *   1. Signal generators with known crest factors, since crest factor is an
 *      input to the thing under test and not just a property of the test.
 *   2. Steady-state probes — drive a tone, let the envelope settle, read the
 *      output level. This is the static-curve instrument.
 *   3. Gain-reduction trajectories plus exponential fits, which is how the
 *      timing and the dual-time-constant release get measured rather than
 *      eyeballed.
 *
 * The reporting framework comes from test_report.h, shared with the EQ
 * harness. Prototype filter responses come from eq_test_common.h, which is
 * where all the analog prototypes live.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "dsp_compressor.h"
#include "test_report.h"

namespace comptest {

constexpr double kPi = 3.14159265358979323846;
constexpr float  kFs = 48000.f;

inline double DbToLin(double db) { return std::pow(10.0, db / 20.0); }
inline double LinToDb(double x)  { return 20.0 * std::log10(std::max(x, 1e-300)); }

/* ── Parameter construction ───────────────────────────────────────────── */

inline mastering_dsp::CompParams MakeParams(uint8_t character,
                                            float threshold_db,
                                            float ratio,
                                            float attack_ms  = 10.f,
                                            float release_ms = 200.f,
                                            float makeup_db  = 0.f,
                                            float mix        = 1.f,
                                            bool  bypass     = false)
{
    mastering_dsp::CompParams p{};
    p.threshold_db = threshold_db;
    p.ratio        = ratio;
    p.attack_ms    = attack_ms;
    p.release_ms   = release_ms;
    p.makeup_db    = makeup_db;
    p.mix          = mix;
    p.character    = character;
    p.bypass       = bypass;
    return p;
}

/** A compressor configured and ready, with its sidechain filters designed.
 *  Every test starts from one of these so no state leaks between cases. */
inline mastering_dsp::Compressor Fresh(const mastering_dsp::CompParams& p,
                                       float fs = kFs)
{
    mastering_dsp::Compressor c;
    c.InitSidechain(fs);
    c.Configure(p, fs);
    return c;
}

inline const char* CharName(uint8_t ch)
{
    switch (ch)
    {
        case mastering_dsp::kCompPrecise:  return "Precise";
        case mastering_dsp::kCompAdaptive: return "Adaptive";
        case mastering_dsp::kCompGlue:     return "Glue";
    }
    return "?";
}

/* ── Signal generators ────────────────────────────────────────────────── */

/** Stereo sine at a given peak level. Both channels identical, so the
 *  power-sum link reads exactly the peak amplitude in dBFS. */
struct Sine {
    double phase = 0.0, inc;
    float  amp;
    Sine(double f_hz, double peak_db, float fs = kFs)
        : inc(2.0 * kPi * f_hz / fs), amp(float(DbToLin(peak_db))) {}
    void Next(float& l, float& r)
    {
        const float s = amp * float(std::sin(phase));
        phase += inc;
        if (phase > 2.0 * kPi) phase -= 2.0 * kPi;
        l = s; r = s;
    }
};

/**
 * Impulse train: `duty` fraction of each period at full amplitude, silence
 * otherwise, scaled so the *mean square* matches a target RMS. Crest factor is
 * therefore 10*log10(1/duty) dB above that — 13 dB at 5 % duty — which is what
 * makes it the high-crest partner to Sine in the adaptation tests.
 *
 * The bursts are windowed sine rather than DC steps so the sidechain high-pass
 * does not turn each one into a decaying transient of its own.
 */
struct ImpulseTrain {
    int   n = 0, period, on;
    float amp;
    double inc;
    ImpulseTrain(double rate_hz, double duty, double rms_db, float fs = kFs)
        : period(int(fs / rate_hz)), on(std::max(1, int(fs / rate_hz * duty)))
    {
        // Burst is a sine, so its own mean square is amp^2/2 over the on-time.
        const double target_ms = DbToLin(rms_db) * DbToLin(rms_db);
        amp = float(std::sqrt(2.0 * target_ms / duty));
        inc = 2.0 * kPi * 200.0 / fs;
    }
    void Next(float& l, float& r)
    {
        float s = 0.f;
        if (n < on) s = amp * float(std::sin(inc * n * (double(period) / on)));
        if (++n >= period) n = 0;
        l = s; r = s;
    }
};

/** Deterministic white noise, xorshift32, flat and repeatable. */
struct Noise {
    uint32_t s;
    float amp;
    explicit Noise(double rms_db, uint32_t seed = 0x1234567u)
        : s(seed), amp(float(DbToLin(rms_db) * std::sqrt(3.0))) {}
    float One()
    {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return amp * (float(int32_t(s)) * (1.f / 2147483648.f));
    }
    void Next(float& l, float& r) { l = One(); r = One(); }
};

/* ── Steady-state probes ──────────────────────────────────────────────── */

/**
 * Drive a 1 kHz sine at `in_db` peak until the envelope settles, then measure
 * the output peak over a whole number of cycles. Returns dBFS.
 *
 * settle_s must comfortably exceed the slowest time constant in play; the Glue
 * character's slow reservoir reaches 4 s, so callers testing it pay for it.
 */
inline double SteadyOutDb(mastering_dsp::Compressor& c, double in_db,
                          double settle_s = 1.0, float fs = kFs)
{
    Sine sig(1000.0, in_db, fs);
    const int n_settle = int(settle_s * fs);
    float l, r;
    for (int i = 0; i < n_settle; i++) { sig.Next(l, r); c.ProcessSample(l, r); }

    double peak = 0.0;
    for (int i = 0; i < 480; i++)      // 10 cycles at 1 kHz
    {
        sig.Next(l, r);
        c.ProcessSample(l, r);
        peak = std::max(peak, double(std::fabs(l)));
    }
    return LinToDb(peak);
}

/** Steady-state gain reduction in positive dB, same settling contract. */
inline double SteadyGrDb(mastering_dsp::Compressor& c, double in_db,
                         double settle_s = 1.0, float fs = kFs)
{
    Sine sig(1000.0, in_db, fs);
    const int n = int(settle_s * fs);
    float l, r;
    for (int i = 0; i < n; i++) { sig.Next(l, r); c.ProcessSample(l, r); }

    // Average over ten cycles: gr ripples slightly at twice the tone frequency.
    double acc = 0.0;
    for (int i = 0; i < 480; i++) { sig.Next(l, r); c.ProcessSample(l, r); acc += -c.gr_db; }
    return acc / 480.0;
}

/* ── Gain-reduction trajectories ──────────────────────────────────────── */

/**
 * Gain reduction (positive dB) sampled every `step` samples while a signal
 * plays. `gen` is anything with a Next(float&, float&), so any of the sources
 * above can drive it.
 */
template <typename Gen>
inline std::vector<double> GrTrajectory(mastering_dsp::Compressor& c, Gen& gen,
                                        int n_samples, int step)
{
    std::vector<double> out;
    out.reserve(size_t(n_samples / step) + 1);
    float l, r;
    for (int i = 0; i < n_samples; i++)
    {
        gen.Next(l, r);
        c.ProcessSample(l, r);
        if (i % step == 0) out.push_back(-c.gr_db);
    }
    return out;
}

/**
 * Index at which a trajectory first covers `frac` of the way from its first to
 * its last value. Returns a fractional index by linear interpolation, or -1 if
 * it never gets there. Works in both directions, which is what lets the same
 * function measure attack and release.
 */
inline double TimeToFrac(const std::vector<double>& v, double frac)
{
    if (v.size() < 2) return -1.0;
    const double a = v.front(), b = v.back();
    const double target = a + frac * (b - a);
    const bool rising = b > a;
    for (size_t i = 1; i < v.size(); i++)
    {
        const bool hit = rising ? (v[i] >= target) : (v[i] <= target);
        if (hit)
        {
            const double d = v[i] - v[i - 1];
            const double t = (std::fabs(d) < 1e-12) ? 0.0 : (target - v[i - 1]) / d;
            return double(i - 1) + std::max(0.0, std::min(1.0, t));
        }
    }
    return -1.0;
}

/** Largest absolute second difference of a sequence — the discrete curvature
 *  probe the C1-continuity tests use. A kink in the first derivative shows up
 *  here as a spike no amount of smoothing elsewhere can hide. */
inline double MaxSecondDiff(const std::vector<double>& v)
{
    double worst = 0.0;
    for (size_t i = 2; i < v.size(); i++)
        worst = std::max(worst, std::fabs(v[i] - 2.0 * v[i - 1] + v[i - 2]));
    return worst;
}

/* ── Exponential fits ─────────────────────────────────────────────────── */

struct ExpFit {
    double tau_fast = 0.0, tau_slow = 0.0, w_fast = 1.0, residual = 0.0;
};

/** Normalised RMS residual of a model against the decay `v` (which is assumed
 *  to start at v[0] and decay toward zero), sampled at `dt` seconds. */
template <typename Model>
inline double Residual(const std::vector<double>& v, double dt, Model m)
{
    double acc = 0.0;
    for (size_t i = 0; i < v.size(); i++)
    {
        const double e = v[i] - m(double(i) * dt);
        acc += e * e;
    }
    return std::sqrt(acc / double(v.size())) / std::max(1e-12, v.front());
}

/** Best single exponential A*exp(-t/tau) by a coarse-to-fine sweep over tau.
 *  A grid search rather than a solver: the search space is one-dimensional and
 *  bounded, and this cannot fall into a local minimum or fail to converge. */
inline ExpFit FitSingleExp(const std::vector<double>& v, double dt)
{
    const double A = v.front();
    ExpFit best; best.residual = 1e300;
    for (double tau = 0.002; tau < 8.0; tau *= 1.02)
    {
        const double res = Residual(v, dt, [&](double t) { return A * std::exp(-t / tau); });
        if (res < best.residual) { best.residual = res; best.tau_fast = tau; }
    }
    best.tau_slow = best.tau_fast;
    return best;
}

/** Best w*exp(-t/tf) + (1-w)*exp(-t/ts), scaled by v[0]. Same reasoning:
 *  three bounded dimensions, swept, no convergence to worry about. */
inline ExpFit FitDualExp(const std::vector<double>& v, double dt)
{
    const double A = v.front();
    ExpFit best; best.residual = 1e300;
    for (double tf = 0.005; tf < 1.5; tf *= 1.06)
        for (double ts = tf * 1.5; ts < 10.0; ts *= 1.06)
            for (double w = 0.2; w <= 0.85; w += 0.025)
            {
                const double res = Residual(v, dt, [&](double t) {
                    return A * (w * std::exp(-t / tf) + (1.0 - w) * std::exp(-t / ts));
                });
                if (res < best.residual)
                {
                    best.residual = res;
                    best.tau_fast = tf; best.tau_slow = ts; best.w_fast = w;
                }
            }
    return best;
}

/* ── RMS of a rendered buffer ─────────────────────────────────────────── */

inline double RmsDb(const std::vector<float>& v)
{
    double acc = 0.0;
    for (float x : v) acc += double(x) * double(x);
    return 10.0 * std::log10(std::max(acc / double(std::max<size_t>(1, v.size())), 1e-300));
}

} // namespace comptest
