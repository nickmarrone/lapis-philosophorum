/**
 * chain_test_common.h — measurement support for the *whole* chain.
 *
 * Every other harness in this directory drives one stage in isolation, which is
 * the right way to measure what a stage does and is structurally blind to what
 * happens between stages. This one drives the real
 * mastering_dsp::Set* / Process at the real block size and the real control
 * frame, and asks only questions that no single-stage harness can answer:
 *
 *   - Does the chain's latency match what the header promises, and is it the
 *     same number regardless of which stages are bypassed? Delay-matching is a
 *     per-stage claim, but "75 samples" is a chain-level fact and every stage
 *     has to agree on it or a bypass A/B becomes a click.
 *   - Does the ceiling still hold once the stages upstream of the limiter are
 *     allowed to be as loud as their knobs permit?
 *   - Do the two channels stay identical when the input is?
 *   - Does moving a knob — any knob — do anything the knob is not supposed to
 *     do, like overshoot both of its own endpoints on the way between them?
 *
 * ── The DSP's state is global, and that shapes this file ──────────────────────
 *
 * mastering_dsp keeps its chain state in a file-scope anonymous namespace, and
 * Init() deliberately does not reset most of it — the limiter's delay line, the
 * EQ's filter state, the compressor's envelope and every `primed` flag all
 * survive it. That is correct for the module, where Init() runs once against
 * zero-initialised statics and never again, but it means a harness running
 * dozens of scenarios in one process cannot ask for a fresh chain.
 *
 * Boot() stands in for the reset it cannot have: Init(), push, then run silence
 * until the chain has measurably gone quiet. Measurably rather than for a fixed
 * time, because the slowest thing in here is the Glue character's slow release
 * reservoir at tau = 4 s, and a settle long enough for that in every scenario
 * would dominate the harness's runtime while being pure waste in most of them.
 *
 * ── Latency is measured by correlation, not by finding a peak ────────────────
 *
 * The obvious instrument — impulse in, find the largest output sample — is only
 * correct when the chain is a pure delay, i.e. when everything is bypassed. Any
 * active filter (the EQ's shelves, the saturator's emphasis and head bump, the
 * half-band pair itself) moves the peak of the impulse response off the group
 * delay, and the half-band's outermost tap means the *first* nonzero sample
 * arrives 15 samples early rather than on time. Correlating a band-limited
 * noise burst measures the delay the audio actually experiences, and gives the
 * same answer for the bypassed and the active path — which is exactly the
 * property under test.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "test_report.h"
#include "mastering_dsp.h"
#include "dsp_limiter.h"   // TruePeak4x, for the output-side true-peak meter

namespace chaintest {

using testrep::Fmt;
using testrep::Report;

constexpr double kFs    = 48000.0;
constexpr double kPi    = 3.14159265358979323846;
constexpr size_t kBlock = 24;     // kEngineBlockSamples, the real audio block

/** Control frame: 24 * 32 = 768 samples = 16 ms, the rate ControlLoop runs
 *  UpdateParams at. Every Set* comment in the DSP is written against it. */
constexpr int kFrameBlocks = 32;

/** What mastering_dsp.h promises: 60 samples of limiter lookahead plus the
 *  saturator's 15-sample oversampling pair. */
constexpr int kChainLatency = 75;

/** Settle granularity and cap. Typical scenarios need two or three chunks; the
 *  cap only matters for a Glue release parked at its 4 s slow reservoir. */
constexpr int kSettleChunk  = 24000;    // 0.5 s
constexpr int kSettleChunks = 40;       // 20 s worst case

/**
 * The floor a settled chain has to reach.
 *
 * Not zero, and it cannot be: the saturator's mix ease approaches zero
 * asymptotically rather than arriving, and with `asym` off centre the shaper
 * emits a DC term that its blocker drives down but never to nothing. Both land
 * around 1e-29 to 1e-44, i.e. 400+ dB below the 24-bit LSB and hundreds of dB
 * below anything the module can express. The host has no flush-to-zero — the
 * firmware sets FPSCR.FZ and FPU->FPDSCR, so on hardware these are literally 0.
 */
constexpr double kSilenceFloor = 1e-20;

/* ── Settings ─────────────────────────────────────────────────────────────── */

/** One complete snapshot of the chain's control surface. */
struct Settings {
    mastering_dsp::EqParams   eq{};
    mastering_dsp::CompParams comp{};
    mastering_dsp::SatParams  sat{};
    mastering_dsp::OutParams  out{};
};

/**
 * Every stage present but doing as close to nothing as its parameters allow.
 * Not the same as all-bypassed: the filters run, the detectors run, and the
 * limiter's delay line runs — this is the setting where any level change the
 * chain produces is an error rather than a knob.
 */
inline Settings Neutral()
{
    Settings s;
    s.eq.ls_freq_hz = 80.f;   s.eq.ls_gain_db  = 0.f;
    s.eq.mid_freq_hz = 1000.f; s.eq.mid_gain_db = 0.f; s.eq.mid_q = 0.707f;
    s.eq.hs_freq_hz = 8000.f; s.eq.hs_gain_db  = 0.f;
    s.eq.bypass = false;

    s.comp.threshold_db = 0.f; s.comp.ratio = 1.f;
    s.comp.attack_ms = 10.f;   s.comp.release_ms = 100.f;
    s.comp.makeup_db = 0.f;    s.comp.mix = 1.f;
    s.comp.character = mastering_dsp::kCompPrecise;
    s.comp.bypass = false;

    s.sat.drive_db = 0.f; s.sat.mix = 1.f; s.sat.emphasis = 0.f;
    s.sat.asym = 0.f;     s.sat.bump = 0.f;
    s.sat.character = mastering_dsp::kSat15Ips;
    s.sat.bypass = false;

    s.out.ceiling_db = -0.1f; s.out.lim_release_ms = 100.f;
    s.out.trim_db = 0.f;      s.out.dither_lsb = 0.f;
    s.out.lim_bypass = false;
    return s;
}

/** Neutral with every stage bypassed — the chain reduced to its pure delay. */
inline Settings AllBypassed()
{
    Settings s = Neutral();
    s.eq.bypass = s.comp.bypass = s.sat.bypass = s.out.lim_bypass = true;
    return s;
}

/**
 * The loudest, most nonlinear thing the panel can ask for: every gain at its
 * stop, the most aggressive character of each stage, +12 dB of trim into the
 * limiter and dither at its maximum. Used wherever the question is "does the
 * chain still hold together at the ends of its own knobs".
 */
inline Settings Extreme()
{
    Settings s = Neutral();
    s.eq.ls_gain_db = 15.f; s.eq.mid_gain_db = 15.f; s.eq.hs_gain_db = 15.f;
    s.eq.mid_q = 4.f;
    s.comp.threshold_db = -40.f; s.comp.ratio = 20.f;
    s.comp.attack_ms = 0.1f;     s.comp.release_ms = 10.f;
    s.comp.makeup_db = 20.f;
    s.comp.character = mastering_dsp::kCompAdaptive;
    s.sat.drive_db = 24.f; s.sat.emphasis = 1.f; s.sat.bump = 1.f;
    s.sat.asym = 0.3f;
    s.sat.character = mastering_dsp::kSatSaturated;
    s.out.trim_db = 12.f; s.out.dither_lsb = 2.f;
    return s;
}

/**
 * Loud, but with the LIMITER as the thing holding the level down.
 *
 * Extreme() is not that setting and it is worth saying why, because it looks
 * like it should be: at 24 dB of drive into the Saturated character the shaper
 * asymptote caps the stage's output at 1/(knee*drive) ~ 0.045, so the chain
 * leaves 13 dB of headroom under its own ceiling and every ceiling assertion
 * passes without the limiter ever engaging. Here the drive is backed off to
 * 6 dB and the gain moved into makeup and trim, so the limiter really is the
 * binding constraint and "the ceiling holds" means something.
 */
inline Settings Loud()
{
    Settings s = Neutral();
    s.eq.ls_gain_db = 12.f; s.eq.hs_gain_db = 12.f;
    s.comp.threshold_db = -24.f; s.comp.ratio = 3.f; s.comp.makeup_db = 12.f;
    s.comp.character = mastering_dsp::kCompGlue;
    s.sat.drive_db = 6.f; s.sat.emphasis = 0.5f; s.sat.bump = 0.5f;
    s.out.trim_db = 12.f; s.out.dither_lsb = 2.f;
    return s;
}

/* ── Programs ─────────────────────────────────────────────────────────────── */

/** Deterministic across platforms — std::rand's sequence is not guaranteed.
 *  Same generator lim_test_common.h uses, for the same reason. */
struct Lcg {
    uint32_t s = 22695477u;
    float Bipolar() { s = s * 1103515245u + 12345u; return (float)(s >> 8) * (1.f / 8388608.f) - 1.f; }
};

/** Steady tone, phase wrapped so the sequence is bit-periodic. */
inline float Tone(long n, double hz, double amp)
{
    const long period = (long)std::llround(kFs / hz);
    return (float)(amp * std::sin(2.0 * kPi * double(n % period) / double(period)));
}

/** Three tones plus periodic stabs — the "is this a mix" program. */
inline float MixProgram(long n, double amp)
{
    const double t = double(n) / kFs;
    const double s = 0.5 * std::sin(2 * kPi * 55 * t)
                   + 0.4 * std::sin(2 * kPi * 440 * t + 1.0)
                   + 0.3 * std::sin(2 * kPi * 3300 * t + 2.0);
    return (float)(amp * s * ((n % 2400 < 8) ? 3.0 : 1.0));
}

/** HF-dense: the program that separates sample peak from true peak. */
inline float HfProgram(long n, double amp)
{
    const double t = double(n) / kFs;
    return (float)(amp * (0.6 * std::sin(2 * kPi * 11000 * t)
                          + 0.6 * std::sin(2 * kPi * 13700 * t + 0.7)
                          + 0.5 * std::sin(2 * kPi * 19000 * t + 2.1)));
}

/* ── Instruments ──────────────────────────────────────────────────────────── */

inline bool AllFinite(const std::vector<float>& v)
{
    for (float x : v) if (!std::isfinite(x)) return false;
    return true;
}

inline double Peak(const std::vector<float>& v, size_t skip = 0)
{
    double p = 0.0;
    for (size_t i = skip; i < v.size(); i++) p = std::fmax(p, std::fabs((double)v[i]));
    return p;
}

inline double PeakDb(const std::vector<float>& v, size_t skip = 0)
{
    return 20.0 * std::log10(Peak(v, skip) + 1e-30);
}

/**
 * 4x true peak of a captured buffer, built from the same TruePeak4x the limiter
 * detects with. The caveat lim_test_common.h states applies here too and is
 * worth restating: this cannot detect an error common to the detector and the
 * meter. It is a check that the *chain* does not undo what the limiter did, not
 * an independent audit of BS.1770.
 */
inline double TruePeakDb(const std::vector<float>& v, size_t skip = 0)
{
    mastering_dsp::TruePeak4x tp;
    double pk = 0.0;
    for (size_t i = 0; i < v.size(); i++)
    {
        const float m = tp.Process(v[i]);
        if (i >= skip) pk = std::fmax(pk, (double)m);
    }
    return 20.0 * std::log10(pk + 1e-30);
}

/** Largest sample-to-sample step — the instrument for "is that a click". */
inline double MaxStep(const std::vector<float>& v, size_t skip = 0)
{
    double m = 0.0;
    for (size_t i = std::max<size_t>(skip, 1); i < v.size(); i++)
        m = std::fmax(m, std::fabs((double)v[i] - (double)v[i - 1]));
    return m;
}

inline double Rms(const std::vector<float>& v, size_t from, size_t to)
{
    double a = 0.0;
    to = std::min(to, v.size());
    if (to <= from) return 0.0;
    for (size_t i = from; i < to; i++) a += (double)v[i] * (double)v[i];
    return std::sqrt(a / double(to - from));
}

inline double Mean(const std::vector<float>& v, size_t from, size_t to)
{
    double a = 0.0;
    to = std::min(to, v.size());
    if (to <= from) return 0.0;
    for (size_t i = from; i < to; i++) a += (double)v[i];
    return a / double(to - from);
}

/**
 * Envelope follower used by the parameter-move tests: peak magnitude over a
 * sliding `win`-sample window, sampled at every position. A knob move that
 * momentarily multiplies the signal shows up here even though it is not a step
 * and MaxStep would not see it.
 */
inline std::vector<double> Envelope(const std::vector<float>& v, size_t win)
{
    std::vector<double> e;
    if (v.size() < win) return e;
    e.reserve(v.size() - win + 1);
    for (size_t i = 0; i + win <= v.size(); i++)
    {
        double m = 0.0;
        for (size_t k = 0; k < win; k++) m = std::fmax(m, std::fabs((double)v[i + k]));
        e.push_back(m);
    }
    return e;
}

inline double MaxOf(const std::vector<double>& v, size_t from = 0)
{
    double m = -1e300;
    for (size_t i = from; i < v.size(); i++) m = std::fmax(m, v[i]);
    return m;
}

/**
 * Integer lag, in samples, that maximises the cross-correlation of `in` with
 * `out`. This is the chain's latency as the audio experiences it — see the
 * header note on why finding the peak of an impulse response is not.
 */
inline int CorrelationLag(const std::vector<float>& in, const std::vector<float>& out,
                          int max_lag)
{
    int    best_lag = -1;
    double best     = -1e300;
    const size_t n  = std::min(in.size(), out.size());
    for (int lag = 0; lag <= max_lag; lag++)
    {
        double a = 0.0;
        for (size_t i = 0; i + (size_t)lag < n; i++) a += (double)in[i] * (double)out[i + lag];
        if (a > best) { best = a; best_lag = lag; }
    }
    return best_lag;
}

/** Band-limited noise burst — flat enough to correlate sharply, and well below
 *  Nyquist so the half-band's rolloff cannot smear the correlation peak. */
inline void FillNoiseBurst(std::vector<float>& v, size_t n, double amp)
{
    Lcg    rng;
    double lp = 0.0;
    v.assign(n, 0.f);
    for (size_t i = 0; i < n; i++)
    {
        lp += 0.25 * (rng.Bipolar() - lp);      // one-pole, corner ~2 kHz
        v[i] = (float)(amp * lp);
    }
}

/** Maximum sample-to-sample slew a sine of this amplitude produces by itself.
 *  The click threshold sat_control_test established, restated at chain level:
 *  a step the program could have made on its own is not a click. */
inline double SineSlew(double amp, double hz) { return amp * 2.0 * kPi * hz / kFs; }

/* ── The chain under test ─────────────────────────────────────────────────── */

/**
 * Drives mastering_dsp at the real block size, pushing the control surface once
 * every kFrameBlocks blocks exactly as ControlLoop::OnFrame does.
 *
 * `gen(n, &l, &r)` fills one input sample. `ctl(frame, &settings)` runs just
 * before each control frame is pushed, so a test can move a knob mid-run and
 * see what the ease actually does with it.
 */
struct Chain {
    Settings s;
    long     n_ = 0;        // running input sample index, across calls

    void Push()
    {
        mastering_dsp::SetEq(s.eq);
        mastering_dsp::SetComp(s.comp);
        mastering_dsp::SetSat(s.sat);
        mastering_dsp::SetOutput(s.out);
    }

    template <class Gen, class Ctl>
    void Run(Gen gen, Ctl ctl, size_t n, std::vector<float>* oL,
             std::vector<float>* oR = nullptr)
    {
        float il[kBlock], ir[kBlock], ol[kBlock], orr[kBlock];
        const float* ip[2] = {il, ir};
        float*       op[2] = {ol, orr};

        int frame = 0;
        for (size_t done = 0; done < n; done += kBlock)
        {
            if ((done / kBlock) % kFrameBlocks == 0) { ctl(frame++, s); Push(); }

            const size_t m = std::min(kBlock, n - done);
            for (size_t i = 0; i < kBlock; i++)
            {
                il[i] = ir[i] = 0.f;
                if (i < m) gen(n_ + (long)i, &il[i], &ir[i]);
            }
            mastering_dsp::Process(ip, op, kBlock);
            n_ += (long)kBlock;

            if (oL) for (size_t i = 0; i < m; i++) oL->push_back(ol[i]);
            if (oR) for (size_t i = 0; i < m; i++) oR->push_back(orr[i]);
        }
    }

    /** The common case: fixed settings, one generator, collect both channels. */
    template <class Gen>
    void RunSteady(Gen gen, size_t n, std::vector<float>* oL, std::vector<float>* oR = nullptr)
    {
        Run(gen, [](int, Settings&) {}, n, oL, oR);
    }

    /**
     * Init() + settle-until-quiet. See the note at the top of the file for why
     * this cannot be a real reset.
     *
     * Dither is forced off for the duration of the settle — with it on, the
     * output never reaches a floor and the loop would always run to its cap.
     * The two conditions together cover both halves of the chain's memory: the
     * output floor catches the filters, the delay line and the eased scalars,
     * and the gain-reduction meter catches the compressor's envelope, which is
     * silent at the output and is the slowest state in here by an order of
     * magnitude.
     */
    void Boot(const Settings& init)
    {
        s  = init;
        n_ = 0;
        mastering_dsp::Init((float)kFs);

        const float want_dither = s.out.dither_lsb;
        s.out.dither_lsb = 0.f;
        Push();

        for (int chunk = 0; chunk < kSettleChunks; chunk++)
        {
            std::vector<float> tail;
            RunSteady([](long, float* l, float* r) { *l = *r = 0.f; }, kSettleChunk, &tail);
            if (Peak(tail) < kSilenceFloor && mastering_dsp::CompGainReductionDb() < 1e-4)
                break;
        }

        s.out.dither_lsb = want_dither;
        Push();
        n_ = 0;
    }
};

} // namespace chaintest
