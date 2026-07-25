/**
 * comp_control_test.cpp — the compressor's control path, driven through the
 * real mastering_dsp::SetComp / Process in 24-sample blocks.
 *
 * The EQ's companion harness asserts that a stationary knob is *bit-identical*
 * to never calling SetEq again. That assertion cannot be made here, and the
 * reason is the point of this file rather than a weakness of it: the
 * compressor has a running envelope and a 5 ms parameter ease, so its output
 * legitimately depends on time in a way a static filter's does not. The honest
 * property — and the one that actually justifies giving this stage a plain
 * direct-forward SetComp instead of the EQ's double-buffered, dirty-checked
 * handoff — is that ADC-scale jitter does not reach the output at any audible
 * level. That is measured here, with a negative control to prove the
 * measurement has teeth.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "mastering_dsp.h"
#include "comp_test_common.h"

using namespace comptest;
using testrep::Fmt;
using testrep::Report;

namespace {

constexpr size_t kBlock = 24;      // kEngineBlockSamples
constexpr int    kFrame = 32;      // blocks per ~16 ms control frame

/** Everything except the compressor made transparent, so these tests observe
 *  the compressor alone. Dither in particular must be off or nothing repeats. */
void SilenceRestOfChain()
{
    mastering_dsp::EqParams e{};
    e.ls_freq_hz = 80.f;    e.ls_gain_db  = 0.f;
    e.mid_freq_hz = 1000.f; e.mid_gain_db = 0.f; e.mid_q = 0.707f;
    e.hs_freq_hz = 8000.f;  e.hs_gain_db  = 0.f;
    e.bypass = true;
    mastering_dsp::SetEq(e);

    mastering_dsp::SatParams s{};
    s.drive_db = 0.f; s.mix = 1.f; s.emphasis = 0.f; s.asym = 0.f; s.bump = 0.f;
    s.character = 0; s.bypass = true;
    mastering_dsp::SetSat(s);

    mastering_dsp::OutParams o{};
    o.ceiling_db = -0.1f; o.lim_release_ms = 100.f;
    o.trim_db = 0.f; o.dither_lsb = 0.f; o.lim_bypass = false;
    mastering_dsp::SetOutput(o);
}

/**
 * Push silence through the chain until every piece of state has forgotten
 * whatever the previous test left in it.
 *
 * This is load-bearing, not hygiene theatre. mastering_dsp keeps its chain
 * state at file scope and Init() deliberately does not clear it — the firmware
 * calls Init() exactly once, and a reset would be dead code on the target. So
 * in a harness that runs several scenarios in one process, run N starts inside
 * run N-1's limiter delay line, compressor envelope and filter state. Without
 * this flush the latency probe finds a leftover sample rather than its own
 * impulse, and two "identical" runs differ for reasons that have nothing to do
 * with the thing under test.
 *
 * Three seconds because the Glue character's slow release reservoir reaches
 * 12 x 200 ms; everything else here is far quicker.
 */
void FlushChain(double seconds = 3.0)
{
    float in_l[kBlock] = {}, in_r[kBlock] = {}, o_l[kBlock], o_r[kBlock];
    const float* ins[2]  = {in_l, in_r};
    float*       outs[2] = {o_l, o_r};
    const int n_blocks = int(seconds * kFs / double(kBlock));
    for (int b = 0; b < n_blocks; b++) mastering_dsp::Process(ins, outs, kBlock);
}

/** Init + make everything but the compressor transparent + flush. */
void ResetChain()
{
    mastering_dsp::Init(kFs);
    SilenceRestOfChain();
    mastering_dsp::SetComp(MakeParams(mastering_dsp::kCompPrecise, 0.f, 1.f,
                                      10.f, 100.f, 0.f, 1.f, true));
    FlushChain();
}

/** Run the whole chain, pushing comp params every kFrame blocks with an
 *  optional multiplicative jitter on threshold, and return the output. */
std::vector<float> RunChain(int n_blocks, float thresh_db, double jitter,
                            uint8_t character, uint32_t seed = 0xC0FFEEu)
{
    ResetChain();

    std::vector<float> out;
    out.reserve(size_t(n_blocks) * kBlock);

    float in_l[kBlock], in_r[kBlock], o_l[kBlock], o_r[kBlock];
    const float* ins[2]  = {in_l, in_r};
    float*       outs[2] = {o_l, o_r};

    Noise    n(-14.0, seed);
    uint32_t js = 0x9E3779B9u;

    for (int b = 0; b < n_blocks; b++)
    {
        if (b % kFrame == 0)
        {
            js ^= js << 13; js ^= js >> 17; js ^= js << 5;
            const double u = double(int32_t(js)) / 2147483648.0;   // -1 .. 1
            const float t = float(thresh_db * (1.0 + jitter * u));
            mastering_dsp::SetComp(MakeParams(character, t, 4.f, 10.f, 200.f, 0.f, 1.f));
        }
        for (size_t i = 0; i < kBlock; i++) n.Next(in_l[i], in_r[i]);
        mastering_dsp::Process(ins, outs, kBlock);
        for (size_t i = 0; i < kBlock; i++) out.push_back(o_l[i]);
    }
    return out;
}

/** Level of the difference between two runs, in dBFS RMS. */
double DiffDb(const std::vector<float>& a, const std::vector<float>& b)
{
    const size_t n = std::min(a.size(), b.size());
    double acc = 0.0;
    for (size_t i = 0; i < n; i++)
    {
        const double d = double(a[i]) - double(b[i]);
        acc += d * d;
    }
    return 10.0 * std::log10(std::max(acc / double(std::max<size_t>(1, n)), 1e-300));
}

/* ── Test 1: ADC jitter rejection ─────────────────────────────────────── */

void TestJitterRejection(Report& r)
{
    r.Section("ADC-scale jitter does not reach the output audibly");

    const auto clean = RunChain(3000, -18.f, 0.0,    mastering_dsp::kCompGlue);
    const auto small = RunChain(3000, -18.f, 0.0005, mastering_dsp::kCompGlue);  // 0.05 %
    const auto gross = RunChain(3000, -18.f, 0.02,   mastering_dsp::kCompGlue);  // 2 %

    const double prog = RmsDb(clean);

    /*
     * Expressed as gain modulation, not as an absolute difference level. That
     * is the perceptually meaningful quantity here and it is what makes the
     * bound defensible: a difference signal 53 dB below the program is a gain
     * that wobbles by 0.02 dB, and the just-noticeable difference for a slow
     * level change is somewhere north of 0.2 dB.
     *
     * Note this is a WEAKER claim than the EQ harness's, on purpose. The EQ
     * dirty-checks its design inputs, so a stationary knob there is bit
     * identical. Nothing here is dirty-checked: threshold goes straight into
     * the gain computer, so jitter genuinely does pass through, attenuated by
     * the ratio slope and smeared by the envelope. The claim is that what
     * arrives is inaudible, and the measurement below says by how much.
     */
    auto GainModDb = [&](const std::vector<float>& v) {
        const double rel = std::pow(10.0, (DiffDb(clean, v) - prog) / 20.0);
        return 20.0 * std::log10(1.0 + rel);
    };
    const double m_small = GainModDb(small);
    const double m_gross = GainModDb(gross);

    r.Check(m_small < 0.05, "0.05 % threshold jitter moves the gain under 0.05 dB",
            Fmt("%.4f dB of gain modulation (%.1f dBFS difference)", m_small, DiffDb(clean, small)));
    r.Check(m_gross > 0.05, "negative control — 2 % jitter IS detected",
            Fmt("%.4f dB of gain modulation (%.1f dBFS difference)", m_gross, DiffDb(clean, gross)));
    r.Info("this is why SetComp needs no dirty check",
           Fmt("program %.1f dBFS; the envelope network is the smoother", prog));
}

/* ── Test 2: character switching is click-free ────────────────────────── */

void TestCharacterSwitch(Report& r)
{
    r.Section("Character switch mid-signal is click-free");

    ResetChain();

    float in_l[kBlock], in_r[kBlock], o_l[kBlock], o_r[kBlock];
    const float* ins[2]  = {in_l, in_r};
    float*       outs[2] = {o_l, o_r};

    Sine sig(220.0, -8.0);
    std::vector<float> out;
    for (int b = 0; b < 2000; b++)
    {
        if (b % kFrame == 0)
        {
            const uint8_t ch = uint8_t((b / 400) % 3);   // switch every ~200 ms
            mastering_dsp::SetComp(MakeParams(ch, -20.f, 4.f, 10.f, 200.f, 0.f, 1.f));
        }
        for (size_t i = 0; i < kBlock; i++) sig.Next(in_l[i], in_r[i]);
        mastering_dsp::Process(ins, outs, kBlock);
        // Discard the tone's own onset: silence -> full-amplitude tone is a
        // step of its own, and it is not what this test is about.
        if (b > 200) for (size_t i = 0; i < kBlock; i++) out.push_back(o_l[i]);
    }

    // The tone's own maximum sample-to-sample slew is the yardstick: a click is
    // a step the signal itself could not have produced.
    const double tone_slew = 2.0 * kPi * 220.0 / double(kFs) * DbToLin(-8.0);
    double worst = 0.0, peak = 0.0;
    for (size_t i = 1; i < out.size(); i++)
    {
        worst = std::max(worst, std::fabs(double(out[i]) - double(out[i - 1])));
        peak  = std::max(peak, std::fabs(double(out[i])));
    }
    r.Check(worst < tone_slew * 1.5, "no step larger than the tone's own slew",
            Fmt("worst %.2e vs tone slew %.2e", worst, tone_slew));
    r.Check(peak < DbToLin(-8.0) * 1.2, "no overshoot across the switch",
            Fmt("peak %.4f", peak));
}

/* ── Test 3: the ratio taper ──────────────────────────────────────────── */

/** Mirrors RatioFromAmount in mastering.cpp. Duplicated rather than included
 *  because mastering.cpp cannot be compiled on a host — it needs the STM32
 *  HAL — so this test pins the *contract*: if the taper there changes, this
 *  has to be changed deliberately alongside it. */
float RatioFromAmount(float a)
{
    const float amt = a < 0.f ? 0.f : (a > 0.95f ? 0.95f : a);
    return 1.f / (1.f - amt);
}

void TestRatioTaper(Report& r)
{
    r.Section("Ratio taper — the useful range gets real knob travel");

    bool  monotone = true;
    float prev = 0.f;
    int   in_glue_range = 0;
    const int steps = 1000;
    for (int i = 0; i <= steps; i++)
    {
        const float a  = 0.95f * float(i) / float(steps);
        const float rt = RatioFromAmount(a);
        if (rt < prev - 1e-6f) monotone = false;
        prev = rt;
        if (rt >= 1.2f && rt <= 3.0f) in_glue_range++;
    }
    const double frac = double(in_glue_range) / double(steps + 1);

    r.Check(monotone, "monotone across the whole travel");
    r.Check(std::fabs(RatioFromAmount(0.f) - 1.f) < 1e-6
            && std::fabs(RatioFromAmount(0.95f) - 20.f) < 1e-3,
            "endpoints are exactly 1:1 and 20:1",
            Fmt("%.4f .. %.4f", double(RatioFromAmount(0.f)), double(RatioFromAmount(0.95f))));
    r.Check(frac > 0.45, "1.2:1 - 3:1 spans most of the lower half",
            Fmt("%.1f %% of travel (was ~9 %% on the old linear taper)", frac * 100.0));
    r.Info("landmarks", Fmt("a=0.50 -> %.2f:1,  a=0.667 -> %.2f:1",
                            double(RatioFromAmount(0.5f)), double(RatioFromAmount(0.667f))));
}

/* ── Test 4: parameter changes reach the audio path ───────────────────── */

void TestParameterChangeGetsThrough(Report& r)
{
    r.Section("Preset recall — one push of new params takes effect");

    // Same signal, two threshold settings, one SetComp call each.
    auto run = [](float thr) {
        ResetChain();
        mastering_dsp::SetComp(MakeParams(mastering_dsp::kCompPrecise, thr, 8.f,
                                          5.f, 100.f, 0.f, 1.f));
        float in_l[kBlock], in_r[kBlock], o_l[kBlock], o_r[kBlock];
        const float* ins[2]  = {in_l, in_r};
        float*       outs[2] = {o_l, o_r};
        Sine sig(1000.0, -6.0);
        std::vector<float> out;
        for (int b = 0; b < 2000; b++)
        {
            for (size_t i = 0; i < kBlock; i++) sig.Next(in_l[i], in_r[i]);
            mastering_dsp::Process(ins, outs, kBlock);
            if (b > 1000) for (size_t i = 0; i < kBlock; i++) out.push_back(o_l[i]);
        }
        return RmsDb(out);
    };

    const double hi_thresh = run(-6.f);    // barely compressing
    const double lo_thresh = run(-30.f);   // compressing hard
    r.Check(hi_thresh - lo_thresh > 6.0,
            "a single SetComp with a lower threshold audibly compresses more",
            Fmt("%.2f dBFS vs %.2f dBFS", hi_thresh, lo_thresh));
}

/* ── Test 5: bypass A/B is level-matched ──────────────────────────────── */

void TestBypassLevelMatch(Report& r)
{
    r.Section("Bypass A/B — auto-makeup keeps the comparison honest");

    for (uint8_t ch : {mastering_dsp::kCompAdaptive, mastering_dsp::kCompGlue})
    {
        auto run = [&](bool bypass) {
            ResetChain();
            mastering_dsp::SetComp(MakeParams(ch, -20.f, 2.f, 10.f, 200.f, 0.f, 1.f, bypass));
            float in_l[kBlock], in_r[kBlock], o_l[kBlock], o_r[kBlock];
            const float* ins[2]  = {in_l, in_r};
            float*       outs[2] = {o_l, o_r};
            Noise n(-14.0, 0xABCDEFu);
            std::vector<float> out;
            for (int b = 0; b < 4000; b++)
            {
                for (size_t i = 0; i < kBlock; i++) n.Next(in_l[i], in_r[i]);
                mastering_dsp::Process(ins, outs, kBlock);
                if (b > 1000) for (size_t i = 0; i < kBlock; i++) out.push_back(o_l[i]);
            }
            return RmsDb(out);
        };
        const double d = std::fabs(run(false) - run(true));
        r.Check(d < 2.5, std::string(CharName(ch)) + ": bypass toggles within 2.5 dB",
                Fmt("difference %.2f dB", d));
    }
}

/* ── Test 6: chain latency guard ──────────────────────────────────────── */

void TestChainLatency(Report& r)
{
    r.Section("Chain latency — the whole signal path, measured not asserted");

    ResetChain();   // flushes the limiter delay line to all zeros

    float in_l[kBlock], in_r[kBlock], o_l[kBlock], o_r[kBlock];
    const float* ins[2]  = {in_l, in_r};
    float*       outs[2] = {o_l, o_r};

    int  found = -1;
    int  n     = 0;
    for (int b = 0; b < 20 && found < 0; b++)
    {
        for (size_t i = 0; i < kBlock; i++)
        {
            in_l[i] = (n == 0) ? 0.5f : 0.f;
            in_r[i] = in_l[i];
            n++;
        }
        mastering_dsp::Process(ins, outs, kBlock);
        for (size_t i = 0; i < kBlock; i++)
            if (found < 0 && std::fabs(o_l[i]) > 1e-4f) found = int(b * kBlock + i);
    }

    // 63 samples: 48 of limiter lookahead plus 15 for the saturator's 2x
    // oversampling pair. The developer guide's latency budget caps the whole
    // chain at 64 samples (~1.3 ms) and earmarked the spare 16 for exactly this
    // stage, so the chain is now one sample under its ceiling.
    //
    // The compressor still spends none of it, and cannot — its bypass takes the
    // same-sample dry signal, so a delay line there would make bypass a click.
    // The saturator can because its bypass takes a *matched-delay* dry, which
    // is why this number does not depend on any bypass state. If it moves,
    // that budget is what to check it against.
    r.Check(found == 63, "impulse emerges at sample 63 (limiter 48 + saturator 15)",
            Fmt("measured %.0f samples = %.2f ms", double(found), double(found) / 48.0));
    r.Check(found <= 64, "within the 64-sample chain budget",
            Fmt("%.0f of 64 samples used", double(found)));
}

} // namespace

int main()
{
    std::printf("\n\033[1mCompressor control-path harness\033[0m  —  real "
                "SetComp/Process, %zu-sample blocks at %.0f Hz\n", kBlock, double(kFs));

    Report r;
    TestJitterRejection(r);
    TestCharacterSwitch(r);
    TestRatioTaper(r);
    TestParameterChangeGetsThrough(r);
    TestBypassLevelMatch(r);
    TestChainLatency(r);
    return r.Finish();
}
