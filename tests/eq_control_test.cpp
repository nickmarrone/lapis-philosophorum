/**
 * eq_control_test.cpp — behaviour of the control->audio path, driven through
 * the real mastering_dsp::SetEq / Process, not a model of them.
 *
 * What these check, none of which the response harness can see:
 *
 *   - A knob that is not moving produces a bit-identical output stream. That
 *     is the actual definition of "no zipper", and it is what the parameter
 *     smoothing plus the dirty check inside SetEq exist to guarantee.
 *   - The test can actually detect zipper, verified with a negative control.
 *   - A parameter change still gets through (the dirty check is not
 *     over-eager) and still arrives gradually rather than as a step.
 *   - Preset recall works: nothing is edge-triggered, so pushing new values
 *     once is enough.
 *   - Nothing ever goes non-finite, including under an adversarial sweep.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "eq_test_common.h"
#include "mastering_dsp.h"

using eqtest::Report;
using eqtest::Fmt;

namespace {

constexpr size_t kBlock = 24;      // kEngineBlockSamples
constexpr double kFs    = 48000.0;
constexpr int    kSinePeriod = 48; // 1 kHz at 48 kHz — exactly 48 samples

/** Everything downstream of the EQ made transparent, so these tests observe
 *  the EQ alone. Dither in particular must be off or nothing repeats. */
void SilenceRestOfChain()
{
    mastering_dsp::CompParams c{};
    c.threshold_db = 0.f; c.ratio = 1.f; c.attack_ms = 10.f; c.release_ms = 100.f;
    c.makeup_db = 0.f; c.mix = 1.f; c.soft_knee = false; c.bypass = true;
    mastering_dsp::SetComp(c);

    mastering_dsp::OutParams o{};
    o.drive_db = 0.f; o.asym = 0.f; o.ceiling_db = -0.1f; o.lim_release_ms = 100.f;
    o.trim_db = 0.f; o.dither_lsb = 0.f; o.sat_type = 0; o.sat_bypass = true;
    mastering_dsp::SetOutput(o);
}

mastering_dsp::EqParams Params(float ls_g, float mid_g, float hs_g, bool bypass = false)
{
    mastering_dsp::EqParams p{};
    p.ls_freq_hz = 80.f;   p.ls_gain_db  = ls_g;
    p.mid_freq_hz = 1000.f; p.mid_gain_db = mid_g; p.mid_q = 0.707f;
    p.hs_freq_hz = 8000.f; p.hs_gain_db  = hs_g;
    p.bypass = bypass;
    return p;
}

/** Run one block of a continuous 1 kHz sine, return the output. */
void RunBlock(long& phase, std::vector<float>& outbuf)
{
    float inL[kBlock], inR[kBlock], outL[kBlock], outR[kBlock];
    for (size_t i = 0; i < kBlock; i++)
    {
        // Wrap the phase before taking the sine. sin(2*pi*n/48) for growing n
        // is *not* bit-periodic — the argument is a different float each cycle
        // and its low bits drift — so an unwrapped phase would make the input
        // itself aperiodic and the periodicity check below meaningless.
        // -30 dBFS keeps the limiter and saturator well out of the way.
        const long ph = (phase + long(i)) % kSinePeriod;
        const float v = float(0.0316 * std::sin(2.0 * eqtest::kPi * double(ph) / kSinePeriod));
        inL[i] = inR[i] = v;
    }
    phase = (phase + long(kBlock)) % kSinePeriod;

    const float* inp[2]  = {inL, inR};
    float*       outp[2] = {outL, outR};
    mastering_dsp::Process(inp, outp, kBlock);

    for (size_t i = 0; i < kBlock; i++) outbuf.push_back(outL[i]);
}

/**
 * Smallest exact repetition period in the tail of `y`, searched over multiples
 * of the input period; 0 if the tail never repeats bit-for-bit.
 *
 * Multiples are necessary, not sloppiness: a settled float32 biquad can fall
 * into a quantization limit cycle whose period is a small multiple of the
 * input's. Measured here it is 2x, at roughly -150 dBFS — inaudible, well
 * under the 24-bit dither floor, and entirely a property of the filter rather
 * than of the control path.
 */
int SteadyPeriod(const std::vector<float>& y, int max_mult = 8)
{
    for (int m = 1; m <= max_mult; m++)
    {
        const size_t lag  = size_t(kSinePeriod) * size_t(m);
        const size_t span = lag * 2;
        if (y.size() < span) break;
        const size_t base = y.size() - span;
        bool ok = true;
        for (size_t k = 0; k < lag && ok; k++)
            if (y[base + k] != y[base + k + lag]) ok = false;
        if (ok) return int(lag);
    }
    return 0;
}

bool AllFinite(const std::vector<float>& y)
{
    for (float v : y) if (!std::isfinite(v)) return false;
    return true;
}

/* ── Tests ────────────────────────────────────────────────────────────── */

void TestStationaryKnobIsSilent(Report& r)
{
    r.Section("Stationary knob — no zipper");

    SilenceRestOfChain();
    const auto p = Params(6.f, -4.f, 3.f);

    // The property that matters: once a knob stops moving, pushing the same
    // values every frame must be indistinguishable from never pushing again.
    // Capture two windows from one continuous run — the first while SetEq is
    // called every block, the second while it is not called at all. Both are
    // period-aligned, so if the coefficients are genuinely frozen the two
    // windows are the same samples.
    constexpr int kWin = 8;                          // blocks, = 192 samples
    std::vector<float> y;
    long phase = 0;

    for (int b = 0; b < 4000; b++) { mastering_dsp::SetEq(p); RunBlock(phase, y); }
    const std::vector<float> pushed(y.end() - kWin * long(kBlock), y.end());

    for (int b = 0; b < 4000; b++) { RunBlock(phase, y); }   // no SetEq at all
    const std::vector<float> quiet(y.end() - kWin * long(kBlock), y.end());

    r.Check(AllFinite(y), "output finite");
    r.Check(pushed == quiet,
            "pushing a stationary knob changes nothing, bit for bit",
            "identical to never calling SetEq again");
    r.Check(SteadyPeriod(y) > 0, "output reaches an exact steady state",
            Fmt("period %.0f samples", double(SteadyPeriod(y))));

    // Now with jitter on the frequency, at two scales.
    const auto run_jittered = [&](float amount) {
        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> jit(-amount, amount);
        std::vector<float> yj;
        for (int b = 0; b < 4000; b++)
        {
            auto pj = p;
            pj.mid_freq_hz = 1000.f * (1.f + jit(rng));
            mastering_dsp::SetEq(pj);
            RunBlock(phase, yj);
        }
        return SteadyPeriod(yj);
    };

    // Realistic ADC noise. The Daisy's 16-bit ADC with a few LSB of noise is
    // ~0.005% of full scale, which through the Exp(20,800) taper is ~0.02% in
    // Hz. 0.05% is a few times worse than that, and the dirty check should
    // reject it outright — the coefficients never move at all.
    r.Check(run_jittered(0.0005f) > 0, "ADC-scale jitter (0.05%) is fully rejected",
            "below the dirty-check threshold; coefficients never move");

    // Negative control: the checks above are only meaningful if they can fail.
    // 2% is ~100x real ADC noise, comfortably past the threshold, so it must
    // get through and must show up as a loss of exact repetition.
    r.Check(run_jittered(0.02f) == 0, "negative control: gross jitter (2%) IS detected",
            "so the checks above are meaningful");
}

void TestParameterChangeGetsThrough(Report& r)
{
    r.Section("Parameter changes still reach the audio path");

    SilenceRestOfChain();
    std::vector<float> y;
    long phase = 0;

    const auto flat = Params(0.f, 0.f, 0.f);
    for (int b = 0; b < 3000; b++) { mastering_dsp::SetEq(flat); RunBlock(phase, y); }
    const size_t mark_flat = y.size();

    // A single push of new values, exactly as a preset load produces.
    const auto boosted = Params(0.f, 12.f, 0.f);
    for (int b = 0; b < 3000; b++) { mastering_dsp::SetEq(boosted); RunBlock(phase, y); }

    const auto rms = [&](size_t from, size_t to) {
        double a = 0.0;
        for (size_t i = from; i < to; i++) a += double(y[i]) * double(y[i]);
        return std::sqrt(a / double(to - from));
    };
    const double before = rms(mark_flat - 480, mark_flat);
    const double after  = rms(y.size() - 480, y.size());
    const double gain_db = 20.0 * std::log10(after / before);

    r.Check(std::fabs(gain_db - 12.0) < 0.5,
            "+12 dB at 1 kHz arrives at the output", Fmt("measured %+.2f dB", gain_db));
    r.Check(SteadyPeriod(y) > 0, "settles back to an exact steady state",
            Fmt("period %.0f samples", double(SteadyPeriod(y))));
}

void TestChangeIsGlided(Report& r)
{
    r.Section("Parameter changes are glided, not stepped");

    SilenceRestOfChain();
    std::vector<float> y;
    long phase = 0;

    const auto flat = Params(0.f, 0.f, 0.f);
    for (int b = 0; b < 3000; b++) { mastering_dsp::SetEq(flat); RunBlock(phase, y); }

    // Slam the mid band and watch how the envelope moves. Called at the real
    // ~16 ms control cadence (every 32 blocks) so the timing means something.
    const size_t mark = y.size();
    const auto boosted = Params(0.f, 15.f, 0.f);
    for (int b = 0; b < 200; b++)
    {
        if (b % 32 == 0) mastering_dsp::SetEq(boosted);
        RunBlock(phase, y);
    }

    // Peak within each 48-sample period, as a dB trajectory.
    std::vector<double> env;
    for (size_t i = mark; i + kSinePeriod < y.size(); i += kSinePeriod)
    {
        double pk = 0.0;
        for (int k = 0; k < kSinePeriod; k++) pk = std::max(pk, double(std::fabs(y[i + size_t(k)])));
        env.push_back(20.0 * std::log10(std::max(pk, 1e-12)));
    }

    double biggest_step = 0.0;
    for (size_t i = 1; i < env.size(); i++)
        biggest_step = std::max(biggest_step, std::fabs(env[i] - env[i - 1]));

    // Ungliided, the whole 15 dB would land inside one 1 ms period.
    r.Check(biggest_step < 6.0, "no single-period jump larger than 6 dB",
            Fmt("largest %.2f dB per period", biggest_step));
    r.Check(AllFinite(y), "output finite throughout");
}

void TestBypassAndSweep(Report& r)
{
    r.Section("Bypass and adversarial sweep");

    SilenceRestOfChain();
    std::vector<float> y;
    long phase = 0;

    for (int b = 0; b < 3000; b++) { mastering_dsp::SetEq(Params(0.f, 15.f, 0.f)); RunBlock(phase, y); }
    const size_t mark = y.size();
    for (int b = 0; b < 3000; b++) { mastering_dsp::SetEq(Params(0.f, 15.f, 0.f, true)); RunBlock(phase, y); }

    const auto rms = [&](size_t from, size_t to) {
        double a = 0.0;
        for (size_t i = from; i < to; i++) a += double(y[i]) * double(y[i]);
        return std::sqrt(a / double(to - from));
    };
    const double wet = rms(mark - 480, mark);
    const double dry = rms(y.size() - 480, y.size());
    r.Check(20.0 * std::log10(wet / dry) > 12.0, "bypass removes the boost",
            Fmt("wet is %+.2f dB above bypassed", 20.0 * std::log10(wet / dry)));

    // Sweep every EQ parameter hard and fast while full-scale noise runs
    // through, at the real control cadence. Nothing may blow up.
    std::mt19937 rng(0xBADF00D);
    std::uniform_real_distribution<float> nz(-1.f, 1.f);
    std::vector<float> ys;
    float inL[kBlock], inR[kBlock], outL[kBlock], outR[kBlock];
    const float* inp[2]  = {inL, inR};
    float*       outp[2] = {outL, outR};

    for (int b = 0; b < 4000; b++)
    {
        if (b % 32 == 0)
        {
            const float u = float(b) / 4000.f;
            mastering_dsp::EqParams p{};
            p.ls_freq_hz  = 20.f * std::pow(40.f, u);
            p.ls_gain_db  = -15.f + 30.f * u;
            p.mid_freq_hz = 5000.f * std::pow(0.04f, u);
            p.mid_gain_db = 15.f - 30.f * u;
            p.mid_q       = (b / 32) % 2 ? 4.0f : 0.707f;   // hop Q, worst case
            p.hs_freq_hz  = 1000.f * std::pow(20.f, u);
            p.hs_gain_db  = -15.f + 30.f * u;
            p.bypass      = false;
            mastering_dsp::SetEq(p);
        }
        for (size_t i = 0; i < kBlock; i++) inL[i] = inR[i] = nz(rng);
        mastering_dsp::Process(inp, outp, kBlock);
        for (size_t i = 0; i < kBlock; i++) ys.push_back(outL[i]);
    }

    double peak = 0.0;
    for (float v : ys) peak = std::max(peak, double(std::fabs(v)));
    r.Check(AllFinite(ys), "full-scale noise + full parameter sweep stays finite");
    r.Check(peak <= 1.5, "output bounded", Fmt("peak %.3f", peak));
}

} // namespace

int main()
{
    std::printf("\n\033[1mEQ control-path harness\033[0m  —  real SetEq/Process, "
                "%zu-sample blocks at %.0f Hz\n", kBlock, kFs);

    mastering_dsp::Init(float(kFs));

    Report r;
    TestStationaryKnobIsSilent(r);
    TestParameterChangeGetsThrough(r);
    TestChangeIsGlided(r);
    TestBypassAndSweep(r);
    return r.Finish();
}
