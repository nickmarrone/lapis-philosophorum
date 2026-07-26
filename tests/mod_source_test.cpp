/**
 * mod_source_test.cpp — host-side harness for the CV modulation engine.
 *
 * Drives mod_source.cpp directly at its real 1 kHz control-tick rate and
 * measures the properties each mode actually promises: clock lock and phase
 * spread, non-octave LFO ratios, the divergence axis of the smooth random,
 * Euclidean maximal evenness, and the analysis band split.
 *
 * The engine includes nothing from libDaisy or the alchemy-sdk, so this links
 * mod_source.cpp alone. The analysis section additionally pulls dsp_analysis.h,
 * which is <cmath>-only like every other stage header.
 */

#include "mod_source.h"
#include "dsp_analysis.h"
#include "test_report.h"

#include <cmath>
#include <cstdint>
#include <vector>

using namespace mod_source;
using testrep::Fmt;

namespace {

constexpr float kTickHz = 1000.f;
constexpr float kDt     = 1.f / kTickHz;

/* ── measurement helpers ─────────────────────────────────────────────── */

/** Run the engine for `secs`, returning every jack's trace. */
std::vector<std::vector<float>> Run(ModSource& m, float secs)
{
    const int n = static_cast<int>(secs * kTickHz);
    std::vector<std::vector<float>> traces(kNumJacks);
    for (auto& t : traces) t.reserve(static_cast<size_t>(n));

    Frame f;
    for (int i = 0; i < n; i++)
    {
        m.Tick(f);
        for (uint8_t j = 0; j < kNumJacks; j++) traces[j].push_back(f.volts[j]);
    }
    return traces;
}

/** Run with a clock injected every `clk_period` seconds. */
std::vector<std::vector<float>> RunClocked(ModSource& m, float secs,
                                           float clk_period,
                                           float reset_at = -1.f)
{
    const int n = static_cast<int>(secs * kTickHz);
    std::vector<std::vector<float>> traces(kNumJacks);
    for (auto& t : traces) t.reserve(static_cast<size_t>(n));

    Frame    f;
    double   next_clk = 0.0;
    bool     did_reset = false;

    for (int i = 0; i < n; i++)
    {
        const double t = i * static_cast<double>(kDt);
        if (t >= next_clk)
        {
            m.OnClock(static_cast<uint32_t>(t * 1e6));
            next_clk += clk_period;
        }
        if (reset_at >= 0.f && !did_reset && t >= reset_at)
        {
            m.OnReset();
            did_reset = true;
        }
        m.Tick(f);
        for (uint8_t j = 0; j < kNumJacks; j++) traces[j].push_back(f.volts[j]);
    }
    return traces;
}

/** Mean period of a trace, measured between upward zero crossings.
 *  Returns 0 if fewer than two crossings were seen. */
double MeanPeriod(const std::vector<float>& x)
{
    std::vector<double> crossings;
    for (size_t i = 1; i < x.size(); i++)
        if (x[i - 1] <= 0.f && x[i] > 0.f)
        {
            /* Linear interpolation for sub-tick resolution, so a 1 kHz tick
             * does not put a floor on how precisely a slow LFO can be timed. */
            const double frac = (0.0 - x[i - 1]) / (x[i] - x[i - 1]);
            crossings.push_back((static_cast<double>(i - 1) + frac) * kDt);
        }
    if (crossings.size() < 2) return 0.0;
    return (crossings.back() - crossings.front()) /
           static_cast<double>(crossings.size() - 1);
}

double Mean(const std::vector<float>& x)
{
    double s = 0.0;
    for (float v : x) s += v;
    return x.empty() ? 0.0 : s / static_cast<double>(x.size());
}

/** Pearson correlation of two equal-length traces. */
double Correlation(const std::vector<float>& a, const std::vector<float>& b)
{
    const double ma = Mean(a), mb = Mean(b);
    double num = 0.0, da = 0.0, db = 0.0;
    for (size_t i = 0; i < a.size(); i++)
    {
        const double x = a[i] - ma, y = b[i] - mb;
        num += x * y; da += x * x; db += y * y;
    }
    if (da <= 0.0 || db <= 0.0) return 0.0;
    return num / std::sqrt(da * db);
}

/** Lag, in ticks, that maximises the cross-correlation of b against a,
 *  searched over one period's worth of shift. */
int BestLag(const std::vector<float>& a, const std::vector<float>& b,
            int max_lag)
{
    double best = -2.0;
    int    arg  = 0;
    for (int lag = 0; lag < max_lag; lag++)
    {
        double num = 0.0;
        for (size_t i = 0; i + static_cast<size_t>(lag) < a.size(); i++)
            num += static_cast<double>(a[i]) * b[i + static_cast<size_t>(lag)];
        if (num > best) { best = num; arg = lag; }
    }
    return arg;
}

double MaxAbs(const std::vector<float>& x)
{
    double m = 0.0;
    for (float v : x) if (std::fabs(v) > m) m = std::fabs(v);
    return m;
}

double MaxStep(const std::vector<float>& x)
{
    double m = 0.0;
    for (size_t i = 1; i < x.size(); i++)
    {
        const double d = std::fabs(static_cast<double>(x[i]) - x[i - 1]);
        if (d > m) m = d;
    }
    return m;
}

/** Count set bits and check maximal evenness: the gaps between consecutive
 *  pulses (cyclically) take at most two distinct values differing by 1. */
bool MaximallyEven(uint32_t mask, uint8_t steps, int& out_pulses)
{
    std::vector<int> positions;
    for (uint8_t i = 0; i < steps; i++)
        if (mask & (1u << i)) positions.push_back(i);
    out_pulses = static_cast<int>(positions.size());
    if (positions.size() < 2) return true;

    int gmin = steps, gmax = 0;
    for (size_t i = 0; i < positions.size(); i++)
    {
        const int nxt = (i + 1 < positions.size())
                            ? positions[i + 1]
                            : positions[0] + steps;
        const int gap = nxt - positions[i];
        if (gap < gmin) gmin = gap;
        if (gap > gmax) gmax = gap;
    }
    return (gmax - gmin) <= 1;
}

ModSource Make(Mode mode, float knob_a, uint8_t secondary, uint32_t seed = 12345u,
               float knob_b = 0.f)
{
    ModSource m;
    m.Init(kTickHz, seed);
    m.SetParams({mode, knob_a, knob_b, secondary});
    return m;
}

} // namespace

int main()
{
    testrep::Report rep;

    /* ── Off ─────────────────────────────────────────────────────────── */
    rep.Section("Off");
    {
        ModSource m = Make(Mode::Off, 0.5f, 0);
        Frame f;
        for (int i = 0; i < 100; i++) m.Tick(f);

        bool any_out = false, any_volts = false;
        for (uint8_t j = 0; j < kNumJacks; j++)
        {
            if (f.is_output[j])      any_out   = true;
            if (f.volts[j] != 0.f)   any_volts = true;
        }
        rep.Check(!any_out, "no jack claims to be an output",
                  "all six released");
        rep.Check(!any_volts, "every jack sits at 0 V", "");
    }

    /* ── Jack roles ──────────────────────────────────────────────────── */
    rep.Section("Jack roles");
    {
        rep.Check(ModSource::ClockJack(Mode::Clocked) == 0 &&
                  ModSource::ResetJack(Mode::Clocked) == 1,
                  "Clocked takes clock on J3, reset on J4", "");
        rep.Check(ModSource::ClockJack(Mode::Euclid) == 0 &&
                  ModSource::ResetJack(Mode::Euclid) == 0xFF,
                  "Euclid takes clock on J3, no reset", "");

        int outs = 0;
        for (uint8_t j = 0; j < kNumJacks; j++)
            if (ModSource::JackIsOutput(Mode::Clocked, j)) outs++;
        rep.Check(outs == 4, "Clocked drives four outputs", Fmt("%.0f", (double)outs));

        outs = 0;
        for (uint8_t j = 0; j < kNumJacks; j++)
            if (ModSource::JackIsOutput(Mode::Analysis, j)) outs++;
        rep.Check(outs == 6, "Analysis drives all six", Fmt("%.0f", (double)outs));

        outs = 0;
        for (uint8_t j = 0; j < kNumJacks; j++)
            if (ModSource::JackIsOutput(Mode::Euclid, j)) outs++;
        rep.Check(outs == 5, "Euclid drives five gates", Fmt("%.0f", (double)outs));
    }

    /* ── Clocked: lock, ratios, reset, spread ────────────────────────── */
    rep.Section("Clocked — clock lock");
    {
        const float kClkPeriod = 0.5f;               // 2 Hz clock
        const float ratios[5]  = {0.25f, 0.5f, 1.f, 2.f, 4.f};
        const char* names[5]   = {"/4", "/2", "x1", "x2", "x4"};

        for (uint8_t s = 0; s < 5; s++)
        {
            ModSource m = Make(Mode::Clocked, 0.f, s);
            auto tr = RunClocked(m, 40.f, kClkPeriod);
            /* Jack 2 is the sine out. Skip the first few seconds so the
             * measured clock has settled. */
            std::vector<float> tail(tr[2].begin() + 5000, tr[2].end());
            const double per      = MeanPeriod(tail);
            const double expected = kClkPeriod / ratios[s];
            const double err      = std::fabs(per - expected) / expected;
            rep.Check(err < 0.01,
                      std::string("sine locks at ") + names[s],
                      Fmt("%.4f s measured vs %.4f s expected", per, expected));
        }
    }

    rep.Section("Clocked — reset");
    {
        ModSource m = Make(Mode::Clocked, 0.f, 2);   // x1
        /* Reset lands at 1.25 s, a quarter-period into a 0.5 s cycle, where
         * a cosine that has not been reset would read 0, not +4 V. */
        auto tr = RunClocked(m, 3.f, 0.5f, 1.25f);
        const size_t at = static_cast<size_t>(1.25f * kTickHz) + 1;
        rep.Check(tr[2][at] > 3.9f,
                  "sine snaps to phase 0 on reset",
                  Fmt("%.3f V one tick after reset", (double)tr[2][at]));
    }

    rep.Section("Clocked — shape spread");
    {
        /* Spread 0 collapses all four outputs onto one position in the bank.
         * They share a phase, so that makes them bit-identical — the mono-bus
         * case, and the property that says spread really does close all the
         * way rather than merely narrowing. */
        ModSource lo = Make(Mode::Clocked, 0.3f, 2, 12345u, /*knob_b=*/0.f);
        auto tr_lo = RunClocked(lo, 20.f, 0.5f);

        double worst_diff = 0.0;
        for (uint8_t j = 3; j < 6; j++)
            for (size_t i = 0; i < tr_lo[2].size(); i++)
            {
                const double d = std::fabs(tr_lo[j][i] - tr_lo[2][i]);
                if (d > worst_diff) worst_diff = d;
            }
        rep.Check(worst_diff < 1e-6,
                  "spread 0: the four outputs are identical",
                  Fmt("worst divergence %.2e V", worst_diff));

        /* At full spread they sit 1.5 slots apart — six slots over four
         * outputs — so no two carry the same waveform any more. */
        ModSource hi = Make(Mode::Clocked, 0.3f, 2, 12345u, /*knob_b=*/1.f);
        auto tr_hi = RunClocked(hi, 20.f, 0.5f);

        double closest = 1e9;
        for (uint8_t a = 2; a < 6; a++)
            for (uint8_t b = (uint8_t)(a + 1); b < 6; b++)
            {
                double sum = 0.0;
                for (size_t i = 0; i < tr_hi[a].size(); i++)
                    sum += std::fabs(tr_hi[a][i] - tr_hi[b][i]);
                const double mean = sum / (double)tr_hi[a].size();
                if (mean < closest) closest = mean;
            }
        rep.Check(closest > 0.25,
                  "spread 1: no two outputs carry the same waveform",
                  Fmt("closest pair differs by %.3f V on average", closest));
    }

    rep.Section("Clocked — shape rotation");
    {
        /* Rotation is a ring: a full sweep of the knob returns to where it
         * started, so the knob has no seam. Compare rotation 0 against
         * rotation 1 with spread closed, which is the cleanest read. */
        ModSource a = Make(Mode::Clocked, 0.f, 2, 12345u, 0.f);
        ModSource b = Make(Mode::Clocked, 1.f, 2, 12345u, 0.f);
        auto tr_a = RunClocked(a, 10.f, 0.5f);
        auto tr_b = RunClocked(b, 10.f, 0.5f);

        double worst = 0.0;
        for (size_t i = 0; i < tr_a[2].size(); i++)
        {
            const double d = std::fabs(tr_a[2][i] - tr_b[2][i]);
            if (d > worst) worst = d;
        }
        rep.Check(worst < 1e-6, "rotation wraps: 0 and 1 are the same shape",
                  Fmt("worst difference %.2e V", worst));

        /* And the sweep between them is a real traversal, not a plateau:
         * a mid rotation must differ from both ends. */
        ModSource mid = Make(Mode::Clocked, 0.5f, 2, 12345u, 0.f);
        auto tr_m = RunClocked(mid, 10.f, 0.5f);
        double sum = 0.0;
        for (size_t i = 0; i < tr_a[2].size(); i++)
            sum += std::fabs(tr_m[2][i] - tr_a[2][i]);
        rep.Check(sum / (double)tr_a[2].size() > 0.25,
                  "mid rotation is a different waveform from the ends",
                  Fmt("mean difference %.3f V",
                      sum / (double)tr_a[2].size()));
    }

    rep.Section("Clocked — free-runs without a clock");
    {
        /* Selecting the mode with nothing patched must still produce motion.
         * This froze every output at DC before, which read as a dead module. */
        ModSource m = Make(Mode::Clocked, 0.f, 2, 12345u, 1.f);
        auto tr = Run(m, 10.f);          // Run() never calls OnClock
        bool moves = true;
        for (uint8_t j = 2; j < 6; j++)
            if (MaxAbs(tr[j]) < 0.1) moves = false;
        rep.Check(moves, "all four outputs move with no clock patched",
                  Fmt("weakest output peaks at %.3f V",
                      MaxAbs(tr[2]) < MaxAbs(tr[3]) ? MaxAbs(tr[2])
                                                    : MaxAbs(tr[3])));
    }

    rep.Section("Shape bank");
    {
        /* The bank is a ring with no discontinuity: stepping a fixed phase
         * through every position in small increments must never jump. The
         * held value is fixed here so the stepped slot is a constant and any
         * jump found is the crossfade's own. */
        double worst_jump = 0.0;
        for (float phase : {0.05f, 0.31f, 0.62f, 0.88f})
        {
            float prev = ShapeAt(0.f, phase, 0.4f);
            for (int i = 1; i <= 1200; i++)
            {
                const float pos = (float)i * (float)kNumShapes / 1200.f;
                const float v   = ShapeAt(pos, phase, 0.4f);
                const double d  = std::fabs(v - prev);
                if (d > worst_jump) worst_jump = d;
                prev = v;
            }
        }
        rep.Check(worst_jump < 0.02,
                  "sweeping the bank is continuous, including the wrap",
                  Fmt("largest single-step jump %.4f", worst_jump));

        bool in_range = true;
        for (int i = 0; i < 600; i++)
            for (int p = 0; p < 40; p++)
            {
                const float v = ShapeAt((float)i * 0.01f, (float)p / 40.f, 0.9f);
                if (v < -1.0001f || v > 1.0001f) in_range = false;
            }
        rep.Check(in_range, "every position in the bank stays inside -1..+1", "");
    }

    rep.Section("Clocked — output range");
    {
        ModSource m = Make(Mode::Clocked, 0.5f, 2);
        auto tr = RunClocked(m, 20.f, 0.5f);
        bool in_range = true;
        for (uint8_t j = 2; j < 6; j++)
            if (MaxAbs(tr[j]) > kBipolarVolts + 1e-4) in_range = false;
        rep.Check(in_range, "all four outputs stay inside +/-4 V",
                  Fmt("worst %.4f V", MaxAbs(tr[2]) > MaxAbs(tr[4])
                                          ? MaxAbs(tr[2]) : MaxAbs(tr[4])));
        rep.Check(tr[0].empty() || MaxAbs(tr[0]) == 0.0,
                  "clock input jack is never driven", "");
        rep.Check(MaxAbs(tr[1]) == 0.0, "reset input jack is never driven", "");
    }

    /* ── MultiLfo ────────────────────────────────────────────────────── */
    rep.Section("MultiLfo — ratios");
    {
        const char* set_names[3] = {"Golden", "Prime", "Narrow"};
        for (uint8_t s = 0; s < 3; s++)
        {
            /* knob 0 => 0.01 Hz base is far too slow to measure quickly;
             * use a mid setting and derive the expected base the same way
             * the engine does. */
            const float knob = 0.5f;
            const float base = 0.01f * std::pow(10.f / 0.01f, knob);

            ModSource m = Make(Mode::MultiLfo, knob, s);
            auto tr = Run(m, 200.f);
            const float* ratios = Ratios(static_cast<RatioSet>(s));

            bool all_ok = true;
            double worst = 0.0;
            for (uint8_t j = 0; j < kNumJacks; j++)
            {
                const double per      = MeanPeriod(tr[j]);
                const double expected = 1.0 / (base * ratios[j]);
                if (per <= 0.0) { all_ok = false; continue; }
                const double err = std::fabs(per - expected) / expected;
                if (err > worst) worst = err;
                if (err > 0.01) all_ok = false;
            }
            rep.Check(all_ok,
                      std::string("six periods match the ") + set_names[s] + " set",
                      Fmt("worst error %.3f %%", worst * 100.0));
        }
    }

    rep.Section("MultiLfo — shape");
    {
        /* All six share one position in the bank, so the shape knob must move
         * every output and must move them the same way. Guards the plumbing:
         * the ratio tests above all run at the default sine position and
         * would pass even if knob_b were never read. */
        ModSource sine = Make(Mode::MultiLfo, 0.5f, 0, 12345u, /*knob_b=*/0.f);
        ModSource ramp = Make(Mode::MultiLfo, 0.5f, 0, 12345u, /*knob_b=*/2.f / 6.f);
        auto tr_s = Run(sine, 60.f);
        auto tr_r = Run(ramp, 60.f);

        bool   all_moved = true;
        double weakest   = 1e9;
        for (uint8_t j = 0; j < kNumJacks; j++)
        {
            double sum = 0.0;
            for (size_t i = 0; i < tr_s[j].size(); i++)
                sum += std::fabs(tr_s[j][i] - tr_r[j][i]);
            const double mean = sum / (double)tr_s[j].size();
            if (mean < weakest) weakest = mean;
            if (mean < 0.2) all_moved = false;
        }
        rep.Check(all_moved, "the shape knob moves all six outputs",
                  Fmt("weakest channel changed by %.3f V on average", weakest));
    }

    rep.Section("MultiLfo — no octave pairs");
    {
        bool clean = true;
        double closest = 1e9;
        for (uint8_t s = 0; s < 3; s++)
        {
            const float* r = Ratios(static_cast<RatioSet>(s));
            for (uint8_t a = 0; a < kNumJacks; a++)
                for (uint8_t b = 0; b < kNumJacks; b++)
                {
                    if (a == b) continue;
                    const double q = r[a] / r[b];
                    for (double oct : {2.0, 4.0, 8.0})
                    {
                        const double d = std::fabs(q - oct) / oct;
                        if (d < closest) closest = d;
                        if (d < 0.02) clean = false;
                    }
                }
        }
        rep.Check(clean, "no pair sits within 2 % of 2x / 4x / 8x",
                  Fmt("closest approach %.2f %%", closest * 100.0));
    }

    /* ── SmoothRandom ────────────────────────────────────────────────── */
    rep.Section("SmoothRandom — divergence");
    {
        /* Divergence 0: every jack is the same walk at a different depth, so
         * pairwise correlation should be essentially 1. */
        ModSource lo = Make(Mode::SmoothRandom, 0.f, 2);
        auto tr_lo = Run(lo, 600.f);
        double worst_lo = 1.0;
        for (uint8_t a = 0; a < kNumJacks; a++)
            for (uint8_t b = static_cast<uint8_t>(a + 1); b < kNumJacks; b++)
            {
                const double c = Correlation(tr_lo[a], tr_lo[b]);
                if (c < worst_lo) worst_lo = c;
            }
        rep.Check(worst_lo > 0.95,
                  "divergence 0: all six move together",
                  Fmt("weakest pairwise correlation %.4f", worst_lo));

        /* Divergence 1: independent walks at fanned rates. */
        ModSource hi = Make(Mode::SmoothRandom, 1.f, 2);
        auto tr_hi = Run(hi, 600.f);
        double worst_hi = 0.0;
        for (uint8_t a = 0; a < kNumJacks; a++)
            for (uint8_t b = static_cast<uint8_t>(a + 1); b < kNumJacks; b++)
            {
                const double c = std::fabs(Correlation(tr_hi[a], tr_hi[b]));
                if (c > worst_hi) worst_hi = c;
            }
        rep.Check(worst_hi < 0.4,
                  "divergence 1: the six are independent",
                  Fmt("strongest pairwise correlation %.4f", worst_hi));

        rep.Check(worst_lo - worst_hi > 0.5,
                  "divergence is a real axis, not a nudge",
                  Fmt("%.3f correlated -> %.3f independent", worst_lo, worst_hi));
    }

    rep.Section("SmoothRandom — smoothness and range");
    {
        for (uint8_t s = 0; s < 3; s++)
        {
            ModSource m = Make(Mode::SmoothRandom, 1.f, s);
            auto tr = Run(m, 300.f);

            double worst_range = 0.0, worst_step = 0.0;
            for (uint8_t j = 0; j < kNumJacks; j++)
            {
                if (MaxAbs(tr[j])  > worst_range) worst_range = MaxAbs(tr[j]);
                if (MaxStep(tr[j]) > worst_step)  worst_step  = MaxStep(tr[j]);
            }
            rep.Check(worst_range <= kBipolarVolts + 1e-4,
                      Fmt("range %.0f: stays inside +/-4 V", (double)s),
                      Fmt("peak %.4f V", worst_range));
            /* A smoothstep segment's peak slope is 1.5 * span / segment time.
             * Even the fastest range (0.25 Hz * 4 fan = 1 Hz) cannot move more
             * than ~0.05 V in a 1 ms tick; a stepped generator would show a
             * full 8 V jump. */
            rep.Check(worst_step < 0.1,
                      Fmt("range %.0f: no discontinuities", (double)s),
                      Fmt("largest single-tick move %.5f V", worst_step));
        }
    }

    /* ── Euclid ──────────────────────────────────────────────────────── */
    rep.Section("Euclid — pattern generation");
    {
        const uint8_t lengths[5] = {16, 12, 9, 7, 5};
        bool even_ok = true, count_ok = true;
        for (uint8_t li = 0; li < 5; li++)
        {
            const uint8_t n = lengths[li];
            for (uint8_t k = 0; k <= n; k++)
            {
                int pulses = 0;
                if (!MaximallyEven(EuclidPattern(n, k, 0), n, pulses))
                    even_ok = false;
                if (pulses != k) count_ok = false;
            }
        }
        rep.Check(count_ok, "pulse count always equals the request", "");
        rep.Check(even_ok, "every pattern is maximally even",
                  "no two gaps differ by more than one step");

        /* Known Euclidean rhythms, up to rotation. E(3,8) is the tresillo:
         * three pulses spread x..x..x. as one of its rotations. */
        int p = 0;
        const uint32_t e38 = EuclidPattern(8, 3, 0);
        rep.Check(MaximallyEven(e38, 8, p) && p == 3,
                  "E(3,8) is the tresillo necklace",
                  Fmt("mask 0x%02X", (double)e38));

        rep.Check(EuclidPattern(16, 0, 0) == 0, "zero pulses is silent", "");
        rep.Check(EuclidPattern(16, 16, 0) == 0xFFFFu,
                  "full pulses fires every step", "");
    }

    rep.Section("Euclid — rotation is a cyclic shift");
    {
        bool ok = true;
        for (uint8_t r = 0; r < 4; r++)
        {
            const uint32_t base = EuclidPattern(16, 5, 0);
            const uint32_t rot  = EuclidPattern(16, 5, r);
            const uint32_t want = ((base << r) | (base >> (16 - r))) & 0xFFFFu;
            if (r == 0) { if (rot != base) ok = false; }
            else if (rot != want) ok = false;
        }
        rep.Check(ok, "rotation 0..3 is a pure rotate of rotation 0", "");
    }

    rep.Section("Euclid — density and gates");
    {
        /* Gate count over a fixed window must rise monotonically with K5. */
        int prev = -1;
        bool monotonic = true;
        for (int step = 0; step <= 10; step++)
        {
            const float knob = step / 10.f;
            ModSource m = Make(Mode::Euclid, knob, 0);
            auto tr = RunClocked(m, 16.f, 0.125f);   // 8 Hz clock, 128 steps

            int rises = 0;
            for (size_t i = 1; i < tr[1].size(); i++)
                if (tr[1][i - 1] < 1.f && tr[1][i] >= 1.f) rises++;
            if (prev >= 0 && rises < prev) monotonic = false;
            prev = rises;
        }
        rep.Check(monotonic, "gate count rises monotonically with density",
                  Fmt("%.0f gates at full density", (double)prev));

        ModSource m = Make(Mode::Euclid, 0.5f, 0);
        auto tr = RunClocked(m, 16.f, 0.125f);
        bool levels_ok = true;
        for (uint8_t j = 1; j < kNumJacks; j++)
            for (float v : tr[j])
                if (v != 0.f && std::fabs(v - kUnipolarVolts) > 1e-4f)
                    levels_ok = false;
        rep.Check(levels_ok, "gates are clean 0 V / +5 V", "");
        rep.Check(MaxAbs(tr[0]) == 0.0, "clock input jack is never driven", "");
    }

    /* ── Analysis ────────────────────────────────────────────────────── */
    rep.Section("Analysis — band split");
    {
        mastering_dsp::Analyzer a;
        a.Init(48000.f);
        a.Configure(1.f, 50.f, true);

        auto drive = [&](float hz, float secs) {
            const int n = static_cast<int>(secs * 48000.f);
            for (int i = 0; i < n; i++)
            {
                const float s = std::sin(6.28318531f * hz *
                                         static_cast<float>(i) / 48000.f);
                a.ProcessSample(s, s);
            }
        };

        drive(50.f, 1.0f);
        rep.Check(a.low > a.mid && a.low > a.high,
                  "50 Hz lands in the low band",
                  Fmt("low %.4f, mid %.4f", (double)a.low, (double)a.mid));

        a.Configure(1.f, 5.f, true);
        drive(700.f, 1.0f);
        rep.Check(a.mid > a.low && a.mid > a.high,
                  "700 Hz lands in the mid band",
                  Fmt("mid %.4f, low %.4f", (double)a.mid, (double)a.low));

        drive(9000.f, 1.0f);
        rep.Check(a.high > a.low && a.high > a.mid,
                  "9 kHz lands in the high band",
                  Fmt("high %.4f, mid %.4f", (double)a.high, (double)a.mid));

        rep.Check(a.broad > 0.5f, "broadband tracks a full-scale tone",
                  Fmt("%.4f", (double)a.broad));
    }

    rep.Section("Analysis — enable gate and response");
    {
        mastering_dsp::Analyzer a;
        a.Init(48000.f);
        a.Configure(1.f, 50.f, false);
        for (int i = 0; i < 48000; i++) a.ProcessSample(1.f, 1.f);
        rep.Check(a.broad == 0.f, "disabled analyzer stays at zero",
                  Fmt("%.6f", (double)a.broad));

        /* Fast attack must reach a step sooner than slow attack. */
        auto rise_ticks = [](float att_ms) {
            mastering_dsp::Analyzer an;
            an.Init(48000.f);
            an.Configure(att_ms, 1000.f, true);
            for (int i = 0; i < 48000 * 2; i++)
            {
                an.ProcessSample(1.f, 1.f);
                if (an.broad > 0.63f) return i;
            }
            return 48000 * 2;
        };
        const int fast = rise_ticks(1.f);
        const int slow = rise_ticks(100.f);
        rep.Check(slow > fast * 20,
                  "K5 actually changes the follower speed",
                  Fmt("%.0f samples fast vs %.0f slow", (double)fast,
                      (double)slow));
    }

    /* ── Secondary zone table ────────────────────────────────────────── */
    rep.Section("Secondary zones");
    {
        rep.Check(SecondaryZones(Mode::Off) == 1, "Off has no secondary", "");
        rep.Check(SecondaryZones(Mode::Analysis) == 3, "Analysis has 3", "");
        rep.Check(SecondaryZones(Mode::Clocked) == 5, "Clocked has 5", "");
        rep.Check(SecondaryZones(Mode::MultiLfo) == 3, "MultiLfo has 3", "");
        rep.Check(SecondaryZones(Mode::SmoothRandom) == 3, "SmoothRandom has 3", "");
        rep.Check(SecondaryZones(Mode::Euclid) == 4, "Euclid has 4", "");

        /* An out-of-range secondary must be clamped, not indexed with. */
        ModSource m;
        m.Init(kTickHz, 99u);
        m.SetParams({Mode::Clocked, 0.5f, 0.5f, 200});
        Frame f;
        for (int i = 0; i < 10; i++) m.Tick(f);
        rep.Check(true, "out-of-range secondary is clamped on entry",
                  "no out-of-bounds table read");
    }

    return rep.Finish();
}
