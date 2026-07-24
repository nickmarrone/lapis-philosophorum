/**
 * comp_response_test.cpp — measurement harness for the glue compressor.
 *
 * Drives mastering_dsp::Compressor directly, with no chain around it, so every
 * number here is attributable to this one stage.
 *
 *   ./comp_response_test            measure and assert
 *   ./comp_response_test --golden   regenerate golden/comp_*.csv
 *
 * Two of these tests carry explicit NEGATIVE CONTROLS — a deliberately broken
 * model that the same assertion must reject. A continuity test with no
 * negative control proves nothing: any bound loose enough to pass will also
 * pass the thing it was supposed to catch, and you cannot tell which from the
 * output. The old branching one-pole this design replaced is the natural
 * control, so it is reproduced locally and required to fail.
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "comp_test_common.h"
#include "eq_test_common.h"

using namespace comptest;
using testrep::Fmt;
using testrep::Report;
using mastering_dsp::kCompPrecise;
using mastering_dsp::kCompAdaptive;
using mastering_dsp::kCompGlue;
using mastering_dsp::kCharSpec;

namespace {

constexpr uint8_t kChars[3] = {kCompPrecise, kCompAdaptive, kCompGlue};

/* ── Test 1: static input/output curve ────────────────────────────────── */

void TestStaticCurve(Report& r)
{
    r.Section("Static curve — unity below the knee, 1/ratio above it");

    for (uint8_t ch : kChars)
    {
        const float W = kCharSpec[ch].knee_db;
        double worst_unity = 0.0, worst_slope_err = 0.0;
        bool   monotone = true;

        for (float thr : {-30.f, -20.f, -12.f})
            for (float rat : {1.2f, 2.f, 4.f, 10.f})
            {
                auto c = Fresh(MakeParams(ch, thr, rat, 5.f, 100.f));

                // Well below the knee: no gain reduction at all. Measured as
                // gain reduction rather than as output-equals-input, because
                // Adaptive and Glue apply auto-makeup — their output below the
                // knee is deliberately *not* unity, and conflating the two
                // would make this test fail for the one reason that is correct.
                const double below = thr - W * 0.5f - 6.0;
                worst_unity = std::max(worst_unity, SteadyGrDb(c, below, 1.5));

                // Well above the knee: slope must be 1/ratio.
                auto c2 = Fresh(MakeParams(ch, thr, rat, 5.f, 100.f));
                const double a_in = thr + W * 0.5f + 2.0;
                const double b_in = a_in + 6.0;
                const double a_out = SteadyOutDb(c2, a_in, 1.5);
                auto c3 = Fresh(MakeParams(ch, thr, rat, 5.f, 100.f));
                const double b_out = SteadyOutDb(c3, b_in, 1.5);
                const double slope = (b_out - a_out) / (b_in - a_in);
                worst_slope_err = std::max(worst_slope_err,
                                           std::fabs(slope - 1.0 / rat) * rat);

                // Monotone: more in must never mean less out.
                double prev = -1e9;
                for (double in = -50.0; in <= 0.0; in += 2.0)
                {
                    auto cm = Fresh(MakeParams(ch, thr, rat, 5.f, 100.f));
                    const double o = SteadyOutDb(cm, in, 1.0);
                    if (o < prev - 1e-3) monotone = false;
                    prev = o;
                }
            }

        r.Check(worst_unity < 0.02, std::string(CharName(ch)) + ": no gain reduction below the knee",
                Fmt("worst %.4f dB GR", worst_unity));
        r.Check(worst_slope_err < 0.01, std::string(CharName(ch)) + ": slope matches 1/ratio",
                Fmt("worst %.3f %% of 1/ratio", worst_slope_err * 100.0));
        r.Check(monotone, std::string(CharName(ch)) + ": curve is monotone", "-50..0 dBFS");
    }
}

/* ── Test 2 + 3: the knee ─────────────────────────────────────────────── */

/** The old hard-knee gain computer, kept as the negative control for the
 *  continuity check below. This is what the file used to do when soft_knee
 *  was false, and its curvature at the threshold is a step. */
double HardKneeAtten(double over, double slope)
{
    return over > 0.0 ? over * slope : 0.0;
}

void TestKnee(Report& r)
{
    r.Section("Knee — C1 continuity and width");

    for (uint8_t ch : kChars)
    {
        const double W = kCharSpec[ch].knee_db;
        auto c = Fresh(MakeParams(ch, -20.f, 2.f));
        const double slope = 0.5;   // ratio 2

        // Sample the gain computer itself across and well beyond the knee.
        std::vector<double> curve, hard;
        const double step = 0.25;
        for (double over = -W - 6.0; over <= W + 6.0; over += step)
        {
            curve.push_back(double(c.AttenDb(float(over))));
            hard.push_back(HardKneeAtten(over, slope));
        }

        const double d2      = MaxSecondDiff(curve);
        const double d2_hard = MaxSecondDiff(hard);

        // A C1 quadratic knee of width W and slope m has constant curvature
        // m/W inside the knee, so the second difference is m*step^2/W — about
        // 0.0017 dB at W = 18. The hard knee's is a full m*step = 0.125 dB
        // step at the threshold. Two orders of magnitude apart; 0.02 sits
        // between them with room on both sides.
        r.Check(d2 < 0.02, std::string(CharName(ch)) + ": knee is C1",
                Fmt("max 2nd difference %.5f dB", d2));
        r.Check(d2_hard > 0.02, std::string(CharName(ch))
                    + ": negative control — hard knee IS rejected",
                Fmt("max 2nd difference %.5f dB", d2_hard));

        // Width: the span between where attenuation first departs from zero
        // and where it rejoins the hard-knee asymptote slope*over. Both edges
        // are found to a tolerance scaled by the slope, so the measurement does
        // not depend on the ratio.
        const double eps = 0.002 * slope;
        double lo = -1e9, hi = -1e9;
        for (double over = -W - 6.0; over <= W + 6.0; over += 0.005)
        {
            const double a = c.AttenDb(float(over));
            if (lo < -1e8 && a > eps) lo = over;
            if (lo > -1e8 && hi < -1e8 && over > 0.0
                && std::fabs(a - slope * over) < eps) hi = over;
        }
        const double measured = hi - lo;
        r.Check(std::fabs(measured - W) < 0.1 * W,
                std::string(CharName(ch)) + ": knee width matches the spec",
                Fmt("measured %.2f dB, spec %.1f dB", measured, W));
    }
}

/* ── Test 4: attack and release timing ────────────────────────────────── */

void TestTiming(Report& r)
{
    r.Section("Attack and release timing — Precise, adaptation off");

    for (float atk : {1.f, 10.f, 50.f})
    {
        // Step -40 -> -10 dBFS. Attack is measured with a release slow enough
        // that it cannot contribute.
        auto c = Fresh(MakeParams(kCompPrecise, -20.f, 4.f, atk, 2000.f));
        Sine quiet(1000.0, -40.0);
        float l, r_;
        for (int i = 0; i < int(0.5 * kFs); i++) { quiet.Next(l, r_); c.ProcessSample(l, r_); }

        Sine loud(1000.0, -10.0);
        auto traj = GrTrajectory(c, loud, int(0.6 * kFs), 48);   // 1 ms resolution
        const double t_atk = TimeToFrac(traj, 0.632);

        r.Check(t_atk > 0 && std::fabs(t_atk - atk) < 0.2 * atk + 1.0,
                Fmt("attack %.0f ms reaches 63 %% on time", atk),
                Fmt("measured %.2f ms", t_atk));
    }

    for (float rel : {50.f, 200.f, 800.f})
    {
        // Release with a fast attack, so the decoupled cascade's second pole
        // contributes ~nothing to the 63 % point.
        auto c = Fresh(MakeParams(kCompPrecise, -20.f, 4.f, 1.f, rel));
        Sine loud(1000.0, -10.0);
        float l, r_;
        for (int i = 0; i < int(1.0 * kFs); i++) { loud.Next(l, r_); c.ProcessSample(l, r_); }

        Sine quiet(1000.0, -40.0);
        auto traj = GrTrajectory(c, quiet, int(std::max(3.0, rel * 6.0 / 1000.0) * kFs), 48);
        const double t_rel = TimeToFrac(traj, 0.632);

        r.Check(t_rel > 0 && std::fabs(t_rel - rel) < 0.2 * rel + 1.0,
                Fmt("release %.0f ms reaches 63 %% on time", rel),
                Fmt("measured %.2f ms", t_rel));
    }

    r.Info("why this test catches a sign inversion",
           "swapping fmaxf/fminf swaps these two measurements");
}

/* ── Test 5: envelope ripple ──────────────────────────────────────────── */

/**
 * Steady-state peak-to-peak ripple of the gain envelope, in dB, for a tone at
 * `f_hz`. Both detectors are fed the identical per-sample gain-computer output
 * c[n], so the only variable is the smoothing topology.
 *
 * Ripple is the measurement that matters here, not curvature. The branching
 * one-pole's increment a*(c - gr) vanishes exactly where the coefficient
 * switches, so its trajectory has no kink to find — the defect is that it
 * begins releasing on every dip of the waveform, twice per cycle, so the gain
 * follows the program at the program's own frequency. A gain that moves with
 * the signal is amplitude modulation, and it puts intermodulation sidebands
 * around everything in the mix.
 */
struct RipplePair { double decoupled, branching, mean_decoupled, mean_branching; };

RipplePair MeasureRipple(double f_hz)
{
    auto c = Fresh(MakeParams(kCompPrecise, -20.f, 4.f, 20.f, 200.f));

    const double atk_a = 1.0 - std::exp(-1000.0 / (20.0  * double(kFs)));
    const double rel_a = 1.0 - std::exp(-1000.0 / (200.0 * double(kFs)));
    double gr_branch = 0.0;

    double lo_d = 1e9, hi_d = -1e9, sum_d = 0.0;
    double lo_b = 1e9, hi_b = -1e9, sum_b = 0.0;
    size_t n_meas = 0;

    Sine sig(f_hz, -6.0);
    const int n = int(2.0 * kFs);
    for (int i = 0; i < n; i++)
    {
        float l, rr;
        sig.Next(l, rr);

        // c[n] exactly as the stage computes it. The sidechain filter is a
        // no-op here: Precise's corner is 30 Hz and the tones are all well
        // above it, which the assertion on mean gain reduction confirms.
        float pw = 0.5f * (l * l + rr * rr);
        if (pw < mastering_dsp::kMinDetPow) pw = mastering_dsp::kMinDetPow;
        const float lvl = mastering_dsp::kLog10ScaleP * std::log(pw);
        const double cn = c.AttenDb(lvl + 20.f);      // over = lvl - (-20)

        gr_branch += ((cn > gr_branch) ? atk_a : rel_a) * (cn - gr_branch);
        c.ProcessSample(l, rr);

        if (i > int(1.5 * kFs))     // steady state only
        {
            const double d = -c.gr_db;
            lo_d = std::min(lo_d, d); hi_d = std::max(hi_d, d); sum_d += d;
            lo_b = std::min(lo_b, gr_branch); hi_b = std::max(hi_b, gr_branch);
            sum_b += gr_branch;
            n_meas++;
        }
    }
    return {hi_d - lo_d, hi_b - lo_b,
            sum_d / double(n_meas), sum_b / double(n_meas)};
}

void TestEnvelopeRipple(Report& r)
{
    r.Section("Envelope ripple — the gain must not follow the waveform");

    for (double f : {50.0, 100.0, 200.0})
    {
        const RipplePair p = MeasureRipple(f);
        r.Check(p.decoupled < 0.02, Fmt("%.0f Hz: decoupled envelope is steady", f),
                Fmt("ripple %.4f dB pk-pk", p.decoupled));
        r.Check(p.branching > p.decoupled * 4.0,
                Fmt("%.0f Hz: negative control — branching one-pole ripples", f),
                Fmt("branching %.4f dB vs decoupled %.4f dB", p.branching, p.decoupled));
    }

    // Holding the peak rather than tracking down between peaks also means the
    // detector acts on the actual peak level, so it settles deeper.
    const RipplePair p = MeasureRipple(100.0);
    r.Check(p.mean_decoupled > p.mean_branching + 0.5,
            "and it acts on the peak, not on a drifting average",
            Fmt("%.3f dB GR vs branching %.3f dB", p.mean_decoupled, p.mean_branching));
}

/* ── Test 6: sidechain high-pass ──────────────────────────────────────── */

void TestSidechain(Report& r)
{
    r.Section("Sidechain high-pass — the detector really is deaf to the bass");

    for (uint8_t ch : kChars)
    {
        const double corner = kCharSpec[ch].hpf_hz;

        // Reference: 1 kHz, effectively unfiltered by a <=90 Hz high-pass.
        auto c1 = Fresh(MakeParams(ch, -30.f, 4.f, 5.f, 100.f));
        Sine hi(1000.0, -12.0);
        float l, rr;
        for (int i = 0; i < int(1.5 * kFs); i++) { hi.Next(l, rr); c1.ProcessSample(l, rr); }
        double gr_hi = 0.0;
        for (int i = 0; i < 480; i++) { hi.Next(l, rr); c1.ProcessSample(l, rr); gr_hi += -c1.gr_db; }
        gr_hi /= 480.0;

        // 40 Hz at the same level: the detector should see it attenuated by
        // exactly the prototype's magnitude at 40 Hz, so the gain reduction
        // must be the one the static curve predicts for that lower level.
        auto c2 = Fresh(MakeParams(ch, -30.f, 4.f, 5.f, 100.f));
        Sine lo(40.0, -12.0);
        for (int i = 0; i < int(1.5 * kFs); i++) { lo.Next(l, rr); c2.ProcessSample(l, rr); }
        double gr_lo = 0.0;
        for (int i = 0; i < 480; i++) { lo.Next(l, rr); c2.ProcessSample(l, rr); gr_lo += -c2.gr_db; }
        gr_lo /= 480.0;

        const double atten_db =
            eqtest::LinToDb(std::abs(eqtest::ProtoHighPass(40.0, corner, 0.70710678)));
        auto cp = Fresh(MakeParams(ch, -30.f, 4.f, 5.f, 100.f));
        const double predicted = cp.AttenDb(float(-12.0 + atten_db + 30.0));

        r.Check(std::fabs(gr_lo - predicted) < 0.5,
                std::string(CharName(ch)) + Fmt(": 40 Hz seen through the %.0f Hz corner", corner),
                Fmt("GR %.2f dB, predicted %.2f dB", gr_lo, predicted));
        r.Check(gr_hi - gr_lo > 1.0,
                std::string(CharName(ch)) + ": bass compresses less than midrange",
                Fmt("1 kHz %.2f dB vs 40 Hz %.2f dB GR", gr_hi, gr_lo));
    }

    // The filter itself, against the analog prototype it approximates.
    for (uint8_t ch : kChars)
    {
        const double corner = kCharSpec[ch].hpf_hz;
        const auto co = mastering_dsp::MakeHighPass<double>(float(corner), 0.70710678f, 48000.f);
        double worst = 0.0, at = 0.0;
        for (double f : eqtest::SweepFreqs())
        {
            const double d = std::fabs(
                eqtest::LinToDb(std::abs(eqtest::ResponseAt(co, f, 48000.0)))
                - eqtest::LinToDb(std::abs(eqtest::ProtoHighPass(f, corner, 0.70710678))));
            if (d > worst) { worst = d; at = f; }
        }
        r.Check(worst < 0.01 && eqtest::IsStable(co) && eqtest::IsFinite(co),
                Fmt("%.0f Hz design matches prototype, stable", corner),
                Fmt("worst %.6f dB at %.0f Hz", worst, at));
        // Zeros are pinned as a double zero at z = 1 — exactly zero at DC.
        // IsMinPhase() is deliberately NOT asserted: it demands zeros strictly
        // inside the unit circle, and these sit exactly on it, which is the
        // whole point of pinning them there.
        r.Check(std::fabs(co.b0 + co.b1 + co.b2) < 1e-15,
                Fmt("%.0f Hz has an exact zero at DC", corner),
                Fmt("sum of numerator %.2e", co.b0 + co.b1 + co.b2));
    }
}

/* ── Test 7: power-sum stereo linking ─────────────────────────────────── */

void TestStereoLink(Report& r)
{
    r.Section("Stereo link — power sum, not max");

    // (a) Mono content: identical to what max-linking would have read.
    auto cm = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 100.f));
    const double gr_mono = SteadyGrDb(cm, -12.0, 1.5);

    // (b) One-sided content at the same per-channel level must read exactly
    // 3.01 dB lower, because half the power is missing.
    auto cs = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 100.f));
    Sine sig(1000.0, -12.0);
    float l, rr;
    for (int i = 0; i < int(1.5 * kFs); i++) { sig.Next(l, rr); rr = 0.f; cs.ProcessSample(l, rr); }
    double gr_side = 0.0;
    for (int i = 0; i < 480; i++) { sig.Next(l, rr); rr = 0.f; cs.ProcessSample(l, rr); gr_side += -cs.gr_db; }
    gr_side /= 480.0;

    auto cp = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 100.f));
    const double predicted = cp.AttenDb(float(-12.0 - 3.0103 + 30.0));
    r.Check(std::fabs(gr_side - predicted) < 0.05,
            "hard-panned content reads 3.01 dB lower",
            Fmt("GR %.3f dB, predicted %.3f dB", gr_side, predicted));
    r.Check(gr_mono > gr_side,
            "and therefore ducks the mix less than mono content would",
            Fmt("mono %.3f dB vs one-sided %.3f dB", gr_mono, gr_side));

    // (c) L/R symmetry, and (d) the same gain applied to both channels.
    auto cl = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 100.f));
    auto cr = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 100.f));
    Sine s1(1000.0, -6.0), s2(1000.0, -6.0);
    double worst_sym = 0.0, worst_ratio = 0.0;
    for (int i = 0; i < int(1.0 * kFs); i++)
    {
        float a, b; s1.Next(a, b); b = 0.f;
        float a2, b2; s2.Next(a2, b2); a2 = 0.f;
        float la = a, ra = b, lb = a2, rb = b2;
        cl.ProcessSample(la, ra);
        cr.ProcessSample(lb, rb);
        worst_sym = std::max(worst_sym, std::fabs(double(cl.gr_db) - double(cr.gr_db)));
        // Same gain on both channels: the ratio out/in must match between L and R.
        if (std::fabs(a) > 1e-4 && std::fabs(b) > 1e-4)
            worst_ratio = std::max(worst_ratio, std::fabs(double(la / a) - double(ra / b)));
    }
    r.Check(worst_sym < 1e-9, "L-only and R-only are treated identically",
            Fmt("worst gr difference %.2e dB", worst_sym));
    r.Check(worst_ratio < 1e-6, "one gain applied to both channels — no image shift",
            Fmt("worst L/R gain difference %.2e", worst_ratio));
}

/* ── Test 8: the Glue character's dual time constant ──────────────────── */

void TestDualTimeConstant(Report& r)
{
    r.Section("Glue — release is provably not a single exponential");

    auto c = Fresh(MakeParams(kCompGlue, -30.f, 4.f, 5.f, 200.f));
    Sine loud(1000.0, -6.0);
    float l, rr;
    for (int i = 0; i < int(3.0 * kFs); i++) { loud.Next(l, rr); c.ProcessSample(l, rr); }

    Sine quiet(1000.0, -60.0);
    auto traj = GrTrajectory(c, quiet, int(8.0 * kFs), 240);   // 5 ms resolution
    const double dt = 240.0 / double(kFs);

    const ExpFit one = FitSingleExp(traj, dt);
    const ExpFit two = FitDualExp(traj, dt);

    r.Check(two.residual * 10.0 < one.residual,
            "a dual exponential fits far better than a single one",
            Fmt("residual %.5f vs %.5f", two.residual, one.residual));
    r.Check(two.w_fast > 0.45 && two.w_fast < 0.75,
            "fast reservoir carries the configured share",
            Fmt("w_fast %.3f (spec %.2f)", two.w_fast, 1.0 - kCharSpec[kCompGlue].slow_weight));
    r.Check(two.tau_slow / two.tau_fast > 4.0,
            "the two time constants really are far apart",
            Fmt("tau_fast %.0f ms, tau_slow %.0f ms", two.tau_fast * 1e3, two.tau_slow * 1e3));

    // The same measurement on Precise must find nothing to gain from a second
    // exponential — otherwise the test above is measuring the fitter, not Glue.
    auto cp = Fresh(MakeParams(kCompPrecise, -30.f, 4.f, 5.f, 200.f));
    Sine loud2(1000.0, -6.0);
    for (int i = 0; i < int(3.0 * kFs); i++) { loud2.Next(l, rr); cp.ProcessSample(l, rr); }
    Sine quiet2(1000.0, -60.0);
    auto traj_p = GrTrajectory(cp, quiet2, int(8.0 * kFs), 240);
    const ExpFit one_p = FitSingleExp(traj_p, dt);
    const ExpFit two_p = FitDualExp(traj_p, dt);
    r.Check(two_p.residual * 10.0 > one_p.residual,
            "negative control — Precise IS a single exponential",
            Fmt("residual %.5f vs %.5f", two_p.residual, one_p.residual));
}

/* ── Test 9 + 10: crest factor ────────────────────────────────────────── */

void TestCrest(Report& r)
{
    r.Section("Crest factor — estimator calibration and adaptation");

    // A sine's crest factor is exactly 3.0103 dB. This pins the peak and RMS
    // followers against each other; if either definition drifts, this moves.
    auto c = Fresh(MakeParams(kCompAdaptive, 0.f, 2.f));
    Sine sig(1000.0, -12.0);
    float l, rr;
    for (int i = 0; i < int(2.0 * kFs); i++) { sig.Next(l, rr); c.ProcessSample(l, rr); }
    r.Check(std::fabs(c.crest_sm - 3.0103) < 0.3, "a sine measures 3.01 dB crest",
            Fmt("measured %.3f dB", double(c.crest_sm)));

    auto c2 = Fresh(MakeParams(kCompAdaptive, 0.f, 2.f));
    ImpulseTrain tr(8.0, 0.05, -12.0);
    for (int i = 0; i < int(3.0 * kFs); i++) { tr.Next(l, rr); c2.ProcessSample(l, rr); }
    r.Check(c2.crest_sm > 10.0, "a 5 % duty impulse train measures a high crest",
            Fmt("measured %.2f dB", double(c2.crest_sm)));

    // Adaptation: the coefficients the two signals leave behind must differ in
    // the documented direction — high crest gives a slower attack and a faster
    // release. Comparing coefficients rather than re-measuring settling times
    // isolates the adaptation from the envelope it drives.
    const double atk_lo = c.atk_a, atk_hi = c2.atk_a;
    const double rel_lo = c.rel_a, rel_hi = c2.rel_a;
    r.Check(atk_hi < atk_lo * 0.6, "high crest slows the attack",
            Fmt("coef %.3e vs %.3e", atk_hi, atk_lo));
    r.Check(rel_hi > rel_lo * 1.6, "high crest speeds the release",
            Fmt("coef %.3e vs %.3e", rel_hi, rel_lo));

    // Precise must not adapt at all — its multipliers are all 1.0.
    auto c3 = Fresh(MakeParams(kCompPrecise, 0.f, 2.f));
    auto c4 = Fresh(MakeParams(kCompPrecise, 0.f, 2.f));
    Sine s3(1000.0, -12.0);
    ImpulseTrain t4(8.0, 0.05, -12.0);
    for (int i = 0; i < int(2.0 * kFs); i++) { s3.Next(l, rr); c3.ProcessSample(l, rr); }
    for (int i = 0; i < int(3.0 * kFs); i++) { t4.Next(l, rr); c4.ProcessSample(l, rr); }
    r.Check(std::fabs(c3.atk_a - c4.atk_a) < 1e-12 && std::fabs(c3.rel_a - c4.rel_a) < 1e-12,
            "negative control — Precise does NOT adapt",
            Fmt("atk %.3e vs %.3e", double(c3.atk_a), double(c4.atk_a)));

    /*
     * Transition behaviour, pinned deliberately.
     *
     * The peak and RMS followers decay at different rates (150 ms against
     * 25 ms), so a sustained drop in level leaves the estimate climbing and
     * the release accelerating — the auto-release behaviour documented in
     * dsp_compressor.h. It is asserted here so that it stays a designed
     * property: if the follower time constants are ever retuned, this is the
     * test that says whether the retune kept it.
     */
    auto CoefAfter = [](double loud_ms, double quiet_ms) {
        auto cc = Fresh(MakeParams(kCompAdaptive, -24.f, 4.f, 10.f, 200.f));
        float a, b;
        Sine loud(1000.0, -10.0);
        for (int i = 0; i < int(loud_ms * kFs / 1000.0); i++) { loud.Next(a, b); cc.ProcessSample(a, b); }
        Sine quiet(1000.0, -40.0);
        for (int i = 0; i < int(quiet_ms * kFs / 1000.0); i++) { quiet.Next(a, b); cc.ProcessSample(a, b); }
        return std::pair<double, double>{double(cc.crest_sm), double(cc.rel_a)};
    };
    const auto at_3   = CoefAfter(500.0, 3.0);
    const auto at_750 = CoefAfter(500.0, 750.0);
    const double ms_3   = -1000.0 / (double(kFs) * std::log(1.0 - at_3.second));
    const double ms_750 = -1000.0 / (double(kFs) * std::log(1.0 - at_750.second));

    r.Check(at_3.first < 4.0 && at_750.first > 12.0,
            "a sustained level drop raises the estimate (auto-release)",
            Fmt("%.2f dB at 3 ms -> %.2f dB at 750 ms", at_3.first, at_750.first));
    r.Check(ms_750 < ms_3 * 0.4, "and the release accelerates accordingly",
            Fmt("%.0f ms -> %.0f ms", ms_3, ms_750));

    // An onset must NOT jolt the timing: both followers rise quickly, so the
    // ratio barely moves. This is the half of the behaviour that has to stay
    // quiet, and it is the one a follower retune would break first.
    auto cc = Fresh(MakeParams(kCompAdaptive, -24.f, 4.f, 10.f, 200.f));
    Sine q0(1000.0, -40.0);
    for (int i = 0; i < int(0.5 * kFs); i++) { q0.Next(l, rr); cc.ProcessSample(l, rr); }
    const double before = cc.crest_sm;
    Sine l0(1000.0, -10.0);
    for (int i = 0; i < int(0.003 * kFs); i++) { l0.Next(l, rr); cc.ProcessSample(l, rr); }
    r.Check(std::fabs(double(cc.crest_sm) - before) < 1.0,
            "but a 30 dB onset barely moves it",
            Fmt("%.2f -> %.2f dB", before, double(cc.crest_sm)));
}

/* ── Test 11: auto-makeup ─────────────────────────────────────────────── */

void TestAutoMakeup(Report& r)
{
    r.Section("Auto-makeup — bypass A/B lands at a comparable level");

    for (uint8_t ch : kChars)
    {
        double worst = 0.0;
        std::string worst_at;
        for (float thr : {-24.f, -18.f, -12.f})
            for (float rat : {1.5f, 2.f, 4.f})
            {
                auto wet = Fresh(MakeParams(ch, thr, rat, 10.f, 200.f));
                auto dry = Fresh(MakeParams(ch, thr, rat, 10.f, 200.f, 0.f, 1.f, true));
                Noise nw(-14.0, 0xBEEF01u), nd(-14.0, 0xBEEF01u);
                std::vector<float> vw, vd;
                for (int i = 0; i < int(4.0 * kFs); i++)
                {
                    float a, b; nw.Next(a, b); wet.ProcessSample(a, b);
                    float e, f; nd.Next(e, f); dry.ProcessSample(e, f);
                    if (i > int(1.0 * kFs)) { vw.push_back(a); vd.push_back(e); }
                }
                const double d = std::fabs(RmsDb(vw) - RmsDb(vd));
                if (d > worst) { worst = d; worst_at = Fmt("thr %.0f, ratio %.1f", thr, rat); }
            }

        if (kCharSpec[ch].auto_makeup > 0.f)
            r.Check(worst < 2.5, std::string(CharName(ch)) + ": level-matched within 2.5 dB",
                    Fmt("worst %.2f dB", worst) + "  (" + worst_at + ")");
        else
            r.Info(std::string(CharName(ch)) + ": manual makeup only, not level-matched",
                   Fmt("worst %.2f dB — expected, auto_makeup = 0", worst));
    }
}

/* ── Test 12 + 13: mix law and latency ────────────────────────────────── */

void TestMixAndLatency(Report& r)
{
    r.Section("Mix law and latency");

    // Wet and dry runs share an identical envelope — the detector never sees
    // the output — so the crossfade can be checked against the exact
    // expression the DSP evaluates: dry + m*(wet - dry).
    const float m = 0.35f;
    auto cw = Fresh(MakeParams(kCompGlue, -24.f, 4.f, 10.f, 200.f, 3.f, 1.f));
    auto cd = Fresh(MakeParams(kCompGlue, -24.f, 4.f, 10.f, 200.f, 3.f, 0.f));
    auto cm = Fresh(MakeParams(kCompGlue, -24.f, 4.f, 10.f, 200.f, 3.f, m));

    Noise n1(-12.0, 0x5150u), n2(-12.0, 0x5150u), n3(-12.0, 0x5150u);
    double worst = 0.0;
    for (int i = 0; i < int(2.0 * kFs); i++)
    {
        float a1, b1; n1.Next(a1, b1); cw.ProcessSample(a1, b1);
        float a2, b2; n2.Next(a2, b2); cd.ProcessSample(a2, b2);
        float a3, b3; n3.Next(a3, b3); cm.ProcessSample(a3, b3);
        const float expect = a2 + m * (a1 - a2);
        worst = std::max(worst, std::fabs(double(a3) - double(expect)));
    }
    r.Check(worst < 1e-6, "mix is the linear crossfade dry + m*(wet - dry)",
            Fmt("worst deviation %.2e", worst));

    // Latency: an impulse must come out on the sample it went in on, at full
    // amplitude. The sidechain biquads are on the detector path only, so they
    // contribute neither delay nor phase shift here.
    //
    // Ratio 1:1 so the gain computer contributes nothing — at any real ratio a
    // 0 dBFS impulse is genuinely attenuated within the first sample (a 0.1 ms
    // attack moves 19 % of the way in one sample at 48 kHz), which is correct
    // behaviour but would make this an amplitude test rather than a timing one.
    auto c = Fresh(MakeParams(kCompPrecise, -40.f, 1.f, 0.1f, 100.f));
    float l = 1.f, rr = 1.f;
    c.ProcessSample(l, rr);
    r.Check(std::fabs(double(l) - 1.0) < 1e-6 && std::fabs(double(rr) - 1.0) < 1e-6,
            "impulse emerges on sample 0, unattenuated — zero latency",
            Fmt("out %.6f", double(l)));
}

/* ── Test 14: adversarial ─────────────────────────────────────────────── */

void TestAdversarial(Report& r)
{
    r.Section("Adversarial — every knob moving, characters cycling");

    auto c = Fresh(MakeParams(kCompPrecise, -20.f, 4.f));
    Noise n(0.0, 0xF00Du);                  // full-scale noise
    bool   finite = true, gr_ok = true;
    double peak = 0.0;
    int    frame = 0;

    for (int blk = 0; blk < 4000; blk++)     // 4000 * 24 = 2 s
    {
        if (blk % 32 == 0)                   // ~16 ms control frame
        {
            const float u = float(frame % 97) / 97.f;
            const uint8_t ch = uint8_t((frame / 100) % 3);
            c.Configure(MakeParams(ch,
                                   -40.f + 40.f * u,
                                   1.f + 19.f * u,
                                   0.1f + 99.f * u,
                                   10.f + 1990.f * (1.f - u),
                                   20.f * u,
                                   u,
                                   (frame % 37) == 0), kFs);
            frame++;
        }
        for (int i = 0; i < 24; i++)
        {
            float a, b; n.Next(a, b);
            c.ProcessSample(a, b);
            if (!std::isfinite(a) || !std::isfinite(b)) finite = false;
            if (c.gr_db > 1e-6f) gr_ok = false;
            peak = std::max(peak, std::max(std::fabs(double(a)), std::fabs(double(b))));
        }
    }

    r.Check(finite, "output stays finite under a full parameter sweep");
    r.Check(gr_ok, "gr_db is never positive — the stage never makes gain by itself");
    r.Check(peak < 40.0, "output stays bounded", Fmt("peak %.2f (makeup reaches +20 dB)", peak));
}

/* ── Test 15: golden regression ───────────────────────────────────────── */

const char* kGoldenCurve = "golden/comp_curve.csv";
const char* kGoldenEnv   = "golden/comp_envelope.csv";
const char* kGoldenHpf   = "golden/comp_hpf.csv";

struct CurvePoint { uint8_t ch; float thr, rat; double in_db, out_db; };

/** The golden grid locks the AUDIBLE contract — the steady-state transfer
 *  curve — rather than internal coefficients. An internal refactor that keeps
 *  the curve is free; a change to the curve is not. */
std::vector<CurvePoint> CurveGrid()
{
    std::vector<CurvePoint> v;
    for (uint8_t ch : kChars)
        for (float thr : {-30.f, -20.f, -12.f})
            for (float rat : {1.2f, 2.f, 4.f, 10.f})
                for (double in = -60.0; in <= 0.0001; in += 2.0)
                {
                    auto c = Fresh(MakeParams(ch, thr, rat, 5.f, 100.f));
                    v.push_back({ch, thr, rat, in, SteadyOutDb(c, in, 1.0)});
                }
    return v;
}

/** And this one locks timing, which is the other half of what a compressor
 *  is — a curve regression alone would not notice a detector change. */
std::vector<double> EnvelopeTrace(uint8_t ch)
{
    auto c = Fresh(MakeParams(ch, -24.f, 4.f, 10.f, 200.f));
    Sine quiet(1000.0, -40.0);
    float l, rr;
    for (int i = 0; i < int(0.3 * kFs); i++) { quiet.Next(l, rr); c.ProcessSample(l, rr); }

    std::vector<double> out;
    Sine loud(1000.0, -10.0);
    for (int i = 0; i < int(0.2 * kFs); i++)
    {
        loud.Next(l, rr); c.ProcessSample(l, rr);
        if (i % 48 == 0) out.push_back(-c.gr_db);
    }
    Sine back(1000.0, -40.0);
    for (int i = 0; i < int(3.0 * kFs); i++)
    {
        back.Next(l, rr); c.ProcessSample(l, rr);
        if (i % 48 == 0) out.push_back(-c.gr_db);
    }
    return out;
}

void WriteGolden()
{
    std::FILE* f = std::fopen(kGoldenCurve, "w");
    if (!f) { std::perror(kGoldenCurve); std::exit(1); }
    std::fprintf(f, "character,threshold_db,ratio,in_db,out_db\n");
    const auto grid = CurveGrid();
    for (const auto& p : grid)
        std::fprintf(f, "%u,%.1f,%.4f,%.4f,%.6f\n",
                     unsigned(p.ch), double(p.thr), double(p.rat), p.in_db, p.out_db);
    std::fclose(f);
    std::printf("wrote %s (%zu rows)\n", kGoldenCurve, grid.size());

    f = std::fopen(kGoldenEnv, "w");
    if (!f) { std::perror(kGoldenEnv); std::exit(1); }
    std::fprintf(f, "character,t_ms,gr_db\n");
    size_t rows = 0;
    for (uint8_t ch : kChars)
    {
        const auto tr = EnvelopeTrace(ch);
        for (size_t i = 0; i < tr.size(); i++)
            std::fprintf(f, "%u,%zu,%.6f\n", unsigned(ch), i, tr[i]);
        rows += tr.size();
    }
    std::fclose(f);
    std::printf("wrote %s (%zu rows)\n", kGoldenEnv, rows);

    f = std::fopen(kGoldenHpf, "w");
    if (!f) { std::perror(kGoldenHpf); std::exit(1); }
    std::fprintf(f, "f_hz,b0,b1,b2,a1,a2\n");
    for (uint8_t ch : kChars)
    {
        const auto c = mastering_dsp::MakeHighPass<double>(kCharSpec[ch].hpf_hz, 0.70710678f, 48000.f);
        std::fprintf(f, "%.1f,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                     double(kCharSpec[ch].hpf_hz), c.b0, c.b1, c.b2, c.a1, c.a2);
    }
    std::fclose(f);
    std::printf("wrote %s (3 rows)\n", kGoldenHpf);
}

void TestGolden(Report& r)
{
    r.Section("Golden regression");

    std::FILE* f = std::fopen(kGoldenCurve, "r");
    if (!f)
    {
        r.Info("no golden files", "run `make test-golden` to create them");
        return;
    }
    char line[256];
    if (!std::fgets(line, sizeof line, f)) { std::fclose(f); return; }

    const auto grid = CurveGrid();
    size_t row = 0, differ = 0;
    double worst = 0.0;
    while (std::fgets(line, sizeof line, f) && row < grid.size())
    {
        unsigned ch; double thr, rat, in_db, out_db;
        if (std::sscanf(line, "%u,%lf,%lf,%lf,%lf", &ch, &thr, &rat, &in_db, &out_db) == 5)
        {
            const double d = std::fabs(out_db - grid[row].out_db);
            if (d > 1e-3) differ++;
            worst = std::max(worst, d);
        }
        row++;
    }
    std::fclose(f);
    r.Check(row == grid.size(), "curve row count matches the grid",
            Fmt("%.0f of %.0f", double(row), double(grid.size())));
    r.Check(differ == 0, "static curve unchanged  (1e-3 dB)",
            Fmt("%.0f differ, worst %.2e dB", double(differ), worst));

    f = std::fopen(kGoldenEnv, "r");
    if (!f) { r.Info("no envelope golden", "run `make test-golden`"); return; }
    if (!std::fgets(line, sizeof line, f)) { std::fclose(f); return; }
    std::vector<double> all;
    for (uint8_t ch : kChars) { const auto t = EnvelopeTrace(ch); all.insert(all.end(), t.begin(), t.end()); }
    size_t erow = 0, ediffer = 0;
    double eworst = 0.0;
    while (std::fgets(line, sizeof line, f) && erow < all.size())
    {
        unsigned ch; size_t t; double gr;
        if (std::sscanf(line, "%u,%zu,%lf", &ch, &t, &gr) == 3)
        {
            const double d = std::fabs(gr - all[erow]);
            if (d > 0.01) ediffer++;
            eworst = std::max(eworst, d);
        }
        erow++;
    }
    std::fclose(f);
    r.Check(erow == all.size(), "envelope row count matches",
            Fmt("%.0f of %.0f", double(erow), double(all.size())));
    r.Check(ediffer == 0, "gain-reduction trajectories unchanged  (0.01 dB)",
            Fmt("%.0f differ, worst %.2e dB", double(ediffer), eworst));

    f = std::fopen(kGoldenHpf, "r");
    if (!f) { r.Info("no sidechain golden", "run `make test-golden`"); return; }
    if (!std::fgets(line, sizeof line, f)) { std::fclose(f); return; }
    size_t hrow = 0, hdiffer = 0;
    for (uint8_t ch : kChars)
    {
        if (!std::fgets(line, sizeof line, f)) break;
        double fh, b0, b1, b2, a1, a2;
        if (std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf", &fh, &b0, &b1, &b2, &a1, &a2) == 6)
        {
            const auto c = mastering_dsp::MakeHighPass<double>(kCharSpec[ch].hpf_hz, 0.70710678f, 48000.f);
            const double rel = std::max({std::fabs(b0 - c.b0), std::fabs(b1 - c.b1),
                                         std::fabs(b2 - c.b2), std::fabs(a1 - c.a1),
                                         std::fabs(a2 - c.a2)});
            if (rel > 1e-6) hdiffer++;
        }
        hrow++;
    }
    std::fclose(f);
    r.Check(hrow == 3 && hdiffer == 0, "sidechain coefficients unchanged  (1e-6)",
            Fmt("%.0f rows, %.0f differ", double(hrow), double(hdiffer)));
}

} // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--golden") == 0) { WriteGolden(); return 0; }

    std::printf("\n\033[1mCompressor measurement harness\033[0m  —  fs = %.0f Hz, "
                "3 characters, direct on mastering_dsp::Compressor\n", double(kFs));

    Report r;
    TestStaticCurve(r);
    TestKnee(r);
    TestTiming(r);
    TestEnvelopeRipple(r);
    TestSidechain(r);
    TestStereoLink(r);
    TestDualTimeConstant(r);
    TestCrest(r);
    TestAutoMakeup(r);
    TestMixAndLatency(r);
    TestAdversarial(r);
    TestGolden(r);
    return r.Finish();
}
