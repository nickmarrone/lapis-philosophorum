/**
 * chain_test.cpp — whole-chain integration harness.
 *
 * The seven harnesses that came before this one each drive a single stage, and
 * between them they cover what each stage does very thoroughly. What none of
 * them can see is the seams: the chain's total latency, whether that latency
 * survives every combination of bypasses, whether the ceiling still means
 * anything once 40-odd dB of upstream gain is available, whether the two
 * channels stay in step, and whether a knob — any knob — does something on its
 * way between two settings that it does not do at either of them.
 *
 * Everything here goes through the real mastering_dsp::Set* / Process at the
 * real 24-sample block and the real 16 ms control frame. Nothing is modelled.
 *
 * ── Two instruments do most of the work, and both are self-calibrating ───────
 *
 * A harness that hard-codes "an output step above 1.3e-3 is a click" is really
 * asserting the output level it happened to see when it was written; move a
 * default and it starts lying in one direction or the other. So the thresholds
 * here are derived from the run they are judging:
 *
 *   ENVELOPE, against the knob's own travel.  A knob is sampled at nine points
 *   across its range and the largest steady envelope of those nine becomes the
 *   bar. Then the knob is moved and the envelope during the move must stay
 *   under it. Nine points rather than two because not every knob is monotone —
 *   Asym is loudest in the middle of its travel, and a two-point bar would
 *   convict it of an overshoot that is simply what the knob does.
 *
 *   STEP, against the program's own slew.  The largest sample-to-sample step
 *   during a move is compared to the slew the probe tone produces by itself at
 *   the output level actually measured during that move. This is the criterion
 *   sat_control_test already uses, with the amplitude taken from the run rather
 *   than from the input, so a knob that legitimately makes the signal four
 *   times louder is not convicted of clicking four times harder.
 *
 * ── Sweep and snap are separate cases, deliberately ──────────────────────────
 *
 * Every parameter test runs twice: a 12-control-frame sweep (~190 ms, a brisk
 * hand movement) and a 1-frame snap. The snap is not a synthetic worst case on
 * this hardware — the pots are smoothed at tau = 8 ms and read at 1 kHz, so a
 * value covers 88 % of a jump inside one 16 ms control frame, and preset recall
 * and page-switch pot-catch release both hand the DSP a genuine step. A stage
 * whose easing only holds up under a slow sweep will pass the first and fail
 * the second, which is exactly the distinction worth keeping visible.
 *
 * ── Sections ─────────────────────────────────────────────────────────────────
 *
 *   1. Latency is 75 samples, in all sixteen bypass combinations.
 *   2. Transparency: bypassed is a bit-exact delay, neutral is unity.
 *   3. Stage order: trim cannot defeat the ceiling; dither is not scaled.
 *   4. The ceiling holds end to end, as dBTP, with the limiter really engaged.
 *   5. Nothing goes non-finite under an adversarial parameter fuzz.
 *   6. Stereo integrity, silence, and DC.
 *   7. The chain's applied gain is continuous.
 *   8. Parameter moves stay inside their own envelope and produce no step.
 *   9. Bypass toggles.
 *  10. Long-run stability.
 *  11. The gain-reduction meter's contract.
 */

#include <cstdio>
#include <functional>
#include <string>

#include "chain_test_common.h"

using namespace chaintest;
using namespace mastering_dsp;

namespace {

constexpr double kProbeHz  = 1000.0;
constexpr double kProbeAmp = 0.01;      // -40 dBFS: no stage is near its knee

/* ── Parameter-move support ───────────────────────────────────────────────── */

struct Knob {
    const char*                           name;
    std::function<void(Settings&, float)> set;
    float                                 a, b;
};

/** Steady output envelope with the knob parked at `v`. */
double SteadyEnvelope(const Knob& k, float v, const Settings& base)
{
    Chain    ch;
    Settings s = base;
    k.set(s, v);
    ch.Boot(s);

    std::vector<float> out;
    ch.RunSteady([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, kProbeAmp); },
                 24000, &out);
    return MaxOf(Envelope(out, 48), 4800);      // skip the first 100 ms
}

/** The loudest the knob gets anywhere in its travel — the bar a move has to
 *  stay under. Nine points; see the note on non-monotone knobs at the top. */
double TravelCeiling(const Knob& k, const Settings& base)
{
    double hi = 0.0;
    for (int i = 0; i < 9; i++)
        hi = std::fmax(hi, SteadyEnvelope(k, k.a + (k.b - k.a) * float(i) / 8.f, base));
    return hi;
}

struct MoveResult { double env_peak; double max_step; };

/**
 * Park at `a`, then travel to `b` over `frames` control frames and watch.
 * Measurement starts just before the move so the boot transient is excluded.
 */
MoveResult Move(const Knob& k, const Settings& base, int frames)
{
    Chain    ch;
    Settings s = base;
    k.set(s, k.a);
    ch.Boot(s);

    std::vector<float> out;
    const Knob* kp = &k;
    ch.Run([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, kProbeAmp); },
           [kp, frames](int frame, Settings& st) {
               const int f = frame - 8;         // hold at `a` for 8 frames first
               if (f < 0) return;
               const float u = (f >= frames) ? 1.f : float(f) / float(frames);
               kp->set(st, kp->a + u * (kp->b - kp->a));
           },
           48000, &out);

    return { MaxOf(Envelope(out, 48), 6000), MaxStep(out, 6000) };
}

/**
 * The gain the whole chain applied, sample by sample, in dB.
 *
 * Only meaningful when the chain is a pure gain — every stage that filters,
 * saturates or delays differently from the dry path has to be out of the way,
 * which is why the caller below bypasses everything except the compressor. The
 * probe is a square wave precisely so that |in| is constant: dividing by a sine
 * near its zero crossings would manufacture exactly the discontinuities this is
 * looking for.
 */
std::vector<double> AppliedGainDb(const std::vector<float>& in, const std::vector<float>& out)
{
    std::vector<double> g;
    for (size_t i = 0; i + (size_t)kChainLatency < out.size() && i < in.size(); i++)
    {
        if (std::fabs((double)in[i]) < 1e-6) { g.push_back(g.empty() ? 0.0 : g.back()); continue; }
        g.push_back(20.0 * std::log10(std::fabs((double)out[i + kChainLatency] / (double)in[i]) + 1e-30));
    }
    return g;
}

double MaxAdjacentStep(const std::vector<double>& v, size_t from, size_t to)
{
    double m = 0.0;
    to = std::min(to, v.size());
    for (size_t i = std::max<size_t>(from, 1); i < to; i++)
        m = std::fmax(m, std::fabs(v[i] - v[i - 1]));
    return m;
}

} // namespace

int main()
{
    Report rep;
    std::printf("\n\033[1mWhole-chain integration harness\033[0m"
                "  —  EQ -> Comp -> Sat -> Trim -> Limiter -> Dither at %.0f Hz\n", kFs);

    /* ── 1. Latency, and its independence from bypass state ──────────────── */
    rep.Section("Latency is 75 samples, whatever is bypassed");
    {
        std::vector<float> noise;
        FillNoiseBurst(noise, 24000, 0.03);     // -30 dBFS, band-limited

        int  worst_combo = -1, worst_lag = kChainLatency;
        bool all_ok = true;
        for (int combo = 0; combo < 16; combo++)
        {
            Settings s = Neutral();
            s.eq.bypass      = (combo & 1) != 0;
            s.comp.bypass    = (combo & 2) != 0;
            s.sat.bypass     = (combo & 4) != 0;
            s.out.lim_bypass = (combo & 8) != 0;

            Chain ch;
            ch.Boot(s);
            std::vector<float>        out;
            const std::vector<float>* np = &noise;
            ch.RunSteady([np](long n, float* l, float* r) {
                             *l = *r = (n >= 0 && (size_t)n < np->size()) ? (*np)[(size_t)n] : 0.f;
                         },
                         noise.size(), &out);

            const int lag = CorrelationLag(noise, out, 200);
            if (lag != kChainLatency) { all_ok = false; worst_combo = combo; worst_lag = lag; }
        }
        rep.Check(all_ok, "all 16 bypass combinations agree",
                  all_ok ? Fmt("%.0f samples", (double)kChainLatency)
                         : Fmt("combo %d measured ", (double)worst_combo)
                               + Fmt("%.0f samples", (double)worst_lag));
    }

    /* ── 2. Transparency ─────────────────────────────────────────────────── */
    rep.Section("The chain does nothing when asked to do nothing");
    {
        // Fully bypassed, the chain is the saturator's dry delay plus the
        // limiter's delay line and nothing else. The residue is the saturator's
        // mix ease, which approaches zero rather than arriving: it is a
        // subnormal here and literally zero on hardware, where FPSCR.FZ is set.
        Chain ch;
        ch.Boot(AllBypassed());
        std::vector<float> out;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = (n == 100) ? 0.5f : 0.f; },
                     4800, &out);

        double err = 0.0;
        for (size_t i = 0; i < out.size(); i++)
        {
            const double want = (i == 100 + (size_t)kChainLatency) ? 0.5 : 0.0;
            err = std::fmax(err, std::fabs((double)out[i] - want));
        }
        rep.Check(err < kSilenceFloor, "bypassed chain is a 75-sample delay to the LSB",
                  Fmt("worst error %.2e", err) + Fmt(", 24-bit LSB is %.2e", (double)kLsb24));
    }
    {
        // Neutral — every stage running, every stage set to unity. What is left
        // is the design's own rounding: NudgeUnity offsets three EQ bands by
        // 8.7e-5 dB each, and the half-band pair has 0.0018 dB of passband
        // ripple. Anything above that is a stage doing something uninvited.
        Chain ch;
        ch.Boot(Neutral());
        std::vector<float> out;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, kProbeAmp); },
                     24000, &out);
        const double g_db = 20.0 * std::log10(Rms(out, 4800, out.size())
                                              / (kProbeAmp / std::sqrt(2.0)));
        rep.Check(std::fabs(g_db) < 0.02, "neutral chain is unity at 1 kHz",
                  Fmt("%+.4f dB", g_db));
        rep.Check(AllFinite(out), "output is finite", "");
    }

    /* ── 3. Stage order ──────────────────────────────────────────────────── */
    rep.Section("Stage order — trim into the limiter, dither last");
    {
        // Trim ahead of the limiter (commit e1067f9). Behind it, +12 dB of trim
        // simply undid the brickwall; ahead of it, the ceiling is the last word.
        Settings s = Neutral();
        s.out.trim_db    = 12.f;
        s.out.ceiling_db = -6.f;
        Chain ch;
        ch.Boot(s);
        std::vector<float> out;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = MixProgram(n, 0.9); },
                     96000, &out);
        const double tp = TruePeakDb(out, 4800);
        rep.Check(tp <= -6.0 + 0.01, "+12 dB of trim cannot lift the -6 dB ceiling",
                  Fmt("%.3f dBTP", tp));
    }
    {
        // Dither is the last thing in the chain, so its level must not depend on
        // any gain upstream of it. If it sat before the trim, +12 dB of trim
        // would raise the noise floor by 12 dB.
        double      floor_db[2];
        const float trims[2] = {0.f, 12.f};
        for (int i = 0; i < 2; i++)
        {
            Settings s = Neutral();
            s.out.dither_lsb = 2.f;
            s.out.trim_db    = trims[i];
            Chain ch;
            ch.Boot(s);
            std::vector<float> out;
            ch.RunSteady([](long, float* l, float* r) { *l = *r = 0.f; }, 48000, &out);
            floor_db[i] = 20.0 * std::log10(Rms(out, 4800, out.size()) + 1e-30);
        }
        rep.Check(std::fabs(floor_db[1] - floor_db[0]) < 0.1,
                  "dither level is independent of trim",
                  Fmt("%.2f dBFS at 0 dB trim, ", floor_db[0])
                      + Fmt("%.2f dBFS at +12 dB", floor_db[1]));
    }

    /* ── 4. The ceiling, end to end ──────────────────────────────────────── */
    rep.Section("The ceiling holds with the whole chain in front of it");
    {
        for (float ceil_db : {-0.1f, -1.f, -6.f})
        {
            double worst = -1e9, quietest = 1e9;
            for (int prog = 0; prog < 3; prog++)
            {
                Settings s = Loud();
                s.out.ceiling_db = ceil_db;
                Chain ch;
                ch.Boot(s);
                std::vector<float> out;
                ch.RunSteady([prog](long n, float* l, float* r) {
                                 *l = *r = prog == 0 ? MixProgram(n, 0.8)
                                         : prog == 1 ? HfProgram(n, 0.6)
                                                     : Tone(n, 997.0, 0.9);
                             }, 96000, &out);
                const double tp = TruePeakDb(out, 9600);
                worst    = std::fmax(worst, tp);
                quietest = std::fmin(quietest, tp);
            }
            // The margin is dither at its 2-LSB maximum riding on the limiter's
            // output: 2.4e-7 absolute, which at these levels is 2e-6 dB.
            // The second half of the assertion matters as much as the first —
            // a chain that undershoots its ceiling is not being held there by
            // the limiter, and the test would be passing for the wrong reason.
            rep.Check(worst <= ceil_db + 0.01 && quietest > ceil_db - 0.5,
                      Fmt("ceiling %.1f dB is met and not exceeded", ceil_db),
                      Fmt("%.3f dBTP worst, ", worst) + Fmt("%.3f dBTP quietest of 3 programs", quietest));
        }
    }

    /* ── 5. Non-finite under fuzz ────────────────────────────────────────── */
    rep.Section("Nothing goes non-finite");
    {
        // Every parameter re-randomised on every control frame, against input
        // that is deliberately hostile: near-Nyquist tone (the half-band's worst
        // case), a DC pedestal, and periodic full-scale impulses. `hold_out`
        // pins the output stage so the run can assert the ceiling as well as
        // finiteness — a fuzz that randomises lim_bypass can only assert the
        // latter, and that is the weaker of the two claims by a distance.
        auto fuzz = [](bool hold_out, std::vector<float>* out) {
            Chain ch;
            ch.Boot(Neutral());
            Lcg  rng;
            Lcg* rp = &rng;
            ch.Run([](long n, float* l, float* r) {
                       const double t = double(n) / kFs;
                       const double v = 2.0 * std::sin(2 * kPi * 23900.0 * t)
                                      + 0.5 + ((n % 1000 == 0) ? 8.0 : 0.0);
                       *l = (float)v;
                       *r = (float)(-0.7 * v);      // decorrelated, so the link works
                   },
                   [rp, hold_out](int, Settings& s) {
                       auto u = [rp]() { return 0.5f * (rp->Bipolar() + 1.f); };
                       s.eq.ls_freq_hz  = 20.f + u() * 780.f;
                       s.eq.ls_gain_db  = -15.f + u() * 30.f;
                       s.eq.mid_freq_hz = 200.f + u() * 4800.f;
                       s.eq.mid_gain_db = -15.f + u() * 30.f;
                       s.eq.mid_q       = 0.707f + u() * 3.3f;
                       s.eq.hs_freq_hz  = 1000.f + u() * 19000.f;
                       s.eq.hs_gain_db  = -15.f + u() * 30.f;
                       s.eq.bypass      = u() > 0.8f;
                       s.comp.threshold_db = -40.f + u() * 40.f;
                       s.comp.ratio        = 1.f / (1.f - 0.95f * u());
                       s.comp.attack_ms    = 0.1f + u() * 99.9f;
                       s.comp.release_ms   = 10.f + u() * 1990.f;
                       s.comp.makeup_db    = u() * 20.f;
                       s.comp.mix          = u();
                       s.comp.character    = (uint8_t)(u() * 2.99f);
                       s.comp.bypass       = u() > 0.8f;
                       s.sat.drive_db  = u() * 24.f;
                       s.sat.mix       = u();
                       s.sat.emphasis  = u();
                       s.sat.asym      = -0.3f + u() * 0.6f;
                       s.sat.bump      = u();
                       s.sat.character = (uint8_t)(u() * 2.99f);
                       s.sat.bypass    = u() > 0.8f;
                       s.out.lim_release_ms = 10.f + u() * 490.f;
                       s.out.dither_lsb     = u() * 2.f;
                       if (hold_out) { s.out.ceiling_db = -1.f; s.out.trim_db = 12.f;
                                       s.out.lim_bypass = false; }
                       else          { s.out.ceiling_db = -6.f + u() * 5.9f;
                                       s.out.trim_db    = -12.f + u() * 24.f;
                                       s.out.lim_bypass = u() > 0.9f; }
                   },
                   480000, out, nullptr);           // 10 s, 625 control frames
        };

        std::vector<float> held, wild;
        fuzz(true,  &held);
        fuzz(false, &wild);

        rep.Check(AllFinite(held) && AllFinite(wild),
                  "10 s x2 of randomised parameters, hostile input",
                  Fmt("peaks %.2f dBFS held, ", PeakDb(held)) + Fmt("%.2f dBFS wild", PeakDb(wild)));
        rep.Check(TruePeakDb(held, 9600) <= -1.0 + 0.01,
                  "the ceiling survives the fuzz",
                  Fmt("%.3f dBTP", TruePeakDb(held, 9600)));
    }

    /* ── 6. Stereo integrity, silence and DC ─────────────────────────────── */
    rep.Section("Stereo integrity, silence and DC");
    {
        Settings s = Extreme();
        s.out.dither_lsb = 0.f;   // the one deliberate L/R asymmetry; see below
        Chain ch;
        ch.Boot(s);
        std::vector<float> L, R;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = MixProgram(n, 0.5); },
                     48000, &L, &R);
        double d = 0.0;
        for (size_t i = 0; i < L.size(); i++)
            d = std::fmax(d, std::fabs((double)L[i] - (double)R[i]));
        rep.Check(d == 0.0, "identical input gives bit-identical output",
                  Fmt("worst |L-R| = %.2e", d));
    }
    {
        // With dither on, the channels must differ by no more than two dither
        // peaks — enough to prove the generators are independent (they are
        // separately seeded) and that nothing else has crept in alongside them.
        Chain ch;
        Settings s = Neutral();
        s.out.dither_lsb = 2.f;
        ch.Boot(s);
        std::vector<float> L, R;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, 0.2); },
                     48000, &L, &R);
        double d = 0.0;
        for (size_t i = 0; i < L.size(); i++)
            d = std::fmax(d, std::fabs((double)L[i] - (double)R[i]));
        const double bound = 2.0 * 2.0 * (double)kLsb24;
        rep.Check(d > 0.0 && d <= bound, "with dither on, the channels differ only by it",
                  Fmt("worst |L-R| = %.3e", d) + Fmt(", two dither peaks = %.3e", bound));
    }
    {
        Settings s = Extreme();
        s.out.dither_lsb = 0.f;
        Chain ch;
        ch.Boot(s);
        std::vector<float> L, R;
        ch.RunSteady([](long n, float* l, float* r) { *l = MixProgram(n, 0.5); *r = 0.f; },
                     48000, &L, &R);
        // Not exactly zero: Extreme() has asym at its stop, so the shaper emits
        // a DC term on a silent input and the blocker drives it to ~1e-29.
        rep.Check(Peak(R) < kSilenceFloor, "a hard-panned channel does not leak",
                  Fmt("silent-channel peak %.2e", Peak(R)));
        rep.Check(Peak(L) > 0.1, "  (and the fed channel is actually running)",
                  Fmt("peak %.3f", Peak(L)));
    }
    {
        Settings s = Extreme();
        s.out.dither_lsb = 0.f;
        Chain ch;
        ch.Boot(s);
        std::vector<float> out;
        ch.RunSteady([](long, float* l, float* r) { *l = *r = 0.f; }, 48000, &out);
        rep.Check(Peak(out) < kSilenceFloor, "silence in, silence out (dither off)",
                  Fmt("peak %.2e", Peak(out)));
    }
    {
        // The saturator's asymmetry generates DC by construction; its DC blocker
        // is what stops that reaching the output and eating headroom.
        Settings s = Neutral();
        s.sat.drive_db = 18.f; s.sat.asym = 0.3f; s.sat.character = kSatSaturated;
        Chain ch;
        ch.Boot(s);
        std::vector<float> out;
        ch.RunSteady([](long n, float* l, float* r) { *l = *r = Tone(n, 220.0, 0.5); },
                     96000, &out);
        const double dc = std::fabs(Mean(out, 48000, out.size()));
        rep.Check(dc < 1e-3, "asymmetric drive leaves no DC at the output",
                  Fmt("mean %.2e", dc));
    }

    /* ── 7. Gain continuity ──────────────────────────────────────────────── */
    rep.Section("The gain the chain applies is continuous");
    {
        // Everything bypassed except the compressor, so the chain reduces to a
        // pure gain and out[n + 75] / in[n] IS that gain. A square wave keeps
        // |in| constant, so the quotient is defined at every sample.
        //
        // The measurement window is a long release tail: input drops well below
        // threshold, c goes to zero, and y coasts down through every value it
        // can take. With a 2000 ms release and ~6 dB of gain to give back, the
        // envelope moves ~5e-5 dB per sample there, so any discontinuity in the
        // gain path shows up two or three orders of magnitude above the floor.
        Settings s = AllBypassed();
        s.comp.bypass       = false;
        s.comp.threshold_db = -20.f;
        s.comp.ratio        = 4.f;
        s.comp.attack_ms    = 50.f;
        s.comp.release_ms   = 2000.f;
        s.comp.character    = kCompPrecise;

        Chain ch;
        ch.Boot(s);

        std::vector<float> in, out;
        const long kLoud = 96000;                  // 2 s loud, then 8 s quiet
        ch.RunSteady([&in, kLoud](long n, float* l, float* r) {
                         const double a = (n < kLoud) ? 0.5 : 0.002;
                         const float  v = (float)((std::sin(2 * kPi * 100.0 * double(n) / kFs) > 0) ? a : -a);
                         *l = *r = v;
                         in.push_back(v);
                     },
                     480000, &out);

        const std::vector<double> g = AppliedGainDb(in, out);
        // Skip the first 0.2 s after the drop, where the release genuinely is
        // moving fast, and stop before the tail flattens into float noise.
        const double step = MaxAdjacentStep(g, (size_t)kLoud + 9600, (size_t)kLoud + 384000);
        rep.Check(step < 2e-3, "no step in the applied gain down a release tail",
                  Fmt("worst single-sample step %.3e dB", step));
    }

    /* ── 8. Parameter moves ──────────────────────────────────────────────── */
    rep.Section("A knob does nothing on the way that it does not do at rest");
    {
        const Settings base = Neutral();     // -0.1 dB ceiling, far above the probe

        const std::vector<Knob> knobs = {
            { "Sat Drive 0 -> 24 dB",   [](Settings& s, float v) { s.sat.drive_db = v; },   0.f,  24.f },
            { "Sat Emphasis 0 -> 1",    [](Settings& s, float v) { s.sat.emphasis = v; },   0.f,   1.f },
            { "Sat Bump 0 -> 1",        [](Settings& s, float v) { s.sat.bump = v; },       0.f,   1.f },
            { "Sat Mix 0 -> 1",         [](Settings& s, float v) { s.sat.mix = v; },        0.f,   1.f },
            { "Sat Asym -0.3 -> +0.3",  [](Settings& s, float v) { s.sat.asym = v; },      -0.3f,  0.3f },
            { "Comp Mix 0 -> 1",        [](Settings& s, float v) { s.comp.mix = v; },       0.f,   1.f },
            { "Comp Makeup 0 -> 12 dB", [](Settings& s, float v) { s.comp.makeup_db = v; }, 0.f,  12.f },
            { "Trim -12 -> +12 dB",     [](Settings& s, float v) { s.out.trim_db = v; },  -12.f,  12.f },
            { "EQ Mid 0 -> +12 dB",     [](Settings& s, float v) { s.eq.mid_gain_db = v; },  0.f, 12.f },
        };

        for (const Knob& k : knobs)
        {
            const double hi    = TravelCeiling(k, base);
            const MoveResult sw = Move(k, base, 12);
            const MoveResult sn = Move(k, base, 1);

            const double over_sw = 20.0 * std::log10(sw.env_peak / hi);
            const double over_sn = 20.0 * std::log10(sn.env_peak / hi);

            // 0.5 dB of slack covers the coefficient lerp: a blend of two stable
            // biquads is stable but is not the design of the blended parameter,
            // so an EQ or emphasis move can ring a little above every rest point.
            rep.Check(over_sw < 0.5 && over_sn < 0.5,
                      std::string(k.name) + " stays inside its own travel",
                      Fmt("%+.2f dB swept, ", over_sw) + Fmt("%+.2f dB snapped", over_sn));
        }

        for (const Knob& k : knobs)
        {
            const MoveResult sw = Move(k, base, 12);
            // Slew of the probe tone at the level this run actually reached,
            // plus 5 % for float noise. See the note on self-calibration.
            const double bound = 1.05 * SineSlew(sw.env_peak, kProbeHz);
            rep.Check(sw.max_step <= bound,
                      std::string(k.name) + " sweeps without a step",
                      Fmt("worst step %.3e vs slew ", sw.max_step) + Fmt("%.3e", bound));
        }
    }

    /* ── 9. Bypass toggles ───────────────────────────────────────────────── */
    rep.Section("Bypass toggles");
    {
        const char* names[4] = {"EQ", "Compressor", "Saturation", "Limiter"};
        // The saturator routes bypass through its 5 ms mix ease and the limiter
        // gates a gain that is already settled, so both claim a click-free
        // toggle. The EQ and the compressor switch the wet path on the sample,
        // so their toggle IS a step, bounded by whatever the stage was doing.
        const bool crossfaded[4] = {false, false, true, true};

        for (int stage = 0; stage < 4; stage++)
        {
            Settings base = Neutral();
            base.eq.mid_gain_db    = 9.f;
            base.comp.threshold_db = -30.f; base.comp.ratio = 4.f; base.comp.makeup_db = 6.f;
            base.sat.drive_db      = 12.f;  base.sat.emphasis = 0.6f; base.sat.bump = 0.5f;
            base.out.ceiling_db    = -1.f;

            Chain ch;
            ch.Boot(base);
            std::vector<float> out;
            ch.Run([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, 0.05); },
                   [stage](int frame, Settings& s) {
                       const bool on = (frame / 8) % 2 == 1;   // toggle every 128 ms
                       switch (stage) {
                           case 0:  s.eq.bypass      = on; break;
                           case 1:  s.comp.bypass    = on; break;
                           case 2:  s.sat.bypass     = on; break;
                           default: s.out.lim_bypass = on; break;
                       }
                   },
                   96000, &out, nullptr);

            const double env   = MaxOf(Envelope(out, 48), 9600);
            const double step  = MaxStep(out, 9600);
            const double slew  = 1.05 * SineSlew(env, kProbeHz);
            const bool   ok    = crossfaded[stage] ? (step <= slew) : (step <= env);
            rep.Check(ok, std::string(names[stage]) + " bypass toggle",
                      Fmt("step %.3e, slew ", step) + Fmt("%.3e", slew)
                          + (crossfaded[stage] ? " (crossfaded)" : " (switched, by design)"));
        }
    }

    /* ── 10. Long-run stability ──────────────────────────────────────────── */
    rep.Section("Long-run stability");
    {
        // The limiter's boxcar accumulates by add-one/subtract-one forever and
        // its header argues that a float accumulator would random-walk off the
        // true window mean, pushing the product back over the ceiling. A minute
        // of dense program is 2.9 M updates; if the ceiling is still met to the
        // same fraction of a dB at the end, it has not drifted.
        Settings s = Loud();
        s.out.ceiling_db = -1.f;
        Chain ch;
        ch.Boot(s);

        auto program = [](long n, float* l, float* r) { *l = *r = MixProgram(n, 0.8); };
        std::vector<float> first, mid, last;
        ch.RunSteady(program,  240000, &first);     //  5 s
        ch.RunSteady(program, 2400000, &mid);       // 50 s, discarded
        ch.RunSteady(program,  240000, &last);      //  5 s

        const double tp_first = TruePeakDb(first, 9600);
        const double tp_last  = TruePeakDb(last);
        rep.Check(tp_last <= -1.0 + 0.01 && AllFinite(last) && AllFinite(mid),
                  "ceiling still holds after 60 s of program",
                  Fmt("%.4f dBTP at the start, ", tp_first) + Fmt("%.4f dBTP at the end", tp_last));
        rep.Check(std::fabs(tp_last - tp_first) < 0.02, "and has not drifted",
                  Fmt("%.4f dB apart", std::fabs(tp_last - tp_first)));
    }

    /* ── 11. Metering ────────────────────────────────────────────────────── */
    rep.Section("Gain-reduction meter");
    {
        Settings s = Neutral();
        s.comp.threshold_db = -30.f; s.comp.ratio = 4.f; s.comp.character = kCompGlue;
        Chain ch;
        ch.Boot(s);
        std::vector<float> out;
        ch.RunSteady([](long, float* l, float* r) { *l = *r = 0.f; }, 24000, &out);
        const double idle = CompGainReductionDb();

        ch.RunSteady([](long n, float* l, float* r) { *l = *r = Tone(n, kProbeHz, 0.5); },
                     192000, &out);
        const double busy = CompGainReductionDb();

        rep.Check(idle >= 0.0 && idle < 0.01 && busy > 1.0,
                  "reads >= 0 dB, zero at rest, positive under signal",
                  Fmt("%.5f dB idle, ", idle) + Fmt("%.2f dB on a -6 dBFS tone", busy));
    }

    return rep.Finish();
}
