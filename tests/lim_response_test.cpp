/**
 * lim_response_test.cpp — brickwall limiter measurement harness.
 *
 * The headline assertion is that the safety Clampf never fires. That is a
 * stronger statement than "the output respects the ceiling" and it is the one
 * worth testing: the output respected the ceiling before the attack was fixed
 * too, by clipping up to 12 dB into the clamp to get there.
 *
 * Everything here drives `Limiter` directly. No mastering_dsp.cpp, no biquads.
 */

#include "lim_test_common.h"

using namespace limtest;
using namespace mastering_dsp;

int main()
{
    Report rep;
    std::printf("\n\033[1mBrickwall limiter response harness\033[0m"
                "  —  %d-sample lookahead at %.0f Hz\n",
                kLookahead, kFs);

    /* ── 1. The invariant: the safety clamp never has anything to do ─────── */
    rep.Section("Zero overshoot — the clamp is never reached");
    for (float ceil_db : {-0.1f, -1.f, -6.f})
    {
        for (float rel_ms : {10.f, 100.f, 500.f})
        {
            double worst = -1e9;
            long   clip  = 0;
            int    worst_prog = 0;
            for (int p = 0; p < kProgramCount; p++)
            {
                Limiter   lim = MakeLimiter(ceil_db, rel_ms);
                Overshoot o   = MeasureOvershoot(lim, p, 192000);
                if (o.worst_db > worst) { worst = o.worst_db; worst_prog = p; }
                clip += o.clipped;
            }
            // Not "small enough to be inaudible" — small enough to be float
            // rounding on the boxcar mean. The bug this replaces measured
            // +12.35 dB here. See kFloatSlack.
            rep.Check(clip == 0 && worst < 1e-3,
                      Fmt("ceiling %.1f dB, release %.0f ms", ceil_db, rel_ms),
                      Fmt("worst %+.5f dB", worst) + ", " + ProgramName(worst_prog)
                          + Fmt(" (%.0f programs)", (double)kProgramCount));
        }
    }

    /* ── 2. The case the one-pole attack got wrong ───────────────────────── */
    rep.Section("Transients shorter than the window");
    {
        // A lone one-sample +12 dB spike. The old one-pole reached the clamp
        // 12.35 dB over the ceiling here; a working peak hold reaches 0.00.
        Limiter lim = MakeLimiter(-1.f, 100.f);
        double  pre = 0.0;
        for (int n = 0; n < 600; n++)
        {
            const int   rd = (lim.write + 1) % (kLookahead + 1);
            const float dl = lim.buf_l[rd];
            float       s  = (n == 300) ? 4.f : 0.f, l = s, r = s;
            lim.ProcessSample(l, r);
            pre = std::fmax(pre, std::fabs(dl * lim.gain));
        }
        const double over = 20.0 * std::log10(pre / lim.ceiling_lin);
        rep.Check(over < 1e-3, "lone +12 dB one-sample spike is held, not clipped",
                  Fmt("%+.5f dB into the clamp (was +12.35)", over));
    }
    {
        // Sweep the transient width across the window boundary. The failure was
        // width-dependent, so a single width is not evidence.
        double worst = -1e9;
        int    worst_w = 0;
        for (int w = 1; w <= 96; w++)
        {
            Limiter lim = MakeLimiter(-1.f, 100.f);
            double  pre = 0.0;
            for (int n = 0; n < 2000; n++)
            {
                const int   rd = (lim.write + 1) % (kLookahead + 1);
                const float dl = lim.buf_l[rd];
                float       s  = (n >= 500 && n < 500 + w) ? 4.f : 0.2f, l = s, r = s;
                lim.ProcessSample(l, r);
                if (n > 100) pre = std::fmax(pre, std::fabs(dl * lim.gain));
            }
            const double over = 20.0 * std::log10(pre / lim.ceiling_lin);
            if (over > worst) { worst = over; worst_w = w; }
        }
        rep.Check(worst < 1e-3, "every burst width 1..96 samples is held",
                  Fmt("worst %+.5f dB at width %.0f", worst, (double)worst_w));
    }

    /* ── 3. It still limits — a ceiling met by turning everything down is
     *      also "zero overshoot", so check the output actually arrives ───── */
    rep.Section("The ceiling is met, not undershot");
    for (float ceil_db : {-0.1f, -1.f, -6.f})
    {
        Limiter      lim = MakeLimiter(ceil_db, 100.f);
        const double pk  = OutputPeakDb(lim, kSineStabs, 192000);
        rep.Check(pk > ceil_db - 0.5 && pk <= ceil_db + 1e-4,
                  Fmt("ceiling %.1f dB is reached within 0.5 dB", ceil_db),
                  Fmt("output peak %+.3f dBFS", pk));
    }

    /* ── 4. Latency is exactly kLookahead and does not depend on bypass ──── */
    rep.Section("Latency");
    for (int byp = 0; byp < 2; byp++)
    {
        Limiter lim = MakeLimiter(-1.f, 100.f, byp != 0);
        int     at  = -1;
        for (int n = 0; n < 400; n++)
        {
            float s = (n == 10) ? 0.5f : 0.f, l = s, r = s;
            lim.ProcessSample(l, r);
            if (at < 0 && std::fabs(l) > 1e-9f) at = n;
        }
        rep.Check(at - 10 == kLookahead,
                  byp ? "bypassed latency equals kLookahead" : "active latency equals kLookahead",
                  Fmt("%.0f samples", (double)(at - 10)));
    }

    /* ── 5. Bypass is bit-exact unity, not "close to" unity ──────────────── */
    rep.Section("Bypass");
    {
        Limiter lim = MakeLimiter(-1.f, 100.f, true);
        Lcg     rng;
        std::vector<float> in;
        double  worst = 0.0;
        for (int n = 0; n < 20000; n++)
        {
            const float s = 4.f * rng.Bipolar();
            in.push_back(s);
            float l = s, r = s;
            lim.ProcessSample(l, r);
            if (n >= kLookahead)
                worst = std::fmax(worst, std::fabs(l - in[n - kLookahead]));
        }
        rep.Check(worst == 0.0, "bypass passes the delayed input bit-exactly",
                  Fmt("peak error %.2e", worst));
    }

    /* ── 6. Release timing is what the knob says ─────────────────────────── */
    rep.Section("Release");
    for (float rel_ms : {10.f, 100.f, 500.f})
    {
        // Drive 12 dB in, then drop to silence and time the recovery to -3 dB
        // of the way back. The boxcar adds kGainWin samples of ramp on top of
        // the one-pole, which at 48 kHz is 1 ms — visible at 10 ms, not at 500.
        Limiter lim = MakeLimiter(-1.f, rel_ms);
        for (int n = 0; n < 48000; n++) { float l = 4.f, r = 4.f; lim.ProcessSample(l, r); }
        const float g0 = lim.gain;
        int         n63 = -1;
        for (int n = 0; n < 96000 && n63 < 0; n++)
        {
            float l = 0.f, r = 0.f;
            lim.ProcessSample(l, r);
            if (lim.gain >= g0 + (1.f - g0) * 0.632f) n63 = n;
        }
        const double meas_ms = 1000.0 * n63 / kFs;
        rep.Check(meas_ms > rel_ms * 0.7 && meas_ms < rel_ms * 1.4 + 2.0,
                  Fmt("release %.0f ms reaches 63%% on time", rel_ms),
                  Fmt("measured %.1f ms", meas_ms));
    }

    /* ── 7. True peak — the ceiling is dBTP, not dBFS ────────────────────── */
    rep.Section("True peak");
    for (float ceil_db : {-0.1f, -1.f, -6.f})
    {
        Limiter lim = MakeLimiter(ceil_db, 100.f);
        TruePeakMeter m;
        for (int n = 0; n < 192000; n++)
        {
            float s = IspSample(n), l = s, r = s;
            lim.ProcessSample(l, r);
            if (n >= 4000) m.Push(l);
        }
        const double tp = 20.0 * std::log10(m.peak + 1e-30);
        // 0.05 dB of slack is the half-band's own passband ripple plus the
        // rolloff at 19 kHz, not limiter headroom. A sample-peak detector
        // measured +1.40 dB over the ceiling on this program.
        rep.Check(tp <= ceil_db + 0.05,
                  Fmt("ceiling %.1f dB holds as dBTP on HF-dense program", ceil_db),
                  Fmt("%+.2f dBTP (%+.2f dB over ceiling)", tp, tp - ceil_db));
    }

    /* ── 8. The detector's group delay is what the sizing assumes ────────── */
    rep.Section("Window sizing");
    {
        // Measure Td rather than trusting the algebra: feed an impulse, find
        // which iteration the 4x detector reports its largest magnitude at.
        TruePeak4x tp;
        int        at   = 0;
        float      best = 0.f;
        for (int n = 0; n < 200; n++)
        {
            const float v = tp.Process(n == 50 ? 1.f : 0.f);
            if (v > best) { best = v; at = n; }
        }
        const double td = at - 50;
        rep.Check(td * 4 >= kTpDelayMinQ && td * 4 <= kTpDelayMaxQ,
                  "measured detector delay is inside [Td_min, Td_max]",
                  Fmt("%.2f samples, declared [%.2f", td, kTpDelayMinQ / 4.0)
                      + Fmt(", %.2f]", kTpDelayMaxQ / 4.0));
    }
    {
        // B <= D - Td + 1 <= A, checked in quarter-samples against both ends of
        // the detector's delay range. Mirrors the static_asserts in the header.
        const int D = kLookahead;
        rep.Check(4 * kHoldWin >= 4 * D - kTpDelayMinQ + 4,
                  "A >= D - Td_min + 1 (peak hold outlasts the delay line)",
                  Fmt("%.0f >= ", (double)(4 * kHoldWin))
                      + Fmt("%.0f (quarter-samples)", (double)(4 * D - kTpDelayMinQ + 4)));
        rep.Check(4 * kRampWin <= 4 * D - kTpDelayMaxQ + 4,
                  "B <= D - Td_max + 1 (ramp has settled on arrival)",
                  Fmt("%.0f <= ", (double)(4 * kRampWin))
                      + Fmt("%.0f (quarter-samples)", (double)(4 * D - kTpDelayMaxQ + 4)));
    }

    return rep.Finish();
}
