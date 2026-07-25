/**
 * dsp_limiter.h — Header-only stereo-linked brickwall limiter with
 * kLookahead-sample lookahead and provably zero overshoot.
 *
 * A circular delay buffer holds kLookahead samples of audio. Peak detection
 * runs on the *current* (pre-delay) input, which is kLookahead samples ahead
 * of what is being output, so there is a whole window in which to get the gain
 * down before the peak it belongs to reaches the output.
 *
 * Using that window correctly is the entire problem, and it is easy to get
 * wrong. A one-pole attack driven straight from the instantaneous gain target
 * does NOT work: the one-pole only converges while its target is *held*, and a
 * transient shorter than the window sets the target low for a sample or two and
 * then releases it. The gain barely moves, the peak arrives at full height, and
 * the safety clamp below hard-clips it. Measured on this chain before the fix,
 * an isolated one-sample transient reached the clamp 12.35 dB over the ceiling
 * and ordinary program material with stabs reached 3-8 dB — i.e. the limiter
 * was a clipper for exactly the signals lookahead exists to catch.
 *
 * The fix is two stages on the gain signal, neither of which is a one-pole:
 *
 *   RunMin  — running minimum over kGainWin samples. This is the peak *hold*:
 *             once a peak sets the target low, the target stays low for the
 *             whole window rather than snapping back on the next sample.
 *   BoxCar  — moving average over kGainWin samples. This turns RunMin's step
 *             into a linear ramp so the gain change is not a click, and unlike
 *             a one-pole it settles exactly rather than asymptotically.
 *
 * Zero overshoot is a property of the window sizing, not a measurement that
 * happened to come out well. With audio delay D, running-min window A, boxcar
 * window B, and a detector whose own group delay is Td, a peak at input index p
 * is seen by the detector at p+Td and leaves the delay line at p+D. The applied
 * gain at p+D is a mean of B running-minima, and every one of those minima is
 * <= the peak's own target exactly when
 *
 *     B <= D - Td + 1 <= A
 *
 * The release is a one-pole, but on the *target* rather than on the applied
 * gain, with an instantaneous attack. Recovery therefore begins only once the
 * peak has left the running-min window, which is what stops the gain crawling
 * back up while the peak is still in the buffer.
 *
 * The post-gain Clampf is now genuinely belt-and-braces: it exists for float
 * rounding, and the harness asserts it never has anything to do.
 *
 * ── True peak ────────────────────────────────────────────────────────────────
 *
 * The detector is 4x oversampled, so the ceiling is a **dBTP** ceiling in the
 * sense of ITU-R BS.1770-4, not a sample-peak one. This is not decoration: a
 * sample-peak limiter guarantees nothing about what the DAC's reconstruction
 * filter — or a lossy encoder downstream — actually produces between samples.
 * Measured on this chain with HF-dense program, a sample-peak limiter set to
 * -1.0 dBFS emitted **+0.40 dBTP**, 1.4 dB over its own ceiling; 4x detection
 * brings that to -0.96 dBTP.
 *
 * That costs latency, because the interpolator has group delay of its own and
 * Td goes from 0 to ~11 samples. Rather than shrink the windows and give up
 * ramp time, D grows to 60 so the full ~1 ms of ramp survives. The chain
 * budget moves from 63 samples to 75 (1.56 ms); see DEVELOPER_GUIDE.md §8,
 * which is where that ceiling is argued rather than here.
 *
 * Td is a *range*, not a number, because the four interpolated positions
 * produced in one call sit at different base times (see TruePeak4x). The sizing
 * must hold across all of them, so A is checked against the earliest and B
 * against the latest. The static_asserts below do that in quarter-samples,
 * which is the coarsest grid all four land on.
 *
 * Latency introduced: kLookahead samples (60 samples = 1.25 ms at 48 kHz).
 */

#pragma once
#include <cmath>
#include <cstring>
#include "dsp_common.h"
#include "dsp_halfband.h"

namespace mastering_dsp {

/** Audio delay line length, and the chain's limiter latency. */
static constexpr int kLookahead = 60;

/**
 * Group delay of the 4x true-peak detector, in quarter-samples, as a closed
 * range. Derived in TruePeak4x's comment and measured by lim_response_test
 * rather than taken on trust.
 */
static constexpr int kTpDelayMinQ = 42;   // 10.50 base samples
static constexpr int kTpDelayMaxQ = 45;   // 11.25 base samples

/** Peak-hold window (A) and gain-ramp window (B). See the inequality above. */
static constexpr int kHoldWin = 52;
static constexpr int kRampWin = 48;

// A >= D - Td_min + 1 and B <= D - Td_max + 1, in quarter-samples.
static_assert(4 * kHoldWin >= 4 * kLookahead - kTpDelayMinQ + 4,
              "peak hold is too short: a true peak can escape before the gain is down");
static_assert(4 * kRampWin <= 4 * kLookahead - kTpDelayMaxQ + 4,
              "gain ramp is too long: it is still averaging in pre-peak gain on arrival");

/**
 * 4x oversampled peak of one channel, via two cascaded half-band interpolators.
 *
 * Both outputs of the first stage go through the *same* second-stage instance,
 * in time order. Giving the even and odd phases their own filter instance is
 * the natural-looking mistake and it is wrong: each instance would then see a
 * stream decimated by two, which is not a 4x interpolation of anything.
 *
 * Group delay, in base samples. A half-band of 31 taps has group delay 15 at
 * its own working rate, so stage 1 contributes 15/2 = 7.5 base and stage 2
 * contributes 15/4 = 3.75 base. The four positions emitted in one call are not
 * coincident — writing m for the 2x index, stage 2 emits 4x samples standing at
 * base times (m-22.5)/2 and (m-22)/2, and the call feeds it m = 2n then 2n+1:
 *
 *     n - 11.25,  n - 11.00,  n - 10.75,  n - 10.50
 *
 * Hence Td in [10.5, 11.25], which is what kTpDelay{Min,Max}Q encode.
 *
 * The raw sample delayed by 11 is folded in by the caller. The half-band rolls
 * off 0.46 dB by 20 kHz, so the interpolated set can read *below* the true
 * sample peak on near-Nyquist content; taking the max with the original sample
 * makes the detector never worse than the sample-peak one it replaces. Base
 * time n-11 sits inside the range above, so it needs no separate sizing.
 */
struct TruePeak4x {
    float Process(float x)
    {
        float e, o;
        up1_.Process(x, e, o);
        float a, b, c, d;
        up2_.Process(e, a, b);
        up2_.Process(o, c, d);
        return fmaxf(fmaxf(fabsf(a), fabsf(b)), fmaxf(fabsf(c), fabsf(d)));
    }

    HalfBandUp up1_, up2_;
};

/**
 * Running minimum over exactly kGainWin samples, O(1) worst case per sample
 * (van Herk / Gil-Werman): the stream is cut into segments of kGainWin, the
 * current segment contributes a forward-running prefix minimum and the previous
 * one a precomputed suffix minimum, and the window is always one of each.
 *
 * A monotonic deque would also be O(1) amortised but has a data-dependent inner
 * loop, which is the wrong shape for an audio ISR. This costs two compares per
 * sample plus one kGainWin-long suffix pass per kGainWin samples — bounded work
 * with no dependence on the signal.
 *
 * 1.f is the identity for the min: every gain target is <= 1 by construction.
 */
template <int W>
struct RunMin {
    RunMin()
    {
        for (int i = 0; i < W; i++) { cur_[i] = 1.f; suf_[i] = 1.f; }
        suf_[W] = 1.f;
    }

    float Process(float x)
    {
        cur_[i_] = x;
        run_     = (i_ == 0) ? x : fminf(run_, x);
        // cur_[0..i_] is i_+1 samples, suf_[i_+1] covers the previous segment's
        // last W-i_-1 — exactly W between them.
        const float y = fminf(run_, suf_[i_ + 1]);
        if (++i_ == W)
        {
            float m = 1.f;
            for (int k = W - 1; k >= 0; --k) { m = fminf(m, cur_[k]); suf_[k] = m; }
            i_ = 0;
        }
        return y;
    }

    float cur_[W];
    float suf_[W + 1];      // suf_[W] is the empty tail, always 1.f
    float run_ = 1.f;
    int   i_   = 0;
};

/**
 * Moving average over kGainWin samples.
 *
 * The accumulator is double, not float. It is updated by add-one/subtract-one
 * forever, so float rounding would random-walk the running sum away from the
 * true window mean over hours of uptime with nothing to pull it back. At
 * double, the same walk is ~2^29 times smaller than the LSB of the float gain
 * it produces. The H7's FPU is double-precision in hardware and the EQ's low
 * shelf already runs in double, so this is not a new cost.
 */
template <int W>
struct BoxCar {
    BoxCar() { for (int i = 0; i < W; i++) d_[i] = 1.f; }

    float Process(float x)
    {
        // Both operands are promoted to double *before* the subtraction. Writing
        // `acc_ += x - d_[w_]` instead rounds the difference to float first and
        // hands the double accumulator an already-lossy term, which random-walks
        // acc_ exactly as a float accumulator would — measured at 1.7e-5
        // relative after 192k samples, enough to push the product back over the
        // ceiling and into the clamp. The difference of two floats is always
        // exact in double, so this form makes acc_ an exact window sum.
        acc_  += (double)x - (double)d_[w_];
        d_[w_] = x;
        if (++w_ == W) w_ = 0;
        return (float)(acc_ * (1.0 / W));
    }

    double acc_ = W;        // starts at the mean of a buffer full of 1.f
    float  d_[W];
    int    w_ = 0;
};

struct Limiter {
    void Configure(float ceiling_db, float release_ms, bool byp, float fs)
    {
        ceiling_lin = DbToLin(ceiling_db);
        rel_coef    = MsToCoef(release_ms, fs);
        bypass      = byp;
        // Do not reset the envelope, the delay line or either window — that
        // would pop on a live parameter change.
    }

    void ProcessSample(float& l, float& r)
    {
        // Write pre-delay audio into circular buffer.
        buf_l[write] = l;
        buf_r[write] = r;

        // Detect from lookahead (current, pre-delay) input, 4x oversampled so
        // the ceiling is a true-peak one. The raw sample delayed by 11 joins the
        // max because the half-band's 0.46 dB rolloff at 20 kHz lets the
        // interpolated set read below the sample peak on near-Nyquist content;
        // base time n-11 is inside the detector's own [n-11.25, n-10.5] span, so
        // it rides along under the same sizing.
        const int   d11  = (write - 11 + (kLookahead + 1)) % (kLookahead + 1);
        const float peak = fmaxf(fmaxf(tp_l.Process(l), tp_r.Process(r)),
                                 fmaxf(fabsf(buf_l[d11]), fabsf(buf_r[d11])));
        const float gd   = (peak > ceiling_lin) ? ceiling_lin / peak : 1.f;

        // Instantaneous attack, exponential release — applied to the TARGET.
        // All attack shaping is the RunMin/BoxCar pair below; putting a
        // one-pole here as well is the bug described in the header comment.
        if (gd < rel) rel = gd;
        else          rel += rel_coef * (gd - rel);

        // Peak hold, then ramp. See the B <= D-Td+1 <= A sizing above.
        gain = box.Process(rmin.Process(rel));

        // Read kLookahead-sample-delayed audio and apply gain.
        //
        // Bypass gates the gain, never the delay line or the envelope: the
        // buffer is the chain's latency, so skipping it would make the module's
        // latency depend on bypass state and turn the A/B into a click. The
        // envelope keeps tracking for the same reason it does in the
        // compressor — it is already settled when the limiter comes back in.
        const int   rd = (write + 1) % (kLookahead + 1);
        const float g  = bypass ? 1.f : gain;
        l = buf_l[rd] * g;
        r = buf_r[rd] * g;
        if (!bypass)
        {
            l = Clampf(l, -ceiling_lin, ceiling_lin);
            r = Clampf(r, -ceiling_lin, ceiling_lin);
        }

        write = (write + 1) % (kLookahead + 1);
    }

    float gain = 1.f;       // applied gain, post-ramp
    float rel  = 1.f;       // release-smoothed target, pre-hold
    bool  bypass = false;
    float ceiling_lin = 1.f, rel_coef = 0.01f;
    float buf_l[kLookahead + 1] = {};
    float buf_r[kLookahead + 1] = {};
    int   write = 0;
    RunMin<kHoldWin> rmin;
    BoxCar<kRampWin> box;
    TruePeak4x       tp_l, tp_r;
};

} // namespace mastering_dsp
