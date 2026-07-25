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
 * Here Td = 0 (the detector reads the raw input) and A = B = D + 1 = kGainWin,
 * which satisfies it with equality on both sides. So the gain is at or below
 * the target for every sample the peak occupies, for any input whatsoever.
 *
 * The release is a one-pole, but on the *target* rather than on the applied
 * gain, with an instantaneous attack. Recovery therefore begins only once the
 * peak has left the running-min window, which is what stops the gain crawling
 * back up while the peak is still in the buffer.
 *
 * The post-gain Clampf is now genuinely belt-and-braces: it exists for float
 * rounding, and the harness asserts it never has anything to do.
 *
 * Latency introduced: kLookahead samples (48 samples = 1 ms at 48 kHz).
 */

#pragma once
#include <cmath>
#include <cstring>
#include "dsp_common.h"

namespace mastering_dsp {

static constexpr int kLookahead = 48;
/** Running-min and boxcar window. See the sizing inequality above. */
static constexpr int kGainWin = kLookahead + 1;

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
struct RunMin {
    RunMin()
    {
        for (int i = 0; i < kGainWin; i++) { cur_[i] = 1.f; suf_[i] = 1.f; }
        suf_[kGainWin] = 1.f;
    }

    float Process(float x)
    {
        cur_[i_] = x;
        run_     = (i_ == 0) ? x : fminf(run_, x);
        // cur_[0..i_] is i_+1 samples, suf_[i_+1] covers the previous segment's
        // last kGainWin-i_-1 — exactly kGainWin between them.
        const float y = fminf(run_, suf_[i_ + 1]);
        if (++i_ == kGainWin)
        {
            float m = 1.f;
            for (int k = kGainWin - 1; k >= 0; --k) { m = fminf(m, cur_[k]); suf_[k] = m; }
            i_ = 0;
        }
        return y;
    }

    float cur_[kGainWin];
    float suf_[kGainWin + 1];   // suf_[kGainWin] is the empty tail, always 1.f
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
struct BoxCar {
    BoxCar() { for (int i = 0; i < kGainWin; i++) d_[i] = 1.f; }

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
        if (++w_ == kGainWin) w_ = 0;
        return (float)(acc_ * (1.0 / kGainWin));
    }

    double acc_ = kGainWin;      // starts at the mean of a buffer full of 1.f
    float  d_[kGainWin];
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

        // Detect from lookahead (current, pre-delay) input.
        const float peak = fmaxf(fabsf(l), fabsf(r));
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

    float  gain = 1.f;      // applied gain, post-ramp
    float  rel  = 1.f;      // release-smoothed target, pre-hold
    bool   bypass = false;
    float  ceiling_lin = 1.f, rel_coef = 0.01f;
    float  buf_l[kLookahead + 1] = {};
    float  buf_r[kLookahead + 1] = {};
    int    write = 0;
    RunMin rmin;
    BoxCar box;
};

} // namespace mastering_dsp
