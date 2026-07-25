/**
 * dsp_limiter.h — Header-only stereo-linked brickwall limiter with
 * kLookahead-sample lookahead, fast one-pole attack, and exponential release.
 *
 * A circular delay buffer holds kLookahead samples of audio.  Peak detection
 * runs on the *current* (pre-delay) input, which is kLookahead samples ahead
 * of what is being output.  The gain envelope uses an asymmetric one-pole:
 *
 *   attack  — fast (tau ≈ kLookahead/5 samples), ramps gain down before the
 *             peak exits the buffer so no transient overshoot reaches the output
 *   release — exponential, user-controlled (ms → coefficient at audio rate)
 *
 * The safety Clampf after gain application catches the <1 % residual from the
 * one-pole not having fully settled, matching the prior belt-and-braces clamp.
 *
 * Latency introduced: kLookahead samples (48 samples = 1 ms at 48 kHz).
 */

#pragma once
#include <cmath>
#include <cstring>
#include "dsp_common.h"

namespace mastering_dsp {

static constexpr int kLookahead = 48;

struct Limiter {
    void Configure(float ceiling_db, float release_ms, bool byp, float fs)
    {
        ceiling_lin = DbToLin(ceiling_db);
        rel_coef    = MsToCoef(release_ms, fs);
        // Attack settles to within ~1 % of target in kLookahead samples
        // (5 time-constants of tau = kLookahead/5).
        atk_coef    = 1.f - expf(-5.f / kLookahead);
        bypass      = byp;
        // Do not reset gain or write_ — avoids a pop on live parameter changes.
    }

    void ProcessSample(float& l, float& r)
    {
        // Write pre-delay audio into circular buffer.
        buf_l[write] = l;
        buf_r[write] = r;

        // Detect from lookahead (current, pre-delay) input.
        const float peak = fmaxf(fabsf(l), fabsf(r));
        const float gd   = (peak > ceiling_lin) ? ceiling_lin / peak : 1.f;

        // Asymmetric smoothing: fast attack ramp, slow exponential release.
        if (gd < gain) gain += atk_coef * (gd - gain);
        else           gain += rel_coef  * (gd - gain);

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

    float gain = 1.f;
    bool  bypass = false;
    float ceiling_lin = 1.f, rel_coef = 0.01f, atk_coef = 0.1f;
    float buf_l[kLookahead + 1] = {};
    float buf_r[kLookahead + 1] = {};
    int   write = 0;
};

} // namespace mastering_dsp
