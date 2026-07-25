/**
 * dsp_analysis.h — three-band + broadband envelope analysis, for driving the
 * CV jacks from the program material passing through the chain.
 *
 * This is an ANALYSIS path, not a signal path: nothing here is ever added back
 * into the audio, so the usual mastering-grade constraints do not apply. That
 * is what licenses the cheap choices below.
 *
 * Band split is two one-pole lowpasses on the mono sum:
 *
 *     low  = lp200
 *     mid  = lp2k - lp200
 *     high = x    - lp2k
 *
 * 6 dB/oct, and the bands sum to unity by construction. A steeper split would
 * need a Linkwitz-Riley pair per crossover — four biquads instead of two
 * one-poles — to buy selectivity that an envelope follower cannot use: the
 * follower's own attack/release smears far more than the skirt does. It also
 * avoids adding a MakeLowPass designer to dsp_biquad.h for one caller.
 *
 * Header-only and <cmath>-only, like every other dsp_*.h here, so the host
 * test harness can link it without libDaisy.
 */

#pragma once
#include <cmath>
#include "dsp_common.h"

namespace mastering_dsp {

constexpr float kAnalysisLowHz  = 200.f;
constexpr float kAnalysisHighHz = 2000.f;

struct Analyzer {
    /** Set the crossover coefficients. Call once at Init; they depend only
     *  on the sample rate. */
    void Init(float fs)
    {
        lp_lo_coef_ = OnePoleCoef(kAnalysisLowHz,  fs);
        lp_hi_coef_ = OnePoleCoef(kAnalysisHighHz, fs);
        fs_         = fs;
    }

    /** Control-rate: follower times from K5, plus the enable gate. */
    void Configure(float attack_ms, float release_ms, bool enabled)
    {
        att_ = MsToCoef(attack_ms,  fs_);
        rel_ = MsToCoef(release_ms, fs_);
        on_  = enabled;

        /* Leave the envelopes alone on a parameter change — they are metering
         * state, and zeroing them would make every K5 nudge punch a hole in
         * whatever the CV jacks are driving. */
    }

    /** Audio-rate. Costs nothing when the analysis mode is not selected —
     *  the caller is expected to branch on Enabled() once per block, but the
     *  early-out here keeps the contract safe either way. */
    void ProcessSample(float l, float r)
    {
        if (!on_) return;

        const float x = 0.5f * (l + r);

        lp_lo_ += lp_lo_coef_ * (x - lp_lo_);
        lp_hi_ += lp_hi_coef_ * (x - lp_hi_);

        Follow(low,   fabsf(lp_lo_));
        Follow(mid,   fabsf(lp_hi_ - lp_lo_));
        Follow(high,  fabsf(x - lp_hi_));
        Follow(broad, fabsf(x));
    }

    bool Enabled() const { return on_; }

    /* Linear amplitude envelopes, 0..1-ish. Read from the control thread. */
    float low = 0.f, mid = 0.f, high = 0.f, broad = 0.f;

  private:
    static float OnePoleCoef(float fc, float fs)
    {
        /* 1 - exp(-2*pi*fc/fs); the exact one-pole pole placement rather than
         * the 2*pi*fc/fs approximation, which is already 8 % off at 2 kHz. */
        return 1.f - expf(-6.28318531f * fc / fs);
    }

    void Follow(float& env, float rect) const
    {
        env += (rect > env ? att_ : rel_) * (rect - env);
    }

    float fs_         = 48000.f;
    float lp_lo_coef_ = 0.f, lp_hi_coef_ = 0.f;
    float lp_lo_      = 0.f, lp_hi_      = 0.f;
    float att_        = 0.1f, rel_       = 0.01f;
    bool  on_         = false;
};

} // namespace mastering_dsp
