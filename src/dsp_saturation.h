/**
 * dsp_saturation.h — Header-only tape saturator.
 *
 * Signal path, per channel:
 *
 *   in ─┬─ emphasis ─ drive ─ [x2] ─ tanh/ADAA ─ [/2] ─ comp ─ DC ─ de-emph ─ bump ─┐
 *       └─────────────── 15-sample dry delay ─────────────────────────────────── mix ─ out
 *
 * What makes this tape rather than a waveshaper is the emphasis pair. A record
 * head pre-emphasises, the medium saturates, the playback head de-emphasises by
 * exactly the inverse amount. High frequencies therefore reach the nonlinearity
 * hotter and saturate first, and the harmonics they generate are then cut on the
 * way out — so drive produces high-frequency *compression* rather than
 * high-frequency fizz. That is the mechanism behind tape softening transients,
 * and no memoryless curve reproduces it at any drive setting.
 *
 * The de-emphasis is built with InvertBiquad() of whatever coefficients the
 * audio side is currently holding, rather than designed as a cut. On a static
 * setting the two agree exactly — the matched shelf design is reciprocal by
 * construction. They diverge while the Emphasis knob is moving, because the
 * coefficients are eased per block and a lerp of two designs is not the design
 * of the lerped parameter. Inverting what is actually in the filter keeps the
 * pair exact through the whole ramp, which is what lets the stage stay
 * inaudible at zero drive no matter where Emphasis is set or is heading.
 *
 * The shaper is tanh, ADAA'd on F(x) = ln(cosh x). Magnetic media follow the
 * Langevin function coth(x) - 1/x, and tanh is its standard approximation
 * (Langevin's 0/0 at the origin needs a series expansion and buys nothing
 * audible). Being C-infinity, its harmonics decay geometrically instead of
 * trailing off in the slow high-order tail a piecewise polynomial's derivative
 * discontinuity produces — and it approaches +/-1 asymptotically, so unlike a
 * clipper it never goes flat.
 *
 * ADAA (Parker et al., DAFx-16) evaluates the integral-mean of f over
 * [x[n-1], x[n]] rather than f(x[n]):
 *
 *   y[n] = (F(x[n]) - F(x[n-1])) / (x[n] - x[n-1])
 *
 * falling back to f of the midpoint when the quotient is ill-conditioned. On top
 * of 2x oversampling that puts alias products below the harmonics the stage is
 * deliberately generating.
 *
 * Note what is NOT here: the ADAA history is never reset. Character selects a
 * knee constant that scales the shaper's *input*; the shaper function itself is
 * the same tanh for every character, so a stored (x, F(x)) pair stays valid
 * across any parameter change. The previous design reset it in Configure(),
 * which the control loop calls every 16 ms unconditionally — one wrong sample
 * 62.5 times a second, measured at 31-35 dB below program. Keeping one shaper
 * family removes the failure mode rather than guarding it.
 *
 * Gain compensation divides by the same knee*drive that multiplied the input,
 * so small-signal gain is exactly unity at every drive setting and every
 * character. Drive changes how hard the tape is hit, not how loud the module is.
 * That holds while the knob is MOVING as well, which takes one deliberate
 * choice rather than none — see the note over inv_drive_ in ProcessSample.
 */

#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include "mastering_dsp.h"
#include "dsp_common.h"
#include "dsp_biquad.h"
#include "dsp_halfband.h"

namespace mastering_dsp {

/**
 * Per-character constants. Same shape as the compressor's character table: the
 * mode index selects an entire machine, not a single coefficient.
 *
 * Faster tape has a shorter wavelength on the medium, so it saturates later,
 * extends further up, and puts its head bump higher and smaller. Slower tape is
 * the opposite, which is why 15 ips is the one everybody means by "tape".
 */
struct SatCharConst {
    float emph_hz, emph_db;          // record-side pre-emphasis shelf
    float bump_hz, bump_db, bump_q;  // playback head bump
    float knee;                      // input scale into tanh
};

constexpr SatCharConst kSatChars[3] = {
    { 3000.f,  6.f, 50.f, 1.5f, 1.2f, 0.8f },   // 30 ips    — tight, extended
    { 2000.f,  9.f, 40.f, 3.0f, 1.4f, 1.0f },   // 15 ips    — the classic
    { 1500.f, 12.f, 35.f, 3.0f, 1.6f, 1.4f },   // Saturated — early knee, audible
};

struct Saturator {
    /** Published coefficient set. Only the emphasis shelf is stored; the
     *  de-emphasis is derived from it per block so the pair is an exact inverse
     *  at every point of a knob move, which a lerp of two inverses would not be. */
    struct Coeffs {
        BiquadCoeffs emph, bump;
    };

    void Init(float fs)
    {
        fs_ = fs;
        for (int ch = 0; ch < 2; ++ch) {
            up_[ch].Reset(); down_[ch].Reset(); dry_[ch].Reset();
            emph_s_[ch] = BiquadState{}; deemph_s_[ch] = BiquadState{};
            bump_s_[ch] = BiquadState{};
            adaa_x1_[ch] = 0.f; adaa_F1_[ch] = 0.f;   // F(0) = ln(cosh 0) = 0
            dc_x1_[ch] = dc_y1_[ch] = 0.f;
        }
        dc_r_       = 1.f - (2.f * 3.14159265f * 5.f) / fs;
        param_ease_ = MsToCoef(5.f,  fs);
        asym_ease_  = MsToCoef(30.f, fs);   // see ProcessSample

        // Force the next Configure() to publish. Init() is reachable from
        // Configure() on a sample-rate change, and without this the dirty check
        // there would see unchanged emphasis/bump values, return early, and
        // leave the audio side holding coefficients designed for the OLD rate.
        // Unreachable on the module, where Init() runs once — but it is one
        // store to close a trap that reads as correct.
        designed_valid_ = false;
    }

    /**
     * Control rate. Eases the two parameters that become filter coefficients,
     * dirty-checks them, and publishes a new set only when something actually
     * moved — the same reasoning as SetEq: the pots feed this straight from the
     * ADC and a biquad redesigned from jitter every 16 ms zippers.
     *
     * The scalars below the design are plain aligned floats read by the ISR.
     * They cannot tear, and every one of them is eased at 5 ms on the audio
     * side, so they need none of the double-buffering the coefficients get.
     */
    void Configure(const SatParams& p, float fs)
    {
        if (fs != fs_) Init(fs);

        const uint8_t ch_idx = (p.character < 3u) ? p.character : 0u;
        const SatCharConst& k = kSatChars[ch_idx];

        const float drive_lin = DbToLin(Clampf(p.drive_db, 0.f, 24.f));
        t_drive_    = k.knee * drive_lin;
        // Bypass is not a branch — it is a mix target of zero. The audio side
        // eases mix at 5 ms, so toggling bypass crossfades to the delayed dry
        // instead of stepping to it. A hard switch would step by however much
        // the stage was compressing at the time (measured at 0.038 against a
        // tone slew of 0.011, i.e. an audible click) and there is no reason to
        // pay that when the ease is already there.
        t_mix_      = p.bypass ? 0.f : Clampf(p.mix, 0.f, 1.f);
        t_asym_     = Clampf(p.asym, -0.3f, 0.3f);
        t_asym_sh_  = tanhf(t_asym_);

        // ~30 ms glide at the 16 ms control frame, matching SetEq.
        constexpr float kGlide = 0.41f;
        const float e_t = Clampf(p.emphasis, 0.f, 1.f);
        const float b_t = Clampf(p.bump,     0.f, 1.f);
        if (!smooth_primed_) {
            e_cur_ = e_t; b_cur_ = b_t;      // boot and preset load snap
            smooth_primed_ = true;
        } else {
            e_cur_ += kGlide * (e_t - e_cur_);
            b_cur_ += kGlide * (b_t - b_cur_);
        }

        constexpr float kEps = 0.0005f;      // far below one LED step of the pot
        if (designed_valid_ && ch_idx == designed_char_
            && fabsf(e_cur_ - designed_e_) < kEps
            && fabsf(b_cur_ - designed_b_) < kEps)
            return;

        Coeffs s;
        s.emph = MakeHighShelf<float>(k.emph_hz, e_cur_ * k.emph_db, fs_);
        s.bump = MakePeaking<float>(k.bump_hz, b_cur_ * k.bump_db, k.bump_q, fs_);

        const uint32_t w = 1u - live_.load(std::memory_order_relaxed);
        bank_[w] = s;
        live_.store(w, std::memory_order_release);

        designed_char_  = ch_idx;
        designed_e_     = e_cur_;
        designed_b_     = b_cur_;
        designed_valid_ = true;
    }

    /**
     * Once per block: pick up the published coefficients and ease toward them,
     * turning the 16 ms control staircase into a ~3 ms ramp. Safe to interpolate
     * raw direct-form coefficients for the reason spelled out over LerpCoeffs.
     */
    void BeginBlock()
    {
        const Coeffs& tgt = bank_[live_.load(std::memory_order_acquire)];
        if (!run_primed_) {
            run_ = tgt;                      // first block snaps, never ramps
            run_primed_ = true;
        } else {
            constexpr float kEase = 0.15f;   // tau ~= 6 blocks ~= 3 ms
            LerpCoeffs(run_.emph, tgt.emph, kEase);
            LerpCoeffs(run_.bump, tgt.bump, kEase);
        }
        deemph_ = InvertBiquad(run_.emph);
    }

    /**
     * ln(cosh x), the antiderivative of tanh.
     *
     * Three regimes, and the small one is not an optimisation — it is a
     * correctness fix. The closed form sums terms of order ln2 to produce a
     * result of order x²/2, so for small x it cancels catastrophically: at
     * x = 0.004 the true value is 8e-6 while float carries the ln2 term to only
     * ~6e-8 absolute, leaving under two correct digits. ADAA then *differences*
     * two of these and divides by a small dx, so the error lands straight in the
     * audio. Measured as 0.06 dB of small-signal gain error before the series
     * went in — small, but it meant "unity gain at any drive" was not true.
     *
     * The series is also the fast path: no transcendentals, and most program
     * material lives there.
     *
     *   ln(cosh x) = x²/2 - x⁴/12 + x⁶/45 - 17x⁸/2520 + ...
     *
     * good to ~1e-8 relative at the |x| = 0.5 crossover, where the closed form
     * has become accurate in turn. Past |x| = 8 the log1p term is below float
     * epsilon and drops out entirely.
     */
    static float Antideriv(float x)
    {
        const float a = fabsf(x);
        if (a < 0.5f) {
            const float x2 = x * x;
            return x2 * (0.5f + x2 * (-1.f / 12.f
                       + x2 * (1.f / 45.f - x2 * (17.f / 2520.f))));
        }
        if (a > 8.f) return a - 0.69314718f;
        return a + log1pf(expf(-2.f * a)) - 0.69314718f;
    }

    void ProcessSample(float& l, float& r)
    {
        // Shared eased scalars — computed once, used by both channels.
        //
        // inv_drive_ is DERIVED from drive_, never eased alongside it. Easing
        // the two independently is the natural-looking version and it breaks
        // the stage's central claim: the compensation only cancels the drive
        // when their product is 1, and two one-poles converging to D and 1/D
        // from a common start do not keep that product at 1 on the way. A step
        // from 0 to 24 dB of drive put the midpoint at 8.42 * 0.53 = 4.5,
        // i.e. **+13 dB of small-signal gain** for the length of the ease —
        // measured at +12.5 dB through the whole chain, which is a level jump
        // on a knob whose entire point is that it does not change level. One
        // divide per sample (~14 cycles on the M7) buys an exact identity.
        drive_    += param_ease_ * (t_drive_ - drive_);
        inv_drive_ = 1.f / drive_;          // drive_ >= knee_min = 0.8, never 0
        mix_      += param_ease_ * (t_mix_  - mix_);

        // The offset pair gets its own, slower ease, and that is not cosmetic
        // either. asym_ is added at the shaper's input while asym_sh_ is
        // subtracted from the down-sampler's output, so the compensation leads
        // the thing it compensates by the half-band's group delay — 7.5 base
        // samples, a fraction of a sample, not something a delay line fixes
        // tidily. What is left is a DC error of (rate * 7.5), so the cure is to
        // hold the rate down: at 5 ms a full-travel snap left 0.018 of DC (a
        // -35 dBFS thump through the DC blocker), at 30 ms it is a sixth of
        // that. Asym is a flavour control; nobody can hear it arrive 25 ms late.
        asym_     += asym_ease_ * (t_asym_    - asym_);
        asym_sh_  += asym_ease_ * (t_asym_sh_ - asym_sh_);

        l = Channel(l, 0);
        r = Channel(r, 1);
    }

    float Channel(float in, int ch)
    {
        // Record side: pre-emphasis, then drive. Both linear, so the order
        // between them is free; emphasis first keeps drive a pure scalar.
        const float e = BiquadProcess(emph_s_[ch], run_.emph, in) * drive_;

        float u0, u1;
        up_[ch].Process(e, u0, u1);
        const float s0 = Shape(u0 + asym_, ch);
        const float s1 = Shape(u1 + asym_, ch);
        float w = down_[ch].Process(s0, s1);

        // Undo the drive so the stage is unity at small signal, and remove the
        // shaper's response to the offset alone.
        w = (w - asym_sh_) * inv_drive_;

        // Block what the nonlinearity made of the offset before it reaches the
        // filters. The static subtraction above handles the steady state; this
        // handles what the shaper does to it under signal.
        const float b = w - dc_x1_[ch] + dc_r_ * dc_y1_[ch];
        dc_x1_[ch] = w;
        dc_y1_[ch] = b;

        // Playback side: exact inverse of the emphasis, then the head bump.
        float wet = BiquadProcess(deemph_s_[ch], deemph_, b);
        wet       = BiquadProcess(bump_s_[ch], run_.bump, wet);

        // The dry delay always runs, so bypass costs the same latency as the
        // wet path and cannot click, and a partial mix stays phase-coherent
        // instead of comb-filtering.
        const float dry = dry_[ch].Process(in);
        return dry + mix_ * (wet - dry);
    }

    /** tanh via first-order ADAA. The threshold is set by float cancellation in
     *  the numerator, not by the maths: F is O(1) while the difference is O(dx),
     *  so below ~1e-4 the quotient loses more precision than the antialiasing is
     *  worth. The signal is barely moving there anyway, which is exactly when
     *  the pointwise value is already correct. */
    float Shape(float x, int ch)
    {
        const float Fx = Antideriv(x);
        const float dx = x - adaa_x1_[ch];
        const float y  = (fabsf(dx) < 1e-4f)
                       ? tanhf(0.5f * (x + adaa_x1_[ch]))
                       : (Fx - adaa_F1_[ch]) / dx;
        adaa_x1_[ch] = x;
        adaa_F1_[ch] = Fx;
        return y;
    }

    /* ── control-side ────────────────────────────────────────────────────── */
    float    fs_ = 48000.f;
    Coeffs   bank_[2];
    std::atomic<uint32_t> live_{0};
    float    e_cur_ = 0.f, b_cur_ = 0.f;
    float    designed_e_ = 0.f, designed_b_ = 0.f;
    uint8_t  designed_char_ = 0xFF;
    bool     designed_valid_ = false, smooth_primed_ = false;

    /* ── audio-side ──────────────────────────────────────────────────────── */
    Coeffs       run_;
    BiquadCoeffs deemph_;
    bool         run_primed_ = false;

    float t_drive_ = 1.f, t_mix_ = 1.f;
    float t_asym_ = 0.f, t_asym_sh_ = 0.f;
    float drive_ = 1.f, inv_drive_ = 1.f, mix_ = 1.f, asym_ = 0.f, asym_sh_ = 0.f;
    float param_ease_ = 0.01f, asym_ease_ = 0.002f, dc_r_ = 0.999f;

    HalfBandUp   up_[2];
    HalfBandDown down_[2];
    DryDelay     dry_[2];
    BiquadState  emph_s_[2], deemph_s_[2], bump_s_[2];
    float        adaa_x1_[2] = {0.f, 0.f}, adaa_F1_[2] = {0.f, 0.f};
    float        dc_x1_[2] = {0.f, 0.f}, dc_y1_[2] = {0.f, 0.f};
};

} // namespace mastering_dsp
