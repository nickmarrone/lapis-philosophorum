/**
 * dsp_compressor.h — Header-only log-domain feed-forward glue compressor for
 * the end of a mastering chain. Two engines, three characters, no latency.
 *
 * The topology is the one Giannoulis, Massberg & Reiss recommend in "Digital
 * Dynamic Range Compressor Design — A Tutorial and Analysis", JAES 60(6):
 * 399-408 (2012) §III-B: everything in dB, gain computer FIRST and envelope
 * smoothing SECOND. Smoothing the *level* instead would make a 10:1 setting
 * attack five times faster than a 2:1 setting at the same knob position;
 * smoothing the *gain* makes attack and release mean the same thing at every
 * ratio.
 *
 *   sidechain HPF -> power-sum link -> log -> gain computer -> smoothing -> exp
 *
 * Four things distinguish it from the textbook version, and all four exist
 * because this sits across a finished mix rather than a single track:
 *
 * 1. POWER-SUM STEREO LINK.  det = 0.5*(l^2 + r^2), not max(|l|,|r|). A real
 *    stereo-linked VCA sums its two sidechain currents; max-linking is a
 *    digital convenience that makes one hard-panned transient duck the whole
 *    image by its full level and audibly collapses the stereo picture.
 *    Correlated (mono) content reads identically either way; a fully
 *    one-sided hit reads 3.01 dB lower, which is the correction. The sqrt
 *    that would turn power back into amplitude is folded into kLog10ScaleP
 *    (10/ln10 instead of 20/ln10), so the better link costs nothing.
 *
 * 2. SIDECHAIN HIGH-PASS.  Without one, kick and bass own the gain envelope
 *    of a full-range mix and the whole record breathes at the kick rate. The
 *    corner is a per-character constant (30 / 60 / 90 Hz, Butterworth); 90 Hz
 *    is the setting on an SSL bus compressor's sidechain filter that produces
 *    the classic "the kick stops pumping the mix" behaviour. The filters sit
 *    on the DETECTOR path only — the audio path is untouched, so this adds no
 *    phase shift and no latency.
 *
 * 3. SMOOTH DECOUPLED PEAK DETECTOR (Giannoulis eq. 17) in place of the
 *    branching one-pole this file used to carry:
 *
 *        y1[n] = max( c[n], y1[n-1] + rel*(c[n] - y1[n-1]) )   release stage
 *        y [n] =      y[n-1] + atk*(y1[n] - y[n-1])            attack stage
 *
 *    The old form picked its coefficient with `(target < gr_db) ? att : rel`.
 *    Its real defect is not a kink at the handover — the increment is
 *    a*(c - gr), which vanishes at the crossing, so the trajectory is smooth
 *    there — but that it starts RELEASING the moment the level dips, twice
 *    per cycle of whatever is playing. The envelope therefore ripples at the
 *    program frequency, and a rippling gain is amplitude modulation of the
 *    whole mix: intermodulation sidebands around everything. Here the max()
 *    holds the peak instead, releasing only at the release rate, and the
 *    second pole smooths what is left.
 *
 *    Measured on a steady tone at 4:1, 20 ms / 200 ms (the harness pins this):
 *
 *        tone     branching pk-pk    decoupled pk-pk
 *         50 Hz       0.140 dB           0.017 dB
 *        100 Hz       0.070 dB           0.005 dB
 *        200 Hz       0.035 dB           0.001 dB
 *
 *    An order of magnitude, and worst exactly where a full-range mix keeps
 *    its energy. The decoupled form also settles ~1.1 dB deeper on the same
 *    input, because it acts on the peak rather than on something drifting
 *    between peak and average.
 *
 * 4. PROGRAM DEPENDENCE, two different ways.  Adaptive drives attack and
 *    release from the running crest factor; Glue runs two release reservoirs
 *    in parallel, one ~12x slower than the other, which is what the "Auto"
 *    mode of an analogue bus compressor does with a second RC network. A sum
 *    of two exponentials recovers fast at first and then trails away for
 *    seconds — a shape a single exponential cannot make, and the one that
 *    reads as glue.
 *
 * SIGN CONVENTION — the one thing to get right in here. This stage carries
 * attenuation as a POSITIVE dB value and negates exactly once, at gr_db,
 * because the literature's decoupled detector is written for a positive
 * control signal. c rising means MORE attenuation, which is the ATTACK
 * direction, which is why the release stage uses fmaxf. In the old negative
 * gain-reduction convention every fmaxf here would be an fminf — that is why
 * the old `(target < gr_db) ? att_coef : rel_coef` was correct. Flip one
 * without the other and you get instant release with a slow attack, which
 * sounds like a broken gate rather than a compressor.
 *
 * Cost: one logf and one expf per sample, plus one logf per kCrestDecim
 * samples. No sqrtf, no powf. Roughly 220 cycles/sample on the M7 against the
 * old design's ~166.
 *
 * Latency: zero, and it must stay zero — mastering_dsp.cpp implements this
 * stage's bypass by taking the SAME-SAMPLE dry signal, so any delay line here
 * would turn bypass into a click and make the module's latency depend on
 * bypass state.
 */

#pragma once
#include <cmath>
#include <cstdint>
#include "mastering_dsp.h"
#include "dsp_common.h"
#include "dsp_biquad.h"

namespace mastering_dsp {

/** Everything a character fixes. Compressor-private: no other stage reads it. */
struct CompCharacterSpec {
    float knee_db;       // total knee width W, dB
    float hpf_hz;        // sidechain high-pass corner, Butterworth Q
    float atk_lo_mul;    // attack  multiplier at crest <= kCrestLoDb
    float atk_hi_mul;    // attack  multiplier at crest >= kCrestHiDb
    float rel_lo_mul;    // release multiplier at crest <= kCrestLoDb
    float rel_hi_mul;    // release multiplier at crest >= kCrestHiDb
    float slow_rel_mul;  // dual-TC: tau_slow / tau_fast   (0 disables)
    float slow_weight;   // dual-TC: weight of the slow reservoir
    float auto_makeup;   // 0..1 fraction of the computed auto-makeup applied
};

/*
 * Knee 6 / 12 / 18 dB: peak departure from the hard-knee curve is W*slope/8,
 * so at 2:1 that is 0.4 / 0.75 / 1.1 dB — onset softness, not a level error.
 * 18 dB is deliberately wide, matching the famously soft "over-easy" knee of
 * an SSL G-series bus compressor.
 *
 * Adaptation polarity: HIGH crest (transient-rich) => SLOWER attack, so the
 * transient survives, and FASTER release, so the gain is back before the next
 * hit instead of pumping. LOW crest (dense, sustained) => faster attack and
 * slower release, i.e. behave as a level rider. Same polarity as the analogue
 * auto modes this is descended from.
 *
 * Glue deliberately does NOT crest-adapt. Its program dependence is entirely
 * the dual time constant; stacking both would make an A/B against Adaptive
 * meaningless.
 *
 * Precise exists so the previous behaviour stays reachable, and because an
 * 18 dB knee makes the threshold knob non-literal — see the note on effective
 * threshold in the user guide.
 */
//                                   knee   hpf    aLo    aHi    rLo    rHi  slowX  wSlow  auto
constexpr CompCharacterSpec kCharSpec[3] = {
    /* 0 Precise  */ {  6.f, 30.f, 1.00f, 1.00f, 1.00f, 1.00f,  0.f, 0.0f, 0.0f },
    /* 1 Adaptive */ { 12.f, 60.f, 0.50f, 3.00f, 2.00f, 0.35f,  0.f, 0.0f, 1.0f },
    /* 2 Glue     */ { 18.f, 90.f, 1.00f, 1.00f, 1.00f, 1.00f, 12.f, 0.4f, 1.0f },
};

static constexpr int   kCrestDecim   = 8;      // crest block runs 1-in-8 (6 kHz)
static constexpr float kCrestLoDb    = 6.f;    // a sine is 3.01; dense masters 8-12
static constexpr float kCrestHiDb    = 18.f;   // sparse / transient-heavy program
static constexpr float kInvCrestSpan = 1.f / (kCrestHiDb - kCrestLoDb);
static constexpr float kAutoRefDb    = -10.f;  // reference bus level for auto-makeup
static constexpr float kScQ          = 0.70710678f;  // Butterworth sidechain

struct Compressor {
    /**
     * Design the three sidechain high-passes. Called once from
     * mastering_dsp::Init(), never from Configure() — the corners are
     * per-character constants, so Configure() only ever selects one by index.
     * That is what keeps the control->audio handoff safe without the EQ's
     * double-buffering: a torn five-coefficient set can be *unstable*, but a
     * torn index cannot exist.
     */
    void InitSidechain(float fs)
    {
        for (int i = 0; i < 3; i++)
            hpf_bank[i] = MakeHighPass(kCharSpec[i].hpf_hz, kScQ, fs);
    }

    /**
     * Gain-computer output as a POSITIVE attenuation in dB.
     *
     *   over  = level_db - threshold_db
     *   slope = 1 - 1/ratio, in [0, 1)   — positive, unlike the old gr_slope
     *
     * Piecewise-quadratic soft knee, Giannoulis eq. (4) rewritten in the
     * positive convention. Continuous in value and in first derivative at both
     * knee edges: at over = +W/2 the quadratic gives slope*W/2 = slope*over and
     * its derivative is slope; at over = -W/2 both are zero.
     *
     * Configure() calls this too, so auto-makeup is computed from the actual
     * curve — knee included — rather than from a hard-knee approximation.
     */
    float AttenDb(float over) const
    {
        if (2.f * over <= -knee_db) return 0.f;
        if (2.f * over >=  knee_db) return comp_slope * over;
        const float t = over + half_knee;
        return comp_slope * t * t * inv_2knee;
    }

    void Configure(const CompParams& p, float fs)
    {
        const uint8_t ch = (p.character < 3) ? p.character : 0;
        const CompCharacterSpec& s = kCharSpec[ch];

        threshold_db = Clampf(p.threshold_db, -60.f, 0.f);
        const float rat = (p.ratio < 1.f) ? 1.f : p.ratio;
        comp_slope   = 1.f - 1.f / rat;         // 0 .. 0.95
        knee_db      = s.knee_db;
        half_knee    = 0.5f * knee_db;
        inv_2knee    = 0.5f / knee_db;

        // Endpoints of the crest adaptation. The live coefficients are blended
        // from these below, using the CURRENT crest estimate, so a control
        // frame reproduces exactly what the crest block would have produced —
        // Configure() never snaps a running adaptation.
        const float atk_ms = Clampf(p.attack_ms, 0.05f, 200.f);
        atk_a_lo = MsToCoef(atk_ms * s.atk_lo_mul, fs);
        atk_a_hi = MsToCoef(atk_ms * s.atk_hi_mul, fs);

        const float rel_ms = Clampf(p.release_ms, 5.f, 4000.f);
        rel_a_lo = MsToCoef(rel_ms * s.rel_lo_mul, fs);
        rel_a_hi = MsToCoef(rel_ms * s.rel_hi_mul, fs);

        // Second, much slower release reservoir for the Glue character. The
        // slow tau is clamped so the 2000 ms end of the Release knob asks for a
        // 4 s tail rather than a 24 s one.
        dual    = (s.slow_rel_mul > 0.f);
        slow_w  = s.slow_weight;
        fast_w  = 1.f - slow_w;
        rel_a_f = MsToCoef(rel_ms, fs);
        rel_a_s = dual ? MsToCoef(Clampf(rel_ms * s.slow_rel_mul, 120.f, 4000.f), fs)
                       : rel_a_f;

        rms_a      = MsToCoef(25.f,  fs);                       // mean-square window
        pk_up_a    = MsToCoef(0.2f,  fs);                       // peak follower, rising
        pk_dn_a    = MsToCoef(150.f, fs);                       // peak follower, falling
        crest_a    = MsToCoef(150.f, fs / float(kCrestDecim));  // adaptation glide
        param_ease = MsToCoef(5.f,   fs);                       // makeup / mix glide

        ApplyAdaptation();

        /*
         * Auto-makeup: evaluate the gain computer at a reference bus level.
         * Because AttenDb() is the same function the audio path uses, the knee
         * is accounted for exactly, and the whole thing costs nothing at audio
         * rate. It is a STATIC compensation against a nominal level, not a
         * measurement of the gain reduction actually happening — dense program
         * whose peaks sit well above kAutoRefDb will still come out a decibel
         * or two shy. That is the deliberate trade: no feedback path, no drift,
         * and the same number every time for a given knob setting.
         */
        const float auto_db = s.auto_makeup * AttenDb(kAutoRefDb - threshold_db);
        makeup_tgt = DbToLin(Clampf(p.makeup_db + auto_db, -12.f, 30.f));
        mix_tgt    = Clampf(p.mix, 0.f, 1.f);

        if (!eased_primed) {          // boot and preset load snap, never glide
            makeup_run    = makeup_tgt;
            mix_run       = mix_tgt;
            eased_primed  = true;
        }

        bypass = p.bypass;

        // Written LAST. Every character-dependent branch below reads this one
        // byte, so an ISR preemption part-way through Configure() sees at worst
        // a single block of new times against the old knee — never a half-built
        // engine.
        character = ch;
    }

    void ProcessSample(float& l, float& r)
    {
        /* 1. Sidechain high-pass, per channel. Per channel rather than on a
         * mono sum, because a sum would cancel out-of-phase low end and hand
         * the power link below a signal that no longer represents either
         * channel. One coefficient set, two state sets. */
        const BiquadCoeffs& hc = hpf_bank[character];
        const float sl = BiquadProcess(hpf_s[0], hc, l);
        const float sr = BiquadProcess(hpf_s[1], hc, r);

        /* 2. Power-sum stereo link, then straight to dB. */
        float pw = 0.5f * (sl * sl + sr * sr);
        if (pw < kMinDetPow) pw = kMinDetPow;
        const float lvl_db = kLog10ScaleP * logf(pw);          // logf, every sample

        /* 3. Crest estimate, tracked entirely in the power domain.
         *
         * Both followers are one-poles on pw, which is what makes this behave:
         * a peak follower running on the *dB* signal would be dragged toward
         * the -90 dB floor on every zero crossing, and would read a wildly
         * inflated crest on low-frequency material. In power, the sag between
         * two peaks of a 40 Hz tone is 0.7 dB rather than 13 dB.
         *
         * In STEADY STATE this is exactly crest factor: for a sine, pk_p -> A^2
         * and rms_p -> A^2/2, so the ratio is 2 and the result is 3.0103 dB.
         * The harness pins that, and a 5 %-duty burst train reads 19 dB.
         *
         * In TRANSITION it is deliberately something more than crest factor,
         * and this is a designed behaviour rather than an artefact to
         * apologise for. The two followers have different decay times (150 ms
         * against 25 ms), so a sustained drop in level leaves pk_p above rms_p
         * for a while and the estimate climbs. Measured by the harness on a
         * 30 dB step down, with a 200 ms release knob:
         *
         *     3 ms after the drop     3.3 dB  ->  release 400 ms
         *     750 ms after           15.3 dB  ->  release  86 ms
         *
         * That is exactly what an analogue auto-release does: once the loud
         * passage ends, get out of the way. The reverse case is quiet, because
         * pk_p and rms_p both rise quickly — a 30 dB step UP moves the estimate
         * by 0.23 dB, so an onset does not jolt the timing.
         *
         * Only the log of the ratio and the coefficient blend are decimated
         * 1-in-8. The adaptation's own time constant is 150 ms, so even 6 kHz
         * oversamples it by three orders of magnitude. */
        rms_p += rms_a * (pw - rms_p);
        pk_p  += ((pw > pk_p) ? pk_up_a : pk_dn_a) * (pw - pk_p);

        if (--crest_ctr <= 0) {
            crest_ctr = kCrestDecim;
            const float raw = kLog10ScaleP * logf(pk_p / rms_p);   // logf, 1-in-8
            crest_sm += crest_a * (raw - crest_sm);
            ApplyAdaptation();
        }

        /* 4. Gain computer — positive attenuation, dB. */
        const float c = AttenDb(lvl_db - threshold_db);

        /* 5. Smooth decoupled peak detector. See the sign note at the top of
         * the file before touching any comparison in here. */
        if (dual) {
            // Two reservoirs discharging in parallel, summed 60/40. After a
            // step release the sum recovers ~60 % at the fast rate and then
            // trails off over seconds.
            y1_f = fmaxf(c, y1_f + rel_a_f * (c - y1_f));
            y1_s = fmaxf(c, y1_s + rel_a_s * (c - y1_s));
            y1   = fast_w * y1_f + slow_w * y1_s;
        } else {
            y1   = fmaxf(c, y1 + rel_a * (c - y1));
        }
        y += atk_a * (y1 - y);

        gr_db = -y;   // the only negation; metering contract unchanged

        /* 6. Ease the two parameters that multiply the audio directly.
         * Threshold, ratio, attack and release are all smoothed for free by the
         * envelope network, which runs 20-2000x slower than the 16 ms control
         * frame. makeup and mix are not — and auto-makeup means makeup now
         * moves whenever threshold or ratio move, so without this a threshold
         * sweep would step the output gain every control frame. tau = 5 ms. */
        makeup_run += param_ease * (makeup_tgt - makeup_run);
        mix_run    += param_ease * (mix_tgt    - mix_run);

        /* 7. Apply. Everything above runs even when bypassed, so the envelope
         * and the sidechain filters stay warm across a bypass toggle. */
        if (!bypass) {
            const float g  = (y < 0.01f) ? makeup_run
                                         : makeup_run * expf(-kDbToLn * y);
            const float wl = l * g, wr = r * g;
            // Linear crossfade, deliberately: wet is a gain-modulated copy of
            // dry and therefore fully correlated with it, so an equal-power law
            // would bump the sum +1.5 dB at mix = 0.5.
            l += mix_run * (wl - l);
            r += mix_run * (wr - r);
        }
    }

    /** Blend the timing coefficients from the current crest estimate.
     *
     *  Linear in the COEFFICIENT, which is harmonic rather than geometric in
     *  tau (alpha ~ 1/(tau*fs) once tau*fs >> 1). The endpoints are exact; the
     *  midpoint of the 2.0/0.35 release pair lands at 0.60x rather than the
     *  geometric 0.84x. That bias toward the fast end is wanted for a release
     *  whose job is to get out of the way, and it costs no transcendental. */
    void ApplyAdaptation()
    {
        const float u = Clampf((crest_sm - kCrestLoDb) * kInvCrestSpan, 0.f, 1.f);
        atk_a = atk_a_lo + u * (atk_a_hi - atk_a_lo);
        rel_a = rel_a_lo + u * (rel_a_hi - rel_a_lo);
    }

    float gr_db = 0.f;    // smoothed gain reduction in dB, always <= 0 (metering)

    // Detector
    BiquadCoeffs hpf_bank[3];
    BiquadState  hpf_s[2];
    float rms_p    = kMinDetPow;
    float pk_p     = kMinDetPow;
    float crest_sm = 12.f;   // mid-window at boot; 150 ms to settle from there
    int   crest_ctr = 0;     // 0 => the crest block runs on the very first sample

    // Envelope — POSITIVE attenuation, dB
    float y = 0.f, y1 = 0.f, y1_f = 0.f, y1_s = 0.f;

    // Coefficients
    float threshold_db = 0.f, comp_slope = 0.f;
    float knee_db = 6.f, half_knee = 3.f, inv_2knee = 1.f / 12.f;
    float atk_a = 0.f, atk_a_lo = 0.f, atk_a_hi = 0.f;
    float rel_a = 0.f, rel_a_lo = 0.f, rel_a_hi = 0.f;
    float rel_a_f = 0.f, rel_a_s = 0.f, fast_w = 1.f, slow_w = 0.f;
    float rms_a = 0.f, pk_up_a = 0.f, pk_dn_a = 0.f, crest_a = 0.f, param_ease = 0.f;
    float makeup_tgt = 1.f, makeup_run = 1.f;
    float mix_tgt = 1.f, mix_run = 1.f;
    uint8_t character = 0;
    bool  dual = false, bypass = false, eased_primed = false;
};

} // namespace mastering_dsp
