/**
 * mastering_dsp.cpp — Stereo-linked mastering chain: EQ -> Compressor ->
 * Saturation -> Limiter -> Output Trim -> TPDF Dither.
 *
 * Chain state lives in an anonymous namespace at file scope, mirroring the
 * eq_dsp pattern from the template's prior EQ-only DSP file. All unit
 * conversions (dB->lin, ms->coef, coefficient recompute) happen in the
 * Set* setters, called at control rate; Process() only does per-sample
 * math — no transcendentals here beyond what the stage headers
 * themselves guard internally.
 */

#include <atomic>
#include <cmath>

#include "mastering_dsp.h"
#include "dsp_common.h"
#include "dsp_biquad.h"
#include "dsp_compressor.h"
#include "dsp_limiter.h"
#include "dsp_saturation.h"
#include "dsp_dither.h"

namespace mastering_dsp {

namespace {

float fs_ = 48000.f;

// EQ: stereo-linked coefficients (one set shared by both channels), with
// per-channel state kept warm even while bypass is set so re-enabling the EQ
// never pops.
//
// The low shelf runs in double, the other two in float. It is the only band
// whose poles reach radius ~0.998 (20 Hz at 48 kHz), which both amplifies the
// recursion's roundoff by ~1/(1-r) and makes it nearly cancel its own poles
// against its zeros. Measured at 20 Hz: float32 costs 38 dB of noise floor
// and ~1e-2 dB of response error. Mid (200 Hz+) and high shelf (1 kHz+) sit
// nowhere near that regime and gain nothing from the extra precision.
struct EqCoeffSet {
    BiquadCoeffsT<double> ls;
    BiquadCoeffs          mid, hs;
    bool                  bypass = false;
};

// Control -> audio handoff. The main loop fills the bank slot the ISR is *not*
// reading, then publishes it by flipping the index.
//
// A seqlock would be wrong here, and dangerously so: preemption is asymmetric.
// The SAI ISR can interrupt the main loop, but the main loop can never
// interrupt the ISR. A reader that spins waiting for a writer to finish would
// be waiting on a thread that cannot run until the reader returns — deadlock,
// not a glitch. Double-buffering is deadlock-free by construction; the ISR
// always dereferences a slot that was complete before it was published.
//
// The dmb that release/acquire emits is architecturally redundant on a single
// core, since exception entry and return are already context-synchronizing.
// The *compiler* barrier is not: at -O2 nothing otherwise stops GCC sinking or
// reordering the plain bank stores past the index store.
EqCoeffSet            eq_bank_[2];
std::atomic<uint32_t> eq_live_{0};

// Audio-side coefficients, eased toward the published set once per block.
EqCoeffSet eq_run_;
bool       eq_primed_ = false;

BiquadStateT<double> ls_s_[2];
BiquadState          mid_s_[2], hs_s_[2];

// Control-side parameter smoothing, in the domains the knobs actually live in:
// log2(Hz) so a glide is constant-ratio and matches the Exp() taper, dB and
// log2(Q) because both are already logarithmic.
struct EqSmooth {
    float ls_lf = 0.f,  ls_g = 0.f;
    float mid_lf = 0.f, mid_g = 0.f, mid_lq = 0.f;
    float hs_lf = 0.f,  hs_g = 0.f;
};
EqSmooth eq_cur_, eq_designed_;
bool     eq_smooth_primed_   = false;
bool     eq_designed_valid_  = false;
bool     eq_designed_bypass_ = false;

template <typename T>
inline void LerpCoeffs(BiquadCoeffsT<T>& c, const BiquadCoeffsT<T>& t, T k)
{
    c.b0 += k * (t.b0 - c.b0);
    c.b1 += k * (t.b1 - c.b1);
    c.b2 += k * (t.b2 - c.b2);
    c.a1 += k * (t.a1 - c.a1);
    c.a2 += k * (t.a2 - c.a2);
}

Compressor comp_;
Saturator  sat_;
Limiter    lim_;

float trim_lin_ = 1.f;

TpdfDither dith_[2] = {TpdfDither(0x2545F491u), TpdfDither(0x9E3779B9u)};

} // namespace

void Init(float sample_rate)
{
    fs_ = sample_rate;
}

/**
 * Recompute and publish the EQ.
 *
 * Called unconditionally with every current value, every control frame — that
 * property is load-bearing for preset recall (nothing is edge-triggered, so a
 * preset load "just works"), so the dirty check lives in here rather than at
 * the call site.
 *
 * Two things happen before the design:
 *
 *   1. The parameters are eased toward their targets. The pots feed this
 *      straight from the ADC, and a biquad redesigned from jittering inputs
 *      every 16 ms zippers.
 *   2. A dirty check compares against what was last designed. A knob that is
 *      not moving therefore yields a bit-identical coefficient set, frame
 *      after frame — which is what actually silences the residual zipper. The
 *      thresholds are far below audibility (0.1% in frequency, 0.002 dB) and
 *      exist only to decide "has this moved at all".
 */
void SetEq(const EqParams& p)
{
    // ~30 ms glide at the 16 ms control frame: 1 - exp(-16/30).
    constexpr float kGlide = 0.41f;

    EqSmooth t;
    t.ls_lf  = std::log2(Clampf(p.ls_freq_hz,  1.f, 0.4999f * fs_));
    t.ls_g   = Clampf(p.ls_gain_db,  -24.f, 24.f);
    t.mid_lf = std::log2(Clampf(p.mid_freq_hz, 1.f, 0.4999f * fs_));
    t.mid_g  = Clampf(p.mid_gain_db, -24.f, 24.f);
    t.mid_lq = std::log2(Clampf(p.mid_q, 0.1f, 40.f));
    t.hs_lf  = std::log2(Clampf(p.hs_freq_hz,  1.f, 0.4999f * fs_));
    t.hs_g   = Clampf(p.hs_gain_db,  -24.f, 24.f);

    if (!eq_smooth_primed_)
    {
        eq_cur_ = t;                    // boot and preset load snap, never glide
        eq_smooth_primed_ = true;
    }
    else
    {
        eq_cur_.ls_lf  += kGlide * (t.ls_lf  - eq_cur_.ls_lf);
        eq_cur_.ls_g   += kGlide * (t.ls_g   - eq_cur_.ls_g);
        eq_cur_.mid_lf += kGlide * (t.mid_lf - eq_cur_.mid_lf);
        eq_cur_.mid_g  += kGlide * (t.mid_g  - eq_cur_.mid_g);
        eq_cur_.mid_lq += kGlide * (t.mid_lq - eq_cur_.mid_lq);
        eq_cur_.hs_lf  += kGlide * (t.hs_lf  - eq_cur_.hs_lf);
        eq_cur_.hs_g   += kGlide * (t.hs_g   - eq_cur_.hs_g);
    }

    // 0.00144 in log2 is 0.1% in Hz; 0.002 dB is ~1/500th of the finest step
    // the panel can produce.
    constexpr float kFEps = 0.00144f, kGEps = 0.002f;
    const EqSmooth& d = eq_designed_;
    const bool clean =
        eq_designed_valid_ && p.bypass == eq_designed_bypass_
        && std::fabs(eq_cur_.ls_lf  - d.ls_lf ) < kFEps
        && std::fabs(eq_cur_.ls_g   - d.ls_g  ) < kGEps
        && std::fabs(eq_cur_.mid_lf - d.mid_lf) < kFEps
        && std::fabs(eq_cur_.mid_g  - d.mid_g ) < kGEps
        && std::fabs(eq_cur_.mid_lq - d.mid_lq) < kFEps
        && std::fabs(eq_cur_.hs_lf  - d.hs_lf ) < kFEps
        && std::fabs(eq_cur_.hs_g   - d.hs_g  ) < kGEps;
    if (clean)
        return;

    EqCoeffSet s;
    s.ls  = MakeLowShelf<double>(std::exp2(eq_cur_.ls_lf),  eq_cur_.ls_g,  fs_);
    s.mid = MakePeaking(std::exp2(eq_cur_.mid_lf), eq_cur_.mid_g,
                        std::exp2(eq_cur_.mid_lq), fs_);
    s.hs  = MakeHighShelf(std::exp2(eq_cur_.hs_lf), eq_cur_.hs_g, fs_);
    s.bypass = p.bypass;

    const uint32_t w = 1u - eq_live_.load(std::memory_order_relaxed);
    eq_bank_[w] = s;
    eq_live_.store(w, std::memory_order_release);

    eq_designed_        = eq_cur_;
    eq_designed_bypass_ = p.bypass;
    eq_designed_valid_  = true;
}

void SetComp(const CompParams& p)
{
    comp_.Configure(p, fs_);
}

void SetOutput(const OutParams& p)
{
    sat_.Configure(p, fs_);
    lim_.Configure(p.ceiling_db, p.lim_release_ms, fs_);
    trim_lin_ = DbToLin(p.trim_db);
    dith_[0].amp = dith_[1].amp = p.dither_lsb * kLsb24;
}

float CompGainReductionDb()
{
    return -comp_.gr_db;
}

void Process(daisy::AudioHandle::InputBuffer  in,
             daisy::AudioHandle::OutputBuffer out,
             size_t                           n)
{
    // Pick up the published coefficients once per block and ease toward them,
    // turning the 16 ms control staircase into a ~3 ms ramp. Interpolating raw
    // direct-form coefficients is safe here and not by luck: the stability
    // region |a1| < 2, |a1| - 1 < a2 < 1 is a triangle, hence convex, and a
    // linear interpolation is a convex combination — so a blend of two stable
    // biquads is always itself stable. (tests/ asserts this over 650k blends.)
    {
        const EqCoeffSet& tgt = eq_bank_[eq_live_.load(std::memory_order_acquire)];
        if (!eq_primed_)
        {
            eq_run_ = tgt;              // first block snaps; never ramps from identity
            eq_primed_ = true;
        }
        else
        {
            constexpr double kEase = 0.15;   // tau ~= 6 blocks ~= 3 ms
            LerpCoeffs(eq_run_.ls,  tgt.ls,  kEase);
            LerpCoeffs(eq_run_.mid, tgt.mid, float(kEase));
            LerpCoeffs(eq_run_.hs,  tgt.hs,  float(kEase));
        }
        eq_run_.bypass = tgt.bypass;    // discrete; takes effect immediately
    }
    const auto& ls_c     = eq_run_.ls;
    const auto& mid_c    = eq_run_.mid;
    const auto& hs_c     = eq_run_.hs;
    const bool  eq_bypass = eq_run_.bypass;

    for (size_t i = 0; i < n; i++)
    {
        float l = in[0][i];
        float r = in[1][i];

        // EQ — always runs so filter state stays warm; bypass just
        // discards the wet output below instead of skipping the biquads.
        float el = BiquadProcess(ls_s_[0],  ls_c,  l);
        el       = BiquadProcess(mid_s_[0], mid_c, el);
        el       = BiquadProcess(hs_s_[0],  hs_c,  el);

        float er = BiquadProcess(ls_s_[1],  ls_c,  r);
        er       = BiquadProcess(mid_s_[1], mid_c, er);
        er       = BiquadProcess(hs_s_[1],  hs_c,  er);

        if (!eq_bypass)
        {
            l = el;
            r = er;
        }

        comp_.ProcessSample(l, r); // handles its own bypass; detector always runs

        l = sat_.ProcessSample(l, 0);
        r = sat_.ProcessSample(r, 1);

        lim_.ProcessSample(l, r);

        l *= trim_lin_;
        r *= trim_lin_;

        l += dith_[0].Sample();
        r += dith_[1].Sample();

        out[0][i] = l;
        out[1][i] = r;
    }
}

} // namespace mastering_dsp
