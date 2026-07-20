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
// per-channel state kept warm even while eq_bypass_ is set so re-enabling
// the EQ never pops.
BiquadCoeffs ls_c_, mid_c_, hs_c_;
BiquadState  ls_s_[2], mid_s_[2], hs_s_[2];
bool         eq_bypass_ = false;

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

void SetEq(const EqParams& p)
{
    ls_c_  = MakeLowShelf (p.ls_freq_hz,  p.ls_gain_db,           fs_);
    mid_c_ = MakePeaking  (p.mid_freq_hz, p.mid_gain_db, p.mid_q, fs_);
    hs_c_  = MakeHighShelf(p.hs_freq_hz,  p.hs_gain_db,           fs_);
    eq_bypass_ = p.bypass;
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
    for (size_t i = 0; i < n; i++)
    {
        float l = in[0][i];
        float r = in[1][i];

        // EQ — always runs so filter state stays warm; bypass just
        // discards the wet output below instead of skipping the biquads.
        float el = BiquadProcess(ls_s_[0],  ls_c_,  l);
        el       = BiquadProcess(mid_s_[0], mid_c_, el);
        el       = BiquadProcess(hs_s_[0],  hs_c_,  el);

        float er = BiquadProcess(ls_s_[1],  ls_c_,  r);
        er       = BiquadProcess(mid_s_[1], mid_c_, er);
        er       = BiquadProcess(hs_s_[1],  hs_c_,  er);

        if (!eq_bypass_)
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
