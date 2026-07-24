/**
 * mastering_dsp.h — Stereo-linked mastering chain: EQ -> Compressor ->
 * Saturation -> Limiter -> Output Trim -> TPDF Dither.
 *
 * The DSP knows nothing about VirtualKnob, ControlLoop, or any SDK surface.
 * The control side computes parameters in engineering units (Hz, dB, ms)
 * and pushes them to the DSP via the Set* setters. The audio callback is a
 * plain libDaisy AudioCallback the caller wires into hw.StartAudio().
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include "daisy_seed.h"

namespace mastering_dsp {

struct EqParams {
    float ls_freq_hz, ls_gain_db;
    float mid_freq_hz, mid_gain_db, mid_q;
    float hs_freq_hz, hs_gain_db;
    bool  bypass;
};
/**
 * Compressor character — selects the engine and every constant that goes with
 * it: knee width, sidechain high-pass corner, timing adaptation, auto-makeup.
 * The table itself is compressor-private; see dsp_compressor.h.
 *
 *   Precise  — no adaptation, narrow knee. The threshold knob is literal.
 *   Adaptive — crest-factor-driven attack and release.
 *   Glue     — SSL-bus-style dual time-constant program-dependent release.
 */
enum CompCharacter : uint8_t { kCompPrecise = 0, kCompAdaptive = 1, kCompGlue = 2 };

struct CompParams {
    float   threshold_db;   // -40..0
    float   ratio;          // 1..20 (the panel tapers this; see mastering.cpp)
    float   attack_ms;      // 0.1..100
    float   release_ms;     // 10..2000
    float   makeup_db;      // 0..20, on top of the character's auto-makeup
    float   mix;            // 0..1
    uint8_t character;      // CompCharacter; clamped in Configure
    bool    bypass;
};
struct OutParams {
    float   drive_db;       // 0..24
    float   asym;           // -0.3..+0.3 (pre-sat DC offset)
    float   ceiling_db;     // -6..-0.1
    float   lim_release_ms; // 10..500
    float   trim_db;        // -12..+12
    float   dither_lsb;     // 0..2 (LSBs @ 24-bit)
    uint8_t sat_type;       // 0 = cubic soft clip, 1 = hard clip
    bool    sat_bypass;
};

void  Init(float sample_rate);
void  SetEq(const EqParams&);       // control-rate; recomputes coeffs
void  SetComp(const CompParams&);   // control-rate; ms→coef, dB→lin
void  SetOutput(const OutParams&);
float CompGainReductionDb();        // >= 0 dB GR, for optional LED meter
void  Process(daisy::AudioHandle::InputBuffer in,
              daisy::AudioHandle::OutputBuffer out, size_t n);
}
