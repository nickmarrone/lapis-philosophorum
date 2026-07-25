/**
 * mastering_dsp.h — Stereo-linked mastering chain: EQ -> Compressor ->
 * Tape Saturation -> Limiter -> Output Trim -> TPDF Dither.
 *
 * Chain latency is 75 samples (1.56 ms): 60 of limiter lookahead and 15 of the
 * saturator's oversampling pair. Both are unconditional — every stage's bypass
 * takes a matched-delay dry path, so latency never depends on bypass state.
 * The limiter's ceiling is a true-peak (dBTP) ceiling: its detector runs 4x
 * oversampled, which is where 12 of its 60 samples of lookahead go.
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
/**
 * Saturation character — selects the tape machine and every constant that goes
 * with it: pre-emphasis corner and depth, head-bump tuning, and how early the
 * shaper's knee arrives. The table itself is saturator-private; see
 * dsp_saturation.h.
 *
 *   30 ips    — tight low end, extended top, saturates late.
 *   15 ips    — the classic: bigger bump, stronger HF-first compression.
 *   Saturated — low bias, early knee, audible.
 */
enum SatCharacter : uint8_t { kSat30Ips = 0, kSat15Ips = 1, kSatSaturated = 2 };

struct SatParams {
    float   drive_db;       // 0..24
    float   mix;            // 0..1
    float   emphasis;       // 0..1 amount; the character sets the curve
    float   asym;           // -0.3..+0.3 (pre-shaper DC offset)
    float   bump;           // 0..1 amount; the character sets f and Q
    uint8_t character;      // SatCharacter; clamped in Configure
    bool    bypass;
};

struct OutParams {
    float ceiling_db;       // -6..-0.1
    float lim_release_ms;   // 10..500
    float trim_db;          // -12..+12
    float dither_lsb;       // 0..2 (LSBs @ 24-bit)
    bool  lim_bypass;
};

void  Init(float sample_rate);
void  SetEq(const EqParams&);       // control-rate; recomputes coeffs
void  SetComp(const CompParams&);   // control-rate; ms→coef, dB→lin
void  SetSat(const SatParams&);     // control-rate; designs emphasis + bump
void  SetOutput(const OutParams&);
float CompGainReductionDb();        // >= 0 dB GR, for optional LED meter
void  Process(daisy::AudioHandle::InputBuffer in,
              daisy::AudioHandle::OutputBuffer out, size_t n);
}
