/**
 * mastering_dsp.h — Stereo-linked mastering chain: EQ -> Compressor ->
 * Tape Saturation -> Output Trim -> Limiter -> TPDF Dither.
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

/**
 * Dither depth the module ships with, in LSBs at 24-bit.
 *
 * This is not a user control. Dither at the output of a Eurorack module can
 * only ever be a formality: the jacks carry an analog voltage, not a stored
 * 24-bit deliverable, and anything an ADC upstream of us contributed is
 * already ~80 LSBs of noise sitting on the signal — three orders of magnitude
 * above anything we add here. It is kept because it is correct at the point
 * of quantisation and costs two xorshift steps per sample, not because it is
 * audible. It never was, at any knob position, which is why the knob is gone.
 */
constexpr float kDitherLsb = 2.f;

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
    /* The module always passes kDitherLsb; there is no control for it. The
     * field stays parameterised only so the test harness can set it to 0 and
     * get a bit-deterministic chain for comparisons that would otherwise be
     * swamped by the noise. */
    float dither_lsb;
    bool  lim_bypass;
};

/**
 * Envelope analysis published for the CV jacks (mod_source's Analysis mode).
 * Bands are linear amplitude; the gain reductions are positive dB.
 *
 * Metering only — nothing here feeds back into the audio.
 */
struct ChainTelemetry {
    float low        = 0.f;
    float mid        = 0.f;
    float high       = 0.f;
    float broad      = 0.f;
    float comp_gr_db = 0.f;
    float lim_gr_db  = 0.f;
};

void  Init(float sample_rate);
void  SetEq(const EqParams&);       // control-rate; recomputes coeffs
void  SetComp(const CompParams&);   // control-rate; ms→coef, dB→lin
void  SetSat(const SatParams&);     // control-rate; designs emphasis + bump
void  SetOutput(const OutParams&);
float CompGainReductionDb();        // >= 0 dB GR, for optional LED meter
float LimGainReductionDb();         // >= 0 dB GR, derived from the applied gain

/** Control-rate: follower times for the analysis path, plus its enable gate.
 *  Disabled is the default and costs one branch per sample. */
void  SetAnalysis(float attack_ms, float release_ms, bool enabled);

/** Snapshot the analysis envelopes and both gain reductions. */
void  ReadTelemetry(ChainTelemetry&);
void  Process(daisy::AudioHandle::InputBuffer in,
              daisy::AudioHandle::OutputBuffer out, size_t n);
}
