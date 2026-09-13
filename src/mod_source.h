/**
 * mod_source.h — six-channel modulation generator for the CV jacks.
 *
 * The mastering chain uses four pots on its Output page and none of the six
 * CV jacks. This fills both gaps: K6 selects one of five modes (plus Off),
 * K5 is that mode's continuous control, and a B3 tap cycles its discrete
 * secondary.
 *
 *   Off           — nothing routed, all six jacks released
 *   Analysis      — 6 out: the chain's own band energies and gain reduction
 *   Clocked       — 2 in (clock, reset), 4 out: sine / tri / ramp / stepped
 *   MultiLfo      — 6 out: six sines at deliberately non-octave ratios
 *   SmoothRandom  — 6 out: six slew-limited random walks
 *   Euclid        — 1 in (clock), 5 out: Euclidean gate patterns
 *
 * This header and its .cpp deliberately include nothing from libDaisy or the
 * alchemy-sdk — only <cstdint> and <cmath>. That is what lets tests/ link the
 * engine directly, the same invariant dsp_biquad.h holds. Everything to do
 * with actual jacks (ADC thresholds, DG411 routing, DAC cadence) lives in
 * mastering.cpp; this file only ever produces volts.
 *
 * Tick() is called from the control thread at a fixed rate, never from audio.
 */

#pragma once

#include <cstdint>

namespace mod_source {

constexpr uint8_t kNumJacks = 6;

/* Bipolar generators swing +/-4 V rather than +/-5 V: J3..J6 are MCP4728-backed
 * and physically bottom out near -4.4 V, so a wider swing would clip on four of
 * the six jacks and make the same mode feel different depending on which jack
 * you patched. 4 V is the largest span every jack can actually reach. */
constexpr float kBipolarVolts = 4.0f;

/* Unipolar generators (envelopes, gates) use the Eurorack 0..+5 V convention. */
constexpr float kUnipolarVolts = 5.0f;

enum class Mode : uint8_t {
    Off = 0,
    Analysis,
    Clocked,
    MultiLfo,
    SmoothRandom,
    Euclid,
    kCount
};

/** Number of B3 zones each mode's secondary cycles through, indexed by Mode.
 *  Off and any future mode without a secondary report 1, which makes the tap
 *  a no-op.
 *
 *  A table rather than a switch so it can be read at compile time: the
 *  firmware describes these to the host as one labelled enum field per mode,
 *  and those label arrays have to agree with these counts or the descriptor
 *  offers a zone the engine will clamp away. mastering.cpp static_asserts
 *  the two against each other. */
constexpr uint8_t kSecondaryZones[static_cast<uint8_t>(Mode::kCount)] = {
    1,  // Off          — nothing to cycle
    2,  // Analysis     — polarity: normal / inverted
    5,  // Clocked      — clock ratio
    3,  // MultiLfo     — ratio set
    4,  // SmoothRandom — rate range
    5,  // Euclid       — step-length set
};

inline uint8_t SecondaryZones(Mode m)
{
    const uint8_t i = static_cast<uint8_t>(m);
    return (i < static_cast<uint8_t>(Mode::kCount)) ? kSecondaryZones[i] : 1u;
}

/**
 * The active mode's controls.
 *
 * `knob_a` is the mode's primary axis and `knob_b` its character; what that
 * means per mode:
 *
 *              knob_a (K3)          knob_b (K4)         secondary (B3)
 *   Analysis   follower speed       sensitivity         polarity
 *   Clocked    shape rotation       shape spread        clock ratio
 *   MultiLfo   base rate            shape               ratio set
 *   SmoothRnd  divergence           smooth <-> stepped  rate range
 *   Euclid     pulse density        gate length         step-length set
 */
struct Params {
    Mode    mode      = Mode::Off;
    float   knob_a    = 0.f;   // K3, 0..1
    float   knob_b    = 0.f;   // K4, 0..1
    uint8_t secondary = 0;     // B3 index, < SecondaryZones(mode)
};

/** Telemetry handed over from the audio chain, consumed by Mode::Analysis.
 *  Linear 0..1 envelopes for the bands, positive dB for the gain reductions. */
struct Analysis {
    float low         = 0.f;
    float mid         = 0.f;
    float high        = 0.f;
    float broad       = 0.f;
    float comp_gr_db  = 0.f;
    float lim_gr_db   = 0.f;
};

/** One tick of output. `is_output[j]` false means the jack is an input (or
 *  unused) this mode and its DG411 must stay open; `volts[j]` is then 0. */
struct Frame {
    float volts[kNumJacks]     = {};
    bool  is_output[kNumJacks] = {};
};

/** Attack/release the Analysis followers should run at, derived from K5.
 *  mastering.cpp forwards these into the audio side; the engine itself never
 *  touches audio. */
struct AnalysisResponse {
    float attack_ms  = 1.f;
    float release_ms = 50.f;
};

/* xorshift32, lifted from the same generator dsp_dither.h uses so there is one
 * PRNG in the project rather than two. Seed must be nonzero. */
struct Rng {
    explicit Rng(uint32_t seed) : state(seed ? seed : 0x1234567u) {}

    uint32_t Next()
    {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        return state;
    }

    /** Uniform in [0, 1). */
    float Unit() { return static_cast<float>(Next() >> 8) * (1.f / 16777216.f); }

    /** Uniform in [-1, 1). */
    float Bipolar() { return Unit() * 2.f - 1.f; }

    uint32_t state;
};

class ModSource {
  public:
    /** @param tick_hz  Rate Tick() will be called at (control thread).
     *  @param seed     Nonzero PRNG seed. */
    void Init(float tick_hz, uint32_t seed);

    void SetParams(const Params& p);
    void SetAnalysis(const Analysis& a) { analysis_ = a; }

    /** Rising edge on the clock jack. @param t_us free-running microseconds. */
    void OnClock(uint32_t t_us);

    /** Rising edge on the reset jack: all phases snap to 0 on the next Tick. */
    void OnReset() { reset_pending_ = true; }

    /** Advance one tick and fill @p out. */
    void Tick(Frame& out);

    /** Follower times Mode::Analysis wants for the current K5 position. */
    AnalysisResponse Response() const;

    /** Full-scale reference in dB for Mode::Analysis, swept by knob_b. */
    float SensitivityDb() const;

    Mode ActiveMode() const { return params_.mode; }

    /* ── Jack roles, so mastering.cpp can drive RouteCvOut without
     *    duplicating the mode table. Both are pure functions of the mode. ── */

    static bool JackIsOutput(Mode m, uint8_t jack);

    /** Jack index carrying the clock input, or 0xFF if this mode has none. */
    static uint8_t ClockJack(Mode m);

    /** Jack index carrying the reset input, or 0xFF if this mode has none. */
    static uint8_t ResetJack(Mode m);

  private:
    void TickAnalysis(Frame& out);
    void TickClocked(Frame& out);
    void TickMultiLfo(Frame& out);
    void TickSmoothRandom(Frame& out);
    void TickEuclid(Frame& out);

    void ApplyReset();

    Params   params_{};
    Analysis analysis_{};

    float dt_ = 1.f / 1000.f;         // seconds per tick

    Rng rng_{0x2545F491u};

    /* Phase bank in turns (0..1), not radians, so wrap detection is a compare.
     * Clocked uses [0] as a single master phase and derives its four outputs
     * by offset; MultiLfo uses all six independently. */
    float phase_[kNumJacks]      = {};
    float prev_phase_[kNumJacks] = {};

    /* ── Clocked ── */
    float    clk_hz_        = 1.f;    // measured incoming clock, Hz
    uint32_t last_clk_us_   = 0;
    bool     have_clk_      = false;
    bool     reset_pending_ = false;

    /* Per-jack sample-and-hold for the bank's stepped slot. One value per jack
     * rather than one shared: the slot can now appear on any output, and two
     * jacks blending into it must not read the same number. */
    float    stepped_[kNumJacks] = {};

    /* ── SmoothRandom ──
     * Each channel interpolates prev -> target with a smoothstep across its
     * own phase, which is what makes the walk C1-continuous rather than a
     * slew-limited staircase. `common_*` is the single shared walk that
     * divergence = 0 collapses every channel onto. */
    float rand_prev_  [kNumJacks] = {};
    float rand_target_[kNumJacks] = {};
    float rand_phase_ [kNumJacks] = {};
    float common_prev_            = 0.f;
    float common_target_          = 0.f;
    float common_phase_           = 0.f;

    /* ── Euclid ── */
    uint32_t clock_count_          = 0;
    float    gate_left_[kNumJacks] = {};   // seconds of gate remaining
};

/* ── Euclidean pattern generation (exposed for the test harness) ──────────
 * Returns a bitmask, LSB = step 0, with `pulses` bits distributed as evenly
 * as possible over `steps` and then rotated by `rotation`. `steps` <= 32.
 *
 * This is the Bresenham form (bit i set iff ((i+1)*pulses) % steps < pulses),
 * not Bjorklund's recursive pairing. The two produce the *same necklace* —
 * they differ only by a rotation — and rotation is already a user parameter
 * here, so the musical result is identical and this version needs no dynamic
 * allocation. What the harness pins is the property that actually matters:
 * maximal evenness, i.e. no two gaps between pulses differ by more than 1. */
uint32_t EuclidPattern(uint8_t steps, uint8_t pulses, uint8_t rotation);

/* ── The shape bank (exposed for the test harness) ────────────────────────
 * Six waveforms arranged in a ring — sine, triangle, ramp up, ramp down,
 * pulse, stepped random — addressed by a continuous position that crossfades
 * between neighbours and wraps, so a knob sweeping the bank has no
 * discontinuity and no dead end.
 *
 * Clocked and MultiLfo both draw from it. Clocked spreads its four outputs
 * across it at a settable gap; MultiLfo puts all six at one position, since
 * its outputs are already differentiated by rate.
 *
 * @param pos    position in the bank; any real number, wrapped into [0, 6).
 * @param phase  oscillator phase in turns, 0..1.
 * @param held   the caller's per-jack sample-and-hold value for the stepped
 *               slot, redrawn on each phase wrap. Ignored by every other slot.
 * @return       -1..+1.
 */
constexpr uint8_t kNumShapes = 6;

float ShapeAt(float pos, float phase, float held);

/* ── Ratio sets for MultiLfo (exposed for the test harness) ─────────────── */
enum class RatioSet : uint8_t { Golden = 0, Prime, Narrow, kCount };
const float* Ratios(RatioSet s);

} // namespace mod_source
