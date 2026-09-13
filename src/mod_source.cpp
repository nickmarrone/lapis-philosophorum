/**
 * mod_source.cpp — implementation of the six-channel modulation generator.
 * See mod_source.h for the mode map and the no-SDK-includes invariant.
 */

#include "mod_source.h"

#include <cmath>

namespace mod_source {
namespace {

constexpr float kTwoPi = 6.28318530718f;

/* ── Mode::Analysis ──────────────────────────────────────────────────── */

/* Full-scale reference, swept by K4. A mastering chain's program material sits
 * within ~20 dB of full scale, so the shallow end suits loud material and the
 * deep end is what you want when the chain is nearly idle.
 *
 * Knob up is more sensitive, i.e. a deeper floor, because that is what the
 * word means on the panel. This lived on B3 as three fixed steps 20 dB apart
 * and was the wrong parameter to put there: it is the gain staging, it gets
 * set by ear against whatever is playing, and cycling it blind past two other
 * values to get back where you were is the worst affordance in the section. */
constexpr float kSensFloorShallowDb = -10.f, kSensFloorDeepDb = -70.f;

/* Gain reduction gets its own full scale rather than borrowing the band
 * sensitivity — 20 dB of reduction is already a lot, and tying it to the band
 * reference would make the GR outs go nearly dead at the -60 setting. */
constexpr float kGrFullScaleDb = 20.f;

/* K5 sweeps the followers from fast peak to slow RMS. Both ends are useful:
 * the fast end tracks transients for percussive modulation, the slow end
 * gives a smooth program-level voltage that will not jitter a filter. */
constexpr float kAttackFastMs =   1.f, kAttackSlowMs =  100.f;
constexpr float kReleaseFastMs = 50.f, kReleaseSlowMs = 2000.f;

/* ── Mode::Clocked ───────────────────────────────────────────────────── */

constexpr float kClockRatio[5] = {0.25f, 0.5f, 1.f, 2.f, 4.f};

/* Anything outside this is a patching accident, not a clock. Clamping rather
 * than ignoring means a very slow or very fast clock still produces motion. */
constexpr float kClockHzMin = 0.02f, kClockHzMax = 100.f;

/* ── Mode::SmoothRandom ──────────────────────────────────────────────── */

/* glacial / slow / medium / quick. The table used to stop at 0.25 Hz — a four
 * second segment — so the mode had no fast setting at all and could not do
 * anything nervous or percussive whatever the knobs were doing. */
constexpr float kRandomBaseHz[4] = {0.008f, 0.05f, 0.25f, 1.5f};

/* Depth taper applied to the shared walk at divergence 0, so the six jacks
 * read as one gesture seen at six scales rather than six identical copies. */
constexpr float kCommonDepth[kNumJacks] = {1.0f, 0.88f, 0.76f, 0.64f, 0.52f, 0.40f};

/* Rate fan applied as divergence opens up. */
constexpr float kRandomFan[kNumJacks] = {1.0f, 1.4f, 1.9f, 2.5f, 3.2f, 4.0f};

/* ── Mode::MultiLfo ──────────────────────────────────────────────────── */

/* Golden: phi^-k. Irrational ratios, so the six never come back into phase.
 * Prime: 1/1, 1/3, 1/5, 1/7, 1/11, 1/13 — rational but co-prime, so the
 *   pattern repeats only every 15015 cycles of the fastest.
 * Narrow: all six inside a 1.75:1 span, so they beat slowly against each
 *   other instead of fanning into unrelated speeds.
 * None of the three contains a 2x, 4x or 8x pair, which is the whole point —
 * octave-related LFOs lock into an audible common downbeat, and the mode
 * exists precisely to avoid that. mod_source_test pins this: it sweeps every
 * ordered pair in all three sets against 2x/4x/8x and fails under 2 %.
 *
 * The Narrow set earns its guarantee structurally rather than by inspection.
 * Its extremes are only 1.75:1 apart, so no pair CAN reach 2x. The first draft
 * spanned 1.0..0.44 and quietly contained 0.87/0.44 = 1.977 — a 1.1 % octave,
 * which is exactly the failure the mode is supposed to rule out. */
constexpr float kRatiosGolden[kNumJacks] =
    {1.0f, 0.618034f, 0.381966f, 0.236068f, 0.145898f, 0.090170f};
constexpr float kRatiosPrime[kNumJacks] =
    {1.0f, 1.f / 3.f, 1.f / 5.f, 1.f / 7.f, 1.f / 11.f, 1.f / 13.f};
constexpr float kRatiosNarrow[kNumJacks] =
    {1.0f, 0.893f, 0.797f, 0.719f, 0.638f, 0.571f};

constexpr float kLfoBaseHzMin = 0.01f, kLfoBaseHzMax = 10.f;

/* ── Mode::Euclid ────────────────────────────────────────────────────── */

/* Step-length sets, cycled by B3 and ordered by how long the five patterns
 * take to reline — the LCM of the row, i.e. the full combined cycle.
 *
 * Only the grid changes; density and gate length are the same knobs against
 * all five. Before this the lengths were hardwired to the Classic row, so the
 * only thing you could change about the rhythm was how dense it was.
 *
 * B3 used to hold a rotation applied identically to all five patterns, which
 * is a pure time-shift of the whole ensemble — with no external downbeat to
 * hear it against, four zones that all sound the same. EuclidPattern still
 * takes a rotation because it is a pure function worth testing as one; nothing
 * passes it anything but zero now. */
constexpr uint8_t kEuclidSets[5][5] = {
    { 8,  7,  6,  5,  4},   // tight   — relines every 840 clocks
    {12,  9,  8,  7,  5},   // compact — 2520, busy but with a 4/4 anchor
    {16, 12,  9,  7,  5},   // classic — 5040, the set this shipped with
    {13, 11,  9,  7,  5},   // odd     — 45045, no power of two anywhere
    {17, 15, 13, 11,  9},   // long    — 109395, evolves rather than repeats
};

/* No two rows share a first element, which matters more than it looks: a patch
 * that only uses one output still hears the button move. The Long row started
 * 16, 15, ... for that reason alone — it collided with Classic on jack 1, so
 * those two sets were indistinguishable to anyone patching a single gate. */

/* Floor on gate length. The ceiling is a fraction of the measured clock
 * period rather than a constant, so the knob means the same thing at any
 * tempo. */
constexpr float kGateMinSec = 0.002f;
constexpr float kGateMaxDuty = 0.95f;

/* ── helpers ─────────────────────────────────────────────────────────── */

inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float Clamp01(float x) { return Clamp(x, 0.f, 1.f); }

inline float Wrap01(float x)
{
    x -= static_cast<float>(static_cast<int>(x));
    return x < 0.f ? x + 1.f : x;
}

inline float Lerp(float a, float b, float t) { return a + t * (b - a); }

/* C1-continuous ease, so the random walks have no velocity discontinuity at
 * a segment boundary — that kink is exactly what makes a naive interpolated
 * random sound like a stepped one through a filter. */
inline float SmoothStep(float t) { return t * t * (3.f - 2.f * t); }

/* The same ease, compressed into a window at the head of the segment. A window
 * of 1 is SmoothStep unchanged; shrinking it toward 0 turns the interpolation
 * into a hold, so one control spans liquid drift through eased staircase to
 * hard sample-and-hold without ever introducing a discontinuity. */
inline float EasedStep(float t, float window)
{
    return SmoothStep(Clamp01(t / window));
}

/* Peaks at phase 0 so every waveform in Mode::Clocked lines up on the
 * downbeat when spread is 0. */
inline float SineAt(float phase) { return cosf(kTwoPi * phase); }

inline float TriangleAt(float phase)
{
    const float m = phase < 0.5f ? phase : 1.f - phase;
    return 1.f - 4.f * m;
}

/* Ramp up: resets on the downbeat rather than peaking on it — that reset IS
 * the downbeat marker for a saw. */
inline float RampAt(float phase) { return 2.f * phase - 1.f; }

inline float RampDownAt(float phase) { return 1.f - 2.f * phase; }

inline float PulseAt(float phase) { return phase < 0.5f ? 1.f : -1.f; }

/* One slot of the shape bank. `held` is the caller's per-jack sample-and-hold
 * value, used by the stepped slot; every other slot ignores it. */
inline float ShapeSlot(uint8_t slot, float phase, float held)
{
    switch (slot)
    {
        case 0:  return SineAt(phase);
        case 1:  return TriangleAt(phase);
        case 2:  return RampAt(phase);
        case 3:  return RampDownAt(phase);
        case 4:  return PulseAt(phase);
        default: return held;                       // 5 — stepped random
    }
}

/* Linear amplitude -> 0..1 against a dB floor. */
inline float EnvToNorm(float lin, float floor_db)
{
    if (lin <= 1e-7f) return 0.f;
    const float db = 20.f * log10f(lin);
    return Clamp01((db - floor_db) / -floor_db);
}

} // namespace

/* ── Free functions ──────────────────────────────────────────────────── */

float ShapeAt(float pos, float phase, float held)
{
    /* Wrap into [0, kNumShapes) via the unit interval, so the bank is a ring:
     * sweeping past the stepped slot arrives back at the sine rather than
     * dead-ending, and the knob has no discontinuity anywhere in its travel. */
    pos = Wrap01(pos * (1.f / static_cast<float>(kNumShapes)))
          * static_cast<float>(kNumShapes);

    uint8_t i = static_cast<uint8_t>(pos);
    if (i >= kNumShapes) i = 0;                 // float edge at the wrap point
    const float f = pos - static_cast<float>(i);

    const uint8_t next = static_cast<uint8_t>((i + 1u) % kNumShapes);
    return Lerp(ShapeSlot(i, phase, held), ShapeSlot(next, phase, held), f);
}

const float* Ratios(RatioSet s)
{
    switch (s)
    {
        case RatioSet::Prime:  return kRatiosPrime;
        case RatioSet::Narrow: return kRatiosNarrow;
        default:               return kRatiosGolden;
    }
}

uint32_t EuclidPattern(uint8_t steps, uint8_t pulses, uint8_t rotation)
{
    if (steps == 0 || steps > 32) return 0;
    if (pulses == 0)              return 0;
    if (pulses >= steps)          return (steps == 32) ? 0xFFFFFFFFu
                                                       : ((1u << steps) - 1u);

    uint32_t mask = 0;
    for (uint8_t i = 0; i < steps; i++)
    {
        /* Bresenham: a pulse lands on step i wherever the running fraction
         * i*pulses/steps ticks over an integer. Maximally even by
         * construction — gaps differ by at most one step. */
        const uint32_t here = (static_cast<uint32_t>(i) + 1u) * pulses;
        if (here % steps < pulses)
            mask |= (1u << i);
    }

    /* Rotate pulses toward later steps, so increasing rotation delays the
     * pattern rather than advancing it. */
    const uint8_t r = static_cast<uint8_t>(rotation % steps);
    if (r == 0) return mask;
    const uint32_t full = (steps == 32) ? 0xFFFFFFFFu : ((1u << steps) - 1u);
    return ((mask << r) | (mask >> (steps - r))) & full;
}

/* ── Jack roles ──────────────────────────────────────────────────────── */

uint8_t ModSource::ClockJack(Mode m)
{
    return (m == Mode::Clocked || m == Mode::Euclid) ? 0u : 0xFFu;
}

uint8_t ModSource::ResetJack(Mode m)
{
    return (m == Mode::Clocked) ? 1u : 0xFFu;
}

bool ModSource::JackIsOutput(Mode m, uint8_t jack)
{
    if (jack >= kNumJacks)  return false;
    if (m == Mode::Off)     return false;
    if (jack == ClockJack(m)) return false;
    if (jack == ResetJack(m)) return false;
    return true;
}

/* ── Lifecycle ───────────────────────────────────────────────────────── */

void ModSource::Init(float tick_hz, uint32_t seed)
{
    dt_  = (tick_hz > 0.f) ? (1.f / tick_hz) : 0.001f;
    rng_ = Rng(seed);

    for (uint8_t i = 0; i < kNumJacks; i++)
    {
        phase_[i]       = 0.f;
        prev_phase_[i]  = 0.f;
        rand_prev_[i]   = rng_.Bipolar();
        rand_target_[i] = rng_.Bipolar();
        /* Stagger the initial phases so the six walks do not all turn a
         * corner on the same tick right after boot. */
        rand_phase_[i]  = rng_.Unit();
        gate_left_[i]   = 0.f;
    }
    common_prev_   = rng_.Bipolar();
    common_target_ = rng_.Bipolar();
    common_phase_  = 0.f;
    for (uint8_t i = 0; i < kNumJacks; i++) stepped_[i] = rng_.Bipolar();
}

void ModSource::SetParams(const Params& p)
{
    const bool mode_changed = (p.mode != params_.mode);
    params_ = p;

    const uint8_t zones = SecondaryZones(params_.mode);
    if (params_.secondary >= zones) params_.secondary = 0;
    params_.knob_a = Clamp01(params_.knob_a);
    params_.knob_b = Clamp01(params_.knob_b);

    /* A mode change restarts the generators rather than resuming mid-gesture,
     * so switching in reads as a deliberate new patch instead of picking up
     * whatever phase the previous mode happened to leave behind. */
    if (mode_changed)
    {
        for (uint8_t i = 0; i < kNumJacks; i++)
        {
            phase_[i]      = 0.f;
            prev_phase_[i] = 0.f;
            gate_left_[i]  = 0.f;
        }
        clock_count_ = 0;
    }
}

void ModSource::OnClock(uint32_t t_us)
{
    clock_count_++;

    if (have_clk_)
    {
        /* Unsigned subtraction, so a 32-bit microsecond wrap (every ~71 min)
         * still yields the correct interval instead of a giant one. */
        const uint32_t dt_us = t_us - last_clk_us_;
        if (dt_us > 0u)
            clk_hz_ = Clamp(1000000.f / static_cast<float>(dt_us),
                            kClockHzMin, kClockHzMax);
    }
    last_clk_us_ = t_us;
    have_clk_    = true;

    if (params_.mode == Mode::Euclid)
    {
        /* Gate length spans a 2 ms trigger to almost the whole clock period,
         * as a fraction of the measured period so the knob means the same
         * thing at any tempo.
         *
         * This used to be derived as half the period and then clamped to
         * 100 ms, which meant the clamp won across the entire normal musical
         * range rather than the extremes it was written for: at 2 Hz — 120 BPM
         * eighths — "half the period" came out as 20 %, and at 1 Hz as 10 %.
         * Every pattern was a short trigger no matter the tempo, and no knob
         * could say otherwise. */
        const float max_gate = kGateMaxDuty / clk_hz_;
        const float min_gate = max_gate < kGateMinSec ? max_gate : kGateMinSec;
        const float gate     = Lerp(min_gate, max_gate, params_.knob_b);

        const uint8_t* lengths = kEuclidSets[params_.secondary % 5];

        for (uint8_t k = 0; k < 5; k++)
        {
            const uint8_t steps  = lengths[k];
            const uint8_t pulses = static_cast<uint8_t>(
                params_.knob_a * static_cast<float>(steps) + 0.5f);
            const uint32_t pattern = EuclidPattern(steps, pulses, 0);
            const uint8_t  step    = static_cast<uint8_t>(clock_count_ % steps);
            if (pattern & (1u << step))
                gate_left_[k + 1] = gate;   // jack 0 is the clock input
        }
    }
}

void ModSource::ApplyReset()
{
    reset_pending_ = false;
    for (uint8_t i = 0; i < kNumJacks; i++)
    {
        phase_[i]      = 0.f;
        prev_phase_[i] = 0.f;
        stepped_[i]    = rng_.Bipolar();
    }
}

/* ── Response / sensitivity ──────────────────────────────────────────── */

AnalysisResponse ModSource::Response() const
{
    const float k = params_.knob_a;
    AnalysisResponse r;
    /* Geometric interpolation — these span two decades, so a linear sweep
     * would spend most of the knob in the slow half. */
    r.attack_ms  = kAttackFastMs  * powf(kAttackSlowMs  / kAttackFastMs,  k);
    r.release_ms = kReleaseFastMs * powf(kReleaseSlowMs / kReleaseFastMs, k);
    return r;
}

float ModSource::SensitivityDb() const
{
    return Lerp(kSensFloorShallowDb, kSensFloorDeepDb, params_.knob_b);
}

/* ── Tick ────────────────────────────────────────────────────────────── */

void ModSource::Tick(Frame& out)
{
    for (uint8_t i = 0; i < kNumJacks; i++)
    {
        out.volts[i]     = 0.f;
        out.is_output[i] = JackIsOutput(params_.mode, i);
    }

    if (reset_pending_) ApplyReset();

    switch (params_.mode)
    {
        case Mode::Analysis:     TickAnalysis(out);     break;
        case Mode::Clocked:      TickClocked(out);      break;
        case Mode::MultiLfo:     TickMultiLfo(out);     break;
        case Mode::SmoothRandom: TickSmoothRandom(out); break;
        case Mode::Euclid:       TickEuclid(out);       break;
        default:                                        break;   // Off
    }
}

void ModSource::TickAnalysis(Frame& out)
{
    const float floor_db = SensitivityDb();

    out.volts[0] = kUnipolarVolts * EnvToNorm(analysis_.low,   floor_db);
    out.volts[1] = kUnipolarVolts * EnvToNorm(analysis_.mid,   floor_db);
    out.volts[2] = kUnipolarVolts * EnvToNorm(analysis_.high,  floor_db);
    out.volts[3] = kUnipolarVolts * EnvToNorm(analysis_.broad, floor_db);

    out.volts[4] = kUnipolarVolts *
                   Clamp01(analysis_.comp_gr_db / kGrFullScaleDb);
    out.volts[5] = kUnipolarVolts *
                   Clamp01(analysis_.lim_gr_db / kGrFullScaleDb);

    /* Inverted polarity is the duck-on-loud patch: full volts at silence,
     * falling to zero as the chain fills up. There is no way to get it
     * downstream without an external inverter, and these outputs are unipolar
     * so they can only ever push in one direction on their own. Applied to
     * the gain-reduction pair as well, where it reads as headroom remaining
     * rather than reduction applied. */
    if (params_.secondary != 0)
        for (uint8_t j = 0; j < kNumJacks; j++)
            out.volts[j] = kUnipolarVolts - out.volts[j];
}

void ModSource::TickClocked(Frame& out)
{
    const float ratio = kClockRatio[params_.secondary % 5];

    /* Free-runs at clk_hz_'s 1 Hz default until a clock actually arrives.
     * Gating on have_clk_ instead froze every output at DC, so selecting this
     * mode with nothing patched looked like a dead module — the only one of
     * the five that did not move on its own. */
    const float hz = clk_hz_ * ratio;

    prev_phase_[0] = phase_[0];
    phase_[0]      = Wrap01(phase_[0] + hz * dt_);
    const bool wrapped = phase_[0] < prev_phase_[0];

    /* All four outputs share the one phase and differ only in where they sit
     * in the shape bank: rotation slides the whole set around the ring,
     * spread opens the gap between adjacent members. Full spread is 1.5 slots
     * apart, which is 6/4 — the four spaced evenly around the six-slot ring.
     * At spread 0 they collapse onto one shape and the four jacks carry
     * identical voltages, which is the mono-bus case and is deliberate. */
    const float rot = params_.knob_a * static_cast<float>(kNumShapes);
    const float gap = params_.knob_b * (static_cast<float>(kNumShapes) / 4.f);

    /* Jacks 0 and 1 are clock and reset in; the four outputs are 2..5. */
    for (uint8_t k = 0; k < 4; k++)
    {
        const uint8_t j = static_cast<uint8_t>(k + 2);

        /* Each jack holds its own S&H value, drawn independently on the
         * shared downbeat, so the stepped slot stays live wherever it lands
         * in the blend instead of being one output's private waveform. */
        if (wrapped) stepped_[j] = rng_.Bipolar();

        out.volts[j] = kBipolarVolts *
            ShapeAt(rot + static_cast<float>(k) * gap, phase_[0], stepped_[j]);
    }
}

void ModSource::TickMultiLfo(Frame& out)
{
    const float base = kLfoBaseHzMin *
        powf(kLfoBaseHzMax / kLfoBaseHzMin, params_.knob_a);

    const float* ratios = Ratios(
        static_cast<RatioSet>(params_.secondary % 3));

    /* One shared position in the bank, no spread: these six are already
     * differentiated by rate, and giving them different shapes as well would
     * only blur the one thing the mode exists to show. */
    const float shape = params_.knob_b * static_cast<float>(kNumShapes);

    for (uint8_t i = 0; i < kNumJacks; i++)
    {
        const float prev = phase_[i];
        phase_[i] = Wrap01(phase_[i] + base * ratios[i] * dt_);
        if (phase_[i] < prev) stepped_[i] = rng_.Bipolar();

        out.volts[i] = kBipolarVolts * ShapeAt(shape, phase_[i], stepped_[i]);
    }
}

void ModSource::TickSmoothRandom(Frame& out)
{
    const float d    = params_.knob_a;                       // divergence
    const float base = kRandomBaseHz[params_.secondary % 4];

    /* K4 stiffens the ease. The minimum window is 2 % of a segment, short
     * enough to read as a hard step at every rate the mode offers while
     * staying continuous — a literal jump would put a click on any audio path
     * this drives. */
    const float win = Lerp(1.f, 0.02f, params_.knob_b);

    /* The shared walk always runs; at divergence 0 it is the only thing on
     * the jacks, at divergence 1 it is fully crossfaded out. */
    common_phase_ += base * dt_;
    if (common_phase_ >= 1.f)
    {
        common_phase_ -= static_cast<float>(static_cast<int>(common_phase_));
        common_prev_   = common_target_;
        common_target_ = rng_.Bipolar();
    }
    const float common = Lerp(common_prev_, common_target_,
                              EasedStep(common_phase_, win));

    for (uint8_t i = 0; i < kNumJacks; i++)
    {
        /* Rates fan out only as divergence opens; at 0 every channel runs at
         * the base rate, which is what makes them read as one gesture. */
        const float rate = base * Lerp(1.f, kRandomFan[i], d);

        rand_phase_[i] += rate * dt_;
        if (rand_phase_[i] >= 1.f)
        {
            rand_phase_[i] -=
                static_cast<float>(static_cast<int>(rand_phase_[i]));
            rand_prev_[i]   = rand_target_[i];
            rand_target_[i] = rng_.Bipolar();
        }
        const float indep = Lerp(rand_prev_[i], rand_target_[i],
                                 EasedStep(rand_phase_[i], win));

        const float v = Lerp(common * kCommonDepth[i], indep, d);
        out.volts[i]  = kBipolarVolts * Clamp(v, -1.f, 1.f);
    }
}

void ModSource::TickEuclid(Frame& out)
{
    /* Gates are fired by OnClock and simply time out here. Jack 0 is the
     * clock input; the five patterns live on jacks 1..5. */
    for (uint8_t k = 1; k < kNumJacks; k++)
    {
        if (gate_left_[k] > 0.f)
        {
            gate_left_[k] -= dt_;
            out.volts[k]   = kUnipolarVolts;
        }
        else
        {
            gate_left_[k] = 0.f;
            out.volts[k]  = 0.f;
        }
    }
}

} // namespace mod_source
