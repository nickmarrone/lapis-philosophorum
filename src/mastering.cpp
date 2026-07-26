/**
 * mastering.cpp — Alchemy Lab stereo-linked mastering chain.
 * In -> EQ -> Compressor -> Tape Saturation -> Trim -> Limiter -> Dither -> Out.
 *
 * Pure DSP in mastering_dsp.* implements:
 * - Three-band EQ (low shelf / peaking mid / high shelf), stereo-linked params
 * - Log-domain feed-forward glue compressor (three characters, two engines)
 * - Tape saturation: emphasis/de-emphasis around a 2x-oversampled ADAA tanh,
 *   head bump, three tape-machine characters
 * - Lookahead peak limiter
 * - Output trim into the limiter + continuous TPDF dither
 *
 * This file implements:
 * - All the required hardware and audio callback management.
 * - Four pages of controls (EQ / Compressor / Saturation / Output), by B1
 * - Pot-catch on page switch (no value jumps), via Pager
 * - A short tap on B2 toggles the current page's bypass
 * - A short tap on B3 cycles the current page's secondary mode
 *   (EQ mid Q, compressor character, tape machine, modulation secondary)
 * - A CV modulation source on the six jacks, selected by the Output page's
 *   K6: Off, Analysis, Clocked, MultiLfo, SmoothRandom, Euclid. K5 is the
 *   active mode's continuous control. See mod_source.h for the mode map;
 *   everything here is the hardware seam (edge detection, DG411 routing,
 *   and the two different DAC update rates).
 * - LED ring per pot displaying its value; B2/B3 button LEDs show bypass
 *   state and secondary mode
 * - Save and recall presets with flash wear leveling
 * - Settings menu for managing basics
 */

#include "daisy_seed.h"
#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/control/cv_edge.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/settings.h"
#include "alchemy/surface/virtual_knob.h"

#include "mastering_dsp.h"
#include "mastering_palette.h"
#include "mod_source.h"

using namespace alchemy;

/* ── Page 1 — EQ (amber) ─────────────────────────────────────────────────
 * K1/K3/K5 are Level rings (freq); K2/K4/K6 are Bipolar rings (gain). */

static VirtualKnob ls_freq = VirtualKnob(0, "LS Freq")
    .Exp(20.f, 800.f)
    .Ring(Level(kEqPalette.arc));

static VirtualKnob ls_gain = VirtualKnob(1, "LS Gain")
    .Linear(-15.f, 15.f)
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

static VirtualKnob mid_freq = VirtualKnob(2, "Mid Freq")
    .Exp(200.f, 5000.f)
    .Ring(Level(kEqPalette.arc));

static VirtualKnob mid_gain = VirtualKnob(3, "Mid Gain")
    .Linear(-15.f, 15.f)
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

static VirtualKnob hs_freq = VirtualKnob(4, "HS Freq")
    .Exp(1000.f, 20000.f)
    .Ring(Level(kEqPalette.arc));

static VirtualKnob hs_gain = VirtualKnob(5, "HS Gain")
    .Linear(-15.f, 15.f)
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

/* ── Page 2 — Compressor (blue) ───────────────────────────────────────────
 * All six knobs are Level rings; nothing on this page is bipolar. */

static VirtualKnob thresh = VirtualKnob(0, "Thresh")
    .Linear(-40.f, 0.f)
    .Ring(Level(kCompPalette.arc));

/* Ratio is tapered in "compression amount" a = 1 - 1/ratio, which is literally
 * the slope the gain computer uses — so the taper is linear in the quantity
 * that actually does the work rather than in the ratio's reciprocal.
 *
 * The old Linear(1, 20) buried every setting a mastering glue compressor is
 * ever used at (1.2:1 - 3:1) inside the first 10 % of knob travel, and .Exp()
 * cannot fix it either: Exp(1.2, 20) still only gives that span a third of the
 * knob. In compression amount, a = 0 -> 1:1, 0.33 -> 1.5:1, 0.5 -> 2:1,
 * 0.667 -> 3:1, 0.9 -> 10:1, 0.95 -> 20:1 — half the travel sits below 2:1.
 *
 * The mapping lives here, at the control layer, so CompParams keeps carrying
 * plain engineering units across the seam. */
static VirtualKnob ratio = VirtualKnob(1, "Ratio")
    .Linear(0.f, 0.95f)
    .Ring(Level(kCompPalette.arc));

static inline float RatioFromAmount(float a)
{
    const float amt = a < 0.f ? 0.f : (a > 0.95f ? 0.95f : a);
    return 1.f / (1.f - amt);
}

static VirtualKnob attack = VirtualKnob(2, "Attack")
    .Exp(0.1f, 100.f)
    .Ring(Level(kCompPalette.arc));

static VirtualKnob release = VirtualKnob(3, "Release")
    .Exp(10.f, 2000.f)
    .Ring(Level(kCompPalette.arc));

static VirtualKnob makeup = VirtualKnob(4, "Makeup")
    .Linear(0.f, 20.f)
    .Ring(Level(kCompPalette.arc));

static VirtualKnob mix = VirtualKnob(5, "Mix")
    .Linear(0.f, 1.f)
    .Ring(Level(kCompPalette.arc));

/* ── Page 3 — Tape saturation (gold) ──────────────────────────────────────
 * K1/K2/K3/K5 are Level rings; K4 (Asym) is Bipolar. K6 is unassigned —
 * there is no sixth axis worth inventing one for, and a short page is fine. */

static VirtualKnob drive = VirtualKnob(0, "Drive")
    .Linear(0.f, 24.f)
    .Ring(Level(kSatPalette.arc));

static VirtualKnob sat_mix = VirtualKnob(1, "Sat Mix")
    .Linear(0.f, 1.f)
    .Ring(Level(kSatPalette.arc));

static VirtualKnob emphasis = VirtualKnob(2, "Emphasis")
    .Linear(0.f, 1.f)
    .Ring(Level(kSatPalette.arc));

static VirtualKnob asym = VirtualKnob(3, "Asym")
    .Linear(-0.3f, 0.3f)
    .Ring(Bipolar(kSatPalette.bipolar_pos,
                  kSatPalette.bipolar_neg,
                  kSatPalette.bipolar_center));

static VirtualKnob bump = VirtualKnob(4, "Head Bump")
    .Linear(0.f, 1.f)
    .Ring(Level(kSatPalette.arc));

/* ── Page 4 — Output (red) ────────────────────────────────────────────────
 * K1/K2 are the limiter and K6 is Trim; K3/K4/K5 are the modulation source.
 *
 * The modulation block is contiguous and sits between them on purpose: K5
 * picks the mode and the two knobs to its left are that mode's parameters, so
 * everything whose meaning changes with the mode is grouped, and everything
 * with a fixed meaning (Ceiling, Lim Rel, Trim) is outside the group.
 *
 * K4 used to be a 0..2 LSB Dither depth. Dither is now fixed at
 * mastering_dsp::kDitherLsb and the pot went to the modulation source — see
 * that constant for why a control over it was never doing anything a patch
 * could hear. Trim moved to K6 to keep the three mod pots adjacent. */

static VirtualKnob ceiling = VirtualKnob(0, "Ceiling")
    .Linear(-6.f, -0.1f)
    .Ring(Level(kOutPalette.arc));

static VirtualKnob lim_rel = VirtualKnob(1, "Lim Rel")
    .Exp(10.f, 500.f)
    .Ring(Level(kOutPalette.arc));

static VirtualKnob trim = VirtualKnob(5, "Trim")
    .Linear(-12.f, 12.f)
    .Ring(Bipolar(kOutPalette.bipolar_pos,
                  kOutPalette.bipolar_neg,
                  kOutPalette.bipolar_center));

/* ── Page 4, K3/K4/K5 — CV modulation source ─────────────────────────────
 * The chain is stereo-linked with a single parameter set, so there is nothing
 * on the panel worth CV-modulating and the six jacks sat unused. K5 turns them
 * into a modulation source instead: five modes plus Off, with K3 and K4 as the
 * active mode's two continuous controls and a B3 tap as its discrete secondary.
 *
 * The split between the two knobs is per-mode and documented in mod_source.h,
 * but the shape of it is consistent: K3 is the mode's primary axis — how much,
 * how fast, how spread — and K4 is its character. The one exception is
 * Analysis, which has no waveform to characterise and uses K4 for sensitivity.
 *
 * K5 pairs a Selector transform with a Gradient ring on purpose. Value()
 * snaps to a zone while Norm() stays continuous, so the arc morphs smoothly
 * between mode colours as you sweep and the snap pip confirms where it
 * landed — the pattern virtual_knob.h documents for exactly this case.
 *
 * K3 and K4 both use GradientFill against the same snap table with
 * src_pot = 4, so both parameter knobs are always painted in the colour of the
 * mode controlling them. Without that, three adjacent knobs would be red on a
 * page where two of them mean something different in each of six positions. */
static VirtualKnob mod_a = VirtualKnob(2, "Mod A")
    .Linear(0.f, 1.f)
    .Ring(GradientFill(kModeSnaps, 6, /*src_pot=*/4, kPageRed));

static VirtualKnob mod_b = VirtualKnob(3, "Mod B")
    .Linear(0.f, 1.f)
    .Ring(GradientFill(kModeSnaps, 6, /*src_pot=*/4, kPageRed));

static VirtualKnob mod_mode = VirtualKnob(4, "Mod Mode")
    .Selector(static_cast<uint8_t>(mod_source::Mode::kCount))
    .Ring(Gradient(kModeSnaps, 6))
    .Pip(GradientSnapPip());

/* Bind knobs to page */
static Page eq_page   = Page(0).Knobs(ls_freq, ls_gain, mid_freq, mid_gain,
                                      hs_freq, hs_gain);
static Page comp_page = Page(1).Knobs(thresh, ratio, attack, release,
                                      makeup, mix);
static Page sat_page  = Page(2).Knobs(drive, sat_mix, emphasis, asym, bump);
static Page out_page  = Page(3).Knobs(ceiling, lim_rel, mod_a, mod_b,
                                      mod_mode, trim);

/* Get our SDK surfaces and opt in to everything (no ParamLock, no CvMatrix —
 * this chain is stereo-linked with a single param set, nothing to record). */
static AlchemyLab  hw;
static ControlLoop loop    (hw);
static Pager       pager   (hw.buttons[0], 4, kNumPots);
static Presets     presets (hw.seed.qspi);
static Settings    settings(hw, &pager);

/* ── Persistence: per-page mode/bypass state not carried by any knob ─────
 * Seven single-byte fields; Deserialize clamps so a corrupt/foreign slot can
 * never push an out-of-range index into the DSP. */
struct ChainModes : public alchemy::Serializable
{
    uint8_t eq_bypass      = 0;
    uint8_t comp_bypass    = 0;
    uint8_t sat_bypass     = 0;
    uint8_t mid_q_index    = 0;
    uint8_t comp_character = 0;
    uint8_t sat_character  = 0;
    uint8_t lim_bypass     = 0;

    /* One secondary index per modulation mode, so each remembers its own —
     * switching from Clocked to Euclid and back returns to the clock ratio
     * you left, not to whatever the step set happened to be. Indexed by
     * mod_source::Mode; slot 0 (Off) is unused and always reads 0.
     *
     * Zero is the right default everywhere except Euclid, whose zones are
     * ordered by cycle length rather than by which one the module has always
     * shipped with. Index 2 is the Classic 16/12/9/7/5 grid, so a virgin
     * module still comes up sounding like the firmware it replaces. */
    uint8_t mod_secondary[static_cast<uint8_t>(mod_source::Mode::kCount)] = {
        0,  // Off
        0,  // Analysis     — normal polarity
        0,  // Clocked      — 1/4 clock ratio
        0,  // MultiLfo     — Golden ratios
        0,  // SmoothRandom — glacial
        2,  // Euclid       — Classic step set
    };

    static constexpr size_t kNumModes =
        static_cast<size_t>(mod_source::Mode::kCount);

    size_t SerializedSize() const override { return 7 + kNumModes; }

    void Serialize(uint8_t* out) const override
    {
        out[0] = eq_bypass;
        out[1] = comp_bypass;
        out[2] = sat_bypass;
        out[3] = mid_q_index;
        out[4] = comp_character;
        out[5] = sat_character;
        out[6] = lim_bypass;
        for (size_t i = 0; i < kNumModes; i++)
            out[7 + i] = mod_secondary[i];
    }

    bool Deserialize(const uint8_t* in) override
    {
        eq_bypass      = in[0] & 1u;
        comp_bypass    = in[1] & 1u;
        sat_bypass     = in[2] & 1u;
        mid_q_index    = in[3] % 3u;
        comp_character = in[4] % 3u;
        sat_character  = in[5] % 3u;
        lim_bypass     = in[6] & 1u;
        /* Clamp against each mode's own zone count, so a corrupt or foreign
         * slot can never index past kClockRatio[] or the ratio tables. */
        for (size_t i = 0; i < kNumModes; i++)
        {
            const uint8_t zones =
                mod_source::SecondaryZones(static_cast<mod_source::Mode>(i));
            mod_secondary[i] = static_cast<uint8_t>(in[7 + i] % zones);
        }
        return true;
    }

    /* MST5. Bumped from MST4 when the Output page was relaid out: Trim moved
     * from pot 2 to pot 5, the mode selector from pot 5 to pot 4, and pots 2
     * and 3 became the modulation source's two parameter knobs. Nothing about
     * the *serialised* layout changed — but every stored pot position on page
     * 4 now means something different, and Pager's own SchemaHash cannot see
     * that, since num_pages and num_pots are both unchanged. Loading an MST4
     * slot would silently apply the old Trim value as a shape rotation and the
     * old Dither depth as a shape spread. The bump is the only thing standing
     * between a preset and that.
     *
     * MST4 was the CV modulation source arriving with six per-mode secondary
     * bytes; MST3 predates it.
     *
     * The Pager does NOT invalidate independently this time — its own
     * SchemaHash folds in num_pages, which is still 4, and num_pots, still 6.
     * But Presets::SchemaHash is the XOR of every managed component's hash, so
     * bumping this one invalidates the whole slot: an MST3 preset fails the
     * gate in HasValid and nothing is deserialized, Pager included.
     *
     * That matters more than it sounds. Pager's default stored value is 0.5,
     * and 0.5 on K5's Selector(6) is MultiLfo — so without the guard in main()
     * a module with an old preset (or no preset at all) would boot with six
     * LFOs already driving the jacks. main() forces K5 to Off whenever
     * BootLoad returns false; that is what makes this bump safe. */
    uint32_t SchemaHash() const override { return 0x4D535435u; }
};
static ChainModes modes;

/* ── B2/B3 tap detection ──────────────────────────────────────────────────
 * Settings enters on B2+B3 both held 2000 ms and exits on the next B2/B3
 * rising edge; it never consumes edge flags itself. To coexist we derive
 * our own edges from Pressed() only — RisingEdge()/FallingEdge() must never
 * be called anywhere in this file, since Settings depends on seeing them
 * live. A tap that turns out to be part of a chord (either button already
 * down, or Settings active/was-active) is swallowed rather than fired. */
static constexpr uint32_t kTapMs = 400;

struct TapDetector {
    bool prev_pressed = false, chorded = false;
    uint32_t press_t = 0;
    bool tap = false;                            // consumed by OnFrame
    void Poll(uint32_t t, bool pressed, bool other_pressed,
              bool settings_active, bool settings_was_active) {
        if (pressed && !prev_pressed) {
            press_t = t;
            chorded = settings_was_active || settings_active || other_pressed;
        }
        if (pressed && (other_pressed || settings_active)) chorded = true;
        if (!pressed && prev_pressed)
            if (!chorded && (t - press_t) < kTapMs) tap = true;
        prev_pressed = pressed;
    }
};
static TapDetector b2_tap, b3_tap;

static void PollTaps(uint32_t t_ms)
{
    static bool settings_was_active = false;

    const bool settings_active = settings.IsActive();
    const bool b2_pressed      = hw.buttons[1].Pressed();
    const bool b3_pressed      = hw.buttons[2].Pressed();

    b2_tap.Poll(t_ms, b2_pressed, b3_pressed, settings_active, settings_was_active);
    b3_tap.Poll(t_ms, b3_pressed, b2_pressed, settings_active, settings_was_active);

    /* Must be updated after both detectors ran — it suppresses the
     * settings-exit press on the frame Settings becomes active/inactive. */
    settings_was_active = settings_active;
}

/* ControlLoop takes a single OnPoll hook, so the tap detector and the
 * modulation tick share one. Both want the same 1 ms cadence and neither
 * depends on the other; PollModulation is defined below. */
static void PollModulation(uint32_t t_ms);

static void PollControls(uint32_t t_ms)
{
    PollTaps(t_ms);
    PollModulation(t_ms);
}

/* ── CV modulation runtime ────────────────────────────────────────────────
 * All of this lives on the control thread. The engine itself is pure float
 * math in mod_source.cpp; everything here is the hardware seam.
 *
 * Two update rates, because the six jacks are not the same hardware:
 *
 *   J7/J8 (jack 4,5) are the STM32's own DAC — one register write. Updated
 *   every 1 ms poll, which is why the ramp and the stepped-random output live
 *   there: their discontinuities are what stepping shows up in.
 *
 *   J3..J6 (jack 0..3) are the MCP4728 over I2C. Even batched through
 *   SetMcpCvOutVolts that is one WriteAll plus an LDAC pulse, about 430 us —
 *   far too much for every poll. Flushed every 4th poll instead, so all four
 *   land together at 250 Hz for roughly 11 % of the main thread. Audio is in
 *   the SAI ISR and is not affected either way.
 *
 * kMcpFlushTicks is the one number to turn if the Euclid gates on J4..J6 feel
 * loose on hardware: it trades main-thread load against up to 4 ms of gate
 * jitter. J7/J8 already carry two of the five gates at 1 ms. */
static constexpr uint8_t kMcpFlushTicks = 4;

static mod_source::ModSource mod_engine;
static mod_source::Frame     mod_frame;

/* Unipolar 0/+5 V Eurorack clocks and resets, NOT CvEdge. A +5 V clock reads
 * ~0.75 here and 0 V reads ~0.5, so CvEdge's symmetric 0.30/0.70 thresholds
 * would see every rise and never a fall — the "+5 V triggers, 0 V holds
 * forever" failure its own header warns about. CvGate's 0.55/0.65 sit above
 * the rest point and release cleanly. */
static alchemy::CvGate mod_gate;

/* Last mode we programmed the DG411 switches for. 0xFF forces the first
 * Apply to run, so boot always establishes a known routing. */
static uint8_t mod_routed_mode = 0xFFu;
static uint8_t mod_tick_count  = 0;

/** Point every jack at 0 V, then set its DG411 to match the new mode.
 *  Only called when the mode actually changes — these are I2C transactions,
 *  and RouteCvOut goes through the expander. */
static void ApplyModRouting(mod_source::Mode mode)
{
    /* Drive the DACs to 0 V *before* touching the switches, so a jack never
     * connects to a stale voltage from the previous mode. */
    const float zeros[mod_source::kNumJacks] = {};
    hw.SetMcpCvOutVolts(zeros);
    hw.SetCvOutVolts(4, 0.f);
    hw.SetCvOutVolts(5, 0.f);

    for (uint8_t j = 0; j < mod_source::kNumJacks; j++)
        hw.RouteCvOut(j, mod_source::ModSource::JackIsOutput(mode, j));

    mod_routed_mode = static_cast<uint8_t>(mode);
}

/** One 1 ms tick: read the clock/reset jacks, advance the engine, push volts.
 *  Called from ControlLoop's inner poll, same thread as UpdateParams. */
static void PollModulation(uint32_t t_ms)
{
    (void)t_ms;

    const mod_source::Mode mode = mod_engine.ActiveMode();

    if (static_cast<uint8_t>(mode) != mod_routed_mode)
        ApplyModRouting(mode);

    if (mode == mod_source::Mode::Off) return;

    const uint32_t now_us = daisy::System::GetUs();

    /* Edge detection runs on every mode; the jack lookups return 0xFF for the
     * modes with no clock or reset, so the masks simply never match. */
    mod_gate.Tick(loop.Cv(), loop.NumCv(), now_us);

    const uint8_t clk_jack = mod_source::ModSource::ClockJack(mode);
    const uint8_t rst_jack = mod_source::ModSource::ResetJack(mode);
    if (clk_jack != 0xFFu && mod_gate.JustRose(clk_jack))
        mod_engine.OnClock(now_us);
    if (rst_jack != 0xFFu && mod_gate.JustRose(rst_jack))
        mod_engine.OnReset();

    /* Analysis reads the chain at tick rate rather than at the 16 ms frame:
     * with a 1 ms follower the CV should track transients, and a 62.5 Hz
     * refresh would throw most of that away. Six float loads. */
    if (mode == mod_source::Mode::Analysis)
    {
        mastering_dsp::ChainTelemetry t;
        mastering_dsp::ReadTelemetry(t);
        mod_engine.SetAnalysis({t.low, t.mid, t.high,
                                t.broad, t.comp_gr_db, t.lim_gr_db});
    }

    mod_engine.Tick(mod_frame);

    /* J7/J8 every tick — cheap. */
    if (mod_frame.is_output[4]) hw.SetCvOutVolts(4, mod_frame.volts[4]);
    if (mod_frame.is_output[5]) hw.SetCvOutVolts(5, mod_frame.volts[5]);

    /* J3..J6 batched, every kMcpFlushTicks. Non-output jacks carry 0 V in the
     * frame and their DG411 is open, so writing all four is harmless. */
    if (++mod_tick_count >= kMcpFlushTicks)
    {
        mod_tick_count = 0;
        hw.SetMcpCvOutVolts(mod_frame.volts);
    }
}

/* Mid-band Q choices cycled by a B3 tap on the EQ page. */
static constexpr float kMidQTable[3] = {0.707f, 1.5f, 4.0f};

/* Consume pending taps contextually on the active page, then push every
 * knob + mode field to the DSP unconditionally (preset loads apply for
 * free — nothing here is edge-triggered). */
static void UpdateParams()
{
    const uint8_t page = pager.Page();

    /* B2 always bypasses the stage the current page owns. */
    if (b2_tap.tap) {
        b2_tap.tap = false;
        switch (page) {
            case 0:  modes.eq_bypass   = !modes.eq_bypass;   break;
            case 1:  modes.comp_bypass = !modes.comp_bypass; break;
            case 2:  modes.sat_bypass  = !modes.sat_bypass;  break;
            default: modes.lim_bypass  = !modes.lim_bypass;  break;
        }
    }
    /* B3 cycles the current page's secondary mode. On the Output page that is
     * the active modulation mode's own secondary, whose zone count varies by
     * mode — so the modulus comes from the table rather than a literal. */
    const uint8_t mod_mode_idx = static_cast<uint8_t>(mod_mode.Value());
    if (b3_tap.tap) {
        b3_tap.tap = false;
        switch (page) {
            case 0: modes.mid_q_index    = (modes.mid_q_index + 1) % 3;    break;
            case 1: modes.comp_character = (modes.comp_character + 1) % 3; break;
            case 2: modes.sat_character  = (modes.sat_character + 1) % 3;  break;
            case 3: {
                const uint8_t zones = mod_source::SecondaryZones(
                    static_cast<mod_source::Mode>(mod_mode_idx));
                modes.mod_secondary[mod_mode_idx] =
                    static_cast<uint8_t>(
                        (modes.mod_secondary[mod_mode_idx] + 1) % zones);
                break;
            }
            default: break;
        }
    }

    mastering_dsp::SetEq({
        ls_freq.Value(),  ls_gain.Value(),
        mid_freq.Value(), mid_gain.Value(), kMidQTable[modes.mid_q_index],
        hs_freq.Value(),  hs_gain.Value(),
        modes.eq_bypass != 0,
    });

    mastering_dsp::SetComp({
        thresh.Value(), RatioFromAmount(ratio.Value()),
        attack.Value(), release.Value(),
        makeup.Value(), mix.Value(),
        modes.comp_character,
        modes.comp_bypass != 0,
    });

    mastering_dsp::SetSat({
        drive.Value(), sat_mix.Value(), emphasis.Value(),
        asym.Value(), bump.Value(),
        modes.sat_character,
        modes.sat_bypass != 0,
    });

    mastering_dsp::SetOutput({
        ceiling.Value(), lim_rel.Value(), trim.Value(),
        mastering_dsp::kDitherLsb,
        modes.lim_bypass != 0,
    });

    /* Modulation params are pushed unconditionally too, for the same reason
     * every Set* above is: nothing here is edge-triggered, so a preset load
     * applies for free. SetParams restarts the generators when the mode
     * changes, so switching in reads as a deliberate new patch rather than
     * resuming whatever phase the previous mode left behind. */
    const mod_source::Mode mode =
        static_cast<mod_source::Mode>(mod_mode_idx);
    mod_engine.SetParams({mode, mod_a.Norm(), mod_b.Norm(),
                          modes.mod_secondary[mod_mode_idx]});

    /* Gate the analysis followers on the mode that consumes them, so the
     * other five cost one predictable branch per sample instead of a band
     * split. The times come from the same K3 the mode is reading. */
    const mod_source::AnalysisResponse resp = mod_engine.Response();
    mastering_dsp::SetAnalysis(resp.attack_ms, resp.release_ms,
                               mode == mod_source::Mode::Analysis);
}

/* ── Button LEDs ───────────────────────────────────────────────────────── */
static void RenderButtons(uint32_t t_ms)
{
    (void)t_ms;
    if (settings.IsActive()) return;   // OnRender also fires in settings mode

    const uint8_t page = pager.Page();

    LedPanel::Rgb page_color;
    bool          bypassed;
    switch (page) {
        case 0:  page_color = kPageAmber; bypassed = modes.eq_bypass;   break;
        case 1:  page_color = kPageBlue;  bypassed = modes.comp_bypass; break;
        case 2:  page_color = kPageGold;  bypassed = modes.sat_bypass;  break;
        default: page_color = kPageRed;   bypassed = modes.lim_bypass;  break;
    }
    hw.leds.SetButtonPair(1, bypassed ? LedPanel::Scale(page_color, kBypassDim)
                                      : page_color);

    /* On the Output page B3 wears the active modulation mode's colour — the
     * same hue K5's arc and K3/K4's fills are showing, so one glance ties the
     * three knobs and the button together. Off maps to kModOffColor, the
     * dim "nothing to cycle here" the page used to show unconditionally. */
    LedPanel::Rgb mode_color;
    switch (page) {
        case 0:  mode_color = kQColors   [modes.mid_q_index];    break;
        case 1:  mode_color = kCharColors[modes.comp_character]; break;
        case 2:  mode_color = kSatColors [modes.sat_character];  break;
        case 3:  mode_color = kModeColors[
                     static_cast<uint8_t>(mod_mode.Value())];    break;
        default: mode_color = kModeInert;                        break;
    }
    hw.leds.SetButtonPair(2, mode_color);

    /* B1 is left to Pager — it paints the active-page indicator itself. */
}

/**
 * Flush denormals to zero in the FPU.
 *
 * Two writes, and the second one is the one that matters. On exception entry
 * the hardware initializes the handler's FPSCR from FPU->FPDSCR, so setting
 * FPSCR here in thread mode would NOT apply inside the SAI interrupt — which
 * is the only place audio is processed. FPDSCR covers the ISR; the FPSCR write
 * covers thread mode, where the coefficient design runs.
 *
 * Nothing else in this firmware, the SDK, or libDaisy ever touches FPSCR — and
 * this is a BOOT_SRAM build, where Reset_Handler skips SystemInit() entirely,
 * so no startup path has written it either.
 *
 * Safe for audio: a denormal in an IIR decay tail is ~200 dB below audibility,
 * so truncating it at ~1e-38 instead of ~1e-45 changes nothing anyone can hear.
 * Safe for the design math too — its smallest intermediate is fc^4 ~ 5e-13,
 * computed in double, where the normal range reaches 2e-308.
 */
static void EnableFlushToZero()
{
    constexpr uint32_t kFz = 1u << 24;      // FPSCR.FZ
    FPU->FPDSCR |= kFz;                     // every exception handler, incl. SAI
    __set_FPSCR(__get_FPSCR() | kFz);       // thread mode
}

int main()
{
    EnableFlushToZero();

    hw.Init();
    mastering_dsp::Init(hw.SampleRate());

    /* The pots ship with slew = 0, which SetCoeff clamps to a pass-through, so
     * raw ADC jitter reaches the biquad design every frame and zippers. Set the
     * filter coefficient directly rather than a slew time: AnalogControl::Init
     * was handed AudioCallbackRate() (2 kHz), but ProcessAllControls() actually
     * runs from ControlLoop's 1 ms poll, so any slew_seconds would come out 2x
     * wrong. 0.125 is tau ~= 8 ms at the real 1 kHz rate. */
    for (uint8_t i = 0; i < kNumPots; i++)
        hw.pots[i].SetCoeff(0.125f);

    pager.SetPageColor(0, kPageAmber);
    pager.SetPageColor(1, kPageBlue);
    pager.SetPageColor(2, kPageGold);
    pager.SetPageColor(3, kPageRed);

    /* Opting into default settings gestures and controls. */
    settings.UseBrightness();
    settings.UsePresets(presets);

    /* Preset payload — every Serializable surface gets walked on Save/Load. */
    presets.Manage(pager);
    presets.Manage(settings);
    presets.Manage(modes);
    presets.Init();
    const bool restored = presets.BootLoad();

    /* The modulation engine ticks from the 1 ms poll. */
    mod_engine.Init(1000.f, 0x5EEDBEEFu);
    mod_gate.Init(kNumCvInputs);

    /* Force K5 to Off unless a current-schema preset actually restored it.
     *
     * Presets::SchemaHash is the XOR of every managed component, so the MST4
     * -> MST5 bump invalidates the whole slot: HasValid fails and nothing is
     * deserialized, Pager included. Pager's default stored value is 0.5, and
     * 0.5 on K5's Selector(6) is MultiLfo — so a module with an old preset,
     * or a virgin module with no preset at all, would otherwise come up with
     * six LFOs already driving the jacks before the user touched anything.
     *
     * SetStored re-arms catch against the physical pot position, so this does
     * not fight the user: K5 stays at Off until they sweep through it. Settle
     * the ADC first, or catch would re-arm against a pot buffer still full of
     * zeros and the knob could grab on the first frame. */
    if (!restored)
    {
        for (int i = 0; i < 64; i++)
        {
            hw.ProcessAllControls();
            daisy::System::Delay(1);
        }
        float phys[kNumPots];
        for (uint8_t i = 0; i < kNumPots; i++) phys[i] = hw.pots[i].Value();
        pager.SetStored(3, 4, 0.f, phys);
    }

    UpdateParams();
    hw.StartAudio(mastering_dsp::Process);

    /* ControlLoop is a thin, opt-in driver for the canonical control-rate frame.
     * If desired, you can unroll and modify. */
    loop.Use(pager)
        .Use(settings)
        .Use(eq_page)
        .Use(comp_page)
        .Use(sat_page)
        .Use(out_page)
        .OnFrame(UpdateParams)
        .OnPoll(PollControls)
        .OnRender(RenderButtons);

    for (;;) loop.Tick();
}
