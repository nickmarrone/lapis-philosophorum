/**
 * mastering.cpp — Alchemy Lab stereo-linked mastering chain.
 * In -> EQ -> Compressor -> Saturation -> Limiter -> Output Trim -> Dither -> Out.
 *
 * Pure DSP in mastering_dsp.* implements:
 * - Three-band EQ (low shelf / peaking mid / high shelf), stereo-linked params
 * - Log-domain feed-forward compressor (hard or soft knee)
 * - Waveshaper saturation (cubic soft clip or hard clip) + DC blocker
 * - Zero-attack peak limiter
 * - Output trim + continuous TPDF dither
 *
 * This file implements:
 * - All the required hardware and audio callback management.
 * - Three pages of controls (EQ / Compressor / Output), switched by B1
 * - Pot-catch on page switch (no value jumps), via Pager
 * - A short tap on B2 toggles the current page's bypass
 * - A short tap on B3 cycles/toggles the current page's secondary mode
 *   (EQ mid Q, compressor knee, saturation type)
 * - LED ring per pot displaying its value; B2/B3 button LEDs show bypass
 *   state and secondary mode
 * - Save and recall presets with flash wear leveling
 * - Settings menu for managing basics
 */

#include "daisy_seed.h"
#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/settings.h"
#include "alchemy/surface/virtual_knob.h"

#include "mastering_dsp.h"
#include "mastering_palette.h"

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

static VirtualKnob ratio = VirtualKnob(1, "Ratio")
    .Linear(1.f, 20.f)
    .Ring(Level(kCompPalette.arc));

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

/* ── Page 3 — Output (red) ────────────────────────────────────────────────
 * K1/K3/K4/K6 are Level rings; K2 (Asym) and K5 (Trim) are Bipolar. */

static VirtualKnob drive = VirtualKnob(0, "Drive")
    .Linear(0.f, 24.f)
    .Ring(Level(kOutPalette.arc));

static VirtualKnob asym = VirtualKnob(1, "Asym")
    .Linear(-0.3f, 0.3f)
    .Ring(Bipolar(kOutPalette.bipolar_pos,
                  kOutPalette.bipolar_neg,
                  kOutPalette.bipolar_center));

static VirtualKnob ceiling = VirtualKnob(2, "Ceiling")
    .Linear(-6.f, -0.1f)
    .Ring(Level(kOutPalette.arc));

static VirtualKnob lim_rel = VirtualKnob(3, "Lim Rel")
    .Exp(10.f, 500.f)
    .Ring(Level(kOutPalette.arc));

static VirtualKnob trim = VirtualKnob(4, "Trim")
    .Linear(-12.f, 12.f)
    .Ring(Bipolar(kOutPalette.bipolar_pos,
                  kOutPalette.bipolar_neg,
                  kOutPalette.bipolar_center));

static VirtualKnob dither = VirtualKnob(5, "Dither")
    .Linear(0.f, 2.f)
    .Ring(Level(kOutPalette.arc));

/* Bind knobs to page */
static Page eq_page   = Page(0).Knobs(ls_freq, ls_gain, mid_freq, mid_gain,
                                      hs_freq, hs_gain);
static Page comp_page = Page(1).Knobs(thresh, ratio, attack, release,
                                      makeup, mix);
static Page out_page  = Page(2).Knobs(drive, asym, ceiling, lim_rel,
                                      trim, dither);

/* Get our SDK surfaces and opt in to everything (no ParamLock, no CvMatrix —
 * this chain is stereo-linked with a single param set, nothing to record). */
static AlchemyLab  hw;
static ControlLoop loop    (hw);
static Pager       pager   (hw.buttons[0], 3, kNumPots);
static Presets     presets (hw.seed.qspi);
static Settings    settings(hw, &pager);

/* ── Persistence: per-page mode/bypass state not carried by any knob ─────
 * Six single-byte fields; Deserialize clamps so a corrupt/foreign slot can
 * never push an out-of-range index into the DSP. */
struct ChainModes : public alchemy::Serializable
{
    uint8_t eq_bypass   = 0;
    uint8_t comp_bypass = 0;
    uint8_t sat_bypass  = 0;
    uint8_t mid_q_index = 0;
    uint8_t soft_knee   = 0;
    uint8_t sat_type    = 0;

    size_t SerializedSize() const override { return 6; }

    void Serialize(uint8_t* out) const override
    {
        out[0] = eq_bypass;
        out[1] = comp_bypass;
        out[2] = sat_bypass;
        out[3] = mid_q_index;
        out[4] = soft_knee;
        out[5] = sat_type;
    }

    bool Deserialize(const uint8_t* in) override
    {
        eq_bypass   = in[0] & 1u;
        comp_bypass = in[1] & 1u;
        sat_bypass  = in[2] & 1u;
        mid_q_index = in[3] % 3u;
        soft_knee   = in[4] & 1u;
        sat_type    = in[5] & 1u;
        return true;
    }

    uint32_t SchemaHash() const override { return 0x4D535431u; }
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

/* Mid-band Q choices cycled by a B3 tap on the EQ page. */
static constexpr float kMidQTable[3] = {0.707f, 1.5f, 4.0f};

/* Consume pending taps contextually on the active page, then push every
 * knob + mode field to the DSP unconditionally (preset loads apply for
 * free — nothing here is edge-triggered). */
static void UpdateParams()
{
    const uint8_t page = pager.Page();

    if (b2_tap.tap) {
        b2_tap.tap = false;
        switch (page) {
            case 0: modes.eq_bypass   = !modes.eq_bypass;   break;
            case 1: modes.comp_bypass = !modes.comp_bypass; break;
            default: modes.sat_bypass = !modes.sat_bypass;  break;
        }
    }
    if (b3_tap.tap) {
        b3_tap.tap = false;
        switch (page) {
            case 0: modes.mid_q_index = (modes.mid_q_index + 1) % 3; break;
            case 1: modes.soft_knee   = !modes.soft_knee;            break;
            default: modes.sat_type   = !modes.sat_type;             break;
        }
    }

    mastering_dsp::SetEq({
        ls_freq.Value(),  ls_gain.Value(),
        mid_freq.Value(), mid_gain.Value(), kMidQTable[modes.mid_q_index],
        hs_freq.Value(),  hs_gain.Value(),
        modes.eq_bypass != 0,
    });

    mastering_dsp::SetComp({
        thresh.Value(), ratio.Value(), attack.Value(), release.Value(),
        makeup.Value(), mix.Value(),
        modes.soft_knee   != 0,
        modes.comp_bypass != 0,
    });

    mastering_dsp::SetOutput({
        drive.Value(), asym.Value(), ceiling.Value(), lim_rel.Value(),
        trim.Value(), dither.Value(),
        modes.sat_type,
        modes.sat_bypass != 0,
    });
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
        default: page_color = kPageRed;   bypassed = modes.sat_bypass;  break;
    }
    hw.leds.SetButtonPair(1, bypassed ? LedPanel::Scale(page_color, kBypassDim)
                                      : page_color);

    LedPanel::Rgb mode_color;
    switch (page) {
        case 0:  mode_color = kQColors   [modes.mid_q_index]; break;
        case 1:  mode_color = kKneeColors[modes.soft_knee];   break;
        default: mode_color = kSatColors [modes.sat_type];    break;
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
    pager.SetPageColor(2, kPageRed);

    /* Opting into default settings gestures and controls. */
    settings.UseBrightness();
    settings.UsePresets(presets);

    /* Preset payload — every Serializable surface gets walked on Save/Load. */
    presets.Manage(pager);
    presets.Manage(settings);
    presets.Manage(modes);
    presets.Init();
    presets.BootLoad();

    UpdateParams();
    hw.StartAudio(mastering_dsp::Process);

    /* ControlLoop is a thin, opt-in driver for the canonical control-rate frame.
     * If desired, you can unroll and modify. */
    loop.Use(pager)
        .Use(settings)
        .Use(eq_page)
        .Use(comp_page)
        .Use(out_page)
        .OnFrame(UpdateParams)
        .OnPoll(PollTaps)
        .OnRender(RenderButtons);

    for (;;) loop.Tick();
}
