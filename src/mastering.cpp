/**
 * mastering.cpp — Lapis Philosophorum, an Alchemy Lab stereo-linked
 * mastering chain.
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
 * - HostLink over the panel USB-C: presets and settings are manageable from
 *   the browser, and `make program-live` reboots the module into DFU over
 *   that same connection instead of a power cycle
 */

#include "daisy_seed.h"
#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/control/cv_edge.h"
#include "alchemy/host_link/host.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/jack.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/settings.h"
#include "alchemy/surface/virtual_knob.h"

#include "manual.h"
#include "mastering_dsp.h"
#include "mastering_palette.h"
#include "mod_source.h"
#include "version.h"

using namespace alchemy;

/* Version stamp, compiled into the image so a stray .bin can be identified
 * after the fact: `strings build/lapis_philosophorum.bin | grep Lapis`.
 * `used` stops the *compiler* dropping an unreferenced static, but the link
 * runs with --gc-sections, which discards the section anyway — main() takes
 * the address to anchor it. Keep that reference if you move this. */
__attribute__((used, section(".rodata.version")))
static const char kVersionBanner[] = "LapisPhilosophorum " LAPIS_VERSION_STR;

/* ── Page 1 — EQ (amber) ─────────────────────────────────────────────────
 * K1/K3/K5 are Level rings (freq); K2/K4/K6 are Bipolar rings (gain). */

static VirtualKnob ls_freq = VirtualKnob(0, "LS Freq")
    .Exp(20.f, 800.f).Unit("Hz").Ident("eq.ls.freq")
    .Help("Corner of the low shelf. Everything below it is lifted or cut "
          "together, so this is the control for weight rather than for one "
          "note — put it under the material you want to move.")
    .Ring(Level(kEqPalette.arc));

static VirtualKnob ls_gain = VirtualKnob(1, "LS Gain")
    .Linear(-15.f, 15.f).Unit("dB").Ident("eq.ls.gain")
    .Help("How much the low shelf lifts or cuts. Small moves go a long way "
          "on a full mix; if you need a large one, the corner is probably in "
          "the wrong place.")
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

static VirtualKnob mid_freq = VirtualKnob(2, "Mid Freq")
    .Exp(200.f, 5000.f).Unit("Hz").Ident("eq.mid.freq")
    .Help("Centre of the mid band — the one band that is a peak rather than "
          "a shelf, so it acts on a region and leaves everything either side "
          "alone. Sweep it with the gain up to find what you are looking for, "
          "then set the gain properly.")
    .Ring(Level(kEqPalette.arc));

static VirtualKnob mid_gain = VirtualKnob(3, "Mid Gain")
    .Linear(-15.f, 15.f).Unit("dB").Ident("eq.mid.gain")
    .Help("How much the mid band lifts or cuts. Its width is set by B3, so "
          "the same gain can read as a broad tilt or a narrow notch.")
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

static VirtualKnob hs_freq = VirtualKnob(4, "HS Freq")
    .Exp(1000.f, 20000.f).Unit("Hz").Ident("eq.hs.freq")
    .Help("Corner of the high shelf. Lower corners bring presence and can "
          "get harsh; higher ones buy air without touching the body of the "
          "mix.")
    .Ring(Level(kEqPalette.arc));

static VirtualKnob hs_gain = VirtualKnob(5, "HS Gain")
    .Linear(-15.f, 15.f).Unit("dB").Ident("eq.hs.gain")
    .Help("How much the high shelf lifts or cuts. Remember the compressor is "
          "downstream and listening: a top-end boost here will make it work "
          "harder on the same material.")
    .Ring(Bipolar(kEqPalette.bipolar_pos,
                  kEqPalette.bipolar_neg,
                  kEqPalette.bipolar_center));

/* ── Page 2 — Compressor (blue) ───────────────────────────────────────────
 * All six knobs are Level rings; nothing on this page is bipolar. */

static VirtualKnob thresh = VirtualKnob(0, "Thresh")
    .Linear(-40.f, 0.f).Unit("dB").Ident("comp.thresh")
    .Help("The level above which the compressor starts working. On a bus you "
          "usually want it low enough that it is always doing a little, not "
          "high enough that it only catches the loudest bar.")
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
    .Linear(0.f, 0.95f).Ident("comp.ratio")
    .Help("How hard the compressor pulls once it is over the threshold. The "
          "knob is scaled by compression amount rather than by the ratio "
          "number, so the gentle settings a glue compressor actually lives at "
          "get half the travel instead of the first sliver of it.")
    .Ring(Level(kCompPalette.arc));

static inline float RatioFromAmount(float a)
{
    const float amt = a < 0.f ? 0.f : (a > 0.95f ? 0.95f : a);
    return 1.f / (1.f - amt);
}

static VirtualKnob attack = VirtualKnob(2, "Attack")
    .Exp(0.1f, 100.f).Unit("ms").Ident("comp.attack")
    .Help("How quickly the compressor clamps down. Fast catches transients "
          "and flattens the front of every hit; slow lets them through and "
          "compresses what follows, which is what keeps a mix punchy.")
    .Ring(Level(kCompPalette.arc));

static VirtualKnob release = VirtualKnob(3, "Release")
    .Exp(10.f, 2000.f).Unit("ms").Ident("comp.release")
    .Help("How quickly it lets go again. Set it against the tempo: releasing "
          "roughly in time with the track is what turns gain reduction into "
          "the pumping people mean by glue, and too fast will breathe.")
    .Ring(Level(kCompPalette.arc));

static VirtualKnob makeup = VirtualKnob(4, "Makeup")
    .Linear(0.f, 20.f).Unit("dB").Ident("comp.makeup")
    .Help("Gain added after compression, to put back what the gain reduction "
          "took. Each character also applies its own automatic makeup, so "
          "this is a trim on top of that rather than the whole job.")
    .Ring(Level(kCompPalette.arc));

static VirtualKnob mix = VirtualKnob(5, "Mix")
    .Linear(0.f, 1.f).Ident("comp.mix")
    .Help("Blends the compressed signal back against the dry one. Turning it "
          "down is parallel compression: you keep the transients the "
          "compressor flattened and still get the density underneath.")
    .Ring(Level(kCompPalette.arc));

/* ── Page 3 — Tape saturation (gold) ──────────────────────────────────────
 * K1/K2/K3/K5 are Level rings; K4 (Asym) is Bipolar. K6 is unassigned —
 * there is no sixth axis worth inventing one for, and a short page is fine. */

static VirtualKnob drive = VirtualKnob(0, "Drive")
    .Linear(0.f, 24.f).Unit("dB").Ident("tape.drive")
    .Help("How hard the tape stage is pushed. Program level is held across "
          "the whole range, so this trades peaks for harmonics rather than "
          "simply turning things up — it gets denser, not louder.")
    .Ring(Level(kSatPalette.arc));

static VirtualKnob sat_mix = VirtualKnob(1, "Sat Mix")
    .Linear(0.f, 1.f).Ident("tape.mix")
    .Help("Blends the saturated signal against the dry one, for when a "
          "machine's character is right but the amount of it is not.")
    .Ring(Level(kSatPalette.arc));

static VirtualKnob emphasis = VirtualKnob(2, "Emphasis")
    .Linear(0.f, 1.f).Ident("tape.emphasis")
    .Help("Tilts the record/playback pair around the saturating stage. This "
          "is what makes the distortion frequency-dependent instead of "
          "uniform: turn it up and the highs saturate first, the way tape "
          "does, rather than the whole spectrum breaking up at once.")
    .Ring(Level(kSatPalette.arc));

static VirtualKnob asym = VirtualKnob(3, "Asym")
    .Linear(-0.3f, 0.3f).Ident("tape.asym")
    .Help("Pushes the saturation curve off centre so the two halves of the "
          "waveform clip differently. That asymmetry is what produces even "
          "harmonics — the warm ones — where a symmetric curve gives only "
          "odd.")
    .Ring(Bipolar(kSatPalette.bipolar_pos,
                  kSatPalette.bipolar_neg,
                  kSatPalette.bipolar_center));

static VirtualKnob bump = VirtualKnob(4, "Head Bump")
    .Linear(0.f, 1.f).Ident("tape.bump")
    .Help("The low resonance a tape machine gets from its head geometry. A "
          "little adds weight down low without the EQ having to reach for "
          "it; the selected machine sets where it sits.")
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
    .Linear(-6.f, -0.1f).Unit("dB").Ident("out.ceiling")
    .Help("The hard limit nothing gets past. Set this first — it is the only "
          "control here that makes a promise about the output, and the rest "
          "of the chain is easier to judge once the top is fixed.")
    .Ring(Level(kOutPalette.arc));

static VirtualKnob lim_rel = VirtualKnob(1, "Lim Rel")
    .Exp(10.f, 500.f).Unit("ms").Ident("out.lim_rel")
    .Help("How quickly the limiter recovers after catching a peak. Short is "
          "louder but will distort bass; long is cleaner but audibly ducks "
          "the material after a loud hit.")
    .Ring(Level(kOutPalette.arc));

static VirtualKnob trim = VirtualKnob(5, "Trim")
    .Linear(-12.f, 12.f).Unit("dB").Ident("out.trim")
    .Help("Level going into the limiter — so this, not the ceiling, is how "
          "you decide how hard the limiter works. Push it up for loudness "
          "and more gain reduction; back it off to let the chain breathe.")
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
static const char* const kModeLabels[6] = {
    "Off", "Analysis", "Clocked", "Multi LFO", "Smooth Random", "Euclid"};

static VirtualKnob mod_a = VirtualKnob(2, "Mod A")
    .Linear(0.f, 1.f).Ident("mod.a")
    .Help("The active mode's primary axis — how much, how fast, or how far "
          "apart, depending on the mode. It is painted in the mode's colour "
          "so you can see at a glance which one it is answering to.")
    .Ring(GradientFill(kModeSnaps, 6, /*src_pot=*/4, kPageRed));

static VirtualKnob mod_b = VirtualKnob(3, "Mod B")
    .Linear(0.f, 1.f).Ident("mod.b")
    .Help("The active mode's character — the shape of what Mod A is setting "
          "the amount of. Analysis is the exception: it generates nothing, "
          "so it has no character to shape and spends this on sensitivity "
          "instead.")
    .Ring(GradientFill(kModeSnaps, 6, /*src_pot=*/4, kPageRed));

static VirtualKnob mod_mode = VirtualKnob(4, "Mod Mode")
    .Selector(static_cast<uint8_t>(mod_source::Mode::kCount))
    .Labels(kModeLabels).Ident("mod.mode")
    .Help("What the six CV jacks do. Off releases them entirely; the other "
          "five turn the module into a modulation source for the rest of the "
          "rack. Changing mode re-routes the jacks, and they are driven to "
          "0 V before the switches move, so nothing jumps.")
    .Ring(Gradient(kModeSnaps, 6))
    .Pip(GradientSnapPip());

/* Bind knobs to page. Name and Color label and tint the web programmer's
 * tabs; the colors match the panel's page colors in mastering_palette.h so
 * the browser and the module agree on which page is which. */
static Page eq_page   = Page(0).Name("EQ").Color("#ffa000")
                               .Help("Three bands — low shelf, mid peak, high "
                                     "shelf — before anything else in the "
                                     "chain, so the compressor downstream "
                                     "reacts to the tone you set here. B3 "
                                     "cycles the mid band's width.")
                               .Knobs(ls_freq, ls_gain, mid_freq, mid_gain,
                                      hs_freq, hs_gain);
static Page comp_page = Page(1).Name("Compressor").Color("#0060ff")
                               .Help("A log-domain feed-forward glue "
                                     "compressor, stereo-linked off the two "
                                     "channels' summed power. B3 cycles the "
                                     "character, and each one brings its own "
                                     "knee, sidechain high-pass, and "
                                     "automatic makeup.")
                               .Knobs(thresh, ratio, attack, release,
                                      makeup, mix);
static Page sat_page  = Page(2).Name("Tape").Color("#ffb030")
                               .Help("A record/playback emphasis pair around "
                                     "a saturating knee, oversampled and "
                                     "antialiased. B3 cycles the machine. "
                                     "The page has five controls, not six — "
                                     "there was no sixth axis worth "
                                     "inventing.")
                               .Knobs(drive, sat_mix, emphasis, asym, bump);
static Page out_page  = Page(3).Name("Output").Color("#ff2020")
                               .Help("The limiter and output trim, plus the "
                                     "CV modulation source on the three "
                                     "middle knobs. Dither is always on and "
                                     "has no control — it was never doing "
                                     "audible work at a level worth a knob.")
                               .Knobs(ceiling, lim_rel, mod_a, mod_b,
                                      mod_mode, trim);

/* ── Panel jacks ──────────────────────────────────────────────────────────
 * Descriptor metadata only — no runtime behaviour, no state, nothing that
 * reaches a preset. Declared here rather than in manual.cpp so the SeeAlso
 * cross-references below are checked against the actual knob objects at
 * build time instead of by string.
 *
 * J3..J8 are declared CvBi because that is what they carry in the bipolar
 * modes; Analysis and Euclid drive the same jacks unipolar, which the help
 * text covers. Their direction is mode-dependent too, so the per-jack help
 * is written around "what this carries in each mode" rather than a single
 * fixed role. */
static const Jack kJacks[] = {
    Jack("j1", "Audio In L", JackSig::AudioIn).Short("IN L")
        .Help("Left input to the chain."),
    Jack("j2", "Audio In R", JackSig::AudioIn).Short("IN R")
        .Help("Right input to the chain. The two channels are processed "
              "stereo-linked throughout — one set of controls, one gain — so "
              "they cannot drift apart."),

    Jack("j3", "CV 1", JackSig::CvBi).Short("CV1")
        .Help("Low-band envelope in Analysis. Clock input in Clocked and "
              "Euclid. An LFO or random voltage in the other modes.")
        .SeeAlso(mod_mode),
    Jack("j4", "CV 2", JackSig::CvBi).Short("CV2")
        .Help("Mid-band envelope in Analysis. Reset input in Clocked, and a "
              "gate in Euclid. An LFO or random voltage otherwise.")
        .SeeAlso(mod_mode),
    Jack("j5", "CV 3", JackSig::CvBi).Short("CV3")
        .Help("High-band envelope in Analysis, a gate in Euclid, and one of "
              "the shape or random outputs in the other modes.")
        .SeeAlso(mod_mode),
    Jack("j6", "CV 4", JackSig::CvBi).Short("CV4")
        .Help("Broadband level in Analysis, a gate in Euclid, and one of the "
              "shape or random outputs otherwise.")
        .SeeAlso(mod_mode),
    Jack("j7", "CV 5", JackSig::CvBi).Short("CV5")
        .Help("Compressor gain reduction in Analysis, a gate in Euclid, an "
              "output otherwise. This jack and CV 6 update four times faster "
              "than CV 1-4, which is why the stepped and gated signals are "
              "put here.")
        .SeeAlso(mod_mode),
    Jack("j8", "CV 6", JackSig::CvBi).Short("CV6")
        .Help("Limiter gain reduction in Analysis, a gate in Euclid, an "
              "output otherwise. Fast, like CV 5.")
        .SeeAlso(mod_mode),

    Jack("j9", "Audio Out L", JackSig::AudioOut).Short("OUT L")
        .Help("Left output, after the limiter and dither."),
    Jack("j10", "Audio Out R", JackSig::AudioOut).Short("OUT R")
        .Help("Right output, after the limiter and dither. Latency from "
              "input is a constant 75 samples and does not change with "
              "bypass, so a parallel path stays aligned."),
};

/* Get our SDK surfaces and opt in to everything (no ParamLock, no CvMatrix —
 * this chain is stereo-linked with a single param set, nothing to record). */
static AlchemyLab  hw;
static ControlLoop loop    (hw);
static Pager       pager   (hw.buttons[0], 4, kNumPots);
static Presets     presets (hw.seed.qspi);
static Settings    settings(hw, &pager);

/* HostLink on the panel USB-C. The web programmer's layout is derived from
 * the same objects Presets walks — the knob and page declarations above — so
 * it cannot drift from the firmware, and there is nothing to declare here
 * beyond identity. `.Product` makes the module enumerate under the platform
 * name like every other Hermetic module rather than under its own.
 *
 * It also carries the reboot command, which is what lets `make program-live`
 * drop the module into DFU without reaching for the power switch. */
static hostlink::Host host(presets, "lapis_philosophorum",
                           "Lapis Philosophorum",
                           LAPIS_VERSION_STR, LAPIS_GIT_HASH);

/* The SDK's stock descriptor buffer is 24 KiB, and this module's manual prose
 * alone is over half of that before any of the structural JSON around it. An
 * overflow fails the descriptor build — recoverable, and the host is told why,
 * but the browser gets no layout until someone notices. SDRAM is 64 MiB and
 * this firmware uses none of it, so buy the headroom and stop thinking about
 * it: prose can grow without anyone having to remember this ceiling. */
static char DSY_SDRAM_BSS s_descriptor[64u * 1024u];

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
 * StageVolts is the uniform entry point — no branching here on which backend
 * a jack has — but two update rates still fall out of the hardware underneath:
 *
 *   J7/J8 (jack 4,5) are the STM32's own DAC. StageVolts has nothing to defer
 *   there, so it writes the register through immediately and every 1 ms poll
 *   lands. That is why the ramp and the stepped-random output live on those
 *   jacks: their discontinuities are what stepping shows up in.
 *
 *   J3..J6 (jack 0..3) are the MCP4728 over I2C. StageVolts only moves the
 *   SDK's shared shadow, which is free; the FlushCvOutputs that latches it is
 *   one WriteAll plus an LDAC pulse, about 430 us — far too much for every
 *   poll. Flushed every 4th poll instead, so all four land together at 250 Hz
 *   for roughly 11 % of the main thread. Audio is in the SAI ISR and is not
 *   affected either way.
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
 *  and Enable/DisableCvOutput goes through the expander. */
static void ApplyModRouting(mod_source::Mode mode)
{
    /* Drive the DACs to 0 V *before* touching the switches, so a jack never
     * connects to a stale voltage from the previous mode. */
    for (uint8_t j = 0; j < mod_source::kNumJacks; j++)
        hw.cv_jacks[j].StageVolts(0.f);
    hw.FlushCvOutputs();

    for (uint8_t j = 0; j < mod_source::kNumJacks; j++)
    {
        if (mod_source::ModSource::JackIsOutput(mode, j))
            hw.cv_jacks[j].EnableCvOutput();
        else
            hw.cv_jacks[j].DisableCvOutput();
    }

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

    /* Stage every jack every tick. J7/J8 reach the panel here; J3..J6 only
     * move the shadow and wait for the flush below. Non-output jacks carry
     * 0 V in the frame and their DG411 is open, so staging all six is
     * harmless and saves branching on is_output. */
    for (uint8_t j = 0; j < mod_source::kNumJacks; j++)
        hw.cv_jacks[j].StageVolts(mod_frame.volts[j]);

    /* One WriteAll + LDAC pulse latches J3..J6 together, every kMcpFlushTicks. */
    if (++mod_tick_count >= kMcpFlushTicks)
    {
        mod_tick_count = 0;
        hw.FlushCvOutputs();
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
    /* Anchors kVersionBanner against --gc-sections. The asm consumes the
     * pointer and emits nothing, so this costs no code and no cycles. */
    asm volatile("" : : "r"(kVersionBanner));

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

    /* The manual and the jack table are descriptor-only, so order does not
     * matter here — the descriptor renders at the host's first Poll(), once
     * everything main() declares is attached. `.Product` makes the module
     * enumerate under the platform name like every other Hermetic module. */
    host.Product("Alchemy Lab")
        .DescriptorBuffer(s_descriptor, sizeof s_descriptor)
        .Jacks(kJacks)
        .Attach(lapis::kManual);

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
        .Use(host)
        .OnFrame(UpdateParams)
        .OnPoll(PollControls)
        .OnRender(RenderButtons);

    for (;;) loop.Tick();
}
