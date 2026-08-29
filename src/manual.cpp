/**
 * manual.cpp — module-level manual prose. See manual.h for why this is
 * separate from the per-entity .Help() text in mastering.cpp.
 *
 * House style, inherited from alchemy/surface/manual.h: one layer of
 * CommonMark, and never restate a structured fact. Ranges, units, defaults,
 * page names, and preset slot counts all reach the host as data already —
 * repeating them here just creates a second copy to keep correct. Say what
 * the control is *for* instead.
 */

#include "manual.h"

using alchemy::Manual;

namespace lapis
{

/* ── Sections ─────────────────────────────────────────────────────────── */

static const char* const kSignalPath = R"md(
Everything is stereo-linked. There is one set of controls and one gain
computed from both channels, so the two sides can never drift apart and the
image stays where you put it. The compressor sums the two channels' power the
way a stereo-linked analogue compressor sums its detector currents, and the
limiter's true-peak detector reads whichever channel is louder, oversampled.

The stages run in a fixed order — EQ, then compressor, then tape, then the
output trim, then the limiter, then dither. That order is deliberate: you are
shaping tone before you control dynamics, and controlling dynamics before you
set the ceiling, so each stage sees something the previous one has already
made well-behaved.

Latency is constant at 75 samples — the limiter's lookahead plus the tape
stage's half-band filters. It does not change when you bypass a stage, so a
parallel path stays phase-aligned no matter what you switch.
)md";

static const char* const kControls = R"md(
The six knobs are shared by four pages, and **B1** cycles through them. Knobs
do not jump when you change page: a knob stays where the page left it until
the physical pot passes through that value, then picks it up. So you can
switch pages mid-set without a parameter lurching.

**B2** bypasses the current page's stage. **B3** cycles the current page's
mode — the mid band's width on the EQ page, the compressor's character, the
tape machine, and on the output page the modulation mode's secondary control.
The button lights show you both: B2 dims when the stage is bypassed, and B3
wears the colour of the mode it is on.

Holding **B2 + B3** together for two seconds enters settings.
)md";

static const char* const kModulation = R"md(
A stereo-linked chain has one parameter set and nothing worth CV-modulating,
so rather than leave the six CV jacks idle this firmware spends them the other
way round: as a six-channel modulation source that drives the rest of your
rack. **Mod Mode** on the output page chooses what they do, and the two knobs
to its left are that mode's controls — the primary axis (how much, how fast,
how spread) and its character. A **B3** tap cycles the mode's discrete
secondary.

| Mode | Jacks | What it is |
|---|---|---|
| Off | released | The jacks are disconnected. |
| Analysis | six out | The chain listening to itself — band envelopes, broadband level, and both gain-reduction signals. |
| Clocked | clock and reset in, four out | Four positions in the shape bank, locked to your clock. |
| Multi LFO | six out | Six free-running LFOs at deliberately non-octave ratios, so they never line up. |
| Smooth Random | six out | Wandering voltages, from one shared walk to six independent ones. |
| Euclid | clock in, five gate out | Five co-prime Euclidean gate patterns. |

**Analysis** is the one that belongs to this module rather than to any utility
module: the mastering chain becomes its own modulation source, so a patch can
follow what the chain is actually doing to the program material.

Clocked and Multi LFO share a shape bank — six waveforms arranged in a ring
that crossfades between neighbours and wraps, so the shape knob has no seam
and no dead end. Clocked spreads its outputs around that ring; collapse the
spread to zero and all four jacks carry the same voltage, which is the
mono-bus case. With nothing patched to its clock input it free-runs rather
than sitting still.

Bipolar modes swing ±4 V rather than ±5 V. Four of the six jacks are driven by
a DAC that bottoms out near −4.4 V, and a wider swing would clip on those four
but not the other two — so they are all held to the range every jack can
actually reach. Envelopes and gates use the 0 to +5 V convention.
)md";

static const char* const kGainStaging = R"md(
Set the ceiling first — it is the only control here that makes a promise about
the output, and everything else is easier to judge once you know where the top
is.

Then work left to right. Fix tone with the EQ before you compress, because the
compressor is listening to whatever the EQ hands it and a boost you add later
will change how it triggers. Bring the compressor in until the gain reduction
is doing steady, unhurried work rather than snatching at peaks. Use the tape
stage for the last of the loudness: it holds program level across its whole
drive range, so drive trades peaks for harmonics rather than simply turning
things up, and that is usually what you want at the end of a chain.

Leave the limiter something to do, but not much. If it is working hard on
every bar, the answer is upstream — less makeup, or a lower threshold with a
gentler ratio — not a lower ceiling.

Bypass is per stage and latency-compensated, so A/B a single stage freely; the
comparison stays honest.
)md";

/* ── The manual ───────────────────────────────────────────────────────── */

const Manual kManual = Manual()
    .Tagline("Stereo-linked mastering chain — EQ, glue compression, tape, "
             "and a brickwall limiter, with the six CV jacks turned into a "
             "modulation source.")
    .Preamble(
        "A mastering chain for the end of a bus: three-band EQ, a log-domain "
        "glue compressor, tape saturation, and a true-peak limiter, all "
        "stereo-linked so the two channels move together.\n\n"
        "Because a chain like this has nothing worth CV-modulating, the six "
        "CV jacks do the opposite job — they leave the module as a "
        "six-channel modulation source, and one of its modes is the chain's "
        "own analysis of the program material.")
    .Section("signal-path", "Signal path", kSignalPath)
    .Section("controls", "Pages and buttons", kControls)
    .Section("modulation", "The CV modulation source", kModulation)
    .Section("gain-staging", "Gain staging", kGainStaging);

} // namespace lapis
