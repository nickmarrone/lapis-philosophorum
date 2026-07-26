/**
 * mastering_palette.h — Per-page LED palettes for the 4-page mastering chain.
 *
 * You don't need to implement something like this - this is simply to keep
 * colors organized and out of the main flow of the main file.
 *
 * Each page has an identity color (used for Pager::SetPageColor, the B1
 * button paint, and as the base for the B2 bypass-dim scale) plus a
 * PagePalette: the primary arc color for Level rings (freq/ratio/etc.) and
 * the positive/negative/center colors for Bipolar rings (gain/trim/asym).
 * B3's per-page mode indicator (Q, compressor character, tape machine) is a
 * small fixed color array indexed by the current mode. The Output page has no
 * secondary mode and paints kModeInert instead.
 */

#pragma once

#include "alchemy/led/anims/color_morph_arc.h"   /* MorphSnapPoint */
#include "alchemy/led/panel.h"

/** Ring color set for one page. */
struct PagePalette
{
    alchemy::LedPanel::Rgb arc;            // Level-ring color (freq / ratio / attack / etc.)
    alchemy::LedPanel::Rgb bipolar_pos;    // Bipolar ring, positive side (== page color)
    alchemy::LedPanel::Rgb bipolar_neg;    // Bipolar ring, negative side (contrasting/cooler)
    alchemy::LedPanel::Rgb bipolar_center; // Bipolar ring, center/zero (dim white)
};

/* ── Page identity colors ────────────────────────────────────────────────
 * Pager::SetPageColor(page, ...), B1 button paint (via Pager), and the base
 * color scaled by kBypassDim for the B2 bypass LED. */
constexpr alchemy::LedPanel::Rgb kPageAmber = {0xFF, 0xA0, 0x00}; // page 1: EQ
constexpr alchemy::LedPanel::Rgb kPageBlue  = {0x00, 0x60, 0xFF}; // page 2: Compressor
constexpr alchemy::LedPanel::Rgb kPageGold  = {0xFF, 0xB0, 0x30}; // page 3: Saturation
constexpr alchemy::LedPanel::Rgb kPageRed   = {0xFF, 0x20, 0x20}; // page 4: Output

constexpr alchemy::LedPanel::Rgb kDimWhite = {0x40, 0x40, 0x40}; // Bipolar center, all pages

/* ── Page 1 — EQ (amber) ─────────────────────────────────────────────────
 * K1/K3/K5 (LS/Mid/HS Freq) are Level rings -> arc. K2/K4/K6 (LS/Mid/HS
 * Gain) are Bipolar -> boost uses the page color, cut uses a cool contrast. */
constexpr PagePalette kEqPalette = {
    kPageAmber,
    kPageAmber,
    {0x00, 0x40, 0xC0},
    kDimWhite,
};

/* ── Page 2 — Compressor (blue) ──────────────────────────────────────────
 * All six knobs (Thresh/Ratio/Attack/Release/Makeup/Mix) are Level rings ->
 * arc. No knob on this page is Bipolar; the variants are kept for symmetry
 * with the other pages and in case a future control needs them. */
constexpr PagePalette kCompPalette = {
    kPageBlue,
    kPageBlue,
    {0xFF, 0x60, 0x00},
    kDimWhite,
};

/* ── Page 3 — Tape saturation (gold) ─────────────────────────────────────
 * K1/K2/K3/K5 (Drive/Mix/Emphasis/Head Bump) are Level rings -> arc. K4
 * (Asym) is Bipolar -> positive uses the page color, negative uses a cool
 * contrast. Gold rather than the old red: the page is a tape machine now, and
 * red still belongs to the thing that stops the signal leaving. */
constexpr PagePalette kSatPalette = {
    kPageGold,
    kPageGold,
    {0x00, 0x80, 0xC0},
    kDimWhite,
};

/* ── Page 4 — Output (red) ───────────────────────────────────────────────
 * K1/K2/K4 (Ceiling/Lim Rel/Dither) are Level rings -> arc. K3 (Trim) is
 * Bipolar -> positive uses the page color, negative uses a cool contrast. */
constexpr PagePalette kOutPalette = {
    kPageRed,
    kPageRed,
    {0x00, 0x80, 0xC0},
    kDimWhite,
};

/* ── B3 mode indicators ──────────────────────────────────────────────────
 * Indexed by the current per-page mode value; see mastering_dsp.h and the
 * ChainModes persistence fields in mastering.cpp. Each array's length is the
 * modulus of its B3 cycle in UpdateParams — add a colour and widen the
 * modulus and the Deserialize clamp together, or one of them will be wrong. */

/** Page 1, mid_q_index -> {0.707f, 1.5f, 4.0f}: progressively narrower look. */
constexpr alchemy::LedPanel::Rgb kQColors[3] = {
    {0x00, 0xFF, 0x00}, // 0.707 - wide, green
    {0xFF, 0xC0, 0x00}, // 1.5   - medium, yellow
    {0xFF, 0x00, 0xC0}, // 4.0   - narrow, magenta
};

/** Page 2, comp_character -> {Precise, Adaptive, Glue}. The first two keep the
 *  old hard/soft-knee colours, so muscle memory survives the change from a
 *  two-state knee toggle to a three-state character cycle. */
constexpr alchemy::LedPanel::Rgb kCharColors[3] = {
    {0xE0, 0xE0, 0xE0}, // Precise  - white   (was "hard knee")
    {0x00, 0xC0, 0xA0}, // Adaptive - teal    (was "soft knee")
    {0xFF, 0x30, 0x60}, // Glue     - crimson
};

/** Page 3, sat_character -> {30 ips, 15 ips, Saturated}: the same tape run
 *  progressively harder, so the colour warms as the machine does. */
constexpr alchemy::LedPanel::Rgb kSatColors[3] = {
    {0xFF, 0xE0, 0xA0}, // 30 ips    - pale gold
    {0xFF, 0xB0, 0x30}, // 15 ips    - warm gold
    {0xC0, 0x60, 0x00}, // Saturated - deep amber
};

/** Page 4's B3 used to have nothing to cycle. Dim white reads as "nothing here"
 *  rather than as an unlit LED, which would look like a fault. Still used for
 *  the Off modulation mode, where B3 genuinely has no secondary. */
constexpr alchemy::LedPanel::Rgb kModeInert = {0x30, 0x30, 0x30};

/* ── Page 4 — CV modulation source ───────────────────────────────────────
 * K6 selects the mode, K5 is that mode's continuous control, and B3 cycles
 * its discrete secondary.
 *
 * These colours do double duty and that is deliberate: K6 wears them as a
 * Gradient arc that morphs between them as you sweep, K5 wears the active one
 * as a GradientFill so the amount knob always matches the mode it controls,
 * and B3 wears it too. One hue therefore identifies the mode in three places
 * at once.
 *
 * Off is the page's own red, dimmed hard — the jacks are released and the
 * page is just the Output page again. The five generators then walk the
 * spectrum away from red so adjacent modes never read as the same colour at
 * a glance.
 *
 * Positions are evenly spaced across the six Selector zones, sampled at each
 * zone's centre so the morph reaches the pure colour where the value snaps. */
constexpr alchemy::LedPanel::Rgb kModOffColor      = {0x40, 0x10, 0x10}; // dim red
constexpr alchemy::LedPanel::Rgb kModAnalysisColor = {0xFF, 0x40, 0x00}; // orange
constexpr alchemy::LedPanel::Rgb kModClockedColor  = {0xFF, 0xD0, 0x00}; // yellow
constexpr alchemy::LedPanel::Rgb kModMultiLfoColor = {0x00, 0xD0, 0x40}; // green
constexpr alchemy::LedPanel::Rgb kModRandomColor   = {0x00, 0x80, 0xFF}; // blue
constexpr alchemy::LedPanel::Rgb kModEuclidColor   = {0xB0, 0x40, 0xFF}; // violet

/** Zone centres for a Selector(6): (i + 0.5) / 6. */
constexpr alchemy::MorphSnapPoint kModeSnaps[6] = {
    {1.f / 12.f,  kModOffColor},
    {3.f / 12.f,  kModAnalysisColor},
    {5.f / 12.f,  kModClockedColor},
    {7.f / 12.f,  kModMultiLfoColor},
    {9.f / 12.f,  kModRandomColor},
    {11.f / 12.f, kModEuclidColor},
};

/** Parallel to kModeSnaps, for the B3 button paint. */
constexpr alchemy::LedPanel::Rgb kModeColors[6] = {
    kModOffColor,      kModAnalysisColor, kModClockedColor,
    kModMultiLfoColor, kModRandomColor,   kModEuclidColor,
};

/** Dim factor applied to a page/button color when its stage is bypassed. */
constexpr float kBypassDim = 0.15f;
