/**
 * mastering_palette.h — Per-page LED palettes for the 3-page mastering chain.
 *
 * You don't need to implement something like this - this is simply to keep
 * colors organized and out of the main flow of the main file.
 *
 * Each page has an identity color (used for Pager::SetPageColor, the B1
 * button paint, and as the base for the B2 bypass-dim scale) plus a
 * PagePalette: the primary arc color for Level rings (freq/ratio/etc.) and
 * the positive/negative/center colors for Bipolar rings (gain/trim/asym).
 * B3's per-page mode indicator (Q, compressor character, saturation type) is
 * a small fixed color array indexed by the current mode.
 */

#pragma once

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
constexpr alchemy::LedPanel::Rgb kPageRed   = {0xFF, 0x20, 0x20}; // page 3: Output

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

/* ── Page 3 — Output (red) ───────────────────────────────────────────────
 * K1/K3/K4/K6 (Drive/Ceiling/Lim Rel/Dither) are Level rings -> arc. K2/K5
 * (Asym/Trim) are Bipolar -> positive uses the page color, negative uses a
 * cool contrast. */
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

/** Page 3, sat_type (0 = cubic soft clip, 1 = hard clip). */
constexpr alchemy::LedPanel::Rgb kSatColors[2] = {
    {0xFF, 0x80, 0x00}, // cubic soft clip - orange
    {0xA0, 0x00, 0x00}, // hard clip - deep red
};

/** Dim factor applied to a page/button color when its stage is bypassed. */
constexpr float kBypassDim = 0.15f;
