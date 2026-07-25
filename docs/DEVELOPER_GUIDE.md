# Mastering — Developer Guide

How this firmware is put together, why it's put together that way, and
what to touch when you want to change it.

Assumed background: C++17, embedded audio DSP, and a skim of the
[Alchemy SDK](https://github.com/hermetic-modular/alchemy-sdk) README. If
you want the end-user view first, read [USER_GUIDE.md](USER_GUIDE.md).

---

## 1. Architecture

The firmware is two layers with a deliberately narrow seam between them.

```
┌──────────────────────────────────────────────────────────────┐
│  mastering.cpp                          — control layer      │
│  VirtualKnobs, Pages, Pager, Presets, Settings, LED paint,   │
│  tap detection, ChainModes persistence                       │
└───────────────────────────┬──────────────────────────────────┘
                            │  SetEq() / SetComp() / SetSat() / SetOutput()
                            │  structs of plain floats + bools
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  mastering_dsp.cpp + dsp_*.h            — audio layer        │
│  chain state, per-sample Process(), stage implementations    │
└──────────────────────────────────────────────────────────────┘
```

**The seam is the four parameter structs** in `mastering_dsp.h`
(`EqParams`, `CompParams`, `SatParams`, `OutParams`). They carry engineering units —
Hz, dB, ms, ratio — and nothing else. The DSP layer has no idea that
`VirtualKnob`, `ControlLoop`, or the Alchemy SDK exist; its only external
dependency is `daisy::AudioHandle` for the callback signature.

Two properties fall out of this, and both are worth preserving:

1. **The DSP is portable and testable.** You can compile
   `mastering_dsp.cpp` plus the `dsp_*.h` headers against a host test
   harness by stubbing the `AudioHandle` typedefs; nothing else in the
   SDK is reachable from it.
2. **Unit conversion happens exactly once, at control rate.** dB→linear,
   ms→coefficient, and biquad coefficient computation all live in the
   `Set*` setters. `Process()` does per-sample arithmetic only.

### Two rates

| | Rate | Owner | Where |
|---|---|---|---|
| Audio | 48 kHz, 24-sample blocks | SAI/DMA ISR | `mastering_dsp::Process` |
| Control frame | ~60 Hz (16 ms) | main loop | `ControlLoop::Tick` → `UpdateParams` |
| Button poll | 1 ms | inner loop of `Tick` | `PollTaps` via `OnPoll` |

`kEngineBlockSamples = 24` at 48 kHz means a 0.5 ms audio block. Most
setters are lock-free by virtue of writing plain floats that the ISR
reads. **The two stages that publish biquad coefficients are the
exception** — the EQ and the saturator — and the EQ is worth reading before
you copy the pattern for anything else: it is double-buffered and smoothed,
because plain float writes were not good enough for it.

- **Handoff.** `SetEq` fills the bank slot the ISR is *not* reading, then
  publishes it with one `std::atomic` index store. Without this, the ISR
  can preempt a setter mid-write and process a block with a mixed-generation
  coefficient set. Do **not** replace this with a seqlock — preemption here
  is asymmetric (the ISR can interrupt the main loop, never the reverse), so
  a reader that spins waiting for the writer deadlocks outright.
- **Smoothing.** Parameters are eased in log2(Hz) / dB / log2(Q) at control
  rate, and the published coefficients are eased again per block in
  `Process()`. A dirty check inside `SetEq` then makes a stationary knob
  produce a *bit-identical* coefficient set frame after frame — which is
  what actually silences the zipper, and what `tests/eq_control_test.cpp`
  asserts.

The saturator (§4.2) uses the same shape for its emphasis and head-bump
coefficients: dirty-checked design at control rate, atomic index publish,
per-block lerp on the audio side. Its scalar parameters — drive, mix,
asym — are plain floats eased at 5 ms, which is enough because they cannot
tear into an unstable set the way five biquad coefficients can.

The general constraint still stands for every other stage: **do not add a
parameter whose per-frame step would produce an audible discontinuity**
without adding smoothing for it.

A note on why Direct Form I is still the right topology here, since the
reasoning used to be stated less precisely: DF-I never protected against
*tearing* — that is the double buffer's job now. What it does buy is
continuity across a coefficient swap. DF-I's state is literally past inputs
and outputs, so after a swap the output is still computed from real past
samples. A transposed Direct Form II keeps internal partial sums scaled by
the *old* coefficients, and swapping under those clicks.

---

## 2. File map

```
src/
├── mastering.cpp          control layer — the SDK-facing half
├── mastering_dsp.h        the seam: parameter structs + entry points
├── mastering_dsp.cpp      chain orchestration + audio callback
├── dsp_common.h           shared constexpr + inline helpers
├── dsp_biquad.h           matched-magnitude biquad (Direct Form I) + LerpCoeffs/InvertBiquad
├── dsp_compressor.h       log-domain feed-forward glue compressor
├── dsp_saturation.h       tape saturation — emphasis pair, ADAA tanh, head bump
├── dsp_halfband.h         2x polyphase half-band up/down + matched dry delay
├── dsp_limiter.h          brickwall limiter
├── dsp_dither.h           xorshift32 TPDF generator
└── mastering_palette.h    LED colour constants

tools/
└── halfband_design.py     regenerates dsp_halfband.h's tap table (Parks-McClellan)
```

The `dsp_*.h` stage files are **header-only structs with a `Configure()`
and a `ProcessSample()`**. They are headers rather than translation units
so the compiler can inline the whole per-sample path into `Process()`'s
loop body — at 48 kHz with a per-sample virtual or out-of-line call per
stage, the call overhead would be a meaningful fraction of the budget.

`dsp_common.h` exists solely so those headers can share namespace-scope
`constexpr`/`inline` definitions without redefinition collisions when
several of them land in one translation unit.

---

## 3. Control layer (`mastering.cpp`)

### 3.1 Knobs and pages

Each control is a `VirtualKnob` declared at file scope with a pot index
(0–5), a name, a value transform, and a ring style:

```cpp
static VirtualKnob ls_freq = VirtualKnob(0, "LS Freq")
    .Exp(20.f, 800.f)                    // geometric: v = min·(max/min)^norm
    .Ring(Level(kEqPalette.arc));
```

Transforms available: `.Linear(lo, hi)`, `.Exp(lo, hi)` (requires
`lo > 0`), `.Selector(n)`. `Value()` returns engineering units;
`Norm()` returns the raw 0–1 if you need a curve the SDK doesn't express.
Both are ISR-safe and cheap.

Ring styles used here: `Level` for unipolar parameters and `Bipolar` for
anything with a meaningful centre (gains, asymmetry, trim). Colours all
come from `mastering_palette.h` — keeping them out of the declarations
keeps the knob table readable as a parameter table.

Four `Page` objects bind up to six knobs each — EQ and Compressor use all
six, Tape uses five, Output four. Note the pot indices repeat across pages
(every page starts at pot 0); the `Pager` is what disambiguates
them, holding per-`(page, pot)` stored values and per-`(page, pot)` catch
state.

**Limits:** `Page::kMaxKnobs` is 8, `Pager::kMaxPages` is 8,
`ControlLoop::kMaxPages` is 8. Overflow is silently dropped, not
diagnosed — if a knob mysteriously stops responding after you add a
seventh to a page, that's why.

### 3.2 The frame

```cpp
loop.Use(pager)
    .Use(settings)
    .Use(eq_page).Use(comp_page).Use(sat_page).Use(out_page)
    .OnFrame(UpdateParams)     // once per ~16 ms frame
    .OnPoll(PollTaps)          // once per 1 ms, inside the frame
    .OnRender(RenderButtons);  // once per frame, during LED paint

for (;;) loop.Tick();
```

`Use(...)` order doesn't matter — `ControlLoop` slots surfaces into fixed
roles and sequences them canonically. `OnFrame` fires only while settings
is inactive; `OnRender` fires **always**, which is why `RenderButtons`
early-returns on `settings.IsActive()` (settings owns the panel then).

### 3.3 Parameter push is unconditional

`UpdateParams()` calls all four setters with every current value, every
frame, with no dirty-checking:

```cpp
mastering_dsp::SetEq({ ls_freq.Value(), ls_gain.Value(), ... });
```

This is intentional and load-bearing. Because nothing is edge-triggered,
a preset load "just works" — `Presets::Load` writes new stored values into
the `Pager`, and the next frame pushes them to the DSP with no
apply-callback needed anywhere.

`SetEq` now *does* dirty-check, and it does so exactly where this section
always said to: **inside the setter, never at the call site.** It compares
the smoothed parameters against what was last designed and returns early if
nothing moved, which is what makes a stationary knob produce a bit-identical
coefficient set (§1). `UpdateParams` itself is unchanged and must stay
unconditional — that is what preserves the preset property. A preset load
still just works: the stored values arrive, the comparison fails, the
coefficients get rebuilt.

### 3.4 Tap detection, and why it's hand-rolled

This is the subtlest code in the file. Read the comment block at
`mastering.cpp:185` before changing anything here.

`Settings` implements its enter gesture (B2 + B3 held 2 s) and its exit
(next B2/B3 rising edge) by observing button edges **without consuming
them**. If this file called `RisingEdge()` or `FallingEdge()` on B2 or B3,
it would consume the edge flags that `Settings` is depending on seeing
live, and settings mode would break.

So `TapDetector` derives its own edges from `Pressed()` polling only:

```cpp
struct TapDetector {
    bool prev_pressed = false, chorded = false;
    uint32_t press_t = 0;
    bool tap = false;
    void Poll(uint32_t t, bool pressed, bool other_pressed,
              bool settings_active, bool settings_was_active);
};
```

A press is discarded (`chorded`) if the other button was already down, if
either button goes down during the press, or if settings is or was just
active. A tap only fires on release, un-chorded, within `kTapMs` (400 ms).
`settings_was_active` is updated *after* both detectors run, which is what
suppresses the button press that exits settings from also being read as a
bypass toggle.

**Invariant to preserve: `RisingEdge()` and `FallingEdge()` must never be
called anywhere in `mastering.cpp`.**

Taps are latched by `PollTaps` (1 ms cadence) and consumed by
`UpdateParams` (16 ms cadence), which is the correct direction — a tap
that lands between frames isn't lost.

### 3.5 Persistence

Knob values are persisted for free: `Pager` is itself a `Serializable`
holding the per-`(page, pot)` stored values, and `presets.Manage(pager)`
registers it.

What isn't carried by any knob is the seven bytes of mode/bypass state,
which get their own `Serializable`:

```cpp
struct ChainModes : public alchemy::Serializable
{
    uint8_t eq_bypass, comp_bypass, sat_bypass;         // bytes 0-2
    uint8_t mid_q_index, comp_character, sat_character; // bytes 3-5
    uint8_t lim_bypass;                                 // byte 6 — appended

    size_t   SerializedSize() const override { return 7; }
    void     Serialize(uint8_t* out) const override;
    bool     Deserialize(const uint8_t* in) override;   // clamps every field
    uint32_t SchemaHash() const override { return 0x4D535433u; }
};
```

Two things to note:

**`Deserialize` clamps, it doesn't validate.** `% 3u` for the Q index, the
compressor character and the tape character; `& 1u` for the booleans. A
corrupt or foreign slot can therefore never push an out-of-range index
into `kMidQTable[]`, `kSatChars[]` or into the DSP. This is the cheap,
correct discipline for flash-backed state — don't relax it.

**`SchemaHash()` is a manual constant** (`0x4D535433` = "MST3"). The
preset store XORs every managed component's hash and stamps slots with the
result; a mismatch on load makes the slot read as empty rather than
restoring misaligned bytes. **If you change `ChainModes`'s field layout or
size, bump this constant.** If you forget, old slots will deserialize into
the new layout and produce garbage — the one failure mode the mechanism
exists to prevent.

It was last bumped MST2 → MST3 when the saturation moved to its own page:
the struct grew a `lim_bypass` byte and `sat_type` became a three-state
`sat_character`. That release also took `Pager` from three pages to four,
which changes `Pager::SchemaHash()` independently — either bump alone
would have invalidated old slots, which is the intended outcome, because
the knob assignments moved wholesale.

An earlier bump, MST1 → MST2, was for byte 4 changing from a two-state
soft-knee flag to a three-state compressor character. That release also
re-tapered the Ratio knob (see §4.2), which changes what a stored `Pager`
norm *means* without changing its layout — a hash bump would have been
required for that alone. When a knob's mapping changes, the stored values
are stale even though nothing about the format is.

Registration order in `main()` is the on-flash byte order:

```cpp
presets.Manage(pager);
presets.Manage(settings);
presets.Manage(modes);
presets.Init();
presets.BootLoad();     // auto-load slot 0 at power-on
```

Adding, removing, or reordering any of these changes the combined hash and
invalidates existing slots. That's the intended behaviour.

### 3.6 What this firmware opts *out* of

The SDK is opt-in per surface — each feature you don't construct isn't
linked. This project deliberately omits:

- **`ParamLock`** — no automation recording. A stereo-linked mastering
  chain has one parameter set; there is nothing to record per-voice.
- **`CvMatrix` / `CvRouter`** — no CV modulation. Every knob is a direct,
  unlatched control.

Consequently `hw.cv[]` is read by `ControlLoop` into its buffer and never
used. If you want CV back, construct a `CvRouter`, `loop.Use()` it, and
the `VirtualKnob` CV path activates — but note that adding a
`Serializable` surface to the preset payload invalidates existing slots.

---

## 4. DSP layer

### 4.1 Chain state and the process loop

Chain state lives in an anonymous namespace at file scope in
`mastering_dsp.cpp` — a single instance of each stage, no allocation, no
`this` pointer chasing in the audio path.

```cpp
for (size_t i = 0; i < n; i++) {
    float l = in[0][i], r = in[1][i];

    // EQ always runs; bypass discards the result rather than skipping.
    float el = BiquadProcess(ls_s_[0], ls_c_, l);
    el = BiquadProcess(mid_s_[0], mid_c_, el);
    el = BiquadProcess(hs_s_[0],  hs_c_,  el);
    /* ... same for er ... */
    if (!eq_bypass_) { l = el; r = er; }

    comp_.ProcessSample(l, r);     // stereo-linked, handles own bypass
    l = sat_.ProcessSample(l, 0);
    r = sat_.ProcessSample(r, 1);
    lim_.ProcessSample(l, r);      // stereo-linked
    l *= trim_lin_;  r *= trim_lin_;
    l += dith_[0].Sample();  r += dith_[1].Sample();

    out[0][i] = l;  out[1][i] = r;
}
```

**Bypass never skips computation.** The EQ biquads run and their output is
discarded; the compressor's detector and envelope run and only the gain
application is gated. This costs CPU that a branch could save, and buys
two things worth more: filter and detector state stays warm, so toggling
bypass is click-free and the compressor is already settled when it comes
back in. Preserve this property if you add a stage.

**Stereo linkage is structural**, not a parameter. `Compressor` and
`Limiter` take `(float& l, float& r)` and apply one gain to both — there is
no way to configure them apart. They do *not* detect the same way, though:
the limiter uses `fmaxf(fabsf(l), fabsf(r))`, which is right for a device
whose job is a ceiling no sample may cross, while the compressor sums
**power**, `0.5*(l² + r²)`, which is what a stereo-linked VCA does when it
sums its two sidechain currents. The difference shows on one-sided
material: max-linking reads a hard-panned hit at its full level and ducks
the whole image with it; power-linking reads it 3.01 dB lower.
`Saturator` is per-channel by index — it needs its own ADAA history,
oversampler, dry delay, filter and DC-blocker state per side — but shares
one coefficient set. The EQ shares one coefficient set across two
`BiquadState` arrays.

### 4.2 The stages

**Biquad** (`dsp_biquad.h`) — Direct Form I, **magnitude-matched**
coefficient design, templated on working precision.

This is deliberately *not* the RBJ cookbook any more. RBJ uses the bilinear
transform, which forces the response slope to zero at Nyquist, so a high
shelf parked at 20 kHz with fs = 48 kHz cannot reach its target gain and the
curve around it is squashed. Measured against the analog prototype that was
**3.95 dB** of error at 16.5 kHz. The matched designs bring it to **0.28 dB**
at identical runtime cost — only the control-rate math changed.

- Shelves: Vicanek, *Matched Two-Pole Digital Shelving Filters* —
  <https://vicanek.de/articles/2poleShelvingFits.pdf>
- Peaking: Vicanek, *Matched Second Order Digital Filters* §3.2, §4.4 —
  <https://vicanek.de/articles/BiquadFits.pdf>

Two things to know before editing these:

1. **The gain convention is not RBJ's.** Vicanek's `G` is the full linear
   gain `10^(dB/20)`; RBJ's `A` is its square root, `10^(dB/40)`. Mixing
   them halves every dB value.
2. **The bell's pole damping depends on gain.** The prototype denominator is
   `s² + s·w0/(√G·Q) + w0²`, so it is `1/(2·Q·√G)`, not `1/(2Q)`. Dropping
   the `√G` costs ~5 dB of shape error at +15 dB and is completely invisible
   at unity gain, which makes it an easy mistake to ship.

Phase is not traded away for this. The numerator recovery imposes
`b0 > |b2|` and `b0 + b2 > |b1|`, so the result is minimum phase by
construction — and for a minimum-phase filter, matching magnitude matches
phase. Bilinear's phase near Nyquist was the *worse* of the two.

Coefficients are designed in double and stored at the band's working
precision. **The low shelf is instantiated at `double`, the other two at
`float`.** It is the only band whose poles reach radius ~0.998 (20 Hz at
48 kHz), which both amplifies the recursion's own roundoff by ~1/(1−r) and
makes it nearly cancel its own poles against its zeros. Measured at 20 Hz,
float32 costs **38 dB of noise floor** and ~1e-2 dB of response error;
double costs ~20 cycles a frame. Note that it is the *coefficient storage*
that dominates the response error, not the state — the design solve itself
lands within 1e-10 dB either way. Shelf slope is fixed at Butterworth;
`MakePeaking` takes Q, which is what the mid-band Q cycling drives.

**Compressor** (`dsp_compressor.h`) — feed-forward glue compressor,
computed entirely in the log domain, with two engines behind three
characters:

```
sl, sr = sidechain high-pass (per character), detector path only
pw     = 0.5·(sl² + sr²)            // power-sum stereo link
lvl_db = 10·log10(pw)               // via kLog10ScaleP · logf(pw)
c      = AttenDb(lvl_db − threshold_db)      // POSITIVE attenuation, soft knee
y1     = max(c, y1 + rel·(c − y1))           // release stage (a hold, not a branch)
y      = y  + atk·(y1 − y)                   // attack stage
gr_db  = −y
gain   = 10^(−y/20) · makeup
```

Gain computer first, smoothing second — the arrangement Giannoulis,
Massberg & Reiss recommend (JAES 60(6):399-408, 2012, §III-B). Smoothing
the *level* instead would make a 10:1 setting attack five times faster than
a 2:1 setting at the same knob position.

| Character | Engine | Knee | Sidechain HPF | Auto-makeup |
|---|---|---|---|---|
| Precise | no adaptation | 6 dB | 30 Hz | no |
| Adaptive | crest-driven attack/release | 12 dB | 60 Hz | yes |
| Glue | dual time-constant release | 18 dB | 90 Hz | yes |

Everything in that table lives in `kCharSpec`, compressor-private, indexed
by `CompParams::character`.

**The sign convention is the thing to get right.** Attenuation is carried
as a **positive** dB value throughout the stage and negated exactly once,
at `gr_db`, because the literature's decoupled detector is written for a
positive control signal. `c` rising means more attenuation, which is the
attack direction, which is why the release stage uses `fmaxf`. In the old
negative gain-reduction convention every `fmaxf` here would be an `fminf`
— that is why the old `(target < gr_db) ? att : rel` was correct. Flip one
without the other and you get instant release with a slow attack.
`comp_response_test`'s timing test is what catches it: the two measured
numbers swap.

**Why the smooth decoupled detector.** The branching one-pole this
replaced does *not* have a kink at the handover — its increment
`a·(c − gr)` vanishes exactly where the coefficient switches. Its defect is
that it starts releasing on every dip of the waveform, twice per cycle, so
the envelope ripples at the program frequency and amplitude-modulates the
whole mix. Measured at 4:1, 20 ms / 200 ms: 0.140 dB pk-pk against
0.017 dB at 50 Hz, 0.070 against 0.005 at 100 Hz. Worst exactly where a
full-range mix keeps its energy.

**The crest estimate is more than crest factor, on purpose.** In steady
state it is exact — a sine reads 3.0103 dB, a 5 %-duty burst train 19 dB.
But the peak and RMS followers decay at different rates (150 ms against
25 ms), so a sustained *drop* in level leaves the estimate climbing:
3.3 dB → 15.3 dB at 3 ms / 750 ms after a 30 dB step down, taking the
release from 400 ms to 86 ms. That is an analogue auto-release, and it is
asserted in the harness so a follower retune cannot quietly lose it. A step
*up* moves the estimate 0.23 dB, so onsets do not jolt the timing.

**The sidechain filters are constants, not designs.** `InitSidechain()`
builds all three from `MakeHighPass` once, from `Init()`; `Configure()` only
selects one by index. That is what lets `SetComp` stay a plain direct
forward where `SetEq` needs double-buffering — a torn five-coefficient set
can be unstable, a torn index cannot exist. The reasoning is written out
above `SetComp` in `mastering_dsp.cpp`.

**Cost:** one `logf` and one `expf` per sample, plus one `logf` per
`kCrestDecim` (8) samples. No `sqrtf` — the power-sum's square root is
folded into `kLog10ScaleP` (10/ln10 rather than 20/ln10), so the better
stereo link is free. Roughly 220 cycles/sample against the old design's
~166, or about 2.3 % of one core. Guards: `pw` floors at `kMinDetPow`
(−90 dBFS) rather than taking `logf` of a denormal, and the `expf` is
skipped when `y < 0.01` dB.

**Latency: zero, and it must stay zero.** Bypass here takes the
*same-sample* dry signal, so a lookahead line would turn bypass into a
click and make the module's latency depend on bypass state. The sidechain
biquads are on the detector path, not the audio path.

`gr_db` is public and exposed via `CompGainReductionDb()` (sign-flipped to
a positive dB figure) for a gain-reduction meter. Nothing currently
renders it — it's a ready hook if you want one on the compressor page's
rings.

**Saturator** (`dsp_saturation.h`) — a tape model, not a waveshaper. Per
channel:

```
in ─┬─ emphasis ─ drive ─ [x2] ─ tanh/ADAA ─ [/2] ─ comp ─ DC ─ de-emph ─ bump ─┐
    └─────────────── 15-sample dry delay ─────────────────────────────────── mix ─ out
```

**The emphasis pair is the whole point.** A record head pre-emphasises,
the medium saturates, the playback head de-emphasises by exactly the same
curve. So the highs arrive at the knee already boosted and saturate first,
but any *linear* boost is undone on the way out. Net effect: drive
produces high-frequency **compression**, and the stage is flat when it is
not being driven. This is frequency-dependent saturation with memory, and
no memoryless curve reproduces it at any drive setting. That is the
difference between "tape" and "a soft clipper".

The de-emphasis is `InvertBiquad()` of whatever emphasis coefficients the
audio side is *currently* holding — recomputed once per block in
`BeginBlock()`, after the lerp. Designing the cut from the knob instead
would be wrong mid-move: the ISR holds a lerp of two coefficient sets, and
`lerp(design(a), design(b)) ≠ design(lerp(a, b))`. Inverting the held
coefficients is exact by construction at every point on the ramp;
`sat_response_test` measures both, at 8.3e-07 against 0.0200. `InvertBiquad`
is only safe because the emphasis shelves are minimum-phase — the
inverse's poles are the original's zeros — which the harness also asserts,
with 0.76 of pole margin.

**Three machines**, `kSatChars[]`, each an emphasis shelf, a head-bump
resonance and a knee constant:

| | Emphasis | Head bump | Knee |
|---|---|---|---|
| 30 ips | 3 kHz, +6 dB | 50 Hz, +1.5 dB, Q 1.2 | 0.8 |
| 15 ips | 2 kHz, +9 dB | 40 Hz, +3.0 dB, Q 1.4 | 1.0 |
| Saturated | 1.5 kHz, +12 dB | 35 Hz, +3.0 dB, Q 1.6 | 1.4 |

Slower tape saturates earlier, emphasises lower and harder, and puts a
bigger head bump lower down. The Emphasis and Bump knobs scale `emph_db`
and `bump_db`; the shaper function itself never changes.

**The shaper is `tanh`**, for a reason beyond taste: the magnetisation
curve of a ferromagnetic medium is the Langevin function `coth(x) − 1/x`,
and `tanh` is the same shape without the special-casing at zero. It is
`C^∞`, so it has none of the harmonic splatter a piecewise curve's
derivative discontinuity produces, and it approaches ±1 asymptotically, so
there is no corner to hit.

Evaluation is first-order **ADAA** (Parker et al., DAFx-16) on top of 2×
oversampling — the output is the integral-mean of `f` over
`[x[n−1], x[n]]`:

```
y[n] = (F(x[n]) − F(x[n−1])) / (x[n] − x[n−1]),   F(x) = ln(cosh x)
```

with a midpoint-`tanh` fallback when `|dx| < 1e-4`, where the quotient is
ill-conditioned. **`Antideriv` is not `logf(coshf(x))`** — that overflows
by |x| ≈ 89 and, worse, the stable form `|x| + log1p(exp(−2|x|)) − ln2`
loses catastrophically near zero, where a result of order `x²/2` is
assembled from terms of order `ln 2`. A Taylor series takes over below
0.5. Getting this wrong was worth 0.057 dB of small-signal gain error and
−2.79 dB at 12 kHz, and it was invisible until the response harness looked.

Oversampling and ADAA are not redundant: ADAA on top of 2× buys a further
**15–19 dB** of alias suppression at every drive level, measured. Its cost
is an inherent `cos(π f / 96 kHz)` rolloff — a two-tap average is a comb —
worth −0.17 dB at 6 kHz and −0.69 dB at 12 kHz. That is kept, not
compensated: a gently soft top end is on-character, and correcting it would
boost exactly the band the residual aliases land in. The harness asserts
departure from that *curve* rather than flatness.

**Gain compensation.** The input is multiplied by `knee × drive` and the
output divided by the same, so small-signal gain is exactly unity at every
drive setting and every character. Drive changes tone, not level — which
is what makes bypass an honest A/B and stops the Drive knob from feeding
the limiter.

**The ADAA history is never reset.** The previous saturator reset it in
`Configure()`, because switching between a cubic and a hard clip really
does invalidate the stored `F(x[n−1])`. At the 60 Hz control frame that put
one wrong sample into the output 62.5 times a second — measured at 31–35 dB
below program, and audible as a buzz. One `tanh` for every character
removes the reason for the reset, so the reset is gone rather than
dirty-checked. `sat_control_test::TestReconfigureArtifact` is the
regression, and it now reads bit-identical against a run configured once.

**Asymmetry.** `asym` offsets the shaper input; `tanhf(asym)` — the
shaper's response to the offset alone — is precomputed and subtracted,
removing the steady-state DC, and a one-pole ~5 Hz high-pass per channel
removes what the nonlinearity does to it under signal. Both are needed.

**Cost.** This is by some distance the most expensive stage in the chain:
four biquads, two half-band phases, and two shaper evaluations per sample
per channel, each shaper carrying a divide and — above |x| = 0.5 — an
`expf` and a `log1pf`. Measured on a host at **3.5× the compressor's
per-sample cost**, which against the compressor's ~220 cycles/sample puts
it near 780, or roughly 8 % of one core at 48 kHz. Treat that as an order
of magnitude and not a budget: the ratio was taken on x86, whose libm and
divider are relatively faster than the M7's, so on target the multiple is
probably worse. **It has not been profiled on hardware.** If you add to
this stage, measure there first.

**Bypass is not a branch.** It is a mix target of zero, eased at 5 ms, so
toggling crossfades to the delayed dry path instead of stepping. `mix = 0`
is bit-identical to bypass, and both are the input delayed by exactly 15
samples, so a partial mix is phase-coherent rather than a comb.

**Half-band oversampling** (`dsp_halfband.h`) — a 31-tap Parks-McClellan
half-band FIR, used polyphase. Every even-offset tap is zero and the centre
tap is 0.5, so one phase of each converter is a pure delay and the arithmetic
is 8 multiply-accumulates per phase rather than 31. Latency is
`kHbLatency = 15` base-rate samples for the up/down pair together, which is
what the chain budget spends (§8). `tools/halfband_design.py` regenerates
the table and documents why N = 31 rather than 35: N must be ≡ 3 (mod 4)
for a half-band, and N = 35 would cost 17 base samples against the 16
available.

**Limiter** (`dsp_limiter.h`) — brickwall with **`kLookahead` = 48 samples
(1 ms at 48 kHz)** of lookahead, fast one-pole attack, exponential release:

```cpp
buf_l[write] = l;                       // audio into a circular delay line
gd = (peak > ceiling) ? ceiling / peak : 1.f;    // detect on the PRE-delay input
if (gd < gain) gain += atk_coef * (gd - gain);   // ramp down ahead of the peak
else           gain += rel_coef * (gd - gain);   // smooth recovery
l = buf_l[rd] * gain;                   // apply to the delayed audio
```

Detection runs on the current input, which is `kLookahead` samples *ahead*
of what is being output, so the gain is already down by the time the peak
arrives. `atk_coef` is sized as five time constants across the lookahead
(`1 − exp(−5/kLookahead)`), which settles to within ~1 % in exactly the
window available. Transient limiting is therefore gain riding rather than
nonlinear distortion.

The post-gain `Clampf` to `±ceiling_lin` is a belt-and-braces guard for
that residual ~1 %, not the primary mechanism.

`Configure()` deliberately does not reset `gain` or `write` — that would
pop on a live parameter change.

**Bypass gates the gain, not the delay line.** `lim_bypass` (page 4's B2)
makes the applied gain unity; the circular buffer and the envelope keep
running. Skipping the delay would make the module's latency depend on a
button, which would click on the toggle and desynchronise anything
tracking it. It also means the envelope is already settled when the
limiter comes back in.

**TPDF dither** (`dsp_dither.h`) — two independent xorshift32 draws
subtracted to give a triangular distribution, scaled to `dither_lsb ·
2⁻²³`. The two channels use different nonzero seeds
(`0x2545F491`, `0x9E3779B9`) so the noise is decorrelated between L and R;
a shared seed would produce correlated mono noise. **xorshift32 with a
zero state is stuck at zero** — any new seed must be nonzero.

### 4.3 Ordering decisions worth knowing

- **Trim is post-limiter.** Positive trim can exceed the limiter ceiling
  and clip the codec. This is a real footgun and it's documented in the
  user guide; if you'd rather it be safe, move `trim_lin_` above
  `lim_.ProcessSample`.
- **Dither is post-trim**, which is correct — dither belongs at the final
  quantisation point, and scaling it afterward would defeat it.
- **The compressor detects post-EQ**, so EQ moves change how hard the
  compressor works.

---

## 5. Build system

```
Makefile              → libDaisy core Makefile (standard Daisy workflow)
lib/alchemy-sdk       → submodule, compiled from source
lib/libDaisy          → submodule, prebuilt once
```

```sh
git clone --recurse-submodules <repo> && cd alchemy-mastering
make libdaisy          # once after cloning
make                   # → build/mastering.bin
make program-dfu       # flash (module in DFU mode first)
make clean
```

Key `Makefile` details:

| Setting | Value | Why |
|---|---|---|
| `TARGET` | `mastering` | names the `.bin` |
| `BOARD` | `v2` (default), `v1` | selects BSP sources + `-DALCHEMY_BOARD_V2` |
| `APP_TYPE` | `BOOT_SRAM` | runs from SRAM under the Alchemy bootloader |
| `LDSCRIPT` | `alchemy_stm32h750ib_sram.lds` | matching linker script |
| `CPP_STANDARD` | `-std=gnu++17` | SDK requires C++17; libDaisy defaults to gnu++14 |

The SDK is globbed, not listed:

```make
CPP_SOURCES += $(sort $(shell find $(ALCHEMY_DIR)/framework/src -name '*.cpp'))
CPP_SOURCES += $(sort $(wildcard $(ALCHEMY_DIR)/hardware/alchemy-lab/$(BOARD)/src/*.cpp))
```

so bumping the submodule picks up new SDK files with no Makefile edit.
Unused surfaces still compile but are dropped at link.

**The board stamp.** At the bottom of the Makefile:

```make
BOARD_STAMP := $(BUILD_DIR)/.board-$(BOARD)
ifeq ($(wildcard $(BOARD_STAMP)),)
_BOARD_GUARD := $(shell rm -f $(BUILD_DIR)/*.o ... ; touch $(BOARD_STAMP))
endif
```

Switching `BOARD=` wipes the object tree automatically. Without this you'd
get a silently mislinked binary mixing v1 and v2 objects, since the
filenames are identical.

### Gotchas

- **Object files are flattened into `build/` by basename.** Two source
  files anywhere in the tree cannot share a filename, even in different
  directories. This is a libDaisy build constraint, not ours.
- New app sources must be appended to `CPP_SOURCES` manually — only the
  SDK is globbed.
- `build/` is checked in on this branch, which means a stale `.o` can
  outlive a `git checkout`. `make clean` when a build behaves impossibly.

### Flashing

The Alchemy Lab V2 runs `DaisyBootloader-AlchemyLabV2`, serving DFU on the
front-panel USB-C. Press or hold **B3** during the ~2 s boot window (rings
spin a warm-white comet → slow breathe once latched), then `make
program-dfu`. The
[Web Programmer](https://hermeticmodular.com/program) does the same thing
from a browser.

An app can also request DFU programmatically:

```cpp
daisy::System::ResetToBootloader(
    daisy::System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
```

Note that **B1 + B2 held through boot runs CV factory calibration**, not
DFU — a ~15 s procedure that writes a per-board record to a dedicated QSPI
sector. That record survives reflashes.

---

## 6. Recipes

### Add a knob to an existing page

1. Declare a `VirtualKnob` with the next free pot index (0–5) and a ring
   style from `mastering_palette.h`.
2. Add it to the `Page(...).Knobs(...)` list.
3. Add a field to the relevant `*Params` struct in `mastering_dsp.h`.
4. Push `knob.Value()` in `UpdateParams()`.
5. Consume it in the stage's `Configure()`.

Knob values persist automatically — `Pager` serializes by `(page, pot)`,
so no preset changes are needed as long as the pot count is unchanged.

### Add a new stage

1. New `src/dsp_<stage>.h`: a struct with `Configure(params, fs)` and
   `ProcessSample(...)`. Include `dsp_common.h` for the shared helpers;
   don't add namespace-scope `constexpr` to your own header.
2. Add a params struct (or extend one) in `mastering_dsp.h`.
3. Instantiate in the anonymous namespace of `mastering_dsp.cpp`, call
   `Configure` from the matching setter, splice into `Process()`.
4. Keep it running when bypassed — gate the output, not the computation.

Header-only means no `CPP_SOURCES` edit.

### Add a fifth page

1. `Pager pager(hw.buttons[0], 5, kNumPots);`
2. `pager.SetPageColor(4, kSomeColor);`
3. Declare knobs + a `Page(4)`, and `loop.Use(new_page)`.
4. Extend the `switch (page)` blocks in `UpdateParams` and
   `RenderButtons` — both currently use `default:` for the Output page, so
   a new page will silently inherit its behaviour (limiter bypass, inert
   B3) until you add an explicit `case`.

Changing the page count changes `Pager`'s serialized size, so **existing
presets are invalidated.**

### Add a compressor character

1. One row in `kCharSpec` (`dsp_compressor.h`) — knee, sidechain corner,
   the four adaptation multipliers, the dual-TC pair, auto-makeup fraction.
   `InitSidechain()` designs one high-pass per row, so nothing else is
   needed for the filter.
2. One colour in `kCharColors` (`mastering_palette.h`). Its array length is
   the modulus of the B3 cycle — keep them in step.
3. Widen the `% 3u` in `ChainModes::Deserialize` **and** the `% 3` in
   `UpdateParams`'s B3 case, and bump `SchemaHash()`.
4. Widen the bound check in `Compressor::Configure` (`p.character < 3`).
5. Extend `kChars[]` in `tests/comp_response_test.cpp` and regenerate the
   goldens — the curve and envelope grids are per character.

Steps 3 and 4 are three separate moduli that all have to agree. If they
disagree the symptom is a character that is unreachable, or one that reads
past the end of a table; there is no diagnostic.

### Add a tape machine

Same shape, different tables:

1. One row in `kSatChars` (`dsp_saturation.h`) — emphasis shelf (Hz, dB),
   head bump (Hz, dB, Q), knee.
2. One colour in `kSatColors` (`mastering_palette.h`).
3. Widen the `% 3u` in `ChainModes::Deserialize` **and** the `% 3` in
   `UpdateParams`'s B3 case, and bump `SchemaHash()`.
4. Widen the bound check in `Saturator::Configure` (`p.character < 3u`,
   falling back to 0).
5. Extend the character loops in `tests/sat_response_test.cpp` and
   regenerate the goldens — the curve and harmonic grids are per character.

Do **not** give a machine a different shaper function. The single `tanh`
is what makes the ADAA history valid across a character switch, and
therefore what lets `Configure()` leave it alone (invariant 15). A new
shape means the reset comes back, and the 62.5 Hz artifact with it.

### Add persistent state

Extend `ChainModes` (bump `SchemaHash()`, extend `SerializedSize()`,
clamp in `Deserialize`), or add a new `Serializable` and
`presets.Manage()` it before `presets.Init()`. Either way, existing preset
slots stop loading — by design.

### Add a gain-reduction meter

`CompGainReductionDb()` already returns a positive dB figure.
`VirtualKnob::Overdraw(fn, ctx)` paints a callback on top of a knob's
declarative ring after `PerfRenderer` runs — the `kick` example in the SDK
does exactly this for an envelope meter. The compressor page's threshold
or makeup ring is the natural host.

---

## 7. Invariants

Things that will bite quietly if broken:

1. **Never call `RisingEdge()` / `FallingEdge()` on B2 or B3** in
   `mastering.cpp`. `Settings` needs those edges live. (§3.4)
2. **Bump `ChainModes::SchemaHash()`** whenever its layout changes. (§3.5)
3. **`Deserialize` must clamp every field** — flash contents are
   untrusted input. (§3.5)
4. **Bypass gates output, never computation.** (§4.1)
5. **Unit conversion belongs in `Set*`, never in `Process()`.** (§1)
6. **Keep `UpdateParams` unconditional** — that's what makes preset loads
   apply with no extra plumbing. (§3.3)
7. **Dither seeds must be nonzero and must differ per channel.** (§4.2)
8. **`Exp()` requires `min > 0`.** A zero lower bound gives a degenerate
   geometric interpolation.
9. **No two source files may share a basename.** (§5)
10. **Never write `eq_bank_[eq_live_]`.** The EQ's control→audio handoff is
    double-buffered; the writer fills the *other* slot and publishes by
    storing the index. Writing the live slot reintroduces exactly the tearing
    the buffer exists to prevent. (§1)
11. **`dsp_biquad.h` stays `<cmath>`-only.** That is what lets `tests/`
    compile it on a host with no stubbing at all, and the measurement
    harness is the only thing standing between the coefficient math and a
    silent regression. (§9)
12. **The compressor adds no latency, and must not.** Its bypass takes the
    same-sample dry signal (§4.1), so a lookahead line would turn bypass
    into a click — and the budget it would spend is now fully allocated.
    (§8)
13. **Inside `Compressor`, gain reduction is a POSITIVE attenuation**; only
    `gr_db` is negated. The decoupled release stage uses `fmaxf`
    accordingly. Flipping either without the other silently swaps attack
    and release. (§4.2)
14. **Latency must not depend on bypass state**, for any stage that has
    any. The limiter's bypass gates the gain and keeps the delay line
    running; the saturator's bypass is a mix target of zero over a matched
    dry delay. Neither is allowed to become a branch that skips the delay.
    (§4.2, §8)
15. **Never reset the saturator's ADAA history in `Configure()`.** It is
    valid across every character because the shaper is always the same
    `tanh`; resetting it at the 60 Hz control frame injects a 62.5 Hz
    artifact 31–35 dB below program. This was a real shipped bug.
    `sat_control_test` is the regression. (§4.2)
16. **Derive the de-emphasis from the coefficients the audio side is
    holding**, not from the knob. `InvertBiquad(run_.emph)` in
    `BeginBlock()` after the lerp is exact at every point on a ramp;
    designing the cut from the parameter is not. (§4.2)

---

## 8. Latency budget

**Chain latency today is 63 samples (1.31 ms)** — 48 for the limiter's
lookahead and 15 for the saturator's half-band pair.
`comp_control_test`'s `TestChainLatency` measures it end to end rather than
asserting it from prose — this section has gone stale once already, and now
something fails when it does.

For the chain as a whole, **64 samples (~1.3 ms) is the ceiling** any stage
may claim, so **1 sample remains**: the budget is spent. There is no delay
compensation in a Eurorack rack, so the limit is perceptual rather than
arithmetic; at the end of a mastering chain nothing downstream recombines,
so comb filtering is not the binding constraint.

Anything that wants latency from here has to take it from an existing
stage. The realistic trade is the half-band filter: N = 31 costs 15 base
samples and is what the 16 available bought. Going to N = 35 for a steeper
transition would cost 17 and does not fit; going *down* is where slack
would come from, at the price of alias rejection.

Two stages must never spend any of it:

- **The EQ.** Bypass is implemented by discarding the wet output and keeping
  the *same-sample* dry (`mastering_dsp.cpp`), so any EQ latency would
  compare signals from different times — bypass becomes a click, and the
  module's total latency changes with bypass state. Fixing that means a
  matched dry delay line and bypass that is no longer true bypass, bought
  for nothing: the EQ is linear, so it does not alias, and oversampling it
  would only buy decramping, which the matched designs already deliver free.
- **The compressor**, for exactly the same bypass reason. It is also the
  stage that needs it least: a glue compressor's attack is measured in tens
  of milliseconds, so lookahead buys nothing a slower attack does not.

The 16 samples were spent on the **saturator** — the one genuinely
nonlinear stage, hence the only one where oversampling suppresses aliasing
beyond what ADAA already does. It uses 15 of them and buys 15–19 dB over
ADAA alone. The saturator's own bypass is a matched dry delay, not a
branch, so its 15 samples are present whether or not the stage is engaged
(invariant 14).

---

## 9. Testing

```sh
make test          # build and run all six host harnesses
make test-golden   # regenerate the golden CSVs, deliberately
```

Host-only, no cross-toolchain: `tests/` compiles `dsp_biquad.h`,
`dsp_compressor.h` and `dsp_saturation.h` directly, and `mastering_dsp.cpp`
against a six-line `AudioHandle` stub. `tests/test_report.h` holds the
shared assert-and-print framework; `eq_test_common.h`, `comp_test_common.h`
and `sat_test_common.h` hold the measurement support for their respective
stages.

- `tests/eq_response_test.cpp` — sweeps the realized filter against the
  *analog prototype* with no prewarping (prewarping would hide the very
  cramping error the harness exists to measure), checks every design in the
  parameter grid for stability and minimum phase, DFTs the real impulse
  response to confirm the implementation matches its own coefficients, and
  measures the arithmetic noise floor.
- `tests/eq_control_test.cpp` — drives the real `SetEq`/`Process` and asserts
  that a stationary knob is bit-for-bit identical to never calling `SetEq`
  again. It carries a negative control: gross jitter *must* be detected, or
  the test is not measuring anything.
- `tests/comp_response_test.cpp` — the compressor alone: static curve
  against the analytic gain computer, knee width and C¹-ness, attack and
  release settling, envelope ripple, sidechain response, power-sum linking,
  the dual-time-constant release, crest calibration and adaptation,
  auto-makeup level match, mix law, and zero latency.
- `tests/comp_control_test.cpp` — the real `SetComp`/`Process`, plus the
  end-to-end chain latency measurement that keeps §8 honest.
- `tests/sat_response_test.cpp` — the saturator alone: half-band round-trip
  transparency and image rejection, emphasis/de-emphasis reconstruction
  (including mid-ramp) and inverse-filter stability, unity small-signal
  gain at every drive and character, harmonic decay rate, asymmetry and its
  DC, alias energy, head-bump height and return, the frequency-dependence
  that makes it tape, bypass/mix delay matching, adversarial inputs, and
  the goldens.
- `tests/sat_control_test.cpp` — the real `SetSat` cadence: the 62.5 Hz
  reconfigure regression (invariant 15), knob-sweep zipper, character
  switching, and preset recall.

Measurement in the saturation harnesses is by **Hann-windowed single-bin
DFT**, not FFT. A saturator's output has predictable support — harmonic
`k` of `f0` lands at `|k·f0 − m·fs|` in closed form — so the bins worth
reading are known in advance, and reading them individually keeps the
tests tree at `g++` and nothing else (invariant 11). The window is not
optional: without it, leakage from the tone swamps a −90 dB alias bin, and
the alias frequencies are not ours to place.

**Negative controls are the point, not decoration.** Four of the
compressor's assertions ship with a deliberately broken model that the same
bound must reject — the old branching one-pole for envelope ripple, a hard
knee for knee continuity, `Precise` for both the dual-exponential fit and
the adaptation. The saturation harnesses carry four more: the old cubic
clip against the harmonic-decay bound, pointwise base-rate evaluation
against the alias bound, a knob-designed de-emphasis against the
reconstruction bound, and a hard-stepped knob against the zipper bound. A
continuity bound loose enough to pass will also pass the thing it was meant
to catch, and nothing in the output tells you which. If you add an
assertion of that shape, add its control too.

**Two things the compressor harness does that the EQ's does not**, both
forced by the stage rather than by taste:

- It cannot assert bit-identity for a stationary knob. The compressor has a
  running envelope and a 5 ms parameter ease, and nothing in `SetComp` is
  dirty-checked. The claim it makes instead is that 0.05 % threshold jitter
  moves the output gain by under 0.05 dB (measured: 0.02 dB) — which is
  precisely the evidence for *not* giving this stage the EQ's handoff.
- It flushes the chain with silence between scenarios. `mastering_dsp`
  keeps chain state at file scope and `Init()` does not clear it — correct
  for firmware that calls `Init()` once, fatal for a harness that runs
  several scenarios in one process. Without the flush, the latency probe
  finds a leftover sample instead of its own impulse.

If you change the coefficient math the EQ golden will fail; if you change
the compressor's curve or its timing, `comp_curve.csv` or
`comp_envelope.csv` will; if you touch the knee, the emphasis table or the
shaper, `sat_curve.csv` or `sat_harmonics.csv` will. That is the point —
regenerate only when you meant to change the design, and read the diff
before you commit it.

---

## 10. Reference

- SDK framework headers: `lib/alchemy-sdk/framework/include/alchemy/`
- V2 board class: `lib/alchemy-sdk/hardware/alchemy-lab/v2/include/alchemy/hw/alchemy_lab_v2.h`
- V2 pin/LED layout: `.../alchemy_lab_v2_layout.h`
- SDK examples: `lib/alchemy-sdk/examples/` — `stereo_eq` is the
  maximal-opt-in contrast to this project, `kick` the minimal one.

### Bumping the vendored libraries

```sh
git -C lib/alchemy-sdk pull origin main
git add lib/alchemy-sdk && git commit -m "Bump alchemy-sdk"
```

The pinned libDaisy commit matches the one the Alchemy SDK vendors and
tests against; bump them together.

The SDK is in beta — APIs, surface names, and on-flash preset formats may
change before its first stable release. Re-read the SDK changelog after a
bump, and expect preset invalidation if a managed surface's layout moved.
