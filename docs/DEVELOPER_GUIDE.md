# Lapis Philosophorum — Developer Guide

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
├── dsp_analysis.h         band split + envelope followers (CV telemetry, not signal path)
├── mod_source.h           CV modulation engine — mode map and public seam
├── mod_source.cpp         CV modulation engine — the five generators
├── mastering_palette.h    LED colour constants
└── version.h              firmware version — single source of truth (§5)

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

`mod_source.*` is the one exception to the header-only rule, and
deliberately so: it runs on the control thread at 1 kHz, not per sample, so
inlining buys nothing, and a real translation unit keeps the five
generators out of every file that includes the header. It **includes
nothing from libDaisy or the alchemy-sdk** — only `<cstdint>` and
`<cmath>` — which is the same invariant `dsp_biquad.h` holds and is what
lets `tests/` link it directly. Everything about actual jacks (ADC
thresholds, DG411 routing, DAC cadence) lives in `mastering.cpp`;
`mod_source` only ever produces volts.

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

What isn't carried by any knob is the mode/bypass state, which gets its own
`Serializable`:

```cpp
struct ChainModes : public alchemy::Serializable
{
    uint8_t eq_bypass, comp_bypass, sat_bypass;         // bytes 0-2
    uint8_t mid_q_index, comp_character, sat_character; // bytes 3-5
    uint8_t lim_bypass;                                 // byte 6
    uint8_t mod_secondary[6];                           // bytes 7-12 — appended

    size_t   SerializedSize() const override { return 7 + kNumModes; }
    void     Serialize(uint8_t* out) const override;
    bool     Deserialize(const uint8_t* in) override;   // clamps every field
    uint32_t SchemaHash() const override { return 0x4D535434u; }
};
```

`mod_secondary` is one B3 index **per modulation mode**, not one global
index, so each mode remembers its own — leave Clocked on ÷2, visit Euclid,
come back, and it is still ÷2. Slot 0 (Off) is unused and always reads 0.

Two things to note:

**`Deserialize` clamps, it doesn't validate.** `% 3u` for the Q index, the
compressor character and the tape character; `& 1u` for the booleans. A
corrupt or foreign slot can therefore never push an out-of-range index
into `kMidQTable[]`, `kSatChars[]` or into the DSP. This is the cheap,
correct discipline for flash-backed state — don't relax it. The
`mod_secondary` clamp takes its modulus from
`mod_source::SecondaryZones(mode)` rather than a literal, because the zone
count differs per mode (Clocked 5, Euclid 5, SmoothRandom 4, MultiLfo 3,
Analysis 2) — a fixed modulus would be wrong for all but one of them.

**`SchemaHash()` is a manual constant** (`0x4D535435` = "MST5"). The
preset store XORs every managed component's hash and stamps slots with the
result; a mismatch on load makes the slot read as empty rather than
restoring misaligned bytes. **If you change `ChainModes`'s field layout or
size, bump this constant.** If you forget, old slots will deserialize into
the new layout and produce garbage — the one failure mode the mechanism
exists to prevent.

It was last bumped MST4 → MST5 when the Output page was relaid out. That
bump is worth studying, because **nothing about the serialised layout
changed at all** — `ChainModes` has exactly the same fields and the same
size. What changed is what a stored *pot position* means: Trim moved from
pot 2 to pot 5, the mode selector from pot 5 to pot 4, and pots 2 and 3
became the modulation source's parameter knobs. `Pager::SchemaHash()`
cannot see any of that — page count is still 4 and pot count still 6 — so
an MST4 slot would have loaded cleanly and applied its Trim value as a
shape rotation. **Bump the constant when the *meaning* of stored state
changes, not only when its layout does.**

The bump before it, MST3 → MST4, added the `mod_secondary` array, and
carried a trap worth remembering: `Pager::SchemaHash()` did **not** change
there either, so the only thing invalidating old slots was this constant. Because `Presets::SchemaHash()` is the XOR of every
managed component, that is enough: `HasValid` fails and nothing is
deserialized, `Pager` included. But then `Pager` falls back to its default
stored value of `0.5`, and `0.5` on K5's `Selector(6)` is `MultiLfo` — so
the module would have booted with six LFOs already driving the CV jacks.
`main()` guards it:

```cpp
const bool restored = presets.BootLoad();
...
if (!restored) {
    /* settle the ADC first, then: */
    pager.SetStored(3, 4, 0.f, phys);   // force K5 to Off
}
```

The ADC settle matters — `SetStored` re-arms pot catch against the physical
position, and a zeroed `phys[]` would let K5 grab on the first frame. **Any
future selector knob whose zero position is the safe one needs the same
guard**; the Pager default is 0.5, not 0.

An earlier bump, MST2 → MST3, was for when the saturation moved to its own page:
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
- **`CvMatrix` / `CvRouter`** — **no CV *into* any knob.** Every knob is a
  direct, unlatched control; no `VirtualKnob` here calls `.Cv(...)`.

That last one is worth stating precisely, because the CV jacks are far
from unused. They are a modulation *source* (§3.7) — the jacks are
outputs in four of the six modes, and where they are inputs the firmware
reads them itself through `CvGate` rather than through the `VirtualKnob`
CV path. `loop.Cv()` is consumed directly in `PollModulation`.

If you want CV *modulating knobs* back, construct a `CvRouter`,
`loop.Use()` it, and the `VirtualKnob` CV path activates — but note that
it would then contend with the modulation source for the same six jacks,
and that adding a `Serializable` surface to the preset payload invalidates
existing slots.

### 3.7 The CV modulation source

`mod_source.cpp` is the generator; `mastering.cpp` is the hardware seam.
The split is strict — the engine only ever produces volts and a
`bool is_output[6]`, and knows nothing about ADCs, DG411 switches or DACs.

**Two DAC update rates, because the six jacks are not the same hardware.**

| Jacks | Backing DAC | Rate | Cost |
|---|---|---|---|
| J7, J8 (4, 5) | STM32 internal DAC1 | every 1 ms poll | one register write |
| J3–J6 (0–3) | MCP4728 over I²C | every 4th poll (250 Hz) | ~430 µs, ~11 % of the main thread |

This is why the ramp and the stepped-random output live on J7/J8 in
Clocked mode, and why two of the five Euclidean gates do: their
discontinuities are what stepping shows up in. `kMcpFlushTicks` is the one
number to turn if the gates on J4–J6 feel loose on hardware — it trades
main-thread load against up to 4 ms of gate jitter. Audio is in the SAI
ISR and is unaffected either way.

Even at 250 Hz this only works because of `StageVolts` + `FlushCvOutputs`
(invariant 20). Per-jack `SetVolts` in a loop would be ~1.7 ms.

**Routing is applied only on a mode change.** `ApplyModRouting` drives
every DAC to 0 V *before* touching a DG411, so a jack never connects to a
stale voltage from the previous mode. The switches are expander writes, so
doing this per tick would be absurd; `mod_routed_mode` starts at `0xFF` so
the first poll after boot always establishes a known routing.

**Edge detection runs on all six channels but only two are consulted.** A
jack being driven as an output is still read by the ADC — we see our own
DAC voltage — so output jacks generate spurious edges in the detector.
Harmless, because `ClockJack()`/`ResetJack()` return `0xFF` for modes with
no such input and the masks simply never match.

**Analysis reads telemetry at tick rate, not at the 16 ms frame.** With a
1 ms follower the CV should track transients; a 62.5 Hz refresh would throw
most of that away. It is six float loads.

`PollControls` exists only because `ControlLoop` takes a single `OnPoll`
hook: it calls `PollTaps` and then `PollModulation`. Both want the same
1 ms cadence and neither depends on the other. Note that `OnPoll` and
`OnFrame` run on the same thread, sequentially inside `loop.Tick()`, so
nothing here needs synchronising against `UpdateParams`.

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
    trim_run_ += trim_ease_ * (trim_tgt_ - trim_run_);   // 5 ms, per sample
    l *= trim_run_;  r *= trim_run_;
    lim_.ProcessSample(l, r);      // stereo-linked; the last word on level
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

**Gain compensation is referenced to a program level, not to the origin.**
The input is multiplied by `knee × drive`; the output is divided by the
*chord* from the origin to `kSatRefAmp`, `tanh(drive × a) / a`, rather than
by the `knee × drive` that went in.

Dividing by `knee × drive` is what this used to do, and it is exactly right
about the slope at the origin — which is the trap. Loudness is set by the
curve's **large-signal** gain, and that collapses as `tanh` flattens.
Measured through the whole chain on a mix-like program, drive 0 → 24 dB cost
**11.3 dB** of output RMS, dropped the peak 22 dB, and disengaged the limiter
completely. The harmonics were all present — 17 % THD at −18 dBFS — and
inaudible underneath a level drop of the same size, because an unmatched A/B
is decided by loudness before timbre. The knob read as a volume control that
also dulled the top end. It is now 1.5 dB over the same range.

`kSatRefAmp` is 0.195, a one-parameter fit to the makeup that holds output
RMS constant for noise at −12 dBFS RMS, tracking it within 0.33 dB over
0–24 dB of drive and all three knees. It reads low for a program level
because program saturates on its peaks rather than at its average. The
numerator is normalised per character (`tanh(knee × a) / knee`) so that
drive 0 — where `drive_` equals `knee` — is *exactly* transparent; that
property is load-bearing for the emphasis-pair argument and survives the
change. It is the one scalar here that steps rather than eases, and it can
afford to: across the three knees it spans 0.1902 to 0.1934, a 0.14 dB step
on a B3 press that changes the knee, the emphasis curve and the bump in the
same instant.

Two things the scheme costs, both asserted rather than implied:

- Small-signal gain is **no longer unity** away from drive 0. It reaches
  +7.9 dB at full drive on 30 ips, +9.7 on 15 ips, +12.5 on Saturated, and
  lifts the noise floor with it. That is tape compression — quiet material
  rising relative to loud — not a defect, but Drive is no longer free.
- The makeup follows drive only. Emphasis adds its own HF compression on
  top, so Saturated with emphasis wide open still loses ~7.7 dB across the
  drive range. Compensating that would need the program's spectrum.

Harmonic levels *relative to the fundamental* are untouched by any of this,
since a post gain cannot change a ratio — which is why the harmonic golden
held through the change while the curve golden moved by exactly a constant
per character.

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
`expf` and a `log1pf`. The gain compensation adds a `tanhf`, but only while
Drive is actually moving: `drive_` converges to its target bit-exactly and
then stops, so the dirty check over `comp_` costs one compare per sample in
the steady state. Measured on a host at **3.5× the compressor's
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

**Limiter** (`dsp_limiter.h`) — brickwall with **`kLookahead` = 60 samples
(1.25 ms at 48 kHz)** of lookahead, instantaneous attack, exponential release:

```cpp
buf_l[write] = l;                       // audio into a circular delay line
gd = (peak > ceiling) ? ceiling / peak : 1.f;    // detect on the PRE-delay input
if (gd < rel) rel = gd;                          // instant attack on the TARGET
else          rel += rel_coef * (gd - rel);      // smooth recovery
gain = box.Process(rmin.Process(rel));  // peak hold, then linear ramp
l = buf_l[rd] * gain;                   // apply to the delayed audio
```

Detection runs on the current input, which is `kLookahead` samples *ahead*
of what is being output, so there is a full window in which to get the gain
down before the peak arrives.

**The attack must not be a one-pole, and this is the subtle part.** A
one-pole only converges while its target is *held*. A transient shorter
than the window sets `gd` low for a sample or two and then lets it snap
back to 1.0, so the gain barely moves and the peak arrives at full height.
The chain shipped that way until the limiter review; measured overshoot
into the safety clamp was **+12.35 dB on an isolated one-sample transient
and +3 to +8 dB on ordinary program with stabs** — the limiter was a hard
clipper for exactly the signals lookahead exists to catch. Steady-state
content was unaffected, which is why it hid.

Two stages on the gain signal replace it:

- **`RunMin`** — running minimum over `kHoldWin` samples. This is the peak
  *hold*: a peak keeps the target down for the whole window instead of for
  one sample. O(1) worst case (van Herk / Gil-Werman), so there is no
  data-dependent inner loop in the ISR.
- **`BoxCar`** — moving average over `kRampWin`. Turns `RunMin`'s step into
  a linear ramp, and unlike a one-pole it settles *exactly* rather than
  asymptotically. Its accumulator is `double` because an
  add-one/subtract-one running sum has nothing to pull it back from a
  float random walk over long uptimes.

Zero overshoot is then a property of the window sizing, not a lucky
measurement. With audio delay `D`, running-min window `A`, boxcar window
`B` and detector group delay `Td`, a peak at input index `p` is seen at
`p+Td` and leaves the delay line at `p+D`; the gain applied at `p+D` is a
mean of `B` running-minima, and every one of them is ≤ that peak's target
exactly when

```
B <= D - Td + 1 <= A
```

The harness asserts the resulting overshoot is 0.00000 dB across noise,
impulse trains, square waves, dense mixes and level steps at every ceiling
— see `tests/lim_response_test.cpp`.

**The ceiling is a true-peak (dBTP) ceiling.** The detector runs 4x
oversampled through two cascaded `HalfBandUp` stages, so it sees the
inter-sample peaks the DAC's reconstruction filter — or a lossy encoder
downstream — will actually produce. A sample-peak limiter guarantees
nothing about those: measured on HF-dense program, this chain set to
−1.0 dBFS emitted **+0.40 dBTP**, 1.4 dB over its own ceiling. It now
emits −1.00 dBTP.

Two consequences that are easy to get wrong:

- **Both outputs of the first stage go through the *same* second-stage
  instance**, in time order. Giving the even and odd phases their own
  filter instance looks natural and is wrong — each would then see a
  stream decimated by two, which is not a 4x interpolation of anything.
- **`Td` is a range, not a number.** The four interpolated positions
  emitted in one call stand at base times `n−11.25`, `n−11.00`, `n−10.75`
  and `n−10.50`, so `A` must be sized against the earliest and `B` against
  the latest. `kTpDelay{Min,Max}Q` hold those bounds in quarter-samples,
  the coarsest grid all four land on, and the `static_assert`s in the
  header check the inequality at both ends. The harness measures the delay
  with an impulse rather than trusting the algebra.

The raw sample delayed by 11 is folded into the peak alongside the
interpolated set. The half-band rolls off 0.46 dB by 20 kHz, so on
near-Nyquist content the interpolated values can read *below* the true
sample peak; taking the max makes the detector never worse than the
sample-peak one it replaced. Base time `n−11` is inside the range above,
so it needs no separate sizing.

`kLookahead` is 60 rather than 48 to absorb the detector's delay without
shrinking the ramp window — see §8 for why the chain budget moved instead.

**Cost.** The detector is three `HalfBandUp` calls per channel — stage one
once, stage two twice — at 8 multiply-accumulates each, so 48 MACs per
stereo sample on top of what the limiter did before. Against the ~10 000
cycles an H7 has per sample at 48 kHz that is a low single-digit percent,
and unlike the saturator's cost this one is plain arithmetic rather than a
host-measured ratio: there are no transcendentals and no divides in it.
`RunMin` adds two compares per sample plus one `kHoldWin`-long suffix pass
every `kHoldWin` samples — bounded work with no dependence on the signal,
which is why it is van Herk rather than a monotonic deque. **None of this
has been profiled on hardware.**

The post-gain `Clampf` to `±ceiling_lin` is now genuinely belt-and-braces:
it is there for float rounding, and the harness asserts it never has
anything to do.

`Configure()` deliberately does not reset the envelope, the delay line or
either window — that would pop on a live parameter change.

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

`dither_lsb` is always `kDitherLsb`, i.e. 2. It used to be a knob and is
not one any more: the jacks carry an analog voltage rather than a stored
24-bit deliverable, and whatever ADC feeds the module has already put ~80
LSBs of its own noise on the signal. `OutParams` keeps the field
parameterised anyway, because several chain tests set it to 0 to get a
bit-deterministic chain for comparisons the noise would otherwise swamp.

**The chain quantises to the 24-bit grid itself**, round-to-nearest, in
`Process()` immediately before writing `out[][]`. This is load-bearing for
everything above. libDaisy's `f2s24` is `(int32_t)(x * 8388608.0f)` — a C
cast, so it truncates toward zero, which makes the zero bin two LSBs wide
instead of one. That is not a uniform quantiser and no amount of dither can
linearise it: measured, digital silence with 1 LSB of dither produced
2²¹ consecutive identically-zero output samples, with the dither stage
running the whole time and being arithmetically discarded.

Rounding first leaves `f2s24` nothing to truncate, and is exact by
construction — `roundf` yields an integer N, |N| ≤ 2²³ fits float32's
mantissa without loss, and scaling by a power of two is exact in both
directions, so `f2s24`'s own multiply reproduces N and its cast is a no-op.
`chain_test` pins both halves: every output sample lands on the grid, and
at digital silence 56 % of samples are nonzero against 56.25 % predicted
for 2 LSB TPDF under round-to-nearest.

### 4.3 Ordering decisions worth knowing

- **Trim is pre-limiter**, so the ceiling is the last word on level.
  Behind the limiter, +12 dB of trim simply undid the brickwall and
  clipped the codec — the knob could defeat the stage that exists to
  prevent exactly that. Ahead of it, trim is the limiter's input drive,
  which is also the standard mastering topology: push in for loudness and
  the ceiling still holds. The cost is that trim can no longer raise the
  output above the ceiling at all, which is the point.
- **Trim is eased at 5 ms**, like every other scalar that multiplies the
  audio directly (the compressor's makeup and mix, the saturator's drive
  and mix). It used to be written straight from the control frame, and
  being a plain gain is exactly why that was wrong rather than merely
  inconsistent: a raw gain step *is* the click, with nothing downstream to
  smooth it. At the pot's 8 ms smoothing a brisk sweep of the ±12 dB range
  delivers ~2 dB per 16 ms frame, i.e. a 26 % discontinuity in one sample —
  measured at 9.4e-3 against a program slew of 5.5e-3 before the ease.
- **Dither is post-trim and post-limiter**, which is correct — dither
  belongs at the final quantisation point, and scaling it afterward would
  defeat it. The round-to-nearest quantisation is the only thing after it.
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
git clone --recurse-submodules <repo> && cd lapis-philosophorum
make libdaisy          # once after cloning
make                   # → build/lapis_philosophorum_v0.x.0.bin
make program-dfu       # flash (module in DFU mode first)
make clean
```

Key `Makefile` details:

| Setting | Value | Why |
|---|---|---|
| `TARGET` | `lapis_philosophorum_v$(VERSION)` | names the `.bin`; version read from `src/version.h` (§5) |
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

### Versioning

`src/version.h` is the single source of truth, currently **0.x.0**:

```c
#define LAPIS_VERSION_MAJOR 0
#define LAPIS_VERSION_MINOR 5
#define LAPIS_VERSION_PATCH 0
#define LAPIS_VERSION_STR   "0.x.0"
```

Nothing else defines a version. The Makefile *reads* the string back with
a `sed` one-liner and folds it into `TARGET`, so the artifacts are named
`build/lapis_philosophorum_v0.x.0.{bin,elf,hex,map}` — which is why
`LAPIS_VERSION_STR` has to stay on one line with its value in double
quotes (the `sed` is deliberately strict, and an empty match is a hard
`$(error)` rather than a binary named `..._v.bin`). Bumping the version
therefore changes the artifact name, and the previous release's `.bin`
survives in `build/` until the next `make clean`. `program-dfu` builds its
path from `TARGET`, so flashing needs no extra argument.

`lapis::kVersion` packs the three numbers as `0xMMmmpp` so builds compare
with `<` and `>`.

Bump PATCH for fixes that change no control behaviour, MINOR for new
controls or audible changes, MAJOR for a break in the preset schema or
panel layout. Note that the preset schema has its own guard —
`Presets::SchemaHash` — and it, not the version number, is what actually
invalidates stored slots.

**The version is also compiled into the image**, since a filename is only
a convention and the module has no display to report the version on:

```sh
strings build/lapis_philosophorum_v0.x.0.bin | grep Lapis   # → LapisPhilosophorum 0.x.0
```

That works because of a two-part arrangement in `mastering.cpp`, and both
parts are load-bearing:

```c
__attribute__((used, section(".rodata.version")))
static const char kVersionBanner[] = "LapisPhilosophorum " LAPIS_VERSION_STR;

int main()
{
    asm volatile("" : : "r"(kVersionBanner));   // anchor against --gc-sections
```

`used` only stops the *compiler* discarding an unreferenced static. The
link runs with `-Wl,--gc-sections`, and the linker will happily drop the
whole section anyway — it did, on the first attempt here, silently
producing a binary with no version in it. The empty `asm volatile` takes
the address, costs no code and no cycles, and is the only thing keeping
the string alive. Don't delete it, and if you move the banner, move the
anchor with it. (`KEEP()` in the linker script would be the tidier fix,
but that script belongs to the SDK submodule.)

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

Header-only means no `CPP_SOURCES` edit. A stage that is *not* header-only
does need one — `src/mod_source.cpp` is currently the only such file, and
forgetting it produces a wall of `dangerous relocation: unsupported
relocation` at link time rather than an obvious "no such file".

### Add a CV modulation mode

1. Add the enum value to `mod_source::Mode` in `mod_source.h`, **before**
   `kCount`. `ChainModes::mod_secondary` is sized from `kCount`, so this
   changes the preset layout — bump `ChainModes::SchemaHash()`.
2. Return its B3 zone count from `SecondaryZones()`. Returning 1 makes the
   tap a no-op.
3. If it needs a clock or reset jack, add it to `ModSource::ClockJack()` /
   `ResetJack()`. `JackIsOutput()` is derived from those two, so routing,
   the `Frame`, and `mastering.cpp`'s `ApplyModRouting` all follow with no
   further edits.
4. Write `TickYourMode(Frame&)` and add it to the `switch` in `Tick()`.
5. Add a colour to `kModeSnaps` and `kModeColors` in
   `mastering_palette.h`, and re-space every `position` — they are zone
   centres, `(i + 0.5) / kCount`.
6. Decide what `knob_a` and `knob_b` mean and add the row to the table in
   `mod_source.h`. The convention across the existing modes is **`knob_a`
   is the primary axis — how much, how fast, how spread — and `knob_b` is
   the character.** Analysis is the one exception, and only because it
   generates nothing and so has no character to shape.
7. If the mode has a waveform, reach for `ShapeAt()` before writing your
   own oscillator. It is the shared six-slot ring (sine, triangle, ramp up,
   ramp down, pulse, stepped random) with crossfading and wrapping already
   handled, and the stepped slot works as long as you pass a per-jack held
   value and redraw it on each phase wrap.
8. Add a section to `mod_source_test.cpp`. Measure the property the mode
   promises, not the code you just wrote.

The knob itself needs no edit: K5 is `Selector(Mode::kCount)`.

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

The knee now has a second consequence: it sets the reference chord's
normalisation, so it decides both how much small-signal boost the machine
carries at full drive and how big the level step is when B3 switches to it.
A knee far outside the current 0.8–1.4 span widens both. `TestUnityGain`
measures them per character, so extending its loop in step 5 is what
catches an unreasonable one.

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

`CompGainReductionDb()` and `LimGainReductionDb()` already return positive
dB figures, and `ReadTelemetry()` adds the three band envelopes plus
broadband level. `VirtualKnob::Overdraw(fn, ctx)` paints a callback on top
of a knob's declarative ring after `PerfRenderer` runs — the `kick` example
in the SDK does exactly this for an envelope meter. The compressor page's
threshold or makeup ring is the natural host.

Note the band envelopes only run when the Analysis modulation mode is
selected — `Analyzer` self-gates on its enable flag, set from
`mastering_dsp::SetAnalysis()`. A meter that wants them unconditionally
must enable it unconditionally and accept the per-sample cost.

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
    into a click. That reason stands on its own and does not depend on how
    much of the chain budget happens to be free. (§8)
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
17. **Every scalar that multiplies the audio directly is eased on the
    audio side** — the compressor's makeup and mix, the saturator's drive
    and mix, and the output trim. A plain gain written straight from the
    control frame has nothing downstream to smooth it, so the step *is* the
    click. Trim was the exception until the chain harness measured it.
    (§4.3, §9.1)
18. **Never ease a quantity and its compensation independently.** Both
    endpoints are right and every point between them is wrong: the
    saturator's `comp_` is derived from `drive_` every sample, because
    easing it toward `comp(D)` alongside `drive_` easing toward `D` leaves
    the ease travelling off the curve that relates them. When the pair was
    a reciprocal, that put the midpoint of a 0→24 dB move at a *product*
    of 4.5 — +13 dB of gain on the one knob whose contract is that it does
    not change level. The reference chord has the same trap with the same
    shape, so it is derived the same way; what makes the transcendental
    affordable is a dirty check on `drive_`, not an ease. (§4.2, §9.1)
19. **`mod_source.*` includes nothing from libDaisy or the alchemy-sdk.**
    `<cstdint>` and `<cmath>` only, the same invariant `dsp_biquad.h`
    holds. It is what lets `mod_source_test` link the engine with no
    stubbing, and the harness is the only thing that will catch a
    generator regression — none of this is audible in the audio path.
    (§2, §9)
20. **Never drive an MCP4728 jack with per-jack `SetVolts` in a loop.**
    Each call rewrites all four channels from the shadow *and* pulses
    LDAC, so four jacks cost four complete I²C transactions — ~1.7 ms at
    400 kHz, which does not fit a 1 ms poll. Stage instead:
    `CvJack::StageVolts` on every jack, then one `hw.FlushCvOutputs()`,
    which is one `WriteAll` and one `PulseLdac` for all four. `StageVolts`
    is also correct on J7/J8 — those backends have no latch to defer, so
    it writes through — which is why the poll stages all six uniformly
    and never branches on backend. (§3)
21. **Unipolar Eurorack clocks and gates need `CvGate`, not `CvEdge`.**
    0 V reads ~0.5 here and +5 V reads ~0.75, so `CvEdge`'s symmetric
    0.30/0.70 defaults catch every rise and never a fall — the "+5 V
    triggers, 0 V holds forever" failure its own header warns about.
    `CvGate`'s 0.55/0.65 sit above the rest point and release cleanly.
22. **A selector knob whose zero position is the safe one needs a boot
    guard.** `Pager`'s default stored value is `0.5`, not `0` — so on any
    boot where `presets.BootLoad()` returns false the knob comes up
    mid-range. K5 forces itself to Off via `Pager::SetStored`, after
    settling the ADC so pot-catch re-arms against a real position. (§3.5)

---

## 8. Latency budget

**Chain latency today is 75 samples (1.56 ms)** — 60 for the limiter's
lookahead and 15 for the saturator's half-band pair.
`comp_control_test`'s `TestChainLatency` measures it end to end rather than
asserting it from prose — this section has gone stale once already, and now
something fails when it does. `chain_test` goes further and measures it in
**all sixteen bypass combinations**, which is what invariant 14 actually
claims and what one measurement of one configuration cannot establish.

For the chain as a whole, **80 samples (~1.67 ms) is the ceiling** any stage
may claim, so **5 samples remain**. There is no delay compensation in a
Eurorack rack, so the limit is perceptual rather than arithmetic; at the end
of a mastering chain nothing downstream recombines, so comb filtering is not
the binding constraint. The binding case is someone monitoring live through
the module, where the commonly cited transparency threshold is around 2 ms —
which is what 80 samples is chosen to stay under, with margin.

**This ceiling was 64 samples until the limiter went true-peak, and it moved
deliberately.** The 4x detector carries ~11 samples of group delay, so
holding the old number meant shrinking the gain-ramp window to match and
giving up most of the 1 ms of ramp time that keeps limiting from sounding
like clipping. Raising the ceiling by 16 samples was the cheaper side of
that trade: 63 → 75 samples is 0.25 ms, and neither number is close to
audible in this application. The old 64 was a self-imposed round number
with one sample spare, not a constraint anything downstream imposes — worth
saying plainly, because "the budget is spent" reads like a hard limit and it
never was one.

Anything that wants latency from here has to take it from an existing
stage, or move the ceiling again with an argument like the one above. The
realistic trade is the half-band filter: N = 31 costs 15 base samples.
Going to N = 35 for a steeper transition would cost 17; going *down* is
where slack would come from, at the price of alias rejection.

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
make test          # build and run all nine host harnesses
make test-golden   # regenerate the golden CSVs, deliberately
```

Host-only, no cross-toolchain: `tests/` compiles `dsp_biquad.h`,
`dsp_compressor.h`, `dsp_saturation.h`, `dsp_limiter.h` and
`dsp_analysis.h` directly, `mastering_dsp.cpp` against a six-line
`AudioHandle` stub, and `mod_source.cpp` against nothing at all.
`tests/test_report.h` holds the shared assert-and-print framework;
`eq_test_common.h`, `comp_test_common.h`, `sat_test_common.h`,
`lim_test_common.h` and `chain_test_common.h` hold the measurement support
for their respective scopes.

`mod_source_test.cpp` is the outlier and needs no common header: it drives
the modulation engine at its real 1 kHz control-tick rate and measures the
properties each mode actually promises — clock lock across all five ratios,
shape-bank continuity across the wrap, non-octave LFO ratios, the
divergence axis of the smooth random by pairwise correlation, its
smooth-to-stepped shape axis by largest single-tick move, Euclidean maximal
evenness, tempo-independent gate duty, and the analysis band split.

It has already earned its keep. The first Narrow ratio set spanned
1.0–0.44 and quietly contained `0.87 / 0.44 = 1.977` — a 1.1 % octave, in
the one mode whose entire purpose is that no two LFOs lock into a common
downbeat. Nothing about that is audible in the audio path, and no amount
of listening to the module would have found it. The set now spans only
1.75:1, so no pair *can* reach 2×, and the test sweeps every ordered pair
in all three sets against 2×/4×/8×.

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
  (including mid-ramp) and inverse-filter stability, the program-level
  match across the drive range with the origin-referenced compensation as
  its negative control, zero-drive transparency per character, harmonic
  decay rate, asymmetry and its
  DC, alias energy, head-bump height and return, the frequency-dependence
  that makes it tape, bypass/mix delay matching, adversarial inputs, and
  the goldens.
- `tests/sat_control_test.cpp` — the real `SetSat` cadence: the 62.5 Hz
  reconfigure regression (invariant 15), knob-sweep zipper, character
  switching, and preset recall.
- `tests/lim_response_test.cpp` — the limiter alone, reading the signal
  *before* the safety clamp: zero overshoot across six programs, transients
  shorter than the window, ceiling met rather than undershot, latency,
  bypass, release calibration, dBTP conformance, and the window sizing.
- `tests/chain_test.cpp` — the assembled chain, and the only harness that
  can see the seams. See §9.1.

### 9.1 The chain harness

The other seven each drive one stage. That is the right way to measure
what a stage *does*, and it is structurally blind to what happens between
stages, so `chain_test.cpp` asks only questions none of them can:

- Latency is 75 samples in **all sixteen** bypass combinations, measured by
  correlating a band-limited noise burst. Not by finding the peak of an
  impulse response — that is only the delay when the chain is a pure delay,
  and the half-band's outermost tap puts the *first* nonzero sample 15
  samples early.
- The ceiling holds with the whole chain in front of it, at settings where
  the limiter really is the binding constraint. (`Extreme()` is not such a
  setting and it is instructive why: at 24 dB of drive the shaper asymptote
  caps the saturator's output at ~0.045, so the chain leaves 13 dB of
  headroom and every ceiling assertion passes without the limiter engaging.)
- **An independent true-peak meter.** `TruePeakDb` reconstructs with the
  same 4x half-band the limiter detects with, so it can only ever confirm
  the limiter used its own detector consistently. `SincTruePeakDb`
  reconstructs at 8x with a 129-tap Blackman-windowed sinc in double,
  sharing nothing with the DSP. The gap between them is the number worth
  knowing: **0.14 dB**, which is how much real inter-sample peak a 4x
  detector structurally cannot see, so a −0.1 dB ceiling can touch
  +0.04 dBFS on HF-dense program. That is conformant — 4x is what
  BS.1770-4 specifies — and the check bounds the blind spot rather than
  pretending the ceiling holds through it.
- The chain's **applied gain is continuous**, derived as
  `out[n+75] / in[n]` with everything but the compressor bypassed so the
  chain is a pure gain, and probed with a square wave so `|in|` is constant
  (dividing by a sine near its zero crossings manufactures exactly the
  discontinuities this is looking for).
- **Parameter moves.** Both instruments are self-calibrating, because a
  hard-coded "a step above 1.3e-3 is a click" is really an assertion about
  the output level that happened to be current when it was written.
  Envelope excursions are judged against the loudest of nine rest points
  across the knob's own travel — nine rather than two because Asym is
  loudest in the *middle* and a two-point bar would convict it of an
  overshoot that is simply what the knob does. Steps are judged against the
  probe tone's own slew at the level the run actually reached.
- Every move runs as a 12-frame sweep **and** a 1-frame snap. The snap is
  not a synthetic worst case here: the pots are smoothed at tau = 8 ms and
  read at 1 kHz, so a value covers 88 % of a jump inside one control frame,
  and preset recall hands the DSP a genuine step. A stage whose easing only
  holds up under a slow sweep passes the first and fails the second.

Two traps this harness had to learn the hard way, both worth knowing
before adding to it:

- **Warm the true-peak meter.** `TruePeak4x` boots with a zeroed 16-sample
  history, so a meter started mid-passage sees a step and rings — 2.6 %,
  0.22 dB of Gibbs overshoot, which is larger than any ceiling violation
  worth hunting and looks exactly like a limiter that has drifted.
- **Reaching a branch is not the same as covering it.** The gain-continuity
  test originally used a 2000 ms release over an 8 s window, which never
  brought the envelope down to the 0.01 dB cutover it was aimed at, and so
  passed against a chain that had a 0.115 % step sitting just outside the
  window.

`Boot()` settles until the chain *measures* quiet rather than for a fixed
time. `mastering_dsp` keeps chain state at file scope and `Init()` does not
clear it, so there is no way to hand a test a fresh chain; a settle sized
for the slowest state in here (Glue's 4 s slow reservoir) would otherwise
dominate the runtime of every other scenario. Two residues never reach
zero on a host and are expected: the saturator's mix ease approaches zero
asymptotically, and with `asym` off centre the shaper emits a DC term the
blocker drives to ~1e-29. Both are literally zero on hardware, where
`FPSCR.FZ` and `FPU->FPDSCR` are set.

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
