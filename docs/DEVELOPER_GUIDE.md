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
                            │  SetEq() / SetComp() / SetOutput()
                            │  structs of plain floats + bools
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  mastering_dsp.cpp + dsp_*.h            — audio layer        │
│  chain state, per-sample Process(), stage implementations    │
└──────────────────────────────────────────────────────────────┘
```

**The seam is the three parameter structs** in `mastering_dsp.h`
(`EqParams`, `CompParams`, `OutParams`). They carry engineering units —
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
reads. **The EQ is the exception** and is worth reading before you copy
the pattern for anything else: it is double-buffered and smoothed, because
plain float writes were not good enough for it.

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
├── dsp_biquad.h           matched-magnitude biquad (Direct Form I)
├── dsp_compressor.h       log-domain feed-forward compressor
├── dsp_saturation.h       waveshaper + DC blocker
├── dsp_limiter.h          brickwall limiter
├── dsp_dither.h           xorshift32 TPDF generator
└── mastering_palette.h    LED colour constants
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

Three `Page` objects bind six knobs each. Note the pot indices repeat
across pages (every page uses pots 0–5); the `Pager` is what disambiguates
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
    .Use(eq_page).Use(comp_page).Use(out_page)
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

`UpdateParams()` calls all three setters with every current value, every
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

What isn't carried by any knob is the six bytes of mode/bypass state,
which get their own `Serializable`:

```cpp
struct ChainModes : public alchemy::Serializable
{
    uint8_t eq_bypass, comp_bypass, sat_bypass;
    uint8_t mid_q_index, soft_knee, sat_type;

    size_t   SerializedSize() const override { return 6; }
    void     Serialize(uint8_t* out) const override;
    bool     Deserialize(const uint8_t* in) override;   // clamps every field
    uint32_t SchemaHash() const override { return 0x4D535431u; }
};
```

Two things to note:

**`Deserialize` clamps, it doesn't validate.** `in[3] % 3u` for the Q
index, `& 1u` for the booleans. A corrupt or foreign slot can therefore
never push an out-of-range index into `kMidQTable[]` or into the DSP. This
is the cheap, correct discipline for flash-backed state — don't relax it.

**`SchemaHash()` is a manual constant** (`0x4D535431` = "MST1"). The
preset store XORs every managed component's hash and stamps slots with the
result; a mismatch on load makes the slot read as empty rather than
restoring misaligned bytes. **If you change `ChainModes`'s field layout or
size, bump this constant.** If you forget, old slots will deserialize into
the new layout and produce garbage — the one failure mode the mechanism
exists to prevent.

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
`Limiter` take `(float& l, float& r)` and detect on
`fmaxf(fabsf(l), fabsf(r))` — there is no way to configure them apart.
`Saturator` is per-channel by index (it needs its own DC-blocker state per
side) but shares one coefficient set. The EQ shares one coefficient set
across two `BiquadState` arrays.

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

**Compressor** (`dsp_compressor.h`) — feed-forward, computed entirely in
the log domain:

```
det    = max(|l|, |r|)
lvl_db = 20·log10(det)              // via kLog10Scale · logf(det)
over   = lvl_db − threshold_db
target = over · (1/ratio − 1)       // hard knee, above threshold
gr_db += coef · (target − gr_db)    // coef = attack if going down, else release
gain   = 10^(gr_db/20)
```

The soft knee is a quadratic interpolation across a fixed ±3 dB region
(`kKneeDb = 6`), matching the standard piecewise-quadratic formulation.

Two guards matter: signals below `kMinDetLin` (−90 dBFS) short-circuit to
zero gain reduction rather than taking `logf` of a denormal, and the
`expf` is skipped entirely when `gr_db > -0.01` (the overwhelmingly common
case of no compression). Even so, **this stage runs `logf` per sample** —
it is the single most expensive thing in the chain and the first place to
look if you find yourself over budget.

`gr_db` is public and exposed via `CompGainReductionDb()` (sign-flipped to
a positive dB figure) for a gain-reduction meter. Nothing currently
renders it — it's a ready hook if you want one on the compressor page's
rings.

**Saturator** (`dsp_saturation.h`) — the input is scaled by drive, offset
by `asym`, clamped to `±kSatClamp` (1.25) to bound the cubic's domain, then
shaped:

- cubic soft clip: `1.5·(x − x³/3)`, saturating to ±1 outside |x| ≥ 1
- hard clip: `clamp(x, −1, 1)`

`offset_shaped` (the shaper's response to the offset alone) is precomputed
in `Configure()` and subtracted from the output, which removes most of the
DC the asymmetry introduces; a one-pole ~5 Hz high-pass per channel
removes the remainder. Both are needed — the static subtraction handles
the steady-state offset, the filter handles what the shaper's
nonlinearity does to it under signal.

**Limiter** (`dsp_limiter.h`) — instantaneous attack, exponential release,
no lookahead:

```cpp
gd = (peak > ceiling) ? ceiling / peak : 1.f;
if (gd < gain) gain = gd;               // instant drop
else           gain += rel_coef * (gd - gain);   // smooth recovery
```

The post-gain `Clampf` to `±ceiling_lin` is a belt-and-braces guard, not
the primary mechanism — the gain computation alone satisfies the ceiling
for the current sample. Because there's no lookahead, transient limiting
is nonlinear distortion rather than gain riding; that's the accepted
trade for zero latency.

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

### Add a fourth page

1. `Pager pager(hw.buttons[0], 4, kNumPots);`
2. `pager.SetPageColor(3, kSomeColor);`
3. Declare knobs + a `Page(3)`, and `loop.Use(new_page)`.
4. Extend the `switch (page)` blocks in `UpdateParams` and
   `RenderButtons` — both currently use `default:` for page 2, so a new
   page will silently inherit the Output page's behaviour until you add an
   explicit `case`.

Changing the page count changes `Pager`'s serialized size, so **existing
presets are invalidated.**

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

---

## 8. Latency budget

The chain is zero-latency today, and the **EQ must stay that way**. Bypass
is implemented by discarding the wet output and keeping the *same-sample*
dry (`mastering_dsp.cpp`), so any EQ latency would compare signals from
different times — bypass becomes a click, and the module's total latency
changes with bypass state. Fixing that means a matched dry delay line and
bypass that is no longer true bypass, bought for nothing: the EQ is linear,
so it does not alias, and oversampling it would only buy decramping, which
the matched designs already deliver for free.

For the chain as a whole, **64 samples (~1.3 ms) is the ceiling** any future
stage may claim. There is no delay compensation in a Eurorack rack, so the
limit is perceptual rather than arithmetic; at the end of a mastering chain
nothing downstream recombines, so comb filtering is not the binding
constraint. The two stages that could sensibly spend that budget are the
**limiter** (which has no lookahead at all today, so it can only react after
a peak has passed — 1–2 ms is the standard transparent figure) and the
**saturator** (the one genuinely nonlinear stage, hence the only one where
oversampling actually suppresses aliasing).

---

## 9. Testing

```sh
make test          # build and run both host harnesses
make test-golden   # regenerate the golden coefficient CSV, deliberately
```

Host-only, no cross-toolchain: `tests/` compiles `dsp_biquad.h` directly and
`mastering_dsp.cpp` against a six-line `AudioHandle` stub.

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

If you change the coefficient math, the golden CSV will fail. That is the
point — regenerate it only when you meant to change the design.

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
