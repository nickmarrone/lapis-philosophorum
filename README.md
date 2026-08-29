# Lapis Philosophorum

A stereo-linked mastering-chain firmware for the [Hermetic Modular Alchemy
Lab](https://hermeticmodular.com/modules/alchemy-lab) module, built on the
[Alchemy SDK](https://github.com/hermetic-modular/alchemy-sdk).

Signal path: In → 3-band EQ → Compressor → Tape Saturation → Output Trim →
Limiter → Dither → Out, stereo-linked throughout — one set of controls
and one gain applied to both channels, so they never drift apart. The
limiter's true-peak detector reads `max(|L|,|R|)` 4x oversampled; the compressor sums the two channels'
power, the way a stereo-linked analogue compressor sums its detector
currents. Four pages sharing six knobs:

- **Page 1 — EQ (amber).** Low shelf, mid peak, and high shelf, each with
  freq/gain; the mid band's Q cycles between three widths.
- **Page 2 — Compressor (blue).** A log-domain feed-forward glue
  compressor: threshold, ratio, attack, release, makeup gain, and dry/wet
  mix, with three characters on B3 — Precise, crest-Adaptive, and an
  SSL-style dual-time-constant Glue. Each character carries its own knee
  width (6 / 12 / 18 dB), sidechain high-pass corner (30 / 60 / 90 Hz), and
  auto-makeup.
- **Page 3 — Tape (gold).** A record/playback emphasis pair around a tanh
  knee, which is what makes saturation frequency-dependent rather than
  memoryless: drive, dry/wet mix, emphasis, asymmetry, and head bump, with
  three tape machines on B3 — 30 ips, 15 ips, and Saturated. Each machine
  carries its own emphasis shelf, knee and head-bump resonance. Runs 2×
  oversampled with first-order ADAA, and holds program level across the
  whole drive range — drive trades peaks for harmonics, not for level.
- **Page 4 — Output (red).** Brickwall limiter (ceiling, release) on K1/K2
  and output trim on K6. K3, K4 and K5 drive the CV modulation source
  below. TPDF dither is fixed at 2 LSB and has no control — see
  `mastering_dsp::kDitherLsb` for why one was never doing audible work.

## CV modulation source

A stereo-linked chain has one parameter set and nothing worth
CV-modulating, so the six CV jacks would otherwise sit idle. **K5** on the
Output page turns them into a six-channel modulation source instead;
**K3** and **K4** are the active mode's two continuous controls, and a
**B3** tap cycles its discrete secondary. K5's ring morphs between the mode
colours as you sweep, and K3, K4 and B3 all wear the active mode's colour.

K3 is the mode's primary axis — how much, how fast, how spread — and K4 is
its character. Analysis is the one exception: it generates nothing, so it
has no character to shape and spends K4 on sensitivity instead.

| K5 | Mode | Jacks | K3 | K4 | B3 |
|---|---|---|---|---|---|
| 0 | **Off** | all released | — | — | — |
| 1 | **Analysis** | 6 out | Response, fast peak → slow RMS | Sensitivity, −10 → −70 dBFS | Polarity: normal / inverted |
| 2 | **Clocked** | J3 clock in, J4 reset in, 4 out | Shape rotation | Shape spread, unison → fanned | Clock ratio ÷4 ÷2 ×1 ×2 ×4 |
| 3 | **Multi LFO** | 6 out | Base rate, 0.01–10 Hz | Shape | Ratio set: Golden / Prime / Narrow |
| 4 | **Smooth Random** | 6 out | Divergence, one walk → six | Smooth → stepped | Rate: glacial / slow / medium / quick |
| 5 | **Euclid** | J3 clock in, 5 gate out | Density | Gate length, trigger → legato | Step set: tight / compact / classic / odd / long |

**The shape bank** is shared by Clocked and Multi LFO: six waveforms in a
ring — sine, triangle, ramp up, ramp down, pulse, stepped random —
crossfading between neighbours and wrapping, so the knob has no seam and no
dead end. Clocked spreads its four outputs across the ring at a settable
gap; at zero spread they collapse onto one shape and the four jacks carry
identical voltages, which is the mono-bus case. Multi LFO puts all six at
one position, since its outputs are already differentiated by rate.

**Analysis** is the one that belongs to this module rather than to any
utility: the mastering chain becomes its own modulation source. J3–J5 carry
low/mid/high band envelopes, J6 the broadband level, and J7/J8 the
compressor's and limiter's gain reduction — so a patch can follow what the
chain is actually doing to the program material.

**Clocked** puts four shape-bank positions on J5–J8, locked to the incoming
clock and reset together. With nothing patched to the clock jack it
free-runs at 1 Hz × the ratio rather than sitting still. **Multi LFO** runs
six oscillators at deliberately non-octave ratios, so they never lock into a
common downbeat. **Smooth Random**'s divergence knob sweeps from all six
jacks carrying one shared walk at different depths to six independent
wanderers, and its shape knob stiffens the interpolation from liquid drift
through eased staircase to hard sample-and-hold. **Euclid** puts five
co-prime Euclidean gate patterns on J4–J8; the step set on B3 chooses the
grid, from a tight 8/7/6/5/4 that relines every 840 clocks to a long-form
17/15/13/11/9 that takes 109395.

Bipolar modes swing ±4 V rather than ±5 V: J3–J6 are MCP4728-backed and
bottom out near −4.4 V, so a wider swing would clip on four of the six
jacks. Envelopes and gates use the 0 to +5 V convention. J7/J8 update every
1 ms; J3–J6 are batched over I²C at 250 Hz.

Controls: **B1** tap cycles pages. **B2** tap toggles bypass for the
current page's stage (EQ / compressor / tape / limiter). **B3** tap cycles
the current page's mode (EQ: mid-Q 0.707 / 1.5 / 4.0; compressor: Precise
/ Adaptive / Glue; tape: 30 ips / 15 ips / Saturated; output: the active
modulation mode's secondary). **B2+B3** held for 2 s enters the SDK's
settings mode, same as the template. Presets and settings are kept from the
template; **param lock and CV routing have been removed** — every knob is a
direct, unlatched control.

Total latency is 75 samples (1.56 ms) — 60 for the limiter's lookahead, 15
for the tape stage's half-band filters — and is constant regardless of
bypass state.

The project packages the Alchemy SDK and libDaisy as git submodules and
builds with the standard Daisy `make` workflow.

## Documentation

The module also carries its own manual. Every knob, page, and jack has help
text compiled into the firmware, alongside long-form sections on the signal
path, the panel controls, the modulation source, and gain staging — so a
module plugged into the [web
programmer](https://hermeticmodular.com/program) explains itself, with no
manual to go and find. The prose lives next to the declarations it describes
([`src/mastering.cpp`](src/mastering.cpp)) and in
[`src/manual.cpp`](src/manual.cpp) for the parts that belong to no single
control. None of it reaches a preset's schema hash, so editing it can never
invalidate a saved preset.

- **[User Guide](docs/USER_GUIDE.md)** — how to play the module: the
  signal path, every knob's range and curve, what the LEDs mean, presets,
  settings mode, and gain-staging notes.
- **[Developer Guide](docs/DEVELOPER_GUIDE.md)** — how the firmware is
  built: the control/DSP split, each stage's implementation, the preset
  schema, the build system, recipes for extending it, and the invariants
  that will bite quietly if broken.

## What's inside

```
├── Makefile              standard Daisy Makefile (libDaisy core underneath)
├── src/                  the firmware — this is the part you edit
│   ├── mastering.cpp         hardware wiring, pages, knobs, buttons, LEDs, presets
│   ├── manual.*              the in-firmware manual's long-form sections
│   ├── mastering_dsp.*       chain orchestration + audio callback
│   ├── dsp_common.h          shared constants and helpers
│   ├── dsp_biquad.h          matched-magnitude biquads + coefficient inversion
│   ├── dsp_compressor.h      log-domain bus compressor
│   ├── dsp_limiter.h         brickwall limiter (instant attack, exponential release)
│   ├── dsp_saturation.h      tape saturation: emphasis pair, ADAA tanh, head bump
│   ├── dsp_halfband.h        2x polyphase half-band up/downsamplers + matched dry delay
│   ├── dsp_dither.h          xorshift32 TPDF dither
│   ├── mastering_palette.h   LED color palettes
│   └── version.h             firmware version — single source of truth
├── tools/
│   └── halfband_design.py    regenerates the half-band tap table
└── lib/
    ├── alchemy-sdk/     Alchemy framework + board support   (submodule)
    └── libDaisy/        Electrosmith Daisy library           (submodule)
```

## Requirements

- `git`
- `make`
- `arm-none-eabi-gcc`
- `dfu-util`
- `node` — only for `make program-live`

Ubuntu / Debian:

```sh
sudo apt install git make gcc-arm-none-eabi dfu-util nodejs
```

macOS (Homebrew):

```sh
brew install git make dfu-util node
brew install --cask gcc-arm-embedded
```

## Getting started

```sh
git clone --recurse-submodules <this-repo> lapis-philosophorum
cd lapis-philosophorum

make libdaisy    # build libDaisy once after cloning
make             # build the firmware → build/lapis_philosophorum_v0.5.0.bin
```

The version comes from [`src/version.h`](src/version.h) and lands in two
places: the artifact name, and the image itself. So even a `.bin` that has
been renamed can be identified:

```sh
strings build/lapis_philosophorum_v0.5.0.bin | grep Lapis   # → LapisPhilosophorum 0.5.0
```

`BOARD=v2` is the default; pass `make BOARD=v1` for an original dev board.
Switching boards wipes the object tree automatically.

## Flashing

Connect the front-panel USB-C port and run:

```sh
make program-live
```

The firmware serves HostLink on that port, so this asks the running module to
reboot itself into its bootloader and then flashes it — no power cycle, no
button press, rack untouched.  It needs `node` on your `PATH` for the SDK's
`hostlink-cli`.

If the module isn't running HostLink firmware — a fresh board, or one you've
already bricked mid-experiment — put it into update mode by hand instead:
during the ~2 s window after power-on, while the LED rings spin a warm-white
comet, press or hold **B3.**  The rings switch to a slow breathe and the module
stays in DFU mode until it's flashed or reset.  Then:

```sh
make program-dfu
```

You can also use the [Hermetic Modular Web Programmer](https://hermeticmodular.com/program) straight from the browser.

## Make it yours

The [Developer Guide](docs/DEVELOPER_GUIDE.md) covers the architecture and
has step-by-step recipes for adding a knob, adding a DSP stage, adding a
page, and extending the preset payload. The short version:

1. **The control/DSP seam** is the four parameter structs in
   [`src/mastering_dsp.h`](src/mastering_dsp.h). The DSP layer knows
   nothing about the SDK; the control layer pushes engineering units
   (Hz, dB, ms) into it once per frame.
2. **Rename the firmware** — change `TARGET` at the top of the
   [`Makefile`](Makefile) (this names the `.bin`), and rename the `src/`
   files to taste, updating `CPP_SOURCES` to match.
   [`src/version.h`](src/version.h) carries the version that names the
   `.bin` and is stamped into the image; bump it there and nowhere else.
3. **Add source files** — append them to `CPP_SOURCES` in the Makefile.
   One caveat from the underlying Daisy build: object files are flattened
   into `build/` by basename, so two sources can't share a filename even in
   different directories.
4. **Learn the SDK** — the framework headers live in
   `lib/alchemy-sdk/framework/include/alchemy/`, and the SDK's
   [`examples/`](https://github.com/hermetic-modular/alchemy-sdk/tree/main/examples)
   show other usage styles (the `kick` example is a minimal-opt-in
   contrast to this project).

### Updating the vendored libraries

```sh
git -C lib/alchemy-sdk pull origin main
git add lib/alchemy-sdk && git commit -m "Bump alchemy-sdk"
```

The pinned libDaisy commit matches the one the Alchemy SDK itself vendors
and tests against; if you bump one, consider bumping the other to match.


## License

MIT — see [LICENSE](LICENSE).  libDaisy is independently MIT-licensed by
Electrosmith.
