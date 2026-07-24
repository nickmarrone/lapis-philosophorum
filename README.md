# alchemy-mastering

A stereo-linked mastering-chain firmware for the [Hermetic Modular Alchemy
Lab](https://hermeticmodular.com/modules/alchemy-lab) module, built on the
[Alchemy SDK](https://github.com/hermetic-modular/alchemy-sdk).

Signal path: In → 3-band EQ → Compressor → Saturation → Limiter → Output
Trim → Dither → Out, stereo-linked throughout — one set of controls and one
gain applied to both channels, so they never drift apart. The limiter
detects `max(|L|,|R|)`; the compressor sums the two channels' power, the
way a stereo-linked analogue compressor sums its detector currents. Three
pages of six knobs each:

- **Page 1 — EQ (amber).** Low shelf, mid peak, and high shelf, each with
  freq/gain; the mid band's Q cycles between three widths.
- **Page 2 — Compressor (blue).** A log-domain feed-forward glue
  compressor: threshold, ratio, attack, release, makeup gain, and dry/wet
  mix, with three characters on B3 — Precise, crest-Adaptive, and an
  SSL-style dual-time-constant Glue. Each character carries its own knee
  width (6 / 12 / 18 dB), sidechain high-pass corner (30 / 60 / 90 Hz), and
  auto-makeup.
- **Page 3 — Output (red).** Saturation drive and asymmetry feed a
  brickwall limiter (ceiling, release), followed by output trim and TPDF
  dither depth (0–2 LSB at 24-bit, matching the codec's native word
  length).

Controls: **B1** tap cycles pages. **B2** tap toggles bypass for the
current page's stage (EQ / compressor / saturation). **B3** tap cycles the
current page's mode (EQ: mid-Q 0.707 / 1.5 / 4.0; compressor: Precise /
Adaptive / Glue; output: saturation cubic-soft / hard clip). **B2+B3** held for 2 s
enters the SDK's settings mode, same as the template. Presets and settings
are kept from the template; **param lock and CV routing have been
removed** — every knob is a direct, unlatched control.

The project packages the Alchemy SDK and libDaisy as git submodules and
builds with the standard Daisy `make` workflow.

## Documentation

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
│   ├── mastering_dsp.*       chain orchestration + audio callback
│   ├── dsp_common.h          shared constants and helpers
│   ├── dsp_biquad.h          RBJ biquad (low shelf / peak / high shelf)
│   ├── dsp_compressor.h      log-domain bus compressor
│   ├── dsp_limiter.h         brickwall limiter (instant attack, exponential release)
│   ├── dsp_saturation.h      waveshaper + DC blocker
│   ├── dsp_dither.h          xorshift32 TPDF dither
│   └── mastering_palette.h   LED color palettes
└── lib/
    ├── alchemy-sdk/     Alchemy framework + board support   (submodule)
    └── libDaisy/        Electrosmith Daisy library           (submodule)
```

## Requirements

- `git`
- `make`
- `arm-none-eabi-gcc`
- `dfu-util`

Ubuntu / Debian:

```sh
sudo apt install git make gcc-arm-none-eabi dfu-util
```

macOS (Homebrew):

```sh
brew install git make dfu-util
brew install --cask gcc-arm-embedded
```

## Getting started

```sh
git clone --recurse-submodules <this-repo> alchemy-mastering
cd alchemy-mastering

make libdaisy    # build libDaisy once after cloning
make             # build the firmware → build/mastering.bin
```

`BOARD=v2` is the default; pass `make BOARD=v1` for an original dev board.
Switching boards wipes the object tree automatically.

## Flashing

The Alchemy Lab runs a custom bootloader (`DaisyBootloader-AlchemyLabV2`)
that serves DFU over the front-panel USB-C port.  Connect that port, then put
the module in update mode: during the ~2 s window after power-on — the LED
rings spin a warm-white comet — press or hold **B3.**  The rings switch to a slow breathe, and the module stays in DFU mode until it's flashed or reset.  Then:

```sh
make program-dfu
```

You can also use the [Hermetic Modular Web Programmer](https://hermeticmodular.com/program) straight from the browser.

## Make it yours

The [Developer Guide](docs/DEVELOPER_GUIDE.md) covers the architecture and
has step-by-step recipes for adding a knob, adding a DSP stage, adding a
page, and extending the preset payload. The short version:

1. **The control/DSP seam** is the three parameter structs in
   [`src/mastering_dsp.h`](src/mastering_dsp.h). The DSP layer knows
   nothing about the SDK; the control layer pushes engineering units
   (Hz, dB, ms) into it once per frame.
2. **Rename the firmware** — change `TARGET` at the top of the
   [`Makefile`](Makefile) (this names the `.bin`), and rename the `src/`
   files to taste, updating `CPP_SOURCES` to match.
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
