# alchemy-mastering

A stereo-linked mastering-chain firmware for the [Hermetic Modular Alchemy
Lab](https://hermeticmodular.com/modules/alchemy-lab) module, built on the
[Alchemy SDK](https://github.com/hermetic-modular/alchemy-sdk).

Signal path: In → 3-band EQ → Compressor → Saturation → Limiter → Output
Trim → Dither → Out, stereo-linked throughout (one set of controls; the
compressor and limiter detect `max(|L|,|R|)` so the two channels never
drift apart). Three pages of six knobs each:

- **Page 1 — EQ (amber).** Low shelf, mid peak, and high shelf, each with
  freq/gain; the mid band's Q cycles between three widths.
- **Page 2 — Compressor (blue).** Threshold, ratio, attack, release,
  makeup gain, and dry/wet mix, with a hard/soft knee toggle.
- **Page 3 — Output (red).** Saturation drive and asymmetry feed a
  brickwall limiter (ceiling, release), followed by output trim and TPDF
  dither depth (0–2 LSB at 24-bit, matching the codec's native word
  length).

Controls: **B1** tap cycles pages. **B2** tap toggles bypass for the
current page's stage (EQ / compressor / saturation). **B3** tap cycles the
current page's mode (EQ: mid-Q 0.707 / 1.5 / 4.0; compressor: hard/soft
knee; output: saturation cubic-soft / hard clip). **B2+B3** held for 2 s
enters the SDK's settings mode, same as the template. Presets and settings
are kept from the template; **param lock and CV routing have been
removed** — every knob is a direct, unlatched control.

The project packages the Alchemy SDK and libDaisy as git submodules and
builds with the standard Daisy `make` workflow.

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
git clone --recurse-submodules https://github.com/hermetic-modular/alchemy-template.git my-module
cd my-module

make libdaisy    # build libDaisy once after cloning
make             # build the firmware → build/mastering.bin
```

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

1. **Rename the firmware** — change `TARGET` at the top of the
   [`Makefile`](Makefile) (this names the `.bin`), and rename the `src/`
   files to taste, updating `CPP_SOURCES` to match.
2. **Bring your own DSP** — replace the `dsp_*.h` stages and
   `mastering_dsp.*`, and rewire the knobs and pages in `mastering.cpp`.
   Every framework feature is an explicit constructor call; delete what you
   don't want.
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
