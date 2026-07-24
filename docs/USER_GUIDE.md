# Mastering — User Guide

An end-of-chain stereo mastering processor for the Hermetic Modular
Alchemy Lab. Patch your mix in, get a louder, glued, ceiling-safe mix out.

Everything is **stereo-linked**: there is one set of controls, and one gain
is applied to both channels, so the image never shifts because one side got
squashed harder than the other. The two dynamics stages listen slightly
differently. The limiter takes the louder of the two channels, which is
what a ceiling has to do. The compressor adds the two channels' *power*,
the way a stereo-linked analogue compressor sums its detector currents — so
a hard-panned hit reads 3 dB quieter than a centred one of the same level,
and no longer ducks the whole mix on its own.

---

## Signal flow

```
IN ──▶ 3-band EQ ──▶ Compressor ──▶ Saturation ──▶ Limiter ──▶ Trim ──▶ Dither ──▶ OUT
        (bypass)      (bypass)       (bypass)      always     always    always
```

- **EQ, Compressor, and Saturation** can each be bypassed.
- **Limiter, Trim, and Dither** are always in circuit. Set the limiter
  ceiling to its maximum (−0.1 dB) and dither to 0 if you want them
  effectively out of the way.

Audio I/O is the module's stereo audio in and out, running at 48 kHz
through the Daisy codec's 24-bit converters. The dither stage is scaled in
24-bit LSBs to match.

---

## Controls at a glance

| Control | Action | What it does |
|---|---|---|
| **B1** | tap | Cycle pages: EQ → Compressor → Output → EQ |
| **B2** | tap | Bypass the current page's stage |
| **B3** | tap | Cycle the current page's mode |
| **B2 + B3** | hold 2 s | Enter settings mode |
| **B2** or **B3** | tap (in settings) | Exit settings mode |
| **B1** | tap (in settings) | Cycle settings pages |

A "tap" is a press shorter than 400 ms. Holding a button longer, or
pressing two at once, does nothing on that page — that's deliberate, so a
fumbled settings gesture never flips a bypass by accident.

The six knobs are **K1–K6**, arranged as three rows of two: (K1, K2),
(K3, K4), (K5, K6).

### Pot catch

When you change pages, the knobs do **not** jump to their new positions.
Each knob stays at its stored value until you physically sweep through
that value, at which point it "catches" and starts tracking again. This is
what lets three pages share six physical knobs without a page change
destroying your settings.

The same applies after loading a preset — every knob re-arms, so nothing
moves until you deliberately move it.

---

## Page 1 — EQ (amber)

A classic three-band mastering EQ: low shelf, peaking mid, high shelf,
stereo-linked.

The filters are magnitude-matched rather than the usual bilinear-transform
designs, which matters most at the top of the high-shelf sweep. A digital
shelf built the conventional way cannot actually reach its target gain near
20 kHz — the curve gets squashed into Nyquist, which is where a lot of the
"digital top end" reputation comes from. These track the intended analog
curve to within 0.3 dB across the whole range, at no cost in latency.

| Knob | Parameter | Range | Curve |
|---|---|---|---|
| K1 | Low shelf frequency | 20 Hz – 800 Hz | exponential |
| K2 | Low shelf gain | −15 dB – +15 dB | linear, centre = 0 |
| K3 | Mid frequency | 200 Hz – 5 kHz | exponential |
| K4 | Mid gain | −15 dB – +15 dB | linear, centre = 0 |
| K5 | High shelf frequency | 1 kHz – 20 kHz | exponential |
| K6 | High shelf gain | −15 dB – +15 dB | linear, centre = 0 |

Frequencies (K1/K3/K5) draw as filled level arcs. Gains (K2/K4/K6) draw as
bipolar arcs: amber to the right for boost, blue to the left for cut, dim
white at the centre detent.

- **B2 tap** — bypass the whole EQ section. The button dims to a faint
  amber. The filters keep running underneath while bypassed, so
  re-enabling never pops.
- **B3 tap** — cycle the **mid band's Q**:

| Q | Character | B3 colour |
|---|---|---|
| 0.707 | wide, gentle tone-shaping | green |
| 1.5 | medium | yellow |
| 4.0 | narrow, surgical | magenta |

The shelves have a fixed Q of 0.707 — only the mid band is switchable.

---

## Page 2 — Compressor (blue)

A feed-forward, log-domain bus compressor with a parallel-mix control.

| Knob | Parameter | Range | Curve |
|---|---|---|---|
| K1 | Threshold | −40 dB – 0 dB | linear |
| K2 | Ratio | 1:1 – 20:1 | compression-amount |
| K3 | Attack | 0.1 ms – 100 ms | exponential |
| K4 | Release | 10 ms – 2000 ms | exponential |
| K5 | Makeup gain | 0 dB – +20 dB | linear, on top of auto-makeup |
| K6 | Mix (dry/wet) | 0 – 1 | linear |

All six draw as blue level arcs.

**The Ratio knob is not linear in ratio.** It is linear in how much
compression you are asking for, which puts the settings a mastering
compressor actually lives at where your fingers are: 1.5:1 at a third of
the way up, 2:1 at halfway, 3:1 at two thirds. 10:1 and 20:1 are crammed
into the last tenth, which is the right place for them here.

- **B2 tap** — bypass the compressor. The detector keeps tracking while
  bypassed, so gain reduction is already settled when you switch back in.
- **B3 tap** — cycle the **character**:

| Character | What it does | Knee | Sidechain filter | B3 colour |
|---|---|---|---|---|
| Precise | Times exactly as set. Nothing adapts. | 6 dB | 30 Hz | white |
| Adaptive | Attack and release follow the music | 12 dB | 60 Hz | teal |
| Glue | Two-stage release: quick recovery, long tail | 18 dB | 90 Hz | crimson |

**Adaptive** watches how peaky the material is. Dense, sustained passages
get a faster attack and a slower release, so it behaves as a level rider;
transient-heavy passages get a slower attack, letting the hits through, and
a faster release, so it is out of the way before the next one. It also
speeds its release up as a loud passage dies away.

**Glue** is the classic bus-compressor behaviour. Its release is two
recoveries at once, one quick and one that trails off over seconds, which
is what stops a slow release from choking the mix and a fast one from
pumping it. This is the one to reach for first.

**Wide knees move the effective threshold down.** A knee is centred on the
threshold, so compression starts *half a knee below* the number on the
knob. On Glue, a threshold of −20 begins working at −29 and does not reach
its full ratio until −11. That is why it sounds gentler at the same
setting, and it is the most common surprise on this page. Precise is the
character where the threshold means what it says.

**The sidechain filter** is why bass no longer runs the show. The
compressor does not listen below the corner frequency, so a kick drum stops
pumping the whole mix down with it. The kick is still compressed — it just
no longer *decides* the compression. Note this also means a big low-shelf
boost on page 1 changes the sound without changing how hard the compressor
works.

**About Mix.** The mix control gives you parallel ("New York")
compression: at 0 you hear only the dry signal, at 1 only the compressed
signal.

Adaptive and Glue apply **auto-makeup** — they work out how much level the
compression is costing at a nominal bus level and put it back
automatically, so bypassing (B2) is a fair A/B and the Mix knob crossfades
between two things at roughly the same loudness. The Makeup knob is extra
on top of that. Precise leaves makeup entirely to you.

Makeup applies to the **wet** path only, so with mix below 1 pushing makeup
also makes the compressed layer louder relative to the dry one.

**Starting point for glue:** Glue character, threshold around −22 dB, ratio
2:1, attack 10–30 ms, release ~300 ms, mix at 1.0 — aim for 2–4 dB of gain
reduction on the loud parts. Then back the mix off toward 0.7 if it feels
squashed.

**Starting point for transparent levelling:** Adaptive, threshold −18 dB,
ratio 1.5:1, attack 10 ms, release ~200 ms, mix 1.0.

---

## Page 3 — Output (red)

Saturation, brickwall limiting, output level, and dither.

| Knob | Parameter | Range | Curve |
|---|---|---|---|
| K1 | Saturation drive | 0 dB – +24 dB | linear |
| K2 | Asymmetry | −0.3 – +0.3 | linear, centre = 0 |
| K3 | Limiter ceiling | −6 dB – −0.1 dB | linear |
| K4 | Limiter release | 10 ms – 500 ms | exponential |
| K5 | Output trim | −12 dB – +12 dB | linear, centre = 0 |
| K6 | Dither | 0 – 2 LSB (24-bit) | linear |

K1/K3/K4/K6 draw as red level arcs. K2 and K5 are bipolar — red to the
right, cyan to the left, dim white at centre.

- **B2 tap** — bypass the **saturation** stage only. The limiter, trim,
  and dither keep running.
- **B3 tap** — cycle the **saturation shape**:

| Shape | Character | B3 colour |
|---|---|---|
| Cubic soft clip | rounded, progressive, tube-ish | orange |
| Hard clip | flat-topped, aggressive | deep red |

**Asymmetry** offsets the signal before the waveshaper, so the positive
and negative halves clip differently. That generates even-order harmonics
(a warmer, fuller colouration) instead of the odd-order-only harmonics of
a symmetric shaper. A DC blocker after the shaper removes the offset
itself, so no DC reaches the output. At centre the shaper is perfectly
symmetric.

**The limiter** looks 1 ms ahead. It sees a peak coming before you hear it
and has the gain down by the time it arrives, so loud transients are ridden
rather than clipped. It then recovers at the release time you set. Fast
release settings are transparent on dense material but will pump audibly on
sparse, transient-heavy material — slow the release down if you hear it
breathing.

That 1 ms is the module's entire latency, and it is there whether or not
the limiter is doing anything.

**⚠️ Trim sits *after* the limiter.** Positive trim can push the signal
back above the ceiling you just set, and past 0 dBFS it will clip the
codec. If you want the ceiling honoured at the output jack, keep trim at
or below centre, and use makeup gain on the compressor page for loudness
instead.

**Dither** adds TPDF (triangular) noise at the very end, scaled in 24-bit
LSBs. It decorrelates the quantisation error from the signal, trading a
tiny amount of noise for the removal of quantisation distortion on quiet
fades. 0 is off; 1 LSB is the standard choice; 2 is available if you want
it. It is only worth using if this module is genuinely the last thing
before your converter or recorder.

---

## What the LEDs tell you

| LED | Meaning |
|---|---|
| Knob rings | Current value of that knob on the active page |
| **B1** | Active page: amber = EQ, blue = Compressor, red = Output |
| **B2** | Page colour at full brightness = stage active; heavily dimmed = bypassed |
| **B3** | Current mode for the active page (see the colour tables above) |

Ring brightness is set globally in settings — the LEDs are capable of
getting very bright and quite hot, so the default is deliberately
conservative.

---

## Presets

The module has **16 preset slots** in flash, with wear levelling.

A preset stores everything: all eighteen knob values across all three
pages, all three bypass states, the mid-Q index, the compressor character, the
saturation shape, and your settings (brightness).

### Saving and loading

1. Hold **B2 + B3** for 2 seconds to enter settings mode.
2. **K3** selects the slot (0–15). The ring shows the slot as a colour
   group plus a count of lit segments.
3. **K4** is the action pot. Turn it fully counter-clockwise and **hold
   for 3 seconds to save**; fully clockwise and hold 3 seconds to
   **load**. An arc fills as you hold, then flashes white on success.
4. Tap **B2** or **B3** to exit.

If K4 is already parked at an extreme when you enter settings, nothing
happens — you have to bring it back through the centre first to re-arm it.
This prevents accidental overwrites.

### Boot behaviour

On power-up the module automatically loads **slot 0**. Save your
go-to chain there and it comes up ready every time.

### When presets disappear

Preset slots are stamped with a schema hash. If the firmware is updated in
a way that changes what a preset contains, old slots are treated as empty
rather than being restored incorrectly. Losing your presets after a
firmware update is the safety mechanism working, not a fault.

**This release invalidates existing slots**, for two independent reasons:
the compressor's two-state knee toggle became a three-state character, and
the Ratio knob was re-tapered, so a stored ratio position would restore a
different ratio than the one you saved. You will need to re-save your
chains.

---

## Settings mode

Hold **B2 + B3** for 2 seconds. **B1** cycles settings pages; **B2** or
**B3** exits.

| Control | Function |
|---|---|
| K1 | LED brightness (5 % – 100 %, default 30 %) |
| K3 | Preset slot selector |
| K4 | Preset save / load action |

Settings are stored in presets alongside everything else.

On exit, every performance knob re-arms for pot catch, so nothing jumps
when you come back out.

---

## Practical notes

**This module is not CV-controllable.** Every knob is a direct, unlatched
control — there is no CV routing and no parameter locking in this
firmware. The CV jacks do nothing here. That's a deliberate trade: a
mastering chain wants stable, repeatable settings, not modulation.

**Gain staging.** The compressor detects post-EQ, so a big low-shelf boost
will drive the compressor harder. Set your EQ first, then set the
threshold.

**Order of operations that works:** EQ → compressor → then output stage
last. Set the limiter ceiling before you set drive, so you can hear what
the saturation is actually contributing rather than what the limiter is
taking away.

**Bypass is for comparison, not for saving CPU.** Every stage keeps
running while bypassed — its output is simply discarded. This means A/B
switching is click-free and instantaneous, and filter/detector state is
always warm.

---

## Firmware updates

1. Connect the front-panel USB-C port.
2. Power on the module. During the ~2 second boot window — the LED rings
   spin a warm-white comet — press or hold **B3**. The rings switch to a
   slow breathe; the module now stays in DFU mode until flashed or reset.
3. Flash with the
   [Hermetic Modular Web Programmer](https://hermeticmodular.com/program),
   or from a checkout of this repo with `make program-dfu`.

**CV calibration** (unrelated to this firmware, but stored per-board):
unpatch all CV jacks and hold **B1 + B2** while the board resets. The
panel narrates a ~15 second procedure and reboots. The calibration record
survives reflashes.

---

## Specifications

| | |
|---|---|
| Sample rate | 48 kHz |
| Audio block size | 24 samples |
| Converter word length | 24-bit |
| Channels | Stereo, fully linked |
| EQ | 3 bands, ±15 dB, magnitude-matched biquads, zero latency |
| Compressor | Feed-forward, log-domain, 1:1–20:1, three characters, sidechain HPF, parallel mix |
| Saturation | Cubic soft clip or hard clip, ±0.3 asymmetry, DC-blocked |
| Limiter | Brickwall, 1 ms lookahead, exponential release |
| Dither | TPDF, 0–2 LSB @ 24-bit |
| Latency | 48 samples (1.0 ms), all of it the limiter |
| Presets | 16 slots, flash, wear-levelled |
| Control rate | ~60 Hz frames, 1 ms button polling |

---

See [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md) if you want to modify or
build on this firmware.
