#!/usr/bin/env python3
"""
halfband_design.py — generates the tap table in src/dsp_halfband.h.

Not part of the build. The taps are checked in; this exists so the design is
reproducible and so the band edges are a decision on the record rather than a
magic array. Run it and diff against the header if you want to change them.

    python3 -m venv venv && ./venv/bin/pip install numpy scipy
    ./venv/bin/python tools/halfband_design.py

Design notes
------------
N = 31 is forced by the latency budget, not by the stopband. DEVELOPER_GUIDE §8
allows the chain 64 samples and the limiter already spends 48, so the saturator
has 16. An up/down pair of length-N linear-phase FIRs costs (N-1)/2 samples at
the base rate, so N = 31 costs exactly 15 and N = 35 would cost 17 — over.
(N must also be 3 mod 4 for the half-band structure, so 33 is not a choice.)

The band edges are the free variable. Passband edge 17 kHz buys 79.5 dB with
-0.46 dB at 20 kHz per converter; pushing to 18 kHz would drop the stopband to
69 dB, pulling back to 16 kHz would cost 0.58 dB at 20 kHz. 17 kHz is the knee
of that trade.

Images of content below the passband edge land above the stopband edge and get
the full 79.5 dB. Content in the top 17-20 kHz has its image in the transition
band instead (a 19 kHz tone images to 29 kHz, rejected ~35 dB), which is
accepted: there is little program energy there, and the intermodulation it can
produce sits far below the harmonics the saturator is deliberately generating.
"""

import numpy as np
from scipy.signal import remez, freqz

FS2 = 96000.0   # the 2x working rate
N   = 31        # taps; fixed by the latency budget (see above)
FP  = 17000.0   # passband edge; stopband edge is FS2/2 - FP by half-band symmetry


def design():
    h = remez(N, [0, FP / FS2, 0.5 - FP / FS2, 0.5], [1, 0], weight=[1, 1], fs=1.0)
    # Force the exact half-band structure. Parks-McClellan lands within ~1e-5 of
    # it for these bands; forcing makes it exact, which is what lets one phase of
    # each converter be a pure delay instead of an almost-pure one.
    m = (N - 1) // 2
    for j in range(N):
        if j != m and (j - m) % 2 == 0:
            h[j] = 0.0
    h[m] = 0.5
    return h


def report(h):
    w, H = freqz(h, worN=400000, fs=FS2)
    att = -20 * np.log10(np.abs(H[w >= FS2 / 2 - FP]).max())
    pb = np.abs(H[w <= FP])
    print(f"N={N}  passband edge {FP/1000:.0f} kHz  stopband edge {(FS2/2-FP)/1000:.0f} kHz")
    print(f"  stopband       {att:.1f} dB")
    print(f"  passband ripple {20*np.log10(pb.max()/pb.min())*1000:.2f} m dB")
    for f in (16000, 18000, 19000, 20000):
        g = 20 * np.log10(np.abs(H[np.argmin(np.abs(w - f))]) * 2)
        print(f"  {f/1000:5.1f} kHz     {g:+.3f} dB (one converter)")
    print(f"  pair latency   {(N-1)//2} samples at the base rate")


def emit(h):
    m = (N - 1) // 2
    pairs = [(m - j, h[j]) for j in range(m) if h[j] != 0.0][::-1]
    print(f"\nconstexpr int   kHbPairs      = {len(pairs)};")
    print(f"constexpr float kHbTap[kHbPairs] = {{")
    for off, v in pairs:
        print(f"    {v: .12e}f,   // offset +/- {off:2d}")
    print("};")


if __name__ == "__main__":
    taps = design()
    report(taps)
    emit(taps)
