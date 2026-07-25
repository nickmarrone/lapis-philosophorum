/**
 * sat_response_test.cpp — measurement harness for the tape saturator.
 *
 * Drives mastering_dsp::Saturator directly, with no chain around it, so every
 * number here is attributable to this one stage.
 *
 *   ./sat_response_test            measure and assert
 *   ./sat_response_test --golden   regenerate golden/sat_*.csv
 *
 * Several of these carry explicit NEGATIVE CONTROLS — a deliberately wrong
 * model that the same assertion must reject. A harmonic-decay bound loose
 * enough to pass tanh will also pass the cubic clipper it replaced, and nothing
 * in the output would tell you which; so the cubic is measured alongside and
 * required to fail. Same for the de-emphasis: an independently designed cut
 * looks fine until you ask whether it actually inverts.
 */

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sat_test_common.h"
#include "eq_test_common.h"
#include "dsp_biquad.h"
#include "dsp_halfband.h"

using namespace sattest;
using mastering_dsp::SatParams;

namespace {

bool g_golden = false;

const char* CharName(uint8_t c)
{
    return c == 0 ? "30ips" : (c == 1 ? "15ips" : "sat");
}

/* ── 1. Half-band converters ─────────────────────────────────────────────── */

/**
 * The oversampler is measured before anything that sits on top of it. If the
 * round trip is not transparent then every harmonic number below is measuring
 * the filter, not the shaper.
 */
void TestHalfBand(Report& r)
{
    r.Section("Oversampler — the up/down pair must be transparent");

    // Magnitude through the round trip, no processing between.
    double worst = 0.0, worst_f = 0.0;
    for (double f : {50., 200., 1000., 4000., 8000., 12000., 15000., 17000.})
    {
        mastering_dsp::HalfBandUp   u;
        mastering_dsp::HalfBandDown d;
        const int N = 40000;
        double num = 0.0, den = 0.0;
        for (int n = 0; n < N; n++)
        {
            float a, b;
            u.Process(float(std::sin(2.0 * kPi * f * n / kFs)), a, b);
            const double y = d.Process(a, b);
            if (n > 3000)
            {
                const double xr = std::sin(2.0 * kPi * f * (n - 15) / kFs);
                num += y * y;
                den += xr * xr;
            }
        }
        const double db = 10.0 * std::log10(num / den);
        if (std::fabs(db) > std::fabs(worst)) { worst = db; worst_f = f; }
    }
    r.Check(std::fabs(worst) < 0.02, "round trip is flat to 17 kHz",
            Fmt("worst %+.4f dB at %.0f Hz", worst, worst_f));

    // Latency: an impulse must emerge at exactly 15.
    {
        mastering_dsp::HalfBandUp   u;
        mastering_dsp::HalfBandDown d;
        int    peak_at = -1;
        double peak    = 0.0;
        for (int n = 0; n < 80; n++)
        {
            float a, b;
            u.Process(n == 0 ? 1.f : 0.f, a, b);
            const double y = d.Process(a, b);
            if (std::fabs(y) > peak) { peak = std::fabs(y); peak_at = n; }
        }
        r.Check(peak_at == 15, "pair latency is exactly 15 base-rate samples",
                Fmt("peak at sample %.0f", double(peak_at)));
    }

    // Image rejection: a base-rate tone at f images to fs-f after zero-stuffing.
    // Below the 17 kHz passband edge that image sits in the stopband.
    {
        const double f = 12000.0;
        mastering_dsp::HalfBandUp u;
        const int N = 48000;
        double re = 0, im = 0, re0 = 0, im0 = 0, wsum = 0;
        for (int n = 0; n < N; n++)
        {
            const double win = 0.5 - 0.5 * std::cos(2.0 * kPi * n / (N - 1));
            float a, b;
            u.Process(float(std::sin(2.0 * kPi * f * n / kFs)), a, b);
            for (int k = 0; k < 2; k++)
            {
                const double v  = (k ? b : a) * win;
                const int    m  = 2 * n + k;
                const double wi = 2.0 * kPi * (kFs - f) / (2.0 * kFs);
                const double w0 = 2.0 * kPi * f / (2.0 * kFs);
                re  += v * std::cos(wi * m);  im  -= v * std::sin(wi * m);
                re0 += v * std::cos(w0 * m);  im0 -= v * std::sin(w0 * m);
            }
            wsum += win;
        }
        const double rej = 20.0 * std::log10(std::hypot(re0, im0)
                                             / (std::hypot(re, im) + 1e-30));
        r.Check(rej > 70.0, "image of a 12 kHz tone is rejected",
                Fmt("%.1f dB", rej));
    }
}

/* ── 2. The emphasis pair inverts exactly ────────────────────────────────── */

/**
 * The claim that makes the stage usable: at zero drive it is inaudible no
 * matter where Emphasis sits. That holds only because de-emphasis is built by
 * InvertBiquad rather than designed as an independent cut, so the control is
 * the independent design — which must fail the same bound.
 */
void TestEmphasisInverts(Report& r)
{
    r.Section("Emphasis / de-emphasis — exact inverse, not approximate");

    for (uint8_t ch = 0; ch < 3; ch++)
    {
        const auto& k = mastering_dsp::kSatChars[ch];
        double worst = 0.0, worst_f = 0.0;

        for (double e : {0.25, 0.5, 1.0})
        {
            const auto emph = mastering_dsp::MakeHighShelf<float>(
                k.emph_hz, float(e) * k.emph_db, float(kFs));
            const auto inv = mastering_dsp::InvertBiquad(emph);

            // Perfect reconstruction is a statement about the COMPLEX product,
            // not the magnitude: H_emph * H_deemph must be 1 + 0j. Measuring
            // only |H| would pass a filter that restores the level and leaves
            // the phase rotated, which is audible on transients.
            for (double f : eqtest::SweepFreqs())
            {
                const auto a = eqtest::ResponseAt(emph, f, kFs);
                const auto b = eqtest::ResponseAt(inv, f, kFs);
                const double err = std::abs(a * b - std::complex<double>(1.0, 0.0));
                if (err > worst) { worst = err; worst_f = f; }
            }
        }
        r.Check(worst < 1e-5,
                std::string(CharName(ch)) + ": pair reconstructs exactly",
                Fmt("worst |H*Hinv - 1| = %.2e at %.0f Hz", worst, worst_f));
    }

    /*
     * A static cut designed as MakeHighShelf(fc, -G) is NOT a wrong answer —
     * the matched design is reciprocal by construction, and agrees with
     * InvertBiquad to round-off. Assert that, so the claim is on the record.
     */
    {
        const auto& k = mastering_dsp::kSatChars[1];
        double worst = 0.0;
        for (int i = 1; i <= 8; i++)
        {
            const float g = float(i) / 8.f * k.emph_db;
            const auto up  = mastering_dsp::MakeHighShelf<float>(k.emph_hz, g, float(kFs));
            const auto dn  = mastering_dsp::MakeHighShelf<float>(k.emph_hz, -g, float(kFs));
            const auto inv = mastering_dsp::InvertBiquad(up);
            worst = std::max(worst, double(std::fabs(dn.b0 - inv.b0)
                                         + std::fabs(dn.b1 - inv.b1)
                                         + std::fabs(dn.b2 - inv.b2)
                                         + std::fabs(dn.a1 - inv.a1)
                                         + std::fabs(dn.a2 - inv.a2)));
        }
        r.Check(worst < 1e-6, "the matched shelf is reciprocal by construction",
                Fmt("MakeHighShelf(-g) equals InvertBiquad(+g) to %.1e", worst));
    }

    /*
     * NEGATIVE CONTROL — the failure InvertBiquad actually exists to prevent.
     *
     * Mid knob-move the audio side holds a LERP of two coefficient sets, which
     * is not the design of any single knob value. Inverting what it holds stays
     * exact; designing a cut from the eased knob value does not, and the error
     * appears exactly when the user is turning the control.
     */
    {
        const auto& k = mastering_dsp::kSatChars[1];
        const auto a = mastering_dsp::MakeHighShelf<float>(k.emph_hz, 0.3f * k.emph_db,
                                                           float(kFs));
        const auto b = mastering_dsp::MakeHighShelf<float>(k.emph_hz, 0.7f * k.emph_db,
                                                           float(kFs));
        auto mid = a;
        mastering_dsp::LerpCoeffs(mid, b, 0.5f);          // what the ISR holds

        const auto good = mastering_dsp::InvertBiquad(mid);
        const auto bad  = mastering_dsp::MakeHighShelf<float>(
            k.emph_hz, -0.5f * k.emph_db, float(kFs));    // designed from the knob

        double w_good = 0.0, w_bad = 0.0;
        for (double f : eqtest::SweepFreqs())
        {
            const auto h = eqtest::ResponseAt(mid, f, kFs);
            w_good = std::max(w_good, std::abs(h * eqtest::ResponseAt(good, f, kFs)
                                               - std::complex<double>(1.0, 0.0)));
            w_bad  = std::max(w_bad,  std::abs(h * eqtest::ResponseAt(bad, f, kFs)
                                               - std::complex<double>(1.0, 0.0)));
        }
        r.Check(w_good < 1e-5, "mid-ramp, inverting the held coefficients is exact",
                Fmt("worst |prod - 1| = %.2e", w_good));
        r.Check(w_bad > 1e-3,
                "negative control — designing the cut from the knob IS rejected",
                Fmt("worst |prod - 1| = %.4f", w_bad));
    }

    // The inverse is only stable if the source is minimum phase. Assert it over
    // the whole grid the character table can actually produce.
    {
        double margin = 1e9;
        bool   all_mp = true;
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            const auto& k = mastering_dsp::kSatChars[ch];
            for (int i = 0; i <= 40; i++)
            {
                const auto c = mastering_dsp::MakeHighShelf<float>(
                    k.emph_hz, float(i) / 40.f * k.emph_db, float(kFs));
                if (!eqtest::IsMinPhase(c)) all_mp = false;
                const auto inv = mastering_dsp::InvertBiquad(c);
                margin = std::min(margin, 1.0 - eqtest::PoleMargin(inv));
            }
        }
        r.Check(all_mp, "every emphasis shelf is minimum phase",
                "so InvertBiquad's poles are inside the unit circle");
        r.Check(margin > 0.01, "the inverse filter is comfortably stable",
                Fmt("worst pole margin %.4f from the unit circle", margin));
    }
}

/* ── 3. Unity at rest ────────────────────────────────────────────────────── */

/**
 * Three separate promises, and the first one is the stage's whole reason to be
 * audible.
 *
 * The compensation divides by the CHORD to kSatRefAmp, not by the slope at the
 * origin, so what is held constant across the drive range is the gain at the
 * REFERENCE LEVEL — not the small-signal gain. That is the difference between
 * a Drive knob that trades peaks for harmonics and one that trades them for
 * silence: measured through the whole chain, origin-referenced compensation
 * cost 11.3 dB of output RMS over the drive range and dropped the peak 22 dB,
 * which is what made 17 % THD inaudible.
 *
 * Second, drive at zero is still EXACTLY transparent — the reference chord is
 * normalised per character so that it is, and the emphasis-pair argument
 * depends on it.
 *
 * Third, the low end stays put at zero drive regardless of Emphasis.
 */
void TestUnityGain(Report& r)
{
    r.Section("Gain compensation — Drive trades peaks for harmonics, not level");

    /*
     * The contract, measured the way a listener meets it: output RMS on a
     * program, across the whole drive range.
     *
     * Not the gain at kSatRefAmp, which is what the compensation literally
     * holds constant — that is the INSTANTANEOUS transfer at one amplitude, and
     * a sine's fundamental is compressed by a different amount than its peak
     * chord, so measuring it would report a 1.7 dB spread and prove nothing
     * about either quantity. The reference chord is a one-parameter stand-in for
     * a program-level match (see kSatRefAmp); the match is the claim, so the
     * match is what gets asserted.
     *
     * The band is 3.5 dB across two working levels 6 dB apart. It cannot be
     * tighter with a static makeup: the curve is compressive, so a scheme
     * matched at one level necessarily runs hot at lower ones and shy at higher.
     * That residual IS the tape compression the scheme buys.
     */
    for (uint8_t ch = 0; ch < 3; ch++)
    {
        const int N = 16384;
        double worst = 0.0, worst_d = 0.0, worst_old = 0.0;
        for (double lvl : {-18., -12.})
        {
            const double amp = std::pow(10.0, lvl / 20.0) * std::sqrt(3.0);
            double ref = 0.0, ref_old = 0.0;
            for (double d : {0., 6., 12., 18., 24.})
            {
                auto p = MakeParams();
                p.character = ch;
                p.drive_db  = float(d);
                Rig rig(p);
                const auto in = Noise(amp, N);
                std::vector<float> out(static_cast<size_t>(N));
                rig.Run(in.data(), out.data(), N);
                const double rms = RmsDb(out, 4000);

                // NEGATIVE CONTROL, reconstructed rather than rebuilt: the old
                // compensation differs from this one by the known scalar
                // comp*drive, so dividing it out recovers what the previous
                // build produced, sample for sample.
                const double knee  = double(mastering_dsp::kSatChars[ch].knee);
                const double drive = knee * std::pow(10.0, d / 20.0);
                const double chord = std::tanh(knee * double(mastering_dsp::kSatRefAmp)) / knee;
                const double comp  = chord / std::tanh(drive * double(mastering_dsp::kSatRefAmp));
                const double old   = rms - 20.0 * std::log10(comp * drive);

                if (d == 0.) { ref = rms; ref_old = old; }
                if (std::fabs(rms - ref) > std::fabs(worst))
                {
                    worst   = rms - ref;
                    worst_d = d;
                }
                worst_old = std::min(worst_old, old - ref_old);
            }
        }
        r.Check(std::fabs(worst) < 3.5,
                std::string(CharName(ch)) + ": program level holds across the drive range",
                Fmt("worst %+.2f dB at drive %.0f dB", worst, worst_d));
        r.Check(worst_old < -6.0,
                std::string(CharName(ch))
                    + ": negative control — origin-referenced compensation IS rejected",
                Fmt("it collapses %.2f dB over the same range", worst_old));
    }

    // Zero drive is exactly transparent, per character. This one IS 0.05 dB.
    {
        double worst = 0.0;
        uint8_t worst_ch = 0;
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            auto p = MakeParams();
            p.character = ch;
            p.drive_db  = 0.f;
            // -60 dBFS: far enough down the tanh that the curve is linear.
            const double g = GainDb(p, 1000.0, 0.001);
            if (std::fabs(g) > std::fabs(worst)) { worst = g; worst_ch = ch; }
        }
        r.Check(std::fabs(worst) < 0.05, "at zero drive the stage is transparent",
                Fmt("worst %+.4f dB", worst) + std::string(", on ") + CharName(worst_ch));
    }

    // What the compensation costs, on the record rather than implied: away from
    // zero drive the small-signal gain is a boost, and it rises with the knee.
    // This is tape compression — quiet material coming up relative to loud —
    // and it lifts the noise floor with the drive.
    {
        auto p = MakeParams();
        p.drive_db = 24.f;
        double g[3];
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            p.character = ch;
            g[ch] = GainDb(p, 1000.0, 0.001);
        }
        r.Info("small-signal gain at full drive (the noise-floor cost)",
               Fmt("30ips %+.2f dB, 15ips %+.2f dB", g[0], g[1])
                   + Fmt(", sat %+.2f dB", g[2]));
        r.Check(g[0] < g[1] && g[1] < g[2],
                "and it orders by knee, as the characters' compression does",
                "30ips < 15ips < Saturated");
    }


    // Below 2 kHz the stage must be flat at zero drive whatever Emphasis says,
    // because the emphasis pair reconstructs exactly.
    {
        double worst = 0.0, worst_f = 0.0;
        for (double e : {0.0, 0.5, 1.0})
        {
            auto p = MakeParams();
            p.emphasis = float(e);
            for (double f : {50., 200., 1000., 2000.})
            {
                const double g = GainDb(p, f, 0.001);
                if (std::fabs(g) > std::fabs(worst)) { worst = g; worst_f = f; }
            }
        }
        r.Check(std::fabs(worst) < 0.05,
                "flat below 2 kHz at zero drive, any Emphasis",
                Fmt("worst %+.4f dB at %.0f Hz", worst, worst_f));
    }

    /*
     * Above that the stage is NOT flat, and the reason is worth stating rather
     * than hiding behind a loose bound.
     *
     * First-order ADAA replaces f(x[n]) with the integral-mean of f over
     * [x[n-1], x[n]]. Where f is locally linear that mean is exactly
     * (x[n] + x[n-1]) / 2 — a two-tap average at the 2x rate, whose response is
     * cos(pi*f/96000). So ADAA carries an inherent, drive-independent high-
     * frequency softening: -0.17 dB at 6 kHz, -0.69 dB at 12 kHz, -1.25 dB at
     * 16 kHz.
     *
     * It is kept rather than compensated. Measured against pointwise tanh at the
     * same 2x rate, ADAA buys 15-19 dB of alias rejection at every drive setting
     * (see TestAliasing's negative control), and a gentle top-end softening is
     * what tape does anyway. Compensating it would also mean boosting exactly
     * the band the aliases land in.
     *
     * Asserted as a curve, not a tolerance, so that a change to the oversampling
     * ratio or the ADAA order shows up here as a failure rather than as a
     * quietly different top end.
     */
    {
        auto p = MakeParams();
        double worst = 0.0, worst_f = 0.0;
        for (double f : {4000., 6000., 8000., 12000., 16000.})
        {
            const double predicted = 20.0 * std::log10(std::cos(kPi * f / 96000.0));
            const double measured  = GainDb(p, f, 0.001);
            const double err = std::fabs(measured - predicted);
            if (err > worst) { worst = err; worst_f = f; }
        }
        r.Check(worst < 0.06, "HF softening is exactly ADAA's two-tap average",
                Fmt("worst departure from cos(pi f/96k): %.4f dB at %.0f Hz",
                    worst, worst_f));
        r.Info("the resulting top end",
               Fmt("%.2f dB at 6 kHz, %.2f dB at 12 kHz",
                   GainDb(p, 6000.0, 0.001), GainDb(p, 12000.0, 0.001)));
    }
}

/* ── 4. Harmonic structure ───────────────────────────────────────────────── */

/**
 * What separates a tape curve from a clipper is how fast the series decays.
 * tanh is C-infinity so its harmonics fall away geometrically; the cubic soft
 * clip this replaced is only C1 at its corner and then exactly flat, which
 * leaves a slow high-order tail. Both are measured; the cubic must fail.
 */
void TestHarmonicDecay(Report& r)
{
    r.Section("Harmonic decay — a tape curve, not a clipper");

    auto p = MakeParams();
    p.drive_db = 18.f;
    Rig rig(p);
    const int  N  = 32768;
    const auto in = Sine(1000.0, 0.25, N);
    std::vector<float> out(static_cast<size_t>(N));
    rig.Run(in.data(), out.data(), N);

    const auto h = Harmonics(out, 1000.0, 9, 4000);
    r.Info("harmonic levels re fundamental",
           Fmt("H3 %.1f dB, H5 %.1f dB", h[2], h[4]));
    r.Info("", Fmt("H7 %.1f dB, H9 %.1f dB", h[6], h[8]));

    // Odd harmonics only, and each one well below the last.
    // 10 dB, not 6: the cubic this replaced manages ~7 dB per order at matched
    // compression, so a 6 dB bound would pass both and discriminate nothing.
    // tanh clears 12.
    const double d35 = h[2] - h[4], d57 = h[4] - h[6], d79 = h[6] - h[8];
    r.Check(d35 > 10.0 && d57 > 10.0 && d79 > 10.0,
            "odd series decays by >10 dB per order",
            Fmt("H3->H5 %.1f dB, H5->H7 %.1f dB", d35, d57));
    // -50 dB, not -80: at 18 dB of drive this stage is meant to be audibly
    // saturating, and a ninth harmonic that far down is inaudible under the
    // eighth. The bound that carries the argument is the decay rate above.
    r.Check(h[8] < -50.0, "H9 is far under the audible series",
            Fmt("%.1f dB re fundamental", h[8]));

    // NEGATIVE CONTROL — the cubic soft clip the stage used to use, driven to
    // the same peak so the comparison is at matched saturation.
    {
        // Driven to the same compression of the fundamental as the tanh run
        // above (~5 dB), so the comparison is at matched saturation rather than
        // matched input gain — otherwise the control would be measuring level,
        // not curve shape.
        std::vector<float> cub(static_cast<size_t>(N));
        for (int n = 0; n < N; n++)
        {
            const double x = 10.0 * in[size_t(n)];
            const double c = std::fabs(x) >= 1.0
                           ? (x < 0 ? -1.0 : 1.0)
                           : 1.5 * (x - x * x * x / 3.0);
            cub[size_t(n)] = float(c);
        }
        const auto hc = Harmonics(cub, 1000.0, 9, 4000);
        const double c35 = hc[2] - hc[4], c57 = hc[4] - hc[6], c79 = hc[6] - hc[8];
        r.Info("the cubic's series, at matched compression",
               Fmt("H3 %.1f dB, H5 %.1f dB", hc[2], hc[4]));
        r.Info("", Fmt("H7 %.1f dB, H9 %.1f dB", hc[6], hc[8]));
        r.Check(!(c35 > 10.0 && c57 > 10.0 && c79 > 10.0),
                "negative control — the old cubic clip IS rejected",
                Fmt("H3->H5 %.1f dB, H5->H7 %.1f dB", c35, c57));
        r.Check(hc[8] > -50.0, "negative control — its H9 is NOT far under",
                Fmt("%.1f dB re fundamental", hc[8]));
    }
}

/** Asymmetry is the even-harmonic control; at zero it must produce none. */
void TestAsymmetry(Report& r)
{
    r.Section("Asymmetry — even harmonics on demand, and none without");

    for (double a : {0.0, 0.2})
    {
        auto p = MakeParams();
        p.drive_db = 18.f;
        p.asym     = float(a);
        Rig rig(p);
        const int  N  = 32768;
        const auto in = Sine(1000.0, 0.25, N);
        std::vector<float> out(static_cast<size_t>(N));
        rig.Run(in.data(), out.data(), N);

        const auto h = Harmonics(out, 1000.0, 5, 4000);
        if (a == 0.0)
            r.Check(h[1] < -90.0, "symmetric shaper makes no H2",
                    Fmt("H2 %.1f dB re fundamental", h[1]));
        else
            r.Check(h[1] > -50.0, "asym 0.2 makes audible H2",
                    Fmt("H2 %.1f dB re fundamental", h[1]));

        // Whatever the offset does, no DC may reach the output.
        r.Check(MeanDb(out, 4000) < -80.0,
                Fmt("asym %.1f leaves no DC", a),
                Fmt("mean %.1f dBFS", MeanDb(out, 4000)));
    }
}

/* ── 5. Aliasing ─────────────────────────────────────────────────────────── */

/**
 * The point of ADAA on top of 2x oversampling. An 11 kHz tone puts its 3rd
 * harmonic at 33 kHz, which can only appear in the output by folding to 15 kHz
 * — so every bin measured here is pure artifact.
 *
 * The negative control is the same measurement with the antialiasing removed:
 * pointwise tanh at the base rate, which is what a naive implementation does.
 */
void TestAliasing(Report& r)
{
    r.Section("Aliasing — oversampling plus ADAA, measured not assumed");

    const double f0 = 11000.0;
    const int    N  = 32768;

    auto p = MakeParams();
    p.drive_db = 18.f;
    Rig rig(p);
    const auto in = Sine(f0, 0.25, N);
    std::vector<float> out(static_cast<size_t>(N));
    rig.Run(in.data(), out.data(), N);

    const double al = AliasDb(out, f0, 12, 4000);
    r.Check(al < -50.0, "alias energy is far below the fundamental",
            Fmt("%.1f dB re fundamental", al));

    // NEGATIVE CONTROL — pointwise tanh at the base rate, same drive.
    {
        std::vector<float> naive(static_cast<size_t>(N));
        for (int n = 0; n < N; n++)
            naive[size_t(n)] = float(std::tanh(8.0 * in[size_t(n)]) / 8.0);
        const double an = AliasDb(naive, f0, 12, 4000);
        r.Check(an > al + 20.0,
                "negative control — pointwise at the base rate IS much worse",
                Fmt("%.1f dB, against %.1f dB for the real stage", an, al));
    }
}

/* ── 6. Head bump ────────────────────────────────────────────────────────── */

void TestHeadBump(Report& r)
{
    r.Section("Head bump — a resonant lift the EQ page cannot make");

    for (uint8_t ch = 0; ch < 3; ch++)
    {
        const auto& k = mastering_dsp::kSatChars[ch];
        auto p = MakeParams();
        p.character = ch;
        p.bump      = 1.f;

        const double at_f  = GainDb(p, k.bump_hz, 0.001);
        const double at_1k = GainDb(p, 1000.0, 0.001);
        r.Check(std::fabs(at_f - k.bump_db) < 0.3,
                std::string(CharName(ch)) + ": bump reaches its rated height",
                Fmt("%+.2f dB at %.0f Hz", at_f, k.bump_hz));
        r.Check(std::fabs(at_1k) < 0.2,
                std::string(CharName(ch)) + ": and is gone by 1 kHz",
                Fmt("%+.3f dB", at_1k));
    }
}

/* ── 7. Emphasis makes saturation frequency-dependent ────────────────────── */

/**
 * The whole reason the stage is called tape. With emphasis engaged a high tone
 * must saturate more than a low one at the same input level; with it at zero
 * they must saturate the same. That difference is the effect, so it is measured
 * directly rather than inferred from the filter shapes.
 */
void TestFrequencyDependence(Report& r)
{
    r.Section("HF saturates first — the mechanism that makes it tape");

    auto measure = [&](double emph, double f) {
        auto p = MakeParams();
        p.drive_db = 18.f;
        p.emphasis = float(emph);
        Rig rig(p);
        const int  N  = 32768;
        const auto in = Sine(f, 0.25, N);
        std::vector<float> out(static_cast<size_t>(N));
        rig.Run(in.data(), out.data(), N);
        // Gain of the fundamental. Only DIFFERENCES between these matter here —
        // the absolute figure carries the reference-chord makeup, which is
        // common to both frequencies and so cancels in every check below.
        return 20.0 * std::log10(BinAmp(out, f, 4000) / 0.25);
    };

    const double lo_off = measure(0.0, 200.0), hi_off = measure(0.0, 6000.0);
    const double lo_on  = measure(1.0, 200.0), hi_on  = measure(1.0, 6000.0);

    r.Info("gain at 18 dB drive, emphasis off",
           Fmt("200 Hz %+.2f dB, 6 kHz %+.2f dB", lo_off, hi_off));
    r.Info("gain at 18 dB drive, emphasis on",
           Fmt("200 Hz %+.2f dB, 6 kHz %+.2f dB", lo_on, hi_on));

    r.Check(std::fabs(hi_off - lo_off) < 0.5,
            "with emphasis off, saturation is frequency-flat",
            Fmt("6 kHz within %.2f dB of 200 Hz", std::fabs(hi_off - lo_off)));
    r.Check(hi_on < lo_on - 2.0,
            "with emphasis on, HF compresses substantially more",
            Fmt("6 kHz is %.2f dB below 200 Hz", lo_on - hi_on));
    r.Check(std::fabs(lo_on - lo_off) < 1.0,
            "and the low end is left where it was",
            Fmt("200 Hz moved %.2f dB", lo_on - lo_off));
}

/* ── 8. Bypass and mix ───────────────────────────────────────────────────── */

void TestBypassAndMix(Report& r)
{
    r.Section("Bypass and mix — matched delay, no click, no comb");

    // Bypass must reproduce the input exactly, 15 samples late.
    {
        auto p = MakeParams();
        p.bypass = true;
        p.drive_db = 18.f;
        Rig rig(p);
        const int  N  = 8192;
        const auto in = Noise(0.3, N);
        std::vector<float> out(static_cast<size_t>(N));
        rig.Run(in.data(), out.data(), N);
        double worst = 0.0;
        for (int n = 1000; n < N; n++)
            worst = std::max(worst, std::fabs(double(out[size_t(n)])
                                              - in[size_t(n - 15)]));
        r.Check(worst < 1e-6, "bypass is the dry signal delayed by exactly 15",
                Fmt("worst sample error %.2e", worst));
    }

    // Mix at zero must equal bypass — same path, so a partial mix cannot comb.
    {
        auto p = MakeParams();
        p.mix = 0.f;
        p.drive_db = 18.f;
        Rig rig(p);
        const int  N  = 8192;
        const auto in = Noise(0.3, N);
        std::vector<float> out(static_cast<size_t>(N));
        rig.Run(in.data(), out.data(), N);
        double worst = 0.0;
        for (int n = 1000; n < N; n++)
            worst = std::max(worst, std::fabs(double(out[size_t(n)])
                                              - in[size_t(n - 15)]));
        r.Check(worst < 1e-6, "mix 0 is bit-equal to bypass",
                Fmt("worst sample error %.2e", worst));
    }

    // Toggling bypass mid-signal must not step. The dry delay is what buys
    // this; the old implementation returned early and left its state stale.
    {
        auto p = MakeParams();
        p.drive_db = 12.f;
        Rig rig(p);
        const int  N  = 8192;
        const auto in = Sine(220.0, 0.4, N);
        std::vector<float> out(static_cast<size_t>(N));
        for (int n = 0; n < N; n++)
        {
            if (n % 24 == 0)
            {
                if (n == 4032) { p.bypass = true; rig.sat.Configure(p, float(kFs)); }
                rig.sat.BeginBlock();
            }
            float l = in[size_t(n)], rr = l;
            rig.sat.ProcessSample(l, rr);
            out[size_t(n)] = l;
        }
        double worst = 0.0;
        for (int n = 3000; n < 6000; n++)
            worst = std::max(worst,
                             std::fabs(double(out[size_t(n)]) - out[size_t(n - 1)]));
        const double slew = 0.4 * 2.0 * kPi * 220.0 / kFs;
        r.Check(worst < 3.0 * slew, "bypass toggle produces no step",
                Fmt("worst jump %.2e vs tone slew %.2e", worst, slew));
    }
}

/* ── 9. Adversarial ──────────────────────────────────────────────────────── */

void TestAdversarial(Report& r)
{
    r.Section("Adversarial — nothing here may produce a non-finite sample");

    struct Case { const char* name; double amp; double drive; double asym; };
    const Case cases[] = {
        {"full-scale square-ish noise", 1.0,  24.0,  0.3},
        {"silence at maximum drive",    0.0,  24.0,  0.3},
        {"DC at maximum drive",         0.99, 24.0, -0.3},
    };

    for (const auto& c : cases)
    {
        auto p = MakeParams();
        p.drive_db = float(c.drive);
        p.asym     = float(c.asym);
        p.emphasis = 1.f;
        p.bump     = 1.f;
        Rig rig(p);

        const int N = 8192;
        std::vector<float> in(static_cast<size_t>(N)), out(static_cast<size_t>(N));
        if (std::strcmp(c.name, "DC at maximum drive") == 0)
            for (int n = 0; n < N; n++) in[size_t(n)] = float(c.amp);
        else
            in = Noise(c.amp, N);

        rig.Run(in.data(), out.data(), N);
        bool finite = true;
        double peak = 0.0;
        for (int n = 0; n < N; n++)
        {
            if (!std::isfinite(out[size_t(n)])) finite = false;
            peak = std::max(peak, std::fabs(double(out[size_t(n)])));
        }
        r.Check(finite && peak < 8.0, std::string(c.name) + ": stays finite and bounded",
                Fmt("peak %.3f", peak));
    }

    // DC in must not come out: the blocker plus the static subtraction.
    {
        auto p = MakeParams();
        p.drive_db = 12.f;
        p.asym     = 0.3f;
        Rig rig(p, 500);
        const int N = 48000;
        std::vector<float> in(static_cast<size_t>(N), 0.f), out(static_cast<size_t>(N));
        rig.Run(in.data(), out.data(), N);
        r.Check(MeanDb(out, 24000) < -100.0,
                "a settled stage with full asym emits no DC",
                Fmt("mean %.1f dBFS", MeanDb(out, 24000)));
    }
}

/* ── 10. Golden ──────────────────────────────────────────────────────────── */

/**
 * The golden grids lock the AUDIBLE contract — the steady-state transfer curve
 * and the harmonic structure — rather than any internal coefficient. An
 * internal refactor that keeps both is free; a change to either is not.
 */
struct CurvePoint { uint8_t ch; double in_db, out_db; };

std::vector<CurvePoint> CurveGrid()
{
    std::vector<CurvePoint> v;
    for (uint8_t ch = 0; ch < 3; ch++)
    {
        auto p = MakeParams();
        p.character = ch;
        p.drive_db  = 18.f;
        for (int i = 0; i <= 24; i++)
        {
            const double in_db = -60.0 + i * 2.5;
            const double amp   = std::pow(10.0, in_db / 20.0);
            Rig rig(p);
            const int  N  = 8192;
            const auto in = Sine(1000.0, amp, N);
            std::vector<float> out(static_cast<size_t>(N));
            rig.Run(in.data(), out.data(), N);
            v.push_back({ch, in_db,
                         20.0 * std::log10(BinAmp(out, 1000.0, 2000) + 1e-30)});
        }
    }
    return v;
}

struct HarmPoint { uint8_t ch; double drive_db; double h2, h3, h5, h7; };

std::vector<HarmPoint> HarmGrid()
{
    std::vector<HarmPoint> v;
    for (uint8_t ch = 0; ch < 3; ch++)
        for (double d : {6.0, 12.0, 18.0, 24.0})
        {
            auto p = MakeParams();
            p.character = ch;
            p.drive_db  = float(d);
            p.asym      = 0.1f;             // so H2 is a real number, not noise
            Rig rig(p);
            const int  N  = 16384;
            const auto in = Sine(1000.0, 0.25, N);
            std::vector<float> out(static_cast<size_t>(N));
            rig.Run(in.data(), out.data(), N);
            const auto h = Harmonics(out, 1000.0, 7, 3000);
            v.push_back({ch, d, h[1], h[2], h[4], h[6]});
        }
    return v;
}

void TestGolden(Report& r)
{
    r.Section("Golden — the audible contract, locked");

    const auto curve = CurveGrid();
    const auto harm  = HarmGrid();

    if (g_golden)
    {
        FILE* f = std::fopen("golden/sat_curve.csv", "w");
        std::fprintf(f, "character,in_db,out_db\n");
        for (const auto& c : curve)
            std::fprintf(f, "%u,%.4f,%.6f\n", c.ch, c.in_db, c.out_db);
        std::fclose(f);

        f = std::fopen("golden/sat_harmonics.csv", "w");
        std::fprintf(f, "character,drive_db,h2,h3,h5,h7\n");
        for (const auto& h : harm)
            std::fprintf(f, "%u,%.2f,%.5f,%.5f,%.5f,%.5f\n",
                         h.ch, h.drive_db, h.h2, h.h3, h.h5, h.h7);
        std::fclose(f);

        r.Info("golden files written", "tests/golden/sat_curve.csv, sat_harmonics.csv");
        return;
    }

    FILE* f = std::fopen("golden/sat_curve.csv", "r");
    if (!f)
    {
        r.Info("no golden files", "run `make test-golden` to create them");
        return;
    }
    {
        char line[256];
        (void)!std::fgets(line, sizeof line, f);
        size_t i = 0;
        int    diff = 0;
        double worst = 0.0;
        while (std::fgets(line, sizeof line, f) && i < curve.size())
        {
            unsigned ch; double in_db, out_db;
            if (std::sscanf(line, "%u,%lf,%lf", &ch, &in_db, &out_db) == 3)
            {
                const double d = std::fabs(out_db - curve[i].out_db);
                if (d > 1e-3) diff++;
                worst = std::max(worst, d);
                i++;
            }
        }
        std::fclose(f);
        r.Check(diff == 0 && i == curve.size(), "transfer curve matches golden",
                Fmt("%.0f rows, worst delta %.6f dB", double(i), worst));
    }

    f = std::fopen("golden/sat_harmonics.csv", "r");
    if (f)
    {
        char line[256];
        (void)!std::fgets(line, sizeof line, f);
        size_t i = 0;
        int    diff = 0;
        double worst = 0.0;
        while (std::fgets(line, sizeof line, f) && i < harm.size())
        {
            unsigned ch; double d_db, h2, h3, h5, h7;
            if (std::sscanf(line, "%u,%lf,%lf,%lf,%lf,%lf",
                            &ch, &d_db, &h2, &h3, &h5, &h7) == 6)
            {
                /*
                 * Compared with a floor, not as a flat 0.01 dB on every
                 * harmonic — because a flat dB tolerance is the wrong shape for
                 * this grid and was passing by luck rather than by strictness.
                 *
                 * H7 at 6 dB of drive sits 109 dB below the fundamental. The
                 * arithmetic underneath it (2x oversampling, an ADAA quotient,
                 * a 16-tap half-band, all in float) has a noise floor of its
                 * own around 1e-6 in amplitude, so that bin is roughly ten
                 * parts noise to one part harmonic. Perturbing the LAST BIT of
                 * anything upstream moves it a full decibel while moving the
                 * amplitude by 4e-7 — which is what happened when the drive
                 * compensation changed from an eased reciprocal to a derived
                 * one, a change that is exactly zero in steady state.
                 *
                 * So the comparison is on amplitude with an absolute floor,
                 * which is the quantity that is actually reproducible. kHarmEps
                 * is -100 dBc: above the measured 5.8e-6 spread, and 60 dB
                 * below the quietest harmonic anyone could call audible, which
                 * is the contract this grid says it locks.
                 */
                constexpr double kHarmEps = 1e-5;      // -100 dBc, absolute
                constexpr double kHarmRel = 0.01;      // dB, for harmonics well clear of it
                auto off = [](double gold, double got) {
                    const double ag = std::pow(10.0, gold / 20.0);
                    const double an = std::pow(10.0, got  / 20.0);
                    const double slack = kHarmEps + ag * (std::pow(10.0, kHarmRel / 20.0) - 1.0);
                    return std::fabs(ag - an) / slack;      // >1 means out of tolerance
                };
                const double e = std::max(std::max(off(h2, harm[i].h2), off(h3, harm[i].h3)),
                                          std::max(off(h5, harm[i].h5), off(h7, harm[i].h7)));
                if (e > 1.0) diff++;
                worst = std::max(worst, e);
                i++;
            }
        }
        std::fclose(f);
        r.Check(diff == 0 && i == harm.size(), "harmonic structure matches golden",
                Fmt("%.0f rows, worst %.2f", double(i), worst) + " of tolerance");
    }
}

} // namespace

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; i++)
        if (std::strcmp(argv[i], "--golden") == 0) g_golden = true;

    std::printf("\n\033[1mTape saturator response harness\033[0m  —  the stage "
                "alone at %.0f Hz\n", kFs);

    Report r;
    TestHalfBand(r);
    TestEmphasisInverts(r);
    TestUnityGain(r);
    TestHarmonicDecay(r);
    TestAsymmetry(r);
    TestAliasing(r);
    TestHeadBump(r);
    TestFrequencyDependence(r);
    TestBypassAndMix(r);
    TestAdversarial(r);
    TestGolden(r);
    return r.Finish();
}
