/**
 * eq_test_common.h — host-side measurement support for the EQ.
 *
 * Three things live here:
 *
 *   1. Digital response — evaluate H(e^jw) directly from the BiquadCoeffs
 *      that dsp_biquad.h's Make* functions return.
 *   2. Analog prototype response — evaluate the *s*-domain prototypes the
 *      digital designs are supposed to be approximating, with NO prewarping.
 *      That is the entire point: prewarping would hide exactly the cramping
 *      error we are here to measure.
 *   3. The shared assertion/reporting framework, pulled in from
 *      test_report.h and re-exported under eqtest:: so a run prints the
 *      numbers even when it passes.
 *
 * Prototype sources:
 *   Peaking      — Vicanek, "Matched Second Order Digital Filters" eq (42)
 *                  https://vicanek.de/articles/BiquadFits.pdf
 *   Shelves      — Vicanek, "Matched Two-Pole Digital Shelving Filters" eq (1)
 *                  https://vicanek.de/articles/2poleShelvingFits.pdf
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "dsp_biquad.h"
#include "test_report.h"

namespace eqtest {

using cplx = std::complex<double>;

constexpr double kPi   = 3.14159265358979323846;
constexpr double kFs   = 48000.0;
constexpr double kSqrt2 = 1.41421356237309504880;

inline double DbToLin(double db) { return std::pow(10.0, db / 20.0); }
inline double LinToDb(double x)  { return 20.0 * std::log10(std::max(x, 1e-300)); }

/* ── 1. Digital response ──────────────────────────────────────────────── */

/** H(e^jw) for one biquad section, evaluated from its realized coefficients. */
template <typename T>
inline cplx ResponseAt(const mastering_dsp::BiquadCoeffsT<T>& c, double f, double fs)
{
    const double w  = 2.0 * kPi * f / fs;
    const cplx   z1 = std::polar(1.0, -w);   // z^-1
    const cplx   z2 = z1 * z1;
    const cplx   num = double(c.b0) + double(c.b1) * z1 + double(c.b2) * z2;
    const cplx   den = 1.0          + double(c.a1) * z1 + double(c.a2) * z2;
    return num / den;
}

/* ── 2. Analog prototypes (no prewarping — deliberately) ──────────────── */

/**
 * Peaking / bell. BiquadFits eq (42):
 *   H(s) = (w0^2 + s*w0*sqrt(G)/Q + s^2) / (w0^2 + s*w0/(sqrt(G)*Q) + s^2)
 * Unity at DC and at Nyquist-and-beyond; gain exactly G at f0.
 */
inline cplx ProtoPeaking(double f, double f0, double gain_lin, double q)
{
    const double w0  = 2.0 * kPi * f0;
    const cplx   s   = cplx(0.0, 2.0 * kPi * f);
    const double sqG = std::sqrt(gain_lin);
    const cplx   num = w0 * w0 + s * (w0 * sqG / q) + s * s;
    const cplx   den = w0 * w0 + s * (w0 / (sqG * q)) + s * s;
    return num / den;
}

/**
 * Second-order Butterworth high shelf. 2poleShelvingFits eq (1):
 *   H(s) = (1 + sqrt2*g*s + g^2 s^2) / (1 + sqrt2*s/g + s^2/g^2),
 *   g = G^(1/4),  s = i*f/fc
 * Unity at DC, G at HF, sqrt(G) at fc.
 */
inline cplx ProtoHighShelf(double f, double fc, double gain_lin)
{
    const double g = std::pow(gain_lin, 0.25);
    const cplx   s = cplx(0.0, f / fc);
    const cplx   num = 1.0 + kSqrt2 * g * s + (g * g) * s * s;
    const cplx   den = 1.0 + kSqrt2 * s / g + s * s / (g * g);
    return num / den;
}

/**
 * Second-order Butterworth low shelf. 2poleShelvingFits §4: a low shelf is a
 * high shelf with gain 1/G, numerator scaled by G. G at DC, unity at HF.
 */
inline cplx ProtoLowShelf(double f, double fc, double gain_lin)
{
    return gain_lin * ProtoHighShelf(f, fc, 1.0 / gain_lin);
}

/**
 * Second-order high-pass, the prototype MakeHighPass approximates:
 *   H(s) = s^2 / (s^2 + s*w0/Q + w0^2)
 * Exactly zero at DC, unity at HF, and |H(jw0)| = Q at the corner — which is
 * the point the digital design matches. Used by the compressor harness for the
 * sidechain filter; it lives here because this is where the prototypes live.
 */
inline cplx ProtoHighPass(double f, double f0, double q)
{
    const double w0 = 2.0 * kPi * f0;
    const cplx   s  = cplx(0.0, 2.0 * kPi * f);
    return (s * s) / (s * s + s * (w0 / q) + w0 * w0);
}

/* ── Band description, so tests can drive all three uniformly ─────────── */

enum class Band { LowShelf, Peaking, HighShelf };

inline const char* BandName(Band b)
{
    switch (b)
    {
        case Band::LowShelf:  return "LS";
        case Band::Peaking:   return "MID";
        case Band::HighShelf: return "HS";
    }
    return "?";
}

struct BandSpec {
    Band   band;
    double f_hz;
    double gain_db;
    double q;        // peaking only; ignored by the shelves

    /** The design math at full precision — what the coefficient solve produces
     *  before it is stored. Used for the golden regression and the algebraic
     *  sanity checks, where float rounding would only add noise. */
    mastering_dsp::BiquadCoeffsT<double> Design(double fs = kFs) const
    {
        switch (band)
        {
            case Band::LowShelf:
                return mastering_dsp::MakeLowShelf<double>(float(f_hz), float(gain_db), float(fs));
            case Band::Peaking:
                return mastering_dsp::MakePeaking<double>(float(f_hz), float(gain_db), float(q), float(fs));
            case Band::HighShelf:
                return mastering_dsp::MakeHighShelf<double>(float(f_hz), float(gain_db), float(fs));
        }
        return {};
    }

    /** True where the firmware stores this band's coefficients in double.
     *  Mirrors the instantiation choice in mastering_dsp.cpp. */
    bool ShipsInDouble() const { return band == Band::LowShelf; }

    /** The coefficients as the firmware actually holds them — i.e. rounded to
     *  float for the bands that ship in float. Response accuracy must be
     *  measured against this, not against the full-precision solve. */
    mastering_dsp::BiquadCoeffsT<double> AsShipped(double fs = kFs) const
    {
        const auto d = Design(fs);
        if (ShipsInDouble()) return d;
        return {double(float(d.b0)), double(float(d.b1)), double(float(d.b2)),
                double(float(d.a1)), double(float(d.a2))};
    }

    cplx Prototype(double f) const
    {
        const double G = DbToLin(gain_db);
        switch (band)
        {
            case Band::LowShelf:  return ProtoLowShelf(f, f_hz, G);
            case Band::Peaking:   return ProtoPeaking(f, f_hz, G, q);
            case Band::HighShelf: return ProtoHighShelf(f, f_hz, G);
        }
        return {1.0, 0.0};
    }

    std::string Label() const
    {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%-3s f=%-8.1fHz  g=%+6.1fdB  Q=%.3f",
                      BandName(band), f_hz, gain_db, q);
        return buf;
    }
};

/* ── The 20 Hz – 20 kHz measurement sweep, 1/96 octave ────────────────── */

inline const std::vector<double>& SweepFreqs()
{
    static const std::vector<double> f = [] {
        std::vector<double> v;
        const double step = std::pow(2.0, 1.0 / 96.0);
        for (double x = 20.0; x <= 20000.0 * 1.0000001; x *= step) v.push_back(x);
        return v;
    }();
    return f;
}

/** Max |dB| deviation of the realized filter from its analog prototype. */
struct Deviation {
    double max_db   = 0.0;
    double at_hz    = 0.0;
    double max_deg  = 0.0;
    double deg_at_hz = 0.0;
};

inline Deviation MeasureDeviation(const BandSpec& spec, double fs = kFs)
{
    const auto coeffs = spec.AsShipped(fs);
    Deviation d;
    for (double f : SweepFreqs())
    {
        const cplx   h_dig = ResponseAt(coeffs, f, fs);
        const cplx   h_ana = spec.Prototype(f);
        const double ddb   = std::fabs(LinToDb(std::abs(h_dig)) - LinToDb(std::abs(h_ana)));
        if (ddb > d.max_db) { d.max_db = ddb; d.at_hz = f; }

        double dph = std::arg(h_dig) - std::arg(h_ana);
        while (dph >  kPi) dph -= 2.0 * kPi;
        while (dph < -kPi) dph += 2.0 * kPi;
        dph = std::fabs(dph) * 180.0 / kPi;
        if (dph > d.max_deg) { d.max_deg = dph; d.deg_at_hz = f; }
    }
    return d;
}

/* ── Coefficient sanity ───────────────────────────────────────────────── */

/** Biquad stability triangle: |a1| < 2 and |a1| - 1 < a2 < 1. */
template <typename T>
inline bool IsStable(const mastering_dsp::BiquadCoeffsT<T>& c)
{
    const double a1 = c.a1, a2 = c.a2;
    return std::fabs(a1) < 2.0 && a2 < 1.0 && a2 > std::fabs(a1) - 1.0;
}

/** Distance of the poles from the unit circle. */
template <typename T>
inline double PoleMargin(const mastering_dsp::BiquadCoeffsT<T>& c)
{
    const double a1 = c.a1, a2 = c.a2;
    const double disc = a1 * a1 - 4.0 * a2;
    double r;
    if (disc >= 0.0)
    {
        const double r1 = std::fabs((-a1 + std::sqrt(disc)) * 0.5);
        const double r2 = std::fabs((-a1 - std::sqrt(disc)) * 0.5);
        r = std::max(r1, r2);
    }
    else
    {
        r = std::sqrt(std::fabs(a2));   // complex pair: radius = sqrt(a2)
    }
    return 1.0 - r;
}

/** Vicanek BiquadFits eq (28): zeros inside the unit circle. */
template <typename T>
inline bool IsMinPhase(const mastering_dsp::BiquadCoeffsT<T>& c)
{
    const double b0 = c.b0, b1 = c.b1, b2 = c.b2;
    return b0 > std::fabs(b2) && (b0 + b2) > std::fabs(b1);
}

template <typename T>
inline bool IsFinite(const mastering_dsp::BiquadCoeffsT<T>& c)
{
    return std::isfinite(c.b0) && std::isfinite(c.b1) && std::isfinite(c.b2)
        && std::isfinite(c.a1) && std::isfinite(c.a2);
}

/* ── Tiny reporting framework ─────────────────────────────────────────── */

/* Lives in test_report.h so the compressor harness can share it. Pulled into
 * this namespace so every existing `eqtest::Report` / `eqtest::Fmt` spelling
 * keeps working — the extraction is meant to be invisible from here. */
using testrep::Fmt;
using testrep::Report;

} // namespace eqtest
