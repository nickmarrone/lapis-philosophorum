/**
 * dsp_biquad.h — Header-only biquad (Direct Form I) with magnitude-matched
 * coefficient design.
 *
 * These are *not* the RBJ cookbook designs any more. RBJ uses the bilinear
 * transform, which forces the response slope to zero at Nyquist — so a high
 * shelf parked at 20 kHz with fs = 48 kHz cannot physically reach its target
 * gain, and the curve around it is squashed. Measured error against the
 * analog prototype was 3.95 dB at 16.5 kHz.
 *
 * The designs below instead match the prototype's magnitude at a few chosen
 * frequencies and solve for the coefficients. Same five coefficients, same
 * Direct Form I loop, same cost at audio rate — only the control-rate math
 * changed. Phase is not traded away in the bargain: Vicanek's numerator
 * recovery imposes b0 > |b2| and b0 + b2 > |b1|, so the result is minimum
 * phase by construction, and for a minimum-phase filter matching magnitude
 * matches phase too.
 *
 *   Peaking  — Vicanek, "Matched Second Order Digital Filters" (2016) §3.2, §4.4
 *              https://vicanek.de/articles/BiquadFits.pdf
 *   Shelves  — Vicanek, "Matched Two-Pole Digital Shelving Filters" (2024) App. A
 *              https://vicanek.de/articles/2poleShelvingFits.pdf
 *
 * NOTE the gain convention differs from RBJ: Vicanek's G is the full linear
 * gain, 10^(dB/20), where RBJ's A is its square root, 10^(dB/40).
 *
 * The coefficient math runs in double and returns float. That is not
 * gold-plating — these designs cancel hard at the low end (the low shelf at
 * 20 Hz loses roughly seven decimal digits, which is all of float32), and the
 * design functions only run at control rate, so the precision is free.
 *
 * Stays <cmath>-only by invariant, so tests/ can compile it on a host with
 * no stubbing whatsoever.
 */

#pragma once
#include <cmath>

namespace mastering_dsp {

/**
 * The section is templated on its working precision, T. Signal in and out is
 * always float — T governs how the coefficients are stored and how the
 * recursion accumulates.
 *
 * Most bands are fine in float. A low-frequency shelf is not: its poles sit
 * at radius ~0.998, which amplifies the recursion's own roundoff by ~1/(1-r),
 * and it very nearly cancels its own poles against its zeros, so quantizing
 * the coefficients perturbs a difference of nearly-equal things. Measured on
 * the low shelf at 20 Hz, float32 costs 38 dB of noise floor and ~1e-2 dB of
 * response error; double costs ~20 cycles a frame and erases both.
 */
template <typename T>
struct BiquadCoeffsT { T b0=T(1), b1=T(0), b2=T(0), a1=T(0), a2=T(0); };

template <typename T>
struct BiquadStateT  { T x1=T(0), x2=T(0), y1=T(0), y2=T(0); };

template <typename T>
inline float BiquadProcess(BiquadStateT<T>& s, const BiquadCoeffsT<T>& c, float x)
{
    const T xn = T(x);
    const T y  = c.b0*xn + c.b1*s.x1 + c.b2*s.x2 - c.a1*s.y1 - c.a2*s.y2;
    s.x2 = s.x1;  s.x1 = xn;
    s.y2 = s.y1;  s.y1 = y;
    return float(y);
}

/**
 * Ease one coefficient set toward another, in place.
 *
 * Interpolating raw direct-form coefficients is safe, and not by luck: the
 * stability region |a1| < 2, |a1| - 1 < a2 < 1 is a triangle, hence convex, and
 * a linear interpolation is a convex combination — so a blend of two stable
 * biquads is always itself stable. (eq_control_test asserts this over 650k
 * blends rather than taking the argument's word for it.)
 *
 * Used by both stages that redesign filters from a moving knob: the EQ eases
 * per block toward its published bank, and the saturator does the same with its
 * emphasis shelf and head bump.
 */
template <typename T>
inline void LerpCoeffs(BiquadCoeffsT<T>& c, const BiquadCoeffsT<T>& t, T k)
{
    c.b0 += k * (t.b0 - c.b0);
    c.b1 += k * (t.b1 - c.b1);
    c.b2 += k * (t.b2 - c.b2);
    c.a1 += k * (t.a1 - c.a1);
    c.a2 += k * (t.a2 - c.a2);
}

/**
 * Exact inverse of a biquad: swap numerator and denominator, renormalize so
 * a0 is 1 again. H^-1(z) = A(z)/B(z).
 *
 * This is how the saturator builds its de-emphasis, and the reason is subtler
 * than it first looks. MakeHighShelf(fc, -G) is *already* an exact inverse of
 * MakeHighShelf(fc, +G) — the matched design turns out to be reciprocal by
 * construction, agreeing with this function to 1e-15 (sat_response_test checks
 * it). So on a static setting either route would do.
 *
 * What forces this one is the knob moving. The audio side eases the emphasis
 * *coefficients* toward their published target once per block, and a lerp of
 * two designs is not the design of the lerped parameter. Designing the cut from
 * the eased knob value would therefore invert the wrong filter for the whole
 * duration of a move — audible as the emphasis pair failing to cancel exactly
 * when it is most obvious that it should. Inverting the coefficients the audio
 * side actually holds is exact at every point of the ramp, and costs a divide
 * instead of a design.
 *
 * Stable only if the source's zeros are inside the unit circle, i.e. the source
 * is minimum phase — the inverse's poles are the original's zeros. Every design
 * in this header is minimum phase over the ranges used here, and
 * sat_response_test asserts it across the whole (fc, gain) grid rather than
 * trusting the claim.
 */
template <typename T>
inline BiquadCoeffsT<T> InvertBiquad(const BiquadCoeffsT<T>& c)
{
    const T g = T(1) / c.b0;
    return { g, c.a1 * g, c.a2 * g, c.b1 * g, c.b2 * g };
}

using BiquadCoeffs = BiquadCoeffsT<float>;
using BiquadState  = BiquadStateT<float>;

namespace detail {

constexpr double kPi     = 3.14159265358979323846;
constexpr double kPiHalf = 1.57079632679489661923;

/* Design-time domain guards. Nothing upstream enforces these — the panel
 * ranges happen to stay inside them today, but a shelf at f_hz >= fs/2 is
 * undefined, and a sample-rate change or a widened knob range would walk
 * straight into it. */
inline double ClampFreq(double f_hz, double fs)
{
    const double hi = 0.4999 * fs;
    return f_hz < 1.0 ? 1.0 : (f_hz > hi ? hi : f_hz);
}
inline double ClampQ(double q)
{
    return q < 0.1 ? 0.1 : (q > 40.0 ? 40.0 : q);
}
inline double ClampGainDb(double db)
{
    return db < -24.0 ? -24.0 : (db > 24.0 ? 24.0 : db);
}

/** sqrt with the argument floored at zero — the solved intermediates can dip
 *  a hair negative from roundoff at the domain edges. */
inline double SqrtNn(double x) { return std::sqrt(x < 0.0 ? 0.0 : x); }

template <typename T>
inline BiquadCoeffsT<T> Pack(double b0, double b1, double b2, double a1, double a2)
{
    return {T(b0), T(b1), T(b2), T(a1), T(a2)};
}

/**
 * Shared core of the two matched Butterworth shelves (2poleShelvingFits
 * Appendix A). A low shelf is a high shelf designed for gain 1/G with its
 * numerator scaled by G, so both call this with the appropriate g and
 * numerator scale.
 *
 * fc is normalized to *Nyquist*, not to fs: fc = f_hz / (fs/2).
 */
template <typename T>
inline BiquadCoeffsT<T> MakeShelf(double fc, double g, bool low_shelf)
{
    const double invg = 1.0 / g;
    const double fc2  = fc * fc;
    const double fc4  = fc2 * fc2;

    // Two magnitude-matching frequencies.
    const double f1   = fc / std::sqrt(0.160 + 1.543 * fc2);
    const double f12  = f1 * f1;
    const double f14  = f12 * f12;
    const double sp1  = std::sin(kPiHalf * f1);
    const double phi1 = sp1 * sp1;

    const double f2   = fc / std::sqrt(0.947 + 3.806 * fc2);
    const double f22  = f2 * f2;
    const double f24  = f22 * f22;
    const double sp2  = std::sin(kPiHalf * f2);
    const double phi2 = sp2 * sp2;

    // Target gains at Nyquist and at the two matching frequencies.
    const double hny = (fc4 + g)       / (fc4 + invg);
    const double h1  = (fc4 + f14 * g) / (fc4 + f14 * invg);
    const double h2  = (fc4 + f24 * g) / (fc4 + f24 * invg);

    // 2x2 solve for the denominator's quadratic-form coefficients.
    //
    // These differences do cancel when g is near 1, and each has a
    // cancellation-free closed form. Measured, it makes no difference: this
    // solve lands within 1e-10 dB of the ideal shelf across the whole range.
    // What actually limits low-shelf accuracy is rounding the *result* to
    // float32 — a low-frequency shelf nearly cancels its own poles and zeros,
    // so a 6e-8 coefficient perturbation shows up ~1e-2 dB. Hence the double
    // instantiation for the low shelf; see mastering_dsp.cpp.
    const double d1  = (h1 - 1.0) * (1.0 - phi1);
    const double c11 = -phi1 * d1;
    const double c12 = phi1 * phi1 * (hny - h1);
    const double d2  = (h2 - 1.0) * (1.0 - phi2);
    const double c21 = -phi2 * d2;
    const double c22 = phi2 * phi2 * (hny - h2);

    const double det   = c11 * c22 - c12 * c21;
    const double alfa1 = (c22 * d1 - c12 * d2) / det;

    const double aa1 = (d1 - c11 * alfa1) / c12;
    const double bb1 = hny * aa1;
    const double aa2 = 0.25 * (alfa1 - aa1);
    const double bb2 = 0.25 * (alfa1 - bb1);

    // Recover minimum-phase numerator and denominator from their squared
    // endpoint values.
    const double v = 0.5 * (1.0 + SqrtNn(aa1));
    const double w = 0.5 * (1.0 + SqrtNn(bb1));

    const double a0    = 0.5 * (v + SqrtNn(v * v + aa2));
    const double inva0 = 1.0 / a0;

    const double a1 = (1.0 - v) * inva0;
    const double a2 = -0.25 * aa2 * inva0 * inva0;

    // b0 stays unscaled until b2 has consumed it, then takes the same scale.
    const double b0u   = 0.5 * (w + SqrtNn(w * w + bb2));
    const double scale = low_shelf ? invg * inva0 : inva0;

    const double b1 = (1.0 - w) * scale;
    const double b2 = (-0.25 * bb2 / b0u) * scale;
    const double b0 = b0u * scale;

    return Pack<T>(b0, b1, b2, a1, a2);
}

/** Vicanek nudges unity gain off exactly 1 — the solve is degenerate there,
 *  and 1.00001 is 8.7e-5 dB, far below anything measurable. */
inline double NudgeUnity(double gain)
{
    return std::fabs(1.0 - gain) < 1e-6 ? 1.00001 : gain;
}

} // namespace detail

template <typename T = float>
inline BiquadCoeffsT<T> MakeLowShelf(float f_hz, float gain_db, float fs)
{
    using namespace detail;
    const double fc   = ClampFreq(f_hz, fs) / (0.5 * fs);
    const double gain = std::pow(10.0, ClampGainDb(gain_db) / 20.0);
    // A low shelf is a high shelf at 1/G with the numerator scaled back by G.
    return MakeShelf<T>(fc, 1.0 / NudgeUnity(gain), /*low_shelf=*/true);
}

template <typename T = float>
inline BiquadCoeffsT<T> MakeHighShelf(float f_hz, float gain_db, float fs)
{
    using namespace detail;
    const double fc   = ClampFreq(f_hz, fs) / (0.5 * fs);
    const double gain = std::pow(10.0, ClampGainDb(gain_db) / 20.0);
    return MakeShelf<T>(fc, NudgeUnity(gain), /*low_shelf=*/false);
}

template <typename T = float>
inline BiquadCoeffsT<T> MakePeaking(float f_hz, float gain_db, float q, float fs)
{
    using namespace detail;
    const double f  = ClampFreq(f_hz, fs);
    const double Q  = ClampQ(q);
    const double G  = std::pow(10.0, ClampGainDb(gain_db) / 20.0);
    const double w0 = 2.0 * kPi * f / fs;

    // Poles by impulse invariance (BiquadFits §3.2). qd is the prototype's
    // damping ratio, not the Q: the denominator of the constant-Q bell is
    // s^2 + s*w0/(sqrt(G)*Q) + w0^2, so the sqrt(G) belongs here. Leaving it
    // out costs ~5 dB of shape error at +15 dB and is invisible at unity.
    // The cosh branch covers damping > 1 (very low Q at high gain).
    const double qd = 1.0 / (2.0 * Q * std::sqrt(G));
    const double e  = std::exp(-qd * w0);
    const double a1 = qd <= 1.0
        ? -2.0 * e * std::cos (std::sqrt(1.0 - qd * qd) * w0)
        : -2.0 * e * std::cosh(std::sqrt(qd * qd - 1.0) * w0);
    const double a2 = std::exp(-2.0 * qd * w0);

    const double sh   = std::sin(w0 * 0.5);
    const double phi1 = sh * sh;
    const double phi0 = 1.0 - phi1;
    const double phi2 = 4.0 * phi0 * phi1;

    const double A0 = (1.0 + a1 + a2) * (1.0 + a1 + a2);
    const double A1 = (1.0 - a1 + a2) * (1.0 - a1 + a2);
    const double A2 = -4.0 * a2;

    // Match |H| at DC, Nyquist and f0 (BiquadFits eq 44/45).
    const double G2 = G * G;
    const double B0 = A0;
    const double R1 = (A0 * phi0 + A1 * phi1 + A2 * phi2) * G2;
    const double R2 = (-A0 + A1 + 4.0 * (phi0 - phi1) * A2) * G2;

    const double B2 = (R1 - R2 * phi1 - B0) / (4.0 * phi1 * phi1);
    const double B1 = R2 + B0 + 4.0 * (phi1 - phi0) * B2;

    const double sqB0 = SqrtNn(B0);
    const double sqB1 = SqrtNn(B1);
    const double W    = 0.5 * (sqB0 + sqB1);

    const double b0 = 0.5 * (W + SqrtNn(W * W + B2));
    const double b1 = 0.5 * (sqB0 - sqB1);
    const double b2 = -B2 / (4.0 * b0);

    return Pack<T>(b0, b1, b2, a1, a2);
}

/**
 * Matched second-order high-pass. Not an EQ band — this exists for the
 * compressor's sidechain, where the question "how much bass reaches the
 * detector" makes the shape *at and just below the corner* the specification.
 *
 * Poles come from the same impulse-invariance construction MakePeaking uses
 * (BiquadFits §3.2), with G = 1 so the damping is the plain 1/(2Q); the cosh
 * branch covers Q < 0.5, which ClampQ still admits.
 *
 * The numerator is pinned as a double zero at z = 1, so the response is
 * *exactly* zero at DC rather than merely small — for a sidechain filter that
 * is the property that matters, and it is what the bilinear transform gives
 * you here as well.
 *
 * That leaves one free scalar, matched at the corner itself. The analog
 * prototype
 *
 *     H(s) = s^2 / (s^2 + s*w0/Q + w0^2)
 *
 * evaluated at s = jw0 is exactly jQ, so |H(jw0)| = Q — for Butterworth
 * Q = 1/sqrt(2) that is the familiar -3.01 dB. Matching it:
 *
 *     |1 - e^{-jw0}|^2 = 4 sin^2(w0/2)
 *     b0 = Q * |D(e^{jw0})| / (4 sin^2(w0/2)),   b1 = -2*b0,   b2 = b0
 *
 * Exact at DC, exact at f0, correct 12 dB/oct asymptote. Nyquist is the one
 * point *not* matched, and at the corners a sidechain uses it costs nothing
 * measurable: worst deviation from the prototype over 20 Hz – 20 kHz is below
 * 1e-5 dB at f0 <= 500 Hz. It only becomes visible when the corner climbs into
 * the audio band — 0.003 dB at 5 kHz, 0.047 dB at 10 kHz — so if this ever
 * gets reused as a program high-pass rather than a detector one, re-measure
 * before trusting it.
 *
 * Re(D) at a 30 Hz corner is a difference of three near-unity terms and loses
 * roughly six digits, which is why this — like every other design in this file
 * — solves in double and stores at T.
 */
template <typename T = float>
inline BiquadCoeffsT<T> MakeHighPass(float f_hz, float q, float fs)
{
    using namespace detail;
    const double f  = ClampFreq(f_hz, fs);
    const double Q  = ClampQ(q);
    const double w0 = 2.0 * kPi * f / fs;

    const double qd = 1.0 / (2.0 * Q);
    const double e  = std::exp(-qd * w0);
    const double a1 = qd <= 1.0
        ? -2.0 * e * std::cos (std::sqrt(1.0 - qd * qd) * w0)
        : -2.0 * e * std::cosh(std::sqrt(qd * qd - 1.0) * w0);
    const double a2 = std::exp(-2.0 * qd * w0);

    // |D(e^{jw0})|, with e^{-jw} = cos w - i sin w.
    const double c1 = std::cos(w0),      s1 = std::sin(w0);
    const double c2 = std::cos(2.0 * w0), s2 = std::sin(2.0 * w0);
    const double dre = 1.0 + a1 * c1 + a2 * c2;
    const double dim = -(a1 * s1 + a2 * s2);
    const double dmag = std::sqrt(dre * dre + dim * dim);

    const double sh = std::sin(0.5 * w0);
    const double b0 = Q * dmag / (4.0 * sh * sh);

    return Pack<T>(b0, -2.0 * b0, b0, a1, a2);
}

} // namespace mastering_dsp
