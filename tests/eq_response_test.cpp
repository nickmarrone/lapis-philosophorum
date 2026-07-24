/**
 * eq_response_test.cpp — measurement harness for the mastering EQ.
 *
 * Proves (or disproves) four separate things:
 *
 *   1. That the *coefficients* the Make* functions produce realize the analog
 *      prototype they claim to. This is where bilinear cramping shows up.
 *   2. That the *implementation* (BiquadProcess) realizes those coefficients —
 *      checked by DFT of the actual impulse response, not by trusting (1).
 *   3. That every coefficient set in the parameter grid is stable, minimum
 *      phase, and finite, and stays that way under interpolation.
 *   4. That the arithmetic precision is adequate at the low end, measured as
 *      noise the filter itself adds.
 *
 *   ./eq_response_test            run all checks
 *   ./eq_response_test --golden   regenerate golden/eq_coeffs.csv
 */

#include <cstring>
#include <cstdlib>
#include <random>

#include "eq_test_common.h"

using namespace eqtest;

/** Coefficients as returned by the design solve — always full precision here;
 *  BandSpec::AsShipped() applies each band's real storage precision. */
using Coeffs = mastering_dsp::BiquadCoeffsT<double>;

/* ── Parameter grid ───────────────────────────────────────────────────── */

static const double kGains[] = {-15, -12, -6, -3, -0.5, 0, 0.5, 3, 6, 12, 15};
static const double kMidQ[]  = {0.707, 1.5, 4.0};   // mirrors kMidQTable in mastering.cpp

/** Band min, geometric mid, band max — mirrors the VirtualKnob Exp() ranges. */
static const double kLsFreqs[] = {20.0, 126.491, 800.0};
static const double kMidFreqs[] = {200.0, 1000.0, 5000.0};
static const double kHsFreqs[] = {1000.0, 4472.136, 20000.0};

static std::vector<BandSpec> Grid()
{
    std::vector<BandSpec> g;
    for (double f : kLsFreqs)
        for (double d : kGains) g.push_back({Band::LowShelf, f, d, 0.0});
    for (double f : kMidFreqs)
        for (double d : kGains)
            for (double q : kMidQ) g.push_back({Band::Peaking, f, d, q});
    for (double f : kHsFreqs)
        for (double d : kGains) g.push_back({Band::HighShelf, f, d, 0.0});
    return g;
}

/** Narrow a full-precision design to the working type the firmware uses. */
template <typename T>
static mastering_dsp::BiquadCoeffsT<T> Narrow(const Coeffs& c)
{
    return {T(c.b0), T(c.b1), T(c.b2), T(c.a1), T(c.a2)};
}

/** Impulse response of the *shipped* code path at the given precision. */
template <typename T>
static void ImpulseResponse(const Coeffs& design, std::vector<float>& ir)
{
    const auto c = Narrow<T>(design);
    mastering_dsp::BiquadStateT<T> st{};
    for (size_t n = 0; n < ir.size(); n++)
        ir[n] = mastering_dsp::BiquadProcess(st, c, n == 0 ? 1.f : 0.f);
}

/* ── Test 1: decramping, the headline number ──────────────────────────── */

static void TestDecramping(Report& r)
{
    r.Section("Decramping — realized response vs analog prototype");

    const BandSpec headline{Band::HighShelf, 20000.0, 15.0, 0.0};
    const Deviation d = MeasureDeviation(headline);
    r.Check(d.max_db <= 1.0,
            "HS @ 20 kHz, +15 dB  (<= 1.0 dB)",
            Fmt("max %6.2f dB @ %8.1f Hz", d.max_db, d.at_hz));

    // The same filter cut, and the top of the mid band — the other places the
    // bilinear transform runs out of room.
    for (const BandSpec s : {BandSpec{Band::HighShelf, 20000.0, -15.0, 0.0},
                             BandSpec{Band::HighShelf, 4472.136, 15.0, 0.0},
                             BandSpec{Band::Peaking, 5000.0, 15.0, 4.0}})
    {
        const Deviation dd = MeasureDeviation(s);
        r.Check(dd.max_db <= 1.0, s.Label(),
                Fmt("max %6.2f dB @ %8.1f Hz", dd.max_db, dd.at_hz));
    }
}

/* ── Test 2: whole-grid magnitude accuracy ────────────────────────────── */

static void TestGridAccuracy(Report& r)
{
    r.Section("Grid accuracy — every band/freq/gain/Q combination");

    double worst_small = 0.0, worst_large = 0.0;
    BandSpec worst_small_spec{}, worst_large_spec{};

    for (const BandSpec& s : Grid())
    {
        const Deviation d = MeasureDeviation(s);
        if (std::fabs(s.gain_db) <= 12.0)
        {
            if (d.max_db > worst_small) { worst_small = d.max_db; worst_small_spec = s; }
        }
        else if (d.max_db > worst_large) { worst_large = d.max_db; worst_large_spec = s; }
    }

    r.Check(worst_small <= 0.5, "|gain| <= 12 dB  (<= 0.5 dB)",
            Fmt("worst %6.2f dB", worst_small));
    r.Info("  worst case", worst_small_spec.Label());
    r.Check(worst_large <= 1.0, "|gain| == 15 dB  (<= 1.0 dB)",
            Fmt("worst %6.2f dB", worst_large));
    r.Info("  worst case", worst_large_spec.Label());
}

/* ── Test 3: flat is flat ─────────────────────────────────────────────── */

static void TestFlat(Report& r)
{
    r.Section("Unity gain — all three bands at exactly 0 dB");

    // Cascade all three at 0 dB and confirm the product is flat. Catches
    // degenerate-gain handling (the g = 1.00001 nudge, or a bad short-circuit).
    // Reported per band as well, so a regression here names its own culprit.
    const BandSpec specs[3] = {
        {Band::LowShelf,  126.491,  0.0, 0.0},
        {Band::Peaking,  1000.0,    0.0, 0.707},
        {Band::HighShelf, 4472.136, 0.0, 0.0},
    };
    const Coeffs ls  = specs[0].AsShipped();
    const Coeffs mid = specs[1].AsShipped();
    const Coeffs hs  = specs[2].AsShipped();

    struct { const char* name; const Coeffs* c; } bands[] = {
        {"low shelf",  &ls}, {"peaking", &mid}, {"high shelf", &hs},
    };
    for (const auto& b : bands)
    {
        double m = 0.0, p = 0.0;
        for (double f : SweepFreqs())
        {
            const cplx h = ResponseAt(*b.c, f, kFs);
            m = std::max(m, std::fabs(LinToDb(std::abs(h))));
            p = std::max(p, std::fabs(std::arg(h)) * 180.0 / kPi);
        }
        r.Info(std::string("  ") + b.name, Fmt("%.3e dB   %.3e deg", m, p));
    }

    double max_db = 0.0, max_deg = 0.0;
    for (double f : SweepFreqs())
    {
        const cplx h = ResponseAt(ls, f, kFs) * ResponseAt(mid, f, kFs) * ResponseAt(hs, f, kFs);
        max_db  = std::max(max_db, std::fabs(LinToDb(std::abs(h))));
        max_deg = std::max(max_deg, std::fabs(std::arg(h)) * 180.0 / kPi);
    }
    // Threshold is 0.005 dB: ~1/200th of a dB, three orders of magnitude below
    // audibility, but tight enough to catch a broken degenerate-gain path.
    r.Check(max_db  <= 0.005, "cascade magnitude flat  (<= 0.005 dB)", Fmt("max %.3e dB", max_db));
    r.Check(max_deg <= 0.05,  "cascade phase flat      (<= 0.05 deg)", Fmt("max %.3e deg", max_deg));
}

/* ── Test 4: stability, min phase, finiteness over the grid ───────────── */

static void TestCoeffSanity(Report& r)
{
    r.Section("Coefficient sanity over the grid");

    int unstable = 0, nonmin = 0, nonfinite = 0;
    double worst_margin = 1.0;
    BandSpec worst_spec{};

    for (const BandSpec& s : Grid())
    {
        const Coeffs c = s.Design();
        if (!IsFinite(c)) { nonfinite++; continue; }
        if (!IsStable(c))   unstable++;
        if (!IsMinPhase(c)) nonmin++;
        const double m = PoleMargin(c);
        if (m < worst_margin) { worst_margin = m; worst_spec = s; }
    }

    r.Check(nonfinite == 0, "all coefficients finite",   Fmt("%.0f non-finite", double(nonfinite)));
    r.Check(unstable  == 0, "all inside stability triangle", Fmt("%.0f unstable", double(unstable)));
    r.Check(nonmin    == 0, "all minimum phase (eq 28)", Fmt("%.0f violations", double(nonmin)));
    r.Check(worst_margin > 1e-4, "pole margin  (> 1e-4)", Fmt("min %.3e", worst_margin));
    r.Info("  closest to unit circle", worst_spec.Label());
}

/* ── Test 5: impulse response — does the code realize the coefficients? ── */

static void TestImpulseResponse(Report& r)
{
    r.Section("Impulse response — implementation vs coefficients");

    constexpr int    kN = 1 << 16;      // 1.37 s at 48 kHz; every band fully decays
    constexpr double kTol = 0.05;       // dB; slack for IR truncation

    const BandSpec probes[] = {
        {Band::LowShelf,   20.0,     15.0, 0.0},
        {Band::LowShelf,  800.0,    -15.0, 0.0},
        {Band::Peaking,  1000.0,     15.0, 4.0},
        {Band::Peaking,   200.0,    -15.0, 0.707},
        {Band::HighShelf, 20000.0,   15.0, 0.0},
        {Band::HighShelf,  1000.0,  -15.0, 0.0},
    };

    std::vector<float> ir(static_cast<size_t>(kN));
    double worst = 0.0;

    for (const BandSpec& s : probes)
    {
        // Drive the real shipped code path, at the real shipped precision.
        const Coeffs design = s.Design();
        if (s.ShipsInDouble()) ImpulseResponse<double>(design, ir);
        else                   ImpulseResponse<float>(design, ir);
        const Coeffs c = s.AsShipped();

        // DFT the IR at 128 log-spaced points and compare to the analytic form.
        double band_worst = 0.0;
        for (int k = 0; k < 128; k++)
        {
            const double f = 20.0 * std::pow(1000.0, k / 127.0);
            const double w = 2.0 * kPi * f / kFs;
            double re = 0.0, im = 0.0;
            for (int n = 0; n < kN; n++)
            {
                const double v = ir[size_t(n)];
                if (v == 0.0) continue;
                re += v * std::cos(w * n);
                im -= v * std::sin(w * n);
            }
            const double meas_db = LinToDb(std::hypot(re, im));
            const double calc_db = LinToDb(std::abs(ResponseAt(c, f, kFs)));
            band_worst = std::max(band_worst, std::fabs(meas_db - calc_db));
        }
        worst = std::max(worst, band_worst);
        r.Check(band_worst <= kTol, s.Label(), Fmt("max %.4f dB", band_worst));
    }
    r.Info("worst across probes", Fmt("%.4f dB", worst));
}

/* ── Test 6: interpolation stability ──────────────────────────────────── */

static void TestInterpolationStability(Report& r)
{
    r.Section("Coefficient interpolation stability");

    // The stability region |a1| < 2, |a1|-1 < a2 < 1 is a triangle, i.e. convex,
    // so a convex combination of two stable biquads is stable. This turns that
    // argument into an assertion over real designed coefficient pairs.
    const std::vector<BandSpec> g = Grid();
    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<size_t> pick(0, g.size() - 1);

    int escapes = 0;
    constexpr int kPairs = 10000, kSteps = 65;

    for (int i = 0; i < kPairs; i++)
    {
        const Coeffs a = g[pick(rng)].AsShipped();
        const Coeffs b = g[pick(rng)].AsShipped();
        for (int k = 0; k <= kSteps; k++)
        {
            const double t = double(k) / kSteps;
            Coeffs c;
            c.b0 = a.b0 + t * (b.b0 - a.b0);
            c.b1 = a.b1 + t * (b.b1 - a.b1);
            c.b2 = a.b2 + t * (b.b2 - a.b2);
            c.a1 = a.a1 + t * (b.a1 - a.a1);
            c.a2 = a.a2 + t * (b.a2 - a.a2);
            if (!IsStable(c) || !IsFinite(c)) escapes++;
        }
    }
    r.Check(escapes == 0, Fmt("%.0f pairs x 65 steps stay stable", double(kPairs)),
            Fmt("%.0f escapes", double(escapes)));
}

/* ── Test 7: noise floor — float vs double state on the low shelf ─────── */

template <typename T>
static double MeasureFilterNoise(const Coeffs& design, double sig_hz, double sig_db)
{
    constexpr int kN = 1 << 20;
    constexpr int kSkip = 4096;             // let the filter settle
    const double amp = DbToLin(sig_db);
    const double w   = 2.0 * kPi * sig_hz / kFs;

    const auto c = Narrow<T>(design);
    mastering_dsp::BiquadStateT<T> st{};
    std::vector<double> y(static_cast<size_t>(kN));
    for (int n = 0; n < kN; n++)
        y[size_t(n)] = double(mastering_dsp::BiquadProcess(st, c, float(amp * std::sin(w * n))));

    // Least-squares fit of the fundamental, then subtract it. The filter is
    // linear, so whatever remains is arithmetic roundoff and nothing else.
    double sc = 0.0, ss = 0.0, cc = 0.0, s2 = 0.0;
    for (int n = kSkip; n < kN; n++)
    {
        const double C = std::cos(w * n), S = std::sin(w * n);
        sc += y[size_t(n)] * C;  ss += y[size_t(n)] * S;
        cc += C * C;             s2 += S * S;
    }
    const double A = sc / cc, B = ss / s2;

    double acc = 0.0;
    for (int n = kSkip; n < kN; n++)
    {
        const double res = y[size_t(n)] - (A * std::cos(w * n) + B * std::sin(w * n));
        acc += res * res;
    }
    return LinToDb(std::sqrt(acc / (kN - kSkip)));
}

static void TestNoiseFloor(Report& r)
{
    r.Section("Noise floor — arithmetic precision on the low shelf");

    // 20 Hz / +15 dB is the worst-conditioned point in the whole EQ: poles at
    // radius ~0.998, and DF-I roundoff is shaped by 1/A(z) with no zero
    // cancellation.
    const Coeffs c = BandSpec{Band::LowShelf, 20.0, 15.0, 0.0}.Design();

    const double n_flt = MeasureFilterNoise<float>(c, 1000.0, -20.0);
    const double n_dbl = MeasureFilterNoise<double>(c, 1000.0, -20.0);

    r.Info("float32 state", Fmt("%8.2f dBFS", n_flt));
    r.Info("float64 state", Fmt("%8.2f dBFS", n_dbl));
    r.Check(n_flt - n_dbl >= 20.0, "double at least 20 dB quieter",
            Fmt("delta %.1f dB", n_flt - n_dbl));

    // And the response error the same precision difference causes at 20 Hz.
    const double mag_ref = std::abs(ProtoLowShelf(20.0, 20.0, DbToLin(15.0)));
    const double mag_dig = std::abs(ResponseAt(Narrow<float>(c), 20.0, kFs));
    const double mag_d64 = std::abs(ResponseAt(c, 20.0, kFs));
    r.Info("LS magnitude error at 20 Hz, float64 coeffs",
           Fmt("%.6f dB", std::fabs(LinToDb(mag_d64) - LinToDb(mag_ref))));
    r.Info("LS magnitude error at 20 Hz, float32 coeffs",
           Fmt("%.6f dB", std::fabs(LinToDb(mag_dig) - LinToDb(mag_ref))));
}

/* ── Test 8: modulation stress ────────────────────────────────────────── */

static void TestModulationStress(Report& r)
{
    r.Section("Modulation stress — per-block interpolation under full-scale noise");

    // Models exactly what Step 8 does in Process(): re-design at the 16 ms
    // control frame, lerp the live coefficients toward that target once per
    // 24-sample block.
    constexpr int    kBlock = 24;
    constexpr int    kSecs  = 1;
    constexpr int    kN     = int(kFs) * kSecs;
    constexpr double kLerp  = 0.15;
    const int        kFrameBlocks = int(0.016 * kFs / kBlock);   // ~32 blocks

    std::mt19937 rng(0x5EEDu);
    std::normal_distribution<double> noise(0.0, 0.3);

    auto live = mastering_dsp::MakeLowShelf<double>(20.f, -15.f, float(kFs));
    auto tgt  = live;
    mastering_dsp::BiquadStateT<double> st{};

    double peak = 0.0;
    bool   finite = true;
    int    block = 0;

    for (int n = 0; n < kN; n += kBlock)
    {
        if (block % kFrameBlocks == 0)
        {
            // Sweep freq 20 -> 800 Hz and gain -15 -> +15 dB across the second.
            const double u = double(n) / kN;
            tgt = mastering_dsp::MakeLowShelf<double>(float(20.0 * std::pow(40.0, u)),
                                                      float(-15.0 + 30.0 * u), float(kFs));
        }
        block++;

        live.b0 += kLerp * (tgt.b0 - live.b0);
        live.b1 += kLerp * (tgt.b1 - live.b1);
        live.b2 += kLerp * (tgt.b2 - live.b2);
        live.a1 += kLerp * (tgt.a1 - live.a1);
        live.a2 += kLerp * (tgt.a2 - live.a2);

        if (!IsStable(live)) finite = false;

        for (int i = 0; i < kBlock && n + i < kN; i++)
        {
            const float y = mastering_dsp::BiquadProcess(st, live, float(noise(rng)));
            if (!std::isfinite(y)) finite = false;
            peak = std::max(peak, double(std::fabs(y)));
        }
    }

    r.Check(finite, "all samples finite, filter stayed stable");
    r.Check(peak <= DbToLin(6.0), "peak <= +6 dBFS", Fmt("peak %+.2f dBFS", LinToDb(peak)));
}

/* ── Test 9: golden coefficient regression ────────────────────────────── */

static const char* kGoldenPath = "golden/eq_coeffs.csv";

static void WriteGolden()
{
    std::FILE* f = std::fopen(kGoldenPath, "w");
    if (!f) { std::perror(kGoldenPath); std::exit(1); }
    std::fprintf(f, "band,f_hz,gain_db,q,b0,b1,b2,a1,a2\n");
    for (const BandSpec& s : Grid())
    {
        const Coeffs c = s.Design();
        std::fprintf(f, "%s,%.6f,%.6f,%.6f,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                     BandName(s.band), s.f_hz, s.gain_db, s.q,
                     c.b0, c.b1, c.b2, c.a1, c.a2);
    }
    std::fclose(f);
    std::printf("wrote %s (%zu rows)\n", kGoldenPath, Grid().size());
}

static void TestGolden(Report& r)
{
    r.Section("Golden coefficient regression");

    std::FILE* f = std::fopen(kGoldenPath, "r");
    if (!f)
    {
        r.Info("no golden file", "run `make golden` to create it");
        return;
    }

    char line[512];
    if (!std::fgets(line, sizeof line, f)) { std::fclose(f); return; }   // header

    const std::vector<BandSpec> g = Grid();
    size_t row = 0;
    int    diffs = 0;
    double worst = 0.0;

    while (std::fgets(line, sizeof line, f) && row < g.size())
    {
        char band[8];
        double fh, gd, q, b0, b1, b2, a1, a2;
        if (std::sscanf(line, "%7[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        band, &fh, &gd, &q, &b0, &b1, &b2, &a1, &a2) != 9) break;

        const Coeffs c = g[row].Design();
        const double have[5] = {c.b0, c.b1, c.b2, c.a1, c.a2};
        const double want[5] = {b0, b1, b2, a1, a2};
        for (int k = 0; k < 5; k++)
        {
            const double denom = std::max(std::fabs(want[k]), 1e-9);
            const double rel   = std::fabs(have[k] - want[k]) / denom;
            if (rel > worst) worst = rel;
            if (rel > 1e-6) diffs++;
        }
        row++;
    }
    std::fclose(f);

    r.Check(row == g.size(), "row count matches grid",
            Fmt("%.0f of %.0f", double(row), double(g.size())));
    r.Check(diffs == 0, "coefficients unchanged  (1e-6 relative)",
            Fmt("%.0f differ, worst %.2e", double(diffs), worst));
}

/* ── main ─────────────────────────────────────────────────────────────── */

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--golden") == 0) { WriteGolden(); return 0; }

    std::printf("\n\033[1mEQ measurement harness\033[0m  —  fs = %.0f Hz, "
                "sweep 20 Hz–20 kHz @ 1/96 oct (%zu points), grid = %zu designs\n",
                kFs, SweepFreqs().size(), Grid().size());

    Report r;
    TestDecramping(r);
    TestGridAccuracy(r);
    TestFlat(r);
    TestCoeffSanity(r);
    TestImpulseResponse(r);
    TestInterpolationStability(r);
    TestNoiseFloor(r);
    TestModulationStress(r);
    TestGolden(r);
    return r.Finish();
}
