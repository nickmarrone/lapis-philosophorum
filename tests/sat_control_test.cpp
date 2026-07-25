/**
 * sat_control_test.cpp — the saturator as the firmware actually drives it.
 *
 * Everything here is about *cadence*, not about the shaper. The control loop
 * calls SetSat unconditionally every 16 ms frame with whatever the pots read,
 * and Process runs in 24-sample blocks. A stage can be perfectly correct
 * sample-by-sample and still be broken by how often it is reconfigured — which
 * is not a hypothetical:
 *
 *   The previous saturator reset its ADAA history inside Configure(). At the
 *   60 Hz control frame that put one wrong sample into the output 62.5 times a
 *   second — measured at 31-35 dB below program on tone, noise and a 3-tone
 *   mix. It was invisible to a response harness because a response harness
 *   configures once and then measures.
 *
 * TestReconfigureArtifact is that regression. It reconfigures at the real
 * cadence and compares against a run configured once; anything the control
 * cadence adds shows up as a difference signal.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sat_test_common.h"

using namespace sattest;
using mastering_dsp::SatParams;

namespace {

constexpr int kFrame = 768;   // 16 ms control frame at 48 kHz
constexpr int kBlock = 24;    // audio block

/**
 * Run the stage the way the firmware does: BeginBlock every 24 samples,
 * Configure every 768. When `reconfigure` is false, Configure is called once at
 * the start instead — the reference the cadence is compared against.
 */
std::vector<float> RunDriven(SatParams p, const std::vector<float>& in,
                             bool reconfigure)
{
    mastering_dsp::Saturator sat;
    sat.Init(float(kFs));
    for (int i = 0; i < 8; i++) sat.Configure(p, float(kFs));  // settle the glide

    std::vector<float> out(in.size());
    for (size_t n = 0; n < in.size(); n++)
    {
        if (reconfigure && n % kFrame == 0) sat.Configure(p, float(kFs));
        if (n % kBlock == 0) sat.BeginBlock();
        float l = in[n], r = l;
        sat.ProcessSample(l, r);
        out[n] = l;
    }
    return out;
}

/* ── The regression ──────────────────────────────────────────────────────── */

void TestReconfigureArtifact(Report& r)
{
    r.Section("Control cadence adds nothing — the 62.5 Hz regression");

    struct Case { const char* name; std::vector<float> sig; };
    std::vector<Case> cases;
    // 997 Hz, not 1000: 768 samples is exactly 16 cycles of 1 kHz, so a
    // frame-rate artifact would land on the same phase every time and a tone at
    // 1 kHz would flatter it. 997 is incommensurate, which is why it is the
    // standard test frequency.
    cases.push_back({"997 Hz tone", Sine(997.0, 0.126, 48000)});
    cases.push_back({"white noise",  Noise(0.1, 48000)});
    {
        std::vector<float> m(48000);
        for (int n = 0; n < 48000; n++)
        {
            const double t = double(n) / kFs;
            m[size_t(n)] = float(0.05 * (std::sin(2.0 * kPi * 110.0 * t)
                                       + std::sin(2.0 * kPi * 997.0 * t)
                                       + std::sin(2.0 * kPi * 3313.0 * t)));
        }
        cases.push_back({"3-tone mix", m});
    }

    auto p = MakeParams();
    p.drive_db = 12.f;
    p.emphasis = 0.5f;
    p.bump     = 0.5f;

    for (auto& c : cases)
    {
        const auto driven = RunDriven(p, c.sig, true);
        const auto once   = RunDriven(p, c.sig, false);

        double se = 0.0, ss = 0.0, peak = 0.0;
        for (size_t n = 4800; n < c.sig.size(); n++)
        {
            const double d = double(driven[n]) - once[n];
            se += d * d;
            ss += double(once[n]) * once[n];
            peak = std::max(peak, std::fabs(d));
        }
        const double snr = 10.0 * std::log10(ss / (se + 1e-30));
        r.Check(snr > 120.0,
                std::string(c.name) + ": reconfiguring changes nothing audible",
                Fmt("%.1f dB below program, peak sample error %.2e", snr, peak));
    }

    // And directly, in the frequency domain: nothing at the frame rate or its
    // harmonics.
    //
    // Measured on the DIFFERENCE signal, not on the output. The output does
    // carry roughly -115 dBFS at 62.5 Hz, but so does a run configured once —
    // it is Hann leakage from the 997 Hz tone and the DC blocker's tail, not
    // anything the control loop did. Differencing removes everything common to
    // both runs and leaves exactly the quantity under test: what reconfiguring
    // at 60 Hz adds. On the old saturator this read about -78 dBFS.
    {
        const auto sig    = Sine(997.0, 0.126, 48000);
        const auto driven = RunDriven(p, sig, true);
        const auto once   = RunDriven(p, sig, false);

        std::vector<float> diff(sig.size());
        for (size_t n = 0; n < sig.size(); n++) diff[n] = driven[n] - once[n];

        double worst = -300.0, worst_f = 0.0;
        for (int h = 1; h <= 8; h++)
        {
            const double f  = kFs / kFrame * h;     // 62.5 Hz and up
            const double db = BinDb(diff, f, 4800);
            if (db > worst) { worst = db; worst_f = f; }
        }
        r.Check(worst < -140.0, "the control cadence adds no partial at 62.5 Hz",
                Fmt("worst %.1f dBFS at %.1f Hz", worst, worst_f));
    }
}

/* ── Knob moves ──────────────────────────────────────────────────────────── */

/**
 * The emphasis shelf and head bump are redesigned from a moving pot, so this is
 * the stage's zipper test — the same hazard SetEq's dirty check exists for.
 */
void TestKnobSweepIsQuiet(Report& r)
{
    r.Section("Moving knobs do not zipper");

    auto p = MakeParams();
    p.drive_db = 12.f;

    const auto in = Sine(997.0, 0.126, 96000);

    // Sweep Emphasis and Bump across their whole travel over two seconds.
    mastering_dsp::Saturator sat;
    sat.Init(float(kFs));
    for (int i = 0; i < 8; i++) sat.Configure(p, float(kFs));

    std::vector<float> out(in.size());
    for (size_t n = 0; n < in.size(); n++)
    {
        if (n % kFrame == 0)
        {
            const float t = float(n) / float(in.size());
            p.emphasis = t;
            p.bump     = t;
            sat.Configure(p, float(kFs));
        }
        if (n % kBlock == 0) sat.BeginBlock();
        float l = in[n], r2 = l;
        sat.ProcessSample(l, r2);
        out[n] = l;
    }

    // A zipper is broadband sidebands around the tone. Read a few bins that a
    // clean sweep leaves empty; the tone's own harmonics are avoided.
    double worst = -300.0, worst_f = 0.0;
    for (double f : {150.0, 450.0, 2500.0, 4500.0, 7500.0, 11500.0})
    {
        const double db = BinDb(out, f, 4800);
        if (db > worst) { worst = db; worst_f = f; }
    }
    r.Check(worst < -90.0, "a full Emphasis+Bump sweep stays clean",
            Fmt("worst off-harmonic bin %.1f dBFS at %.0f Hz", worst, worst_f));

    // NEGATIVE CONTROL — the same sweep with the coefficient easing defeated by
    // jumping the knob in big steps, which is what a missing glide looks like.
    {
        mastering_dsp::Saturator s2;
        s2.Init(float(kFs));
        auto q = MakeParams();
        q.drive_db = 12.f;
        for (int i = 0; i < 8; i++) s2.Configure(q, float(kFs));
        std::vector<float> o2(in.size());
        for (size_t n = 0; n < in.size(); n++)
        {
            if (n % kFrame == 0)
            {
                const float t = float(n) / float(in.size());
                q.emphasis = (t < 0.5f) ? 0.f : 1.f;   // one hard step
                q.bump     = q.emphasis;
                s2.Configure(q, float(kFs));
            }
            if (n % kBlock == 0) s2.BeginBlock();
            float l = in[n], r2 = l;
            s2.ProcessSample(l, r2);
            o2[n] = l;
        }
        double w2 = -300.0;
        for (double f : {150.0, 450.0, 2500.0, 4500.0, 7500.0, 11500.0})
            w2 = std::max(w2, BinDb(o2, f, 4800));
        r.Check(w2 > worst, "negative control — a hard step IS noisier",
                Fmt("%.1f dBFS against %.1f dBFS for the glided sweep", w2, worst));
    }
}

/* ── Character switching ─────────────────────────────────────────────────── */

/**
 * Switching character changes the knee, the emphasis curve and the bump at
 * once. It must not click — and specifically it must not need the ADAA history
 * reset that caused the original bug, because the shaper function is the same
 * tanh for every character.
 */
void TestCharacterSwitch(Report& r)
{
    r.Section("Character switch mid-signal is click-free");

    auto p = MakeParams();
    p.drive_db = 12.f;
    p.emphasis = 0.6f;

    const auto in = Sine(220.0, 0.4, 48000);

    mastering_dsp::Saturator sat;
    sat.Init(float(kFs));
    for (int i = 0; i < 8; i++) sat.Configure(p, float(kFs));

    std::vector<float> out(in.size());
    for (size_t n = 0; n < in.size(); n++)
    {
        if (n % kFrame == 0)
        {
            if (n == size_t(kFrame) * 20) p.character = mastering_dsp::kSatSaturated;
            sat.Configure(p, float(kFs));
        }
        if (n % kBlock == 0) sat.BeginBlock();
        float l = in[n], r2 = l;
        sat.ProcessSample(l, r2);
        out[n] = l;
    }

    const size_t sw = size_t(kFrame) * 20;
    double worst = 0.0, peak = 0.0;
    for (size_t n = sw - 2000; n < sw + 6000; n++)
    {
        worst = std::max(worst, std::fabs(double(out[n]) - out[n - 1]));
        peak  = std::max(peak, std::fabs(double(out[n])));
    }
    const double slew = 0.5 * 2.0 * kPi * 220.0 / kFs;
    r.Check(worst < 3.0 * slew, "no step larger than the tone's own slew",
            Fmt("worst %.2e vs tone slew %.2e", worst, slew));
    r.Check(peak < 0.8, "no overshoot across the switch", Fmt("peak %.4f", peak));
}

/* ── Preset recall ───────────────────────────────────────────────────────── */

void TestPresetRecall(Report& r)
{
    r.Section("Preset recall — one push of new params takes effect");

    auto quiet = MakeParams();
    quiet.drive_db = 0.f;
    auto loud = MakeParams();
    loud.drive_db = 24.f;
    loud.asym     = 0.25f;

    const auto in = Sine(1000.0, 0.3, 24000);

    mastering_dsp::Saturator sat;
    sat.Init(float(kFs));
    for (int i = 0; i < 8; i++) sat.Configure(quiet, float(kFs));

    // A single Configure with the loaded values, then measure once settled.
    sat.Configure(loud, float(kFs));
    std::vector<float> out(in.size());
    for (size_t n = 0; n < in.size(); n++)
    {
        if (n % kBlock == 0) sat.BeginBlock();
        float l = in[n], r2 = l;
        sat.ProcessSample(l, r2);
        out[n] = l;
    }
    const auto h = Harmonics(out, 1000.0, 3, 8000);
    r.Check(h[2] > -40.0, "a single Configure audibly saturates",
            Fmt("H3 %.1f dB re fundamental", h[2]));
}

} // namespace

int main()
{
    std::printf("\n\033[1mTape saturator control-path harness\033[0m  —  real "
                "SetSat cadence, %d-sample blocks at %.0f Hz\n", kBlock, kFs);

    Report r;
    TestReconfigureArtifact(r);
    TestKnobSweepIsQuiet(r);
    TestCharacterSwitch(r);
    TestPresetRecall(r);
    return r.Finish();
}
