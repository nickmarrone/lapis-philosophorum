/**
 * test_report.h — the tiny assertion/reporting framework shared by every
 * host-side DSP harness.
 *
 * Extracted verbatim from eq_test_common.h when the compressor harness
 * arrived and needed the same thing. It deliberately knows nothing about
 * biquads, compressors, or dsp_common.h — it is <cstdio>/<string> only, so
 * any harness can include it without dragging a stage header along.
 *
 * The design point worth preserving: Check() prints the measured number on
 * PASS as well as on FAIL. A harness that only speaks up when it breaks tells
 * you nothing about how close to breaking it was.
 */

#pragma once

#include <cstdio>
#include <string>

namespace testrep {

struct Report {
    int failures = 0;
    int checks   = 0;

    void Section(const char* name)
    {
        std::printf("\n\033[1m── %s ", name);
        for (size_t i = std::string(name).size(); i < 66; i++) std::printf("─");
        std::printf("\033[0m\n");
    }

    void Check(bool ok, const std::string& what, const std::string& detail = {})
    {
        checks++;
        if (ok)
        {
            std::printf("  \033[32mPASS\033[0m  %-46s %s\n", what.c_str(), detail.c_str());
        }
        else
        {
            failures++;
            std::printf("  \033[31mFAIL\033[0m  %-46s %s\n", what.c_str(), detail.c_str());
        }
    }

    void Info(const std::string& what, const std::string& detail = {})
    {
        std::printf("        %-46s %s\n", what.c_str(), detail.c_str());
    }

    int Finish()
    {
        std::printf("\n%s%d/%d checks passed\033[0m\n\n",
                    failures ? "\033[31m" : "\033[32m", checks - failures, checks);
        return failures ? 1 : 0;
    }
};

inline std::string Fmt(const char* f, double a)
{
    char buf[128];
    std::snprintf(buf, sizeof buf, f, a);
    return buf;
}

inline std::string Fmt(const char* f, double a, double b)
{
    char buf[128];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}

} // namespace testrep
