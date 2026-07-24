/**
 * daisy_seed.h (host test stub)
 *
 * The absolute minimum needed to compile mastering_dsp.cpp on a host: just
 * the AudioHandle buffer typedefs its Process() signature names. Copied from
 * lib/alchemy-sdk/stubs/daisy_seed.h so the test tree doesn't take a
 * dependency on a submodule's 4.9 KB of unrelated hardware surface.
 *
 * dsp_biquad.h itself needs none of this — it is <cmath>-only by invariant.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace daisy {

struct AudioHandle {
    using InputBuffer   = const float* const*;
    using OutputBuffer  = float**;
    using AudioCallback = void (*)(InputBuffer, OutputBuffer, size_t);
};

} // namespace daisy
