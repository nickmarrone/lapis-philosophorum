# =============================================================================
# tests/Makefile — host-side DSP measurement harness.
#
# Deliberately standalone: no cross-toolchain, no libDaisy, no alchemy-sdk.
# dsp_biquad.h is <cmath>-only by invariant, and mastering_dsp.cpp needs only
# the AudioHandle typedefs, which tests/stubs/daisy_seed.h supplies.
#
#   make          build and run both harnesses
#   make golden   regenerate the golden coefficient CSV
#   make clean
# =============================================================================

CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wshadow
INCLUDES := -I../src -Istubs

BUILD    := build
RESPONSE := $(BUILD)/eq_response_test
CONTROL  := $(BUILD)/eq_control_test

COMMON   := eq_test_common.h ../src/dsp_biquad.h
DSP_SRC  := ../src/mastering_dsp.cpp
DSP_HDRS := ../src/mastering_dsp.h ../src/dsp_common.h ../src/dsp_compressor.h \
            ../src/dsp_limiter.h ../src/dsp_saturation.h ../src/dsp_dither.h

.PHONY: all run golden clean

all: run

$(BUILD):
	@mkdir -p $(BUILD)

$(RESPONSE): eq_response_test.cpp $(COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(CONTROL): eq_control_test.cpp $(COMMON) $(DSP_SRC) $(DSP_HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(DSP_SRC) -o $@

run: $(RESPONSE) $(CONTROL)
	@./$(RESPONSE)
	@./$(CONTROL)

golden: $(RESPONSE)
	@mkdir -p golden
	@./$(RESPONSE) --golden

clean:
	@rm -rf $(BUILD)
