# =============================================================================
# tests/Makefile — host-side DSP measurement harness.
#
# Deliberately standalone: no cross-toolchain, no libDaisy, no alchemy-sdk.
# dsp_biquad.h is <cmath>-only by invariant, and mastering_dsp.cpp needs only
# the AudioHandle typedefs, which tests/stubs/daisy_seed.h supplies.
#
#   make          build and run all six harnesses
#   make golden   regenerate the golden CSVs (EQ coefficients + compressor)
#   make clean
# =============================================================================

CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wshadow
INCLUDES := -I../src -Istubs

BUILD     := build
RESPONSE  := $(BUILD)/eq_response_test
CONTROL   := $(BUILD)/eq_control_test
CRESPONSE := $(BUILD)/comp_response_test
CCONTROL  := $(BUILD)/comp_control_test
SRESPONSE := $(BUILD)/sat_response_test
SCONTROL  := $(BUILD)/sat_control_test

REPORT    := test_report.h
COMMON    := eq_test_common.h $(REPORT) ../src/dsp_biquad.h
CP_COMMON := comp_test_common.h $(COMMON) ../src/dsp_compressor.h \
             ../src/dsp_common.h ../src/mastering_dsp.h
SAT_COMMON := sat_test_common.h $(COMMON) ../src/dsp_saturation.h \
             ../src/dsp_halfband.h ../src/dsp_common.h ../src/mastering_dsp.h
DSP_SRC   := ../src/mastering_dsp.cpp
DSP_HDRS  := ../src/mastering_dsp.h ../src/dsp_common.h ../src/dsp_compressor.h \
             ../src/dsp_limiter.h ../src/dsp_saturation.h ../src/dsp_dither.h \
             ../src/dsp_halfband.h

.PHONY: all run golden clean

all: run

$(BUILD):
	@mkdir -p $(BUILD)

$(RESPONSE): eq_response_test.cpp $(COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(CONTROL): eq_control_test.cpp $(COMMON) $(DSP_SRC) $(DSP_HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(DSP_SRC) -o $@

# comp_response_test drives the Compressor struct alone and links no
# mastering_dsp.cpp — but dsp_compressor.h includes mastering_dsp.h for
# CompParams, which pulls daisy_seed.h, so it still needs -Istubs.
$(CRESPONSE): comp_response_test.cpp $(CP_COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(CCONTROL): comp_control_test.cpp $(CP_COMMON) $(DSP_SRC) $(DSP_HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(DSP_SRC) -o $@

# Both saturator harnesses drive the Saturator struct directly and link no
# mastering_dsp.cpp; the -Istubs is for the daisy_seed.h that mastering_dsp.h
# pulls in for its AudioHandle typedefs.
$(SRESPONSE): sat_response_test.cpp $(SAT_COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(SCONTROL): sat_control_test.cpp $(SAT_COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

run: $(RESPONSE) $(CONTROL) $(CRESPONSE) $(CCONTROL) $(SRESPONSE) $(SCONTROL)
	@./$(RESPONSE)
	@./$(CONTROL)
	@./$(CRESPONSE)
	@./$(CCONTROL)
	@./$(SRESPONSE)
	@./$(SCONTROL)

golden: $(RESPONSE) $(CRESPONSE) $(SRESPONSE)
	@mkdir -p golden
	@./$(RESPONSE) --golden
	@./$(CRESPONSE) --golden
	@./$(SRESPONSE) --golden

clean:
	@rm -rf $(BUILD)
