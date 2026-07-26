# =============================================================================
# tests/Makefile — host-side DSP measurement harness.
#
# Deliberately standalone: no cross-toolchain, no libDaisy, no alchemy-sdk.
# dsp_biquad.h is <cmath>-only by invariant, and mastering_dsp.cpp needs only
# the AudioHandle typedefs, which tests/stubs/daisy_seed.h supplies.
#
#   make          build and run all eight harnesses
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
LRESPONSE := $(BUILD)/lim_response_test
CHAIN     := $(BUILD)/chain_test
MODSRC    := $(BUILD)/mod_source_test

REPORT    := test_report.h
COMMON    := eq_test_common.h $(REPORT) ../src/dsp_biquad.h
CP_COMMON := comp_test_common.h $(COMMON) ../src/dsp_compressor.h \
             ../src/dsp_common.h ../src/mastering_dsp.h
SAT_COMMON := sat_test_common.h $(COMMON) ../src/dsp_saturation.h \
             ../src/dsp_halfband.h ../src/dsp_common.h ../src/mastering_dsp.h
LIM_COMMON := lim_test_common.h $(REPORT) ../src/dsp_limiter.h ../src/dsp_common.h \
              ../src/dsp_halfband.h
CHAIN_COMMON := chain_test_common.h $(REPORT) ../src/mastering_dsp.h \
              ../src/dsp_limiter.h ../src/dsp_common.h ../src/dsp_halfband.h
MOD_COMMON := $(REPORT) ../src/mod_source.h ../src/dsp_analysis.h \
              ../src/dsp_common.h
MOD_SRC   := ../src/mod_source.cpp
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

# lim_response_test drives the Limiter struct alone. dsp_limiter.h is
# <cmath>-only and does not reach for mastering_dsp.h, so this one needs
# neither the stubs nor mastering_dsp.cpp.
$(LRESPONSE): lim_response_test.cpp $(LIM_COMMON) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

# chain_test is the only harness that drives the assembled chain, so it is the
# only one that links mastering_dsp.cpp *and* needs every stage header.
$(CHAIN): chain_test.cpp $(CHAIN_COMMON) $(DSP_SRC) $(DSP_HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(DSP_SRC) -o $@

# mod_source_test drives the CV modulation engine, which includes nothing from
# libDaisy or the alchemy-sdk by design — so this links mod_source.cpp alone and
# needs no stubs. dsp_analysis.h comes along for the band-split section and is
# <cmath>-only like every other stage header.
$(MODSRC): mod_source_test.cpp $(MOD_COMMON) $(MOD_SRC) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(MOD_SRC) -o $@

run: $(RESPONSE) $(CONTROL) $(CRESPONSE) $(CCONTROL) $(SRESPONSE) $(SCONTROL) \
     $(LRESPONSE) $(CHAIN) $(MODSRC)
	@./$(RESPONSE)
	@./$(CONTROL)
	@./$(CRESPONSE)
	@./$(CCONTROL)
	@./$(SRESPONSE)
	@./$(SCONTROL)
	@./$(LRESPONSE)
	@./$(CHAIN)
	@./$(MODSRC)

golden: $(RESPONSE) $(CRESPONSE) $(SRESPONSE)
	@mkdir -p golden
	@./$(RESPONSE) --golden
	@./$(CRESPONSE) --golden
	@./$(SRESPONSE) --golden

clean:
	@rm -rf $(BUILD)
