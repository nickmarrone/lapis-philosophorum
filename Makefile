# =============================================================================
# Lapis Philosophorum — Daisy-bootloader firmware for Hermetic Modular
# Alchemy Lab
#
# Standard Daisy workflow (libDaisy core Makefile underneath):
#   make libdaisy       — build lib/libDaisy once after cloning
#   make                — build firmware (BOARD=v2 by default)
#   make program-live   — reboot the running module into DFU over USB, then
#                         flash it; no power cycle, no button press
#   make program-dfu    — flash a module already in DFU mode (see README)
#   make clean          — remove the build tree
# =============================================================================

# Version lives in src/version.h, which is the single source of truth — this
# only reads it back to name the artifacts, and never sets it.
VERSION := $(shell sed -n 's/^\#define[ \t]*LAPIS_VERSION_STR[ \t]*"\(.*\)".*/\1/p' src/version.h)
ifeq ($(VERSION),)
$(error could not read LAPIS_VERSION_STR from src/version.h)
endif

# Artifacts carry the version: build/lapis_philosophorum_v0.5.0.{bin,elf,hex,map}.
# Bumping version.h therefore produces a differently-named .bin, and the old
# one stays behind — `make clean` between releases if that bothers you.
TARGET = lapis_philosophorum_v$(VERSION)
$(info Building $(TARGET))

# Alchemy Lab board revision: v1 | v2
BOARD ?= v2
ifeq ($(filter $(BOARD),v1 v2),)
$(error BOARD must be 'v1' or 'v2' (got '$(BOARD)'))
endif

ALCHEMY_DIR  = lib/alchemy-sdk
LIBDAISY_DIR = lib/libDaisy

# ── App sources — yours to edit ─────────────────────────────────────────────
CPP_SOURCES = \
    src/mastering.cpp \
    src/mastering_dsp.cpp \
    src/manual.cpp \
    src/mod_source.cpp

# ── Alchemy SDK, compiled straight from the submodule ───────────────────────
CPP_SOURCES += $(sort $(shell find $(ALCHEMY_DIR)/framework/src -name '*.cpp'))
CPP_SOURCES += $(sort $(wildcard $(ALCHEMY_DIR)/hardware/alchemy-lab/$(BOARD)/src/*.cpp))

C_INCLUDES += \
    -Isrc \
    -I$(ALCHEMY_DIR)/framework/include \
    -I$(ALCHEMY_DIR)/hardware/include \
    -I$(ALCHEMY_DIR)/hardware/alchemy-lab/$(BOARD)/include

ifeq ($(BOARD),v2)
C_DEFS += -DALCHEMY_BOARD_V2
endif

# Stamped into the image and reported over HostLink, so a module in the web
# programmer says which commit it is running. Falls back to the definition in
# version.h when git isn't available (tarball build, no .git).
GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null)
ifneq ($(GIT_HASH),)
C_DEFS += -DLAPIS_GIT_HASH=\"$(GIT_HASH)\"
endif

# ── Daisy bootloader build (BOOT_SRAM) ──────────────────────────────────────
APP_TYPE = BOOT_SRAM
LDSCRIPT = $(ALCHEMY_DIR)/cmake/linkers/alchemy_stm32h750ib_sram.lds

# The Alchemy SDK requires C++17 (libDaisy's default is gnu++14).
CPP_STANDARD = -std=gnu++17

# ── libDaisy core Makefile does the rest ────────────────────────────────────
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

BOARD_STAMP := $(BUILD_DIR)/.board-$(BOARD)
ifeq ($(wildcard $(BOARD_STAMP)),)
_BOARD_GUARD := $(shell rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.d $(BUILD_DIR)/*.lst $(BUILD_DIR)/.board-* 2>/dev/null; mkdir -p $(BUILD_DIR); touch $(BOARD_STAMP))
endif

.PHONY: libdaisy
libdaisy:
	$(MAKE) -C $(LIBDAISY_DIR)

# ── Flash a running module without touching it ──────────────────────────────
# The stock program-dfu needs the module already in DFU mode, which means a
# power cycle plus holding B3 through the bootloader window. The firmware runs
# a HostLink host on the panel USB-C, so instead we ask it to reboot into the
# bootloader over that same cable and hand off to dfu-util.
#
# dfu-util needs -w here: the reboot request returns as soon as the module
# ACKs, well before it has re-enumerated as a DFU device, so without -w the
# flash races the re-enumeration and fails "No DFU capable USB device".
#
# Requires node on PATH. If the module is not running HostLink firmware (or is
# already sitting in DFU mode), the reboot step has nothing to talk to — use
# program-dfu for that case.
HOSTLINK_CLI = $(ALCHEMY_DIR)/tools/hostlink-cli/hostlink.mjs

.PHONY: program-live
program-live: all
	node $(HOSTLINK_CLI) reboot bootloader
	dfu-util -w -a 0 -s $(FLASH_ADDRESS):leave -D $(BUILD_DIR)/$(TARGET_BIN) -d ,0483:$(USBPID)

# ── Host-side DSP measurement harness (no cross-toolchain needed) ───────────
.PHONY: test test-golden test-clean
test:
	$(MAKE) -C tests
test-golden:
	$(MAKE) -C tests golden
test-clean:
	$(MAKE) -C tests clean
