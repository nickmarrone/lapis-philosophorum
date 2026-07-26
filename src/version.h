#pragma once

/**
 * version.h — Lapis Philosophorum firmware version.
 *
 * This file is the single source of truth for the version. The Makefile
 * greps LAPIS_VERSION_STR out of it for the build banner, so keep that
 * #define on one line with the version in double quotes.
 *
 * The string is also compiled into the image (see kVersionBanner in
 * mastering.cpp), so a stray .bin can be identified after the fact:
 *
 *     strings build/lapis_philosophorum.bin | grep LapisPhilosophorum
 *
 * Semantic versioning: bump PATCH for fixes that change no control
 * behaviour, MINOR for new controls or audible changes, MAJOR for a break
 * in the preset schema or panel layout.
 */

#define LAPIS_VERSION_MAJOR 0
#define LAPIS_VERSION_MINOR 5
#define LAPIS_VERSION_PATCH 0
#define LAPIS_VERSION_STR   "0.5.0"

namespace lapis
{
/** Version packed as 0xMMmmpp, so releases compare with `<` and `>`. */
constexpr unsigned kVersion = (LAPIS_VERSION_MAJOR << 16)
                            | (LAPIS_VERSION_MINOR << 8)
                            | (LAPIS_VERSION_PATCH);

constexpr const char* kVersionString = LAPIS_VERSION_STR;
} // namespace lapis
