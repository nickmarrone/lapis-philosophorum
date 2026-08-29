#pragma once

/**
 * manual.h — the module's long-form manual, carried in the firmware.
 *
 * Entity-scoped prose (what one knob does, what one jack carries) lives on
 * the entity itself in mastering.cpp, next to the declaration it describes,
 * so the two cannot drift apart. This header is for the prose that has no
 * single object to hang on: the tagline, the preamble, and the sections that
 * explain the chain as a whole.
 *
 * All of it rides the HostLink descriptor to the web programmer. None of it
 * reaches a schema hash, so editing this text can never invalidate a saved
 * preset — see alchemy/surface/manual.h.
 */

#include "alchemy/surface/manual.h"

namespace lapis
{

/** Module-level manual content, attached to the HostLink host in main(). */
extern const alchemy::Manual kManual;

} // namespace lapis
