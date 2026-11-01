#pragma once

#include <vector>

#include "tree.h"

// Loads LED positions from a layout file in place of the built-in tree model.
//
// CSV with a header line, "#" comment lines, and at least these columns:
//   strip,led,x,y,z
// x,y,z are meters (y up). The driver index is strip * PIXELS_PER_STRIP + led.
// Extra columns are ignored. Pixels that are not listed are not shaded.
// Returns an empty vector (after printing why) if the file is unusable.
std::vector<PixInfo> load_layout(const char *path);
