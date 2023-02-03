#pragma once

#include "../../inc/MarlinConfig.h"

void plan_arc(
  const xyze_pos_t &cart,   // Destination position
  const ab_float_t &offset, // Center of rotation relative to current_position
  const uint8_t clockwise   // Clockwise?
);
