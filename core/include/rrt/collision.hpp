#pragma once

#include "rrt/types.hpp"

namespace rrt {

bool point_in_any(Vec2 p, const std::vector<Rect>& obstacles);

bool segment_collides(Vec2 a, Vec2 b, const std::vector<Rect>& obstacles);

} // namespace rrt