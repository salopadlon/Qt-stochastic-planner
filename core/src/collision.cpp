#include "rrt/collision.hpp"

namespace rrt {

bool point_in_any(Vec2 p, const std::vector<Rect>& obstacles)
{
    for (const auto& r : obstacles) {
        if (r.contains(p)) return true;
    }
    return false;
}

bool segment_collides(Vec2, Vec2, const std::vector<Rect>&)
{
    return false;
}

} // namespace rrt