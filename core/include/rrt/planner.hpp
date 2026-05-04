#pragma once

#include "rrt/types.hpp"

namespace rrt {

struct PlannerConfig {
    double goal_threshold     = 20.0;
    double step_distance      = 25.0;
    double v_max              = 250.0;
    double omega_max          = 3.14159;
    std::uint64_t seed        = 0;     // 0 = nondeterministic
    int    max_iterations     = 100000;
};

PlanResult plan(Vec2 start,
                Vec2 goal,
                Rect bounds,
                const std::vector<Rect>& obstacles,
                const PlannerConfig& cfg = {});

} // namespace rrt
