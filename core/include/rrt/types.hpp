#pragma once

#include <cstdint>
#include <vector>

namespace rrt {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Rect {
    Vec2 min;
    Vec2 max;

    bool contains(Vec2 p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
    }
};

struct Node {
    Vec2 pos;
    int  parent = -1;
    double heading = 0.0;
};

struct PlanResult {
    std::vector<Node> tree;
    std::vector<int>  path;
    bool   success = false;
    int    iterations = 0;
    double elapsed_seconds = 0.0;
};

} // namespace rrt