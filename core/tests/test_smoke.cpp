#include "rrt/planner.hpp"
#include "rrt/collision.hpp"

#include <cassert>
#include <iostream>

int main()
{
    rrt::Rect r{{0, 0}, {10, 10}};
    assert(r.contains({5, 5}));
    assert(!r.contains({11, 5}));

    auto result = rrt::plan({0, 0}, {100, 100}, {{0, 0}, {200, 200}}, {});
    (void)result;

    std::cout << "smoke ok\n";
    return 0;
}