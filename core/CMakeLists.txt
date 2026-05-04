add_library(rrt_core STATIC
    src/planner.cpp
    src/collision.cpp
)

target_include_directories(rrt_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(rrt_core PRIVATE
    -Wall -Wextra -Wpedantic
)

if(RRT_BUILD_TESTS)
    add_subdirectory(tests)
endif()