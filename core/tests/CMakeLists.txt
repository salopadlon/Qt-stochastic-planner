add_executable(rrt_core_tests
    test_smoke.cpp
)

target_link_libraries(rrt_core_tests PRIVATE rrt_core)

add_test(NAME rrt_core_tests COMMAND rrt_core_tests)