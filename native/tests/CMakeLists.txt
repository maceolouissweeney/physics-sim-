include(GoogleTest)
find_package(Threads REQUIRED)

# Internal C++ tests: link the static core directly (decision D10).
# alloc_counter.cpp replaces global operator new for the whole executable (zero-allocation tests).
add_executable(frcsim_core_tests
    core/alloc_counter.cpp
    core/test_allocations.cpp
    core/test_body_tag.cpp
    core/test_driven_bodies.cpp
    core/test_event_buffer.cpp
    core/test_field.cpp
    core/test_field_json.cpp
    core/test_kinematic.cpp
    core/test_materials.cpp
    core/test_piece_pool.cpp
    core/test_world.cpp
)
target_include_directories(frcsim_core_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(frcsim_core_tests PRIVATE FRCSIM_REPO_DIR="${PROJECT_SOURCE_DIR}/..")
target_link_libraries(frcsim_core_tests PRIVATE frcsim_core GTest::gtest_main Threads::Threads)
frcsim_configure_target(frcsim_core_tests)
gtest_discover_tests(frcsim_core_tests DISCOVERY_MODE PRE_TEST)

# C ABI tests: link only the shared library, exactly what bindings see.
add_executable(frcsim_capi_tests
    capi/test_capi.cpp
    capi/test_capi_world.cpp
)
target_compile_definitions(frcsim_capi_tests PRIVATE FRCSIM_REPO_DIR="${PROJECT_SOURCE_DIR}/..")
target_link_libraries(frcsim_capi_tests PRIVATE frcsim GTest::gtest_main)
frcsim_configure_target(frcsim_capi_tests)
gtest_discover_tests(frcsim_capi_tests DISCOVERY_MODE PRE_TEST)
