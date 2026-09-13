# Scenario runner: end-to-end step timings for the budgets in CLAUDE.md §7.
add_executable(frcsim_bench
    scenario_runner.cpp
)
target_compile_definitions(frcsim_bench PRIVATE FRCSIM_REPO_DIR="${PROJECT_SOURCE_DIR}/..")
target_link_libraries(frcsim_bench PRIVATE frcsim_core)
frcsim_configure_target(frcsim_bench)
