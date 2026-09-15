configure_file(frcsim_version.h.in "${CMAKE_CURRENT_BINARY_DIR}/generated/frcsim_version.h" @ONLY)

# ---- frcsim_core: all simulation code, static, internal ---------------------------------------------
add_library(frcsim_core STATIC
    drive/battery.cpp
    drive/dc_motor.cpp
    drive/swerve_config.cpp
    drive/swerve_controller.cpp
    drive/swerve_robot.cpp
    field/field.cpp
    field/field_json.cpp
    pieces/piece_pool.cpp
    pieces/piece_type.cpp
    world/driven_bodies.cpp
    world/jolt_runtime.cpp
    world/kinematic_bodies.cpp
    world/materials.cpp
    world/shapes.cpp
    world/world.cpp
)
target_include_directories(frcsim_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
target_link_libraries(frcsim_core
    PUBLIC Jolt
    PRIVATE nlohmann_json::nlohmann_json
)
frcsim_configure_target(frcsim_core)

# ---- frcsim: shared library exporting the C ABI (+ JNI) ---------------------------------------------
add_library(frcsim SHARED
    capi/capi_pieces.cpp
    capi/capi_robots.cpp
    capi/capi_world.cpp
)
target_include_directories(frcsim PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_compile_definitions(frcsim PRIVATE FRCSIM_BUILDING_LIBRARY)
target_link_libraries(frcsim PRIVATE frcsim_core)
frcsim_configure_target(frcsim)

if(FRCSIM_BUILD_JNI)
    find_package(JNI REQUIRED)
    target_sources(frcsim PRIVATE jni/jni_pieces.cpp jni/jni_robots.cpp jni/jni_world.cpp)
    target_include_directories(frcsim SYSTEM PRIVATE ${JNI_INCLUDE_DIRS})
endif()

# ---- Install: <prefix>/<os>/<arch>/shared/ (decision D5) --------------------------------------------
install(TARGETS frcsim
    RUNTIME DESTINATION "${FRCSIM_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${FRCSIM_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${FRCSIM_INSTALL_LIBDIR}"
)
if(MSVC)
    install(FILES "$<TARGET_PDB_FILE:frcsim>" DESTINATION "${FRCSIM_INSTALL_LIBDIR}" OPTIONAL)
endif()
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/" DESTINATION include)
