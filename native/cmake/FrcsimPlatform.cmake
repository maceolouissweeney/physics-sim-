# Derives WPILib-style platform names for the current build.
#
#   FRCSIM_PLATFORM_OS          windows | linux | osx
#   FRCSIM_PLATFORM_ARCH        x86-64 | arm64 | universal
#   FRCSIM_PLATFORM_CLASSIFIER  e.g. windowsx86-64, linuxarm64, osxuniversal (Maven classifier)
#   FRCSIM_INSTALL_LIBDIR       <os>/<arch>/shared  (layout GradleRIO expects inside native zips)

if(WIN32)
    set(FRCSIM_PLATFORM_OS windows)
elseif(APPLE)
    set(FRCSIM_PLATFORM_OS osx)
else()
    set(FRCSIM_PLATFORM_OS linux)
endif()

if(APPLE AND CMAKE_OSX_ARCHITECTURES)
    list(LENGTH CMAKE_OSX_ARCHITECTURES _frcsim_arch_count)
    if(_frcsim_arch_count GREATER 1)
        set(_frcsim_proc universal)
    else()
        set(_frcsim_proc "${CMAKE_OSX_ARCHITECTURES}")
    endif()
else()
    set(_frcsim_proc "${CMAKE_SYSTEM_PROCESSOR}")
endif()
string(TOLOWER "${_frcsim_proc}" _frcsim_proc)

if(_frcsim_proc MATCHES "^(amd64|x86_64|x64)$")
    set(FRCSIM_PLATFORM_ARCH x86-64)
elseif(_frcsim_proc MATCHES "^(arm64|aarch64)$")
    set(FRCSIM_PLATFORM_ARCH arm64)
elseif(_frcsim_proc STREQUAL "universal")
    set(FRCSIM_PLATFORM_ARCH universal)
else()
    message(FATAL_ERROR "frcsim: unsupported processor '${_frcsim_proc}'")
endif()

set(FRCSIM_PLATFORM_CLASSIFIER "${FRCSIM_PLATFORM_OS}${FRCSIM_PLATFORM_ARCH}")
set(FRCSIM_INSTALL_LIBDIR "${FRCSIM_PLATFORM_OS}/${FRCSIM_PLATFORM_ARCH}/shared")

message(STATUS "frcsim platform: ${FRCSIM_PLATFORM_CLASSIFIER} (install: ${FRCSIM_INSTALL_LIBDIR})")
