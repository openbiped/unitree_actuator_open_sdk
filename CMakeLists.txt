cmake_minimum_required(VERSION 3.16)

project(unitree_actuator_open_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Core C++ library (Unitree Actuator SDK compatible API)
# -----------------------------------------------------------------------------
add_library(unitree_actuator_open_sdk_core
    src/unitreeMotor.cpp
    src/SerialPort.cpp
)

# Needed when this static library is linked into the Python extension module.
set_target_properties(unitree_actuator_open_sdk_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(unitree_actuator_open_sdk_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(unitree_actuator_open_sdk_core PRIVATE
    -Wall -Wextra -Wno-unused-parameter
)

# -----------------------------------------------------------------------------
# C++ examples
# -----------------------------------------------------------------------------
option(UNITREE_BUILD_EXAMPLES "Build example programs" ON)
if (UNITREE_BUILD_EXAMPLES)
  add_executable(example_a1_motor_output example/example_a1_motor_output.cpp)
  target_link_libraries(example_a1_motor_output PRIVATE unitree_actuator_open_sdk_core)

  add_executable(example_goM8010_6_motor_output example/example_goM8010_6_motor_output.cpp)
  target_link_libraries(example_goM8010_6_motor_output PRIVATE unitree_actuator_open_sdk_core)

  add_executable(example_go2_motor_output example/example_go2_motor_output.cpp)
  target_link_libraries(example_go2_motor_output PRIVATE unitree_actuator_open_sdk_core)

  add_executable(go1_changeid example/gochangeid.cpp)
  target_compile_definitions(go1_changeid PRIVATE GOVERSION=1)
  target_link_libraries(go1_changeid PRIVATE unitree_actuator_open_sdk_core)

  add_executable(go2_changeid example/gochangeid.cpp)
  target_compile_definitions(go2_changeid PRIVATE GOVERSION=2)
  target_link_libraries(go2_changeid PRIVATE unitree_actuator_open_sdk_core)
endif()

# -----------------------------------------------------------------------------
# Python extension module (pybind11)
# -----------------------------------------------------------------------------
option(UNITREE_BUILD_PYTHON "Build Python bindings" OFF)

# scikit-build-core defines SKBUILD when building a wheel.
if (DEFINED SKBUILD)
  set(UNITREE_BUILD_PYTHON ON CACHE BOOL "Build Python bindings" FORCE)
endif()

if (UNITREE_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_unitree_actuator_open_sdk MODULE
      src/bindings.cpp
  )
  set_target_properties(_unitree_actuator_open_sdk PROPERTIES PREFIX "")

  target_include_directories(_unitree_actuator_open_sdk PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(_unitree_actuator_open_sdk PRIVATE unitree_actuator_open_sdk_core)
  target_compile_options(_unitree_actuator_open_sdk PRIVATE -Wall -Wextra -Wno-unused-parameter)

  # Local dev: place the built module next to the Unitree-style Python examples.
  # Wheel builds: place into the build tree and install from there.
  if (DEFINED SKBUILD)
    set(PKG_DIR "${CMAKE_BINARY_DIR}/unitree_actuator_open_sdk")
  else()
    set(PKG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/lib/unitree_actuator_open_sdk")
  endif()

  file(MAKE_DIRECTORY "${PKG_DIR}")
  file(WRITE "${PKG_DIR}/__init__.py" "from ._unitree_actuator_open_sdk import *\n")

  set_target_properties(_unitree_actuator_open_sdk PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${PKG_DIR}"
      RUNTIME_OUTPUT_DIRECTORY "${PKG_DIR}"
  )

  # scikit-build(-core) packages the results of `cmake --install`.
  if (DEFINED SKBUILD)
    install(TARGETS _unitree_actuator_open_sdk
            LIBRARY DESTINATION unitree_actuator_open_sdk
            RUNTIME DESTINATION unitree_actuator_open_sdk)
    install(FILES "${PKG_DIR}/__init__.py"
            DESTINATION unitree_actuator_open_sdk)
  endif()
endif()
