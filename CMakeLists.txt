cmake_minimum_required(VERSION 3.20)

project(IPVSEAS VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
#
# No machine-specific paths live in this file. Point CMake at your libraries
# in one of these ways:
#   * vcpkg:      cmake --preset vcpkg            (uses $VCPKG_ROOT)
#   * local:      copy CMakeUserPresets.json.example -> CMakeUserPresets.json,
#                 set your paths, then  cmake --preset local
#   * manual:     cmake -B build -DOpenCV_DIR=... -DEIGEN3_INCLUDE_DIR=...
# ---------------------------------------------------------------------------

# Eigen: prefer an installed package config (vcpkg, apt, ...); otherwise
# accept a plain source folder such as C:/libs/eigen-3.4.0.
find_package(Eigen3 3.3 CONFIG QUIET)
if(NOT TARGET Eigen3::Eigen)
    set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Folder that contains Eigen/Dense")
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Dense")
        message(FATAL_ERROR
            "Eigen not found. Install it with vcpkg, or pass "
            "-DEIGEN3_INCLUDE_DIR=<folder containing Eigen/Dense>.")
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

# OpenCV is only needed by the environment-check executable for now.
find_package(OpenCV QUIET)

enable_testing()

# ---------------------------------------------------------------------------
# Part 1 core library, scenario runner and tests (Eigen only)
# ---------------------------------------------------------------------------
add_subdirectory(part_1_road_perception)

# ---------------------------------------------------------------------------
# Environment check (OpenCV + Eigen)
# ---------------------------------------------------------------------------
if(OpenCV_FOUND)
    add_executable(ipvseas src/cpp/main.cpp)
    target_link_libraries(ipvseas PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
    target_include_directories(ipvseas PRIVATE ${OpenCV_INCLUDE_DIRS})
else()
    message(STATUS "OpenCV not found: skipping the 'ipvseas' environment-check target")
endif()
