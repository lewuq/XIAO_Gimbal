cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/src")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Avoid ESP-IDF's git_describe() probe: this repo has no commits yet, which
# makes GetGitRevisionDescription.cmake fail on an empty refs/heads/master.
set(PROJECT_VER "1.0.0")
project(gimbal_recamera)
