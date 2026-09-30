# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

list(APPEND EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_LIST_DIR}/source"
    "${CMAKE_CURRENT_LIST_DIR}/source/driver"
    "${CMAKE_CURRENT_LIST_DIR}/source/control"
    "${CMAKE_CURRENT_LIST_DIR}/source/app"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stepper_project)
