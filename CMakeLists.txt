cmake_minimum_required(VERSION 3.1)
project(usv_libs)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/b796f7d44681514f58a683a3a71ff17c94edb0c1.zip
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(PYBIND11_FINDPYTHON ON)
#find_package (Python3 COMPONENTS Interpreter Development)
add_subdirectory(pybind11)

set(CMAKE_CXX_STANDARD 17)
set(USV_LIBS_SRC src/control/ASMC.cpp src/control/AITSMC.cpp src/model/dynamic_model.cpp src/utils/ControllerUtils.cpp)

# Library compilation
include_directories(src)
add_library(usv_libs ${USV_LIBS_SRC})
target_link_libraries(usv_libs Eigen3::Eigen)

pybind11_add_module(usv_libs_py python/python_bindings.cpp)
target_include_directories(usv_libs_py PRIVATE src)
target_link_libraries(usv_libs_py PRIVATE usv_libs)

install(
        TARGETS
        usv_libs_py
        LIBRARY
        DESTINATION usv_libs_py
)

# Unit test
include(GoogleTest)
file(GLOB_RECURSE USV_TEST_SOURCES test/*.cpp)
add_executable(USV_Tests ${USV_TEST_SOURCES} ${USV_LIBS_SRC})
target_link_libraries(USV_Tests GTest::gtest_main Eigen3::Eigen)
target_include_directories(USV_Tests PRIVATE src/)
gtest_discover_tests(USV_Tests)