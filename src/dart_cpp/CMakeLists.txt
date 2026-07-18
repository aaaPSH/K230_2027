cmake_minimum_required(VERSION 3.16)
project(dart_cpp LANGUAGES CXX)

find_package(OpenCV REQUIRED)

add_library(dart_core
    src/detector.cpp
    src/guidance.cpp
    src/kalman.cpp
)

target_compile_features(dart_core PUBLIC cxx_std_17)
target_include_directories(dart_core PUBLIC src)
target_link_libraries(dart_core PUBLIC ${OpenCV_LIBS})

option(DART_BUILD_TESTS "构建 C++ 回归测试" ON)
if(DART_BUILD_TESTS)
    enable_testing()
    add_executable(dart_core_tests tests/core_tests.cpp)
    target_link_libraries(dart_core_tests PRIVATE dart_core)
    add_test(NAME dart_core_tests COMMAND dart_core_tests)
endif()
