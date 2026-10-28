cmake_minimum_required(VERSION 3.20)
project(reversible_wbc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(rwb_core
    src/closed_loop.cpp
    src/config.cpp
    src/dynamics.cpp
    src/metrics.cpp
    src/monitor.cpp
    src/policy.cpp
    src/safety_shell.cpp
)

target_include_directories(rwb_core PUBLIC include)
target_compile_options(rwb_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rwb_train apps/rwb_train.cpp)
target_link_libraries(rwb_train PRIVATE rwb_core)

add_executable(rwb_benchmark apps/rwb_benchmark.cpp)
target_link_libraries(rwb_benchmark PRIVATE rwb_core)

add_executable(rwb_shell_spec apps/rwb_shell_spec.cpp)
target_link_libraries(rwb_shell_spec PRIVATE rwb_core)

add_executable(rwb_curriculum apps/rwb_curriculum.cpp)
target_link_libraries(rwb_curriculum PRIVATE rwb_core)

enable_testing()
add_executable(rwb_tests tests/core_tests.cpp)
target_link_libraries(rwb_tests PRIVATE rwb_core)
add_test(NAME rwb_core_tests COMMAND rwb_tests)

