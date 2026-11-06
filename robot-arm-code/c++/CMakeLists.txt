cmake_minimum_required(VERSION 3.20)
project(robot_arm CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wdouble-promotion)

add_library(utils INTERFACE)
target_include_directories(utils INTERFACE libs/utils/include)

add_library(linalg INTERFACE)
target_include_directories(linalg INTERFACE libs/linalg/include)

add_library(linalg_io INTERFACE)
target_include_directories(linalg_io INTERFACE libs/linalg/io/include)

add_library(geometry INTERFACE)
target_include_directories(geometry INTERFACE libs/geometry/include)
target_link_libraries(geometry INTERFACE linalg)

add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE libs/kinematics/include)
target_link_libraries(kinematics INTERFACE geometry)
target_link_libraries(kinematics INTERFACE utils)

add_library(kinematics_io INTERFACE)
target_include_directories(kinematics_io INTERFACE libs/kinematics/io/include)
target_link_libraries(kinematics_io INTERFACE kinematics)

add_executable(test_matrix tests/test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE linalg)
target_link_libraries(test_matrix PRIVATE linalg_io)
target_compile_options(test_matrix PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_matrix PRIVATE -fsanitize=address,undefined)

add_executable(test_planar tests/test_planar.cpp)
target_link_libraries(test_planar PRIVATE kinematics)
target_compile_options(test_planar PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_planar PRIVATE -fsanitize=address,undefined)

add_executable(test_arm tests/test_arm.cpp)
target_link_libraries(test_arm PRIVATE kinematics)
target_link_libraries(test_arm PRIVATE kinematics_io)
target_link_libraries(test_arm PRIVATE linalg_io)
target_compile_options(test_arm PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_arm PRIVATE -fsanitize=address,undefined)

add_executable(test_forward_consistency tests/test_forward_consistency.cpp)
target_link_libraries(test_forward_consistency PRIVATE kinematics)
target_compile_options(test_forward_consistency PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_forward_consistency PRIVATE -fsanitize=address,undefined)

add_executable(test_base_3planar tests/test_base+3planar.cpp)
target_link_libraries(test_base_3planar PRIVATE kinematics)
target_compile_options(test_base_3planar PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_base_3planar PRIVATE -fsanitize=address,undefined)

add_executable(test_base_3planar_roll tests/test_base+3planar+roll.cpp)
target_link_libraries(test_base_3planar_roll PRIVATE kinematics)
target_compile_options(test_base_3planar_roll PRIVATE -g -fsanitize=address,undefined)
target_link_options(test_base_3planar_roll PRIVATE -fsanitize=address,undefined)

add_executable(benchmark_planar bench/benchmark_planar.cpp)
target_link_libraries(benchmark_planar PRIVATE kinematics)

add_executable(benchmark_norm1 bench/benchmark_norm1.cpp)
target_link_libraries(benchmark_norm1 PRIVATE linalg)

enable_testing()
add_test(NAME test_matrix COMMAND test_matrix)
add_test(NAME test_planar COMMAND test_planar)
add_test(NAME test_arm COMMAND test_arm)
add_test(NAME test_forward_consistency COMMAND test_forward_consistency)
add_test(NAME test_base_3planar COMMAND test_base_3planar)
add_test(NAME test_base_3planar_roll COMMAND test_base_3planar_roll)