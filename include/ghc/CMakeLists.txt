cmake_minimum_required (VERSION 2.8)
project (tinyraytracer)
set(CMAKE_CXX_STANDARD 14)

include(CheckCXXCompilerFlag)


add_executable(test_fs test_fs.cpp)
add_executable(test_reg test_regex.cpp)
