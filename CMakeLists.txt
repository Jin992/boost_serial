cmake_minimum_required(VERSION 3.14)
project(MavlinkBoost)

set(CMAKE_CXX_STANDARD 17)
find_package(Boost 1.58 COMPONENTS system regex  filesystem REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

add_executable(MavlinkBoost main.cpp)
target_link_libraries(MavlinkBoost ${Boost_LIBRARIES} -pthread)