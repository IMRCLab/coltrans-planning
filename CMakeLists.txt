cmake_minimum_required(VERSION 3.5)

project(coltrans-planning)

find_package(Boost 1.74 REQUIRED COMPONENTS filesystem serialization)
find_package(PkgConfig)
pkg_check_modules(ompl REQUIRED)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

include_directories(
  ${OMPL_INCLUDE_DIRS}
)

# Applications

## RigidBodyPlanning
add_executable(RigidBodyPlanning
  src/RigidBodyPlanning.cpp
)
target_link_libraries(RigidBodyPlanning
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)
