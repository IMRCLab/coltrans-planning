cmake_minimum_required(VERSION 3.5)

project(coltrans-planning)

find_package(Boost 1.74 REQUIRED COMPONENTS filesystem serialization)
find_package(Boost 1.74 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(OMPL ompl REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  )

# # PayloadPlanning
# add_executable(payloadPlanner
#   src/payloadPlanner.cpp
# )

add_executable(nUavsPayloadPlanner
  src/nUavsPayloadPlanner.cpp
)

target_link_libraries(nUavsPayloadPlanner
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)
