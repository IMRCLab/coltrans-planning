cmake_minimum_required(VERSION 3.5)

project(coltrans-planning)

find_package(Boost 1.74 REQUIRED COMPONENTS filesystem serialization)
find_package(Boost 1.74 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(OMPL ompl REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
pkg_check_modules(FCL fcl REQUIRED)
# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# add_subdirectory(deps/fcl EXCLUDE_FROM_ALL)

# add dynoplan
add_subdirectory(deps/dynoplan)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  include
  )

## Payload Planning
add_executable(nUavsPayloadPlanner
  src/nUavsPayloadPlanner.cpp
  src/robots.cpp
  src/fclStateValidityChecker.cpp
  src/optimObj.cpp
)
target_include_directories(nUavsPayloadPlanner
  PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
  PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
)

target_link_libraries(nUavsPayloadPlanner
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${FCL_LIBRARIES}
)

## test_fcl
add_executable(test_fcl
  src/main_fcl.cpp
)
target_include_directories(test_fcl
  PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
  PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
)
target_link_libraries(test_fcl
  ${Boost_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${FCL_LIBRARIES}
)