cmake_minimum_required(VERSION 3.10)
project(PhysicsSimulator)
set(CMAKE_CXX_STANDARD 23)

# Add source files
set(SOURCES 
  src/main.cpp
  src/sim/Particle/particle.cpp
  src/sim/RigidBody/rigidbody.cpp
  src/maths/calculations.cpp
)

find_package(Eigen3 3.3 REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
add_compile_options(-g -Wall)

# Create executable
add_executable(PhysicsSimulator ${SOURCES})

# Link libraries
target_link_libraries(PhysicsSimulator ${CMAKE_DL_LIBS})
target_link_libraries(PhysicsSimulator Eigen3::Eigen)

# google Test
# enable_testing()
#add_executable(
# cpuTest
# test/test.cpp
# ${SOURCES}
#)
#target_link_libraries(
#cpuTest
  #GTest::gtest_main
  #)

#include(GoogleTest)
#gtest_discover_tests(cpuTest)
