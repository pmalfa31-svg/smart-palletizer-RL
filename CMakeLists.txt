cmake_minimum_required(VERSION 3.15)
project(SmartPalletizer)

# Force CMake to tolerate older Bullet Physics configurations
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

# 1. Find Python
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# 2. Download pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1
)
FetchContent_MakeAvailable(pybind11)

# 3. Download Bullet Physics (Disable tests and demos for faster builds)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  bullet3
  GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
  GIT_TAG        3.25
)
FetchContent_MakeAvailable(bullet3)

# ADDED: Tell the compiler where to find our new OOP header (.h) files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 4. Compile the module (Now including all the OOP components)
# 4. Compile the module (Now including all the OOP components)
pybind11_add_module(mio_simulatore 
    src/WorldManager.cpp   
    src/Package.cpp
    src/ConveyorBelt.cpp
    src/RoboticArm.cpp
)

# 5. Link physics to our module
target_link_libraries(mio_simulatore PRIVATE BulletDynamics BulletCollision LinearMath)
# Tell the compiler where to find Bullet's .h files
target_include_directories(mio_simulatore PRIVATE ${bullet3_SOURCE_DIR}/src)

# 6. Copy the file into py_scripts
add_custom_command(TARGET mio_simulatore POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:mio_simulatore>
    ${CMAKE_CURRENT_SOURCE_DIR}/py_scripts/
)