cmake_minimum_required(VERSION 3.15)
project(SmartPalletizer)

# Forza CMake a tollerare le vecchie configurazioni di Bullet Physics
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

# 1. Trova Python
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# 2. Scarica pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1
)
FetchContent_MakeAvailable(pybind11)

# 3. Scarica Bullet Physics (Disabilitiamo test e demo per velocizzare la build)
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

# 4. Compila il modulo
pybind11_add_module(mio_simulatore src/simulatore.cpp)

# 5. COLLEGA LA FISICA AL NOSTRO MODULO (Questa è la riga nuova fondamentale!)
target_link_libraries(mio_simulatore PRIVATE BulletDynamics BulletCollision LinearMath)
# NUOVA RIGA: Diciamo al compilatore dove trovare i file .h di Bullet
target_include_directories(mio_simulatore PRIVATE ${bullet3_SOURCE_DIR}/src)

# 6. Copia il file in py_scripts
add_custom_command(TARGET mio_simulatore POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:mio_simulatore>
    ${CMAKE_CURRENT_SOURCE_DIR}/py_scripts/
)