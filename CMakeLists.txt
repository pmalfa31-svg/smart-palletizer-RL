cmake_minimum_required(VERSION 3.15)
project(smart_palletizer_rl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

# ---------------------------------------------------------------------------
# Bullet3 — ci serve SOLO BulletDynamics/BulletCollision/LinearMath/
# Bullet3Common. btMultiBody (Featherstone) e' gia' incluso in BulletDynamics
# di default: NON serve BUILD_EXTRAS ne' l'importer URDF C++ di bullet3
# (fragile da compilare) perche' il parsing URDF lo facciamo offline in
# Python (vedi tools/urdf_parser.py) e passiamo dati gia' pronti al motore.
# ---------------------------------------------------------------------------
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
set(USE_GLUT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    bullet3
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG        3.25
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(bullet3)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.13.6
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)

# ---------------------------------------------------------------------------
# Motore (libreria statica, nessuna dipendenza da Python)
# ---------------------------------------------------------------------------
add_library(palletizer_engine STATIC
    engine/core/PhysicsWorld.cpp
    engine/robot/ArticulatedArm.cpp
    engine/conveyor/CircularConveyor.cpp
    engine/packaging/Parcel.cpp
    engine/planner/PalletPlanner.cpp
)
target_include_directories(palletizer_engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
    ${bullet3_SOURCE_DIR}/src
)
target_link_libraries(palletizer_engine PUBLIC
    BulletDynamics BulletCollision LinearMath Bullet3Common
)

# ---------------------------------------------------------------------------
# Modulo Python (pybind11) — l'unico punto che sa di Python
# ---------------------------------------------------------------------------
pybind11_add_module(palletizer_core bindings/py_bindings.cpp)
target_link_libraries(palletizer_core PRIVATE palletizer_engine)

# ---------------------------------------------------------------------------
# Test C++ (placeholder — aggiungeremo Catch2/GoogleTest quando servono
# i primi unit test sulla fisica: raggio di presa, planner, ecc.)
# ---------------------------------------------------------------------------
option(BUILD_ENGINE_TESTS "Compila i test C++ del motore" OFF)
if(BUILD_ENGINE_TESTS)
    add_subdirectory(tests)
endif()
