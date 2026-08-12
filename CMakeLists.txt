cmake_minimum_required(VERSION 3.15)
project(SmartPalletizer)

# Trova Python installato nel sistema
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Scarica pybind11 in automatico senza installazioni manuali
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1
)
FetchContent_MakeAvailable(pybind11)

# Compila simulatore.cpp in un modulo Python (mio_simulatore)
pybind11_add_module(mio_simulatore src/simulatore.cpp)

# Copia in automatico il file compilato nella cartella py_scripts per usarlo comodamente
add_custom_command(TARGET mio_simulatore POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:mio_simulatore>
    ${CMAKE_CURRENT_SOURCE_DIR}/py_scripts/
)