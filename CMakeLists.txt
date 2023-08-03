#########################################################################
## NOTE: Do not invoke this cmake yourself, use pip install . (see the ##
## README.md file).                                                    ##
#########################################################################

cmake_minimum_required(VERSION 3.16.3)

project(_pyFRI)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(FRI_BUILD_EXAMPLES OFF)
set(PID_BUILD_EXAMPLES OFF)

add_subdirectory(pybind)
add_subdirectory(FRI-Client-SDK_Cpp)
add_subdirectory(pid)

pybind11_add_module(_pyFRI ${CMAKE_CURRENT_SOURCE_DIR}/pyFRI/src/wrapper.cpp)

target_link_libraries(_pyFRI PRIVATE FRIClient pid)
