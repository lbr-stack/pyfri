#########################################################################
## NOTE: Do not invoke this cmake yourself, use pip install . (see the ##
## README.md file).                                                    ##
#########################################################################

cmake_minimum_required(VERSION 3.16.3)

project(pyFRIClient)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(FRI_BUILD_EXAMPLES OFF)

add_subdirectory(pybind)
add_subdirectory(FRI-Client-SDK_Cpp)

pybind11_add_module(pyFRIClient ${CMAKE_CURRENT_SOURCE_DIR}/pyFRIClient/src/wrapper.cpp)

target_link_libraries(pyFRIClient PRIVATE FRIClient)
