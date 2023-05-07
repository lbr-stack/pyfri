cmake_minimum_required(VERSION 3.16.3)
project(pyFRIClient)

# Remove build/ directory, this avoids linking errors when building
# different versions of FRI
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build")
  execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_SOURCE_DIR}/build")
  message("Removed ${CMAKE_CURRENT_SOURCE_DIR}/build.")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(FRI_BUILD_EXAMPLES OFF)

add_subdirectory(pybind)
add_subdirectory(FRI-Client-SDK_Cpp)

pybind11_add_module(pyFRIClient ${CMAKE_CURRENT_SOURCE_DIR}/pyFRIClient/src/wrapper.cpp)

target_link_libraries(pyFRIClient PRIVATE FRIClient)
