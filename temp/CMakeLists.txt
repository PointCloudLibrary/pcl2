cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test src/test.cpp src/cloud.cpp src/array.cpp src/float_array.cpp src/int_array.cpp)

target_link_libraries (test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

set(CMAKE_CXX_FLAGS "-msse4 ${CMAKE_CXX_FLAGS}")

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)
