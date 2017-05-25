#Change this if you need to target a specific CMake version
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Enable debug symbols by default
# must be done before project() statement
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()
# (you can also set it on the command line: -D CMAKE_BUILD_TYPE=Release)

project(slambot)

include_directories("${PROJECT_BINARY_DIR}")

# Define sources and executable
set(EXECUTABLE_NAME "slambot")
add_executable(${EXECUTABLE_NAME}
  main.cpp
)

set(
  CMAKE_MODULE_PATH 
  cmake_modules
  "${CMAKE_SOURCE_DIR}/cmake_modules" 
  ${CMAKE_MODULE_PATH} 
)

#find_package(SFML 2 REQUIRED system window graphics network audio)

#include_directories(${SFML_INCLUDE_DIR})

target_link_libraries(
  ${EXECUTABLE_NAME} 
#  ${SFML_LIBRARIES} 
)

# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)
