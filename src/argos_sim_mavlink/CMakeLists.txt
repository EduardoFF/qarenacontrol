cmake_minimum_required(VERSION 2.6)
project(argos_sim_mavlink)

# Find the ARGoS package
set(CMAKE_MODULE_PATH /usr/share/argos2 ${CMAKE_SOURCE_DIR}/cmake)
find_package(ARGoS REQUIRED)

# Find the GSL package
#find_package(GSL REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/mavlink)

# Set ARGoS link dir
link_directories(${ARGOS_LINK_DIR})

# Include the simulation compile flags
include(${CMAKE_SOURCE_DIR}/cmake/SimulationConfig.cmake)

# Descend into the controllers directory
add_subdirectory(controllers)

# Descend into the loop_functions directory
#add_subdirectory(loop_functions)

# Descend into the embedding directory
#add_subdirectory(embedding)
