#Project name
project(controlledVectorField)

#Compiler flags
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x -Wall")

#Folder containing library
add_subdirectory(system_dynamics)
add_subdirectory(systems_utils)

#Executable that calls library
add_executable(controlledVectorField main.cpp)

#Link to library 
target_link_libraries(controlledVectorField PRIVATE systems_utils system_dynamics)

install(TARGETS controlledVectorField RUNTIME DESTINATION bin)
