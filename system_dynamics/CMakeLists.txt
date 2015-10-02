#create the library system_dynamics
add_library(system_dynamics system_dynamics.cpp)

#make subfolder visible to cmake
target_include_directories( system_dynamics
   PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#tell cmake that system_dynamics lib uses systems_utils lib
target_link_libraries(system_dynamics PRIVATE systems_utils)
