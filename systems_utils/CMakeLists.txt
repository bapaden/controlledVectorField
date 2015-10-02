find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

add_library(systems_utils systems_utils.cpp)

target_link_libraries(systems_utils ${PYTHON_LIBRARY})
target_include_directories( systems_utils
   PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
   