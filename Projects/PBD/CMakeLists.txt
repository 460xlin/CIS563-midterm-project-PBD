cmake_minimum_required(VERSION 3.5)

find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)

if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

add_cispba_executable(PBD main.cpp tiny_obj_loader.h tiny_obj_loader.cc particles.h particles.cpp distance_constraint.h distance_constraint.cpp)
target_include_directories(PBD SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(PBD eigen partio)
