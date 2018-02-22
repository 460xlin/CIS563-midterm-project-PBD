find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)


if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

# Configure this project
add_cispba_executable(hm03 main.cpp)
target_include_directories(hm03 SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(hm03 eigen)
