cmake_minimum_required(VERSION 3.22)
project(damiao)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -Wall -Wextra -pedantic)

# --- Library ---
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

find_package(Boost REQUIRED COMPONENTS system)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::system)

# --- Install ---
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/damiao DESTINATION include)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE damiao::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# --- Tests ---
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1
  )
  FetchContent_MakeAvailable(Catch2)

  enable_testing()

  add_executable(test_types tests/test_types.cpp)
  target_link_libraries(test_types PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain)
  add_test(NAME test_types COMMAND test_types)

  add_executable(test_motor tests/test_motor.cpp)
  target_link_libraries(test_motor PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain)
  add_test(NAME test_motor COMMAND test_motor)
endif()

# --- Examples ---
option(BUILD_EXAMPLES "Build example executables" OFF)
if(BUILD_EXAMPLES)
  add_executable(mit_control examples/mit_control.cpp)
  target_link_libraries(mit_control PRIVATE ${PROJECT_NAME})
endif()

# --- Documentation ---
option(BUILD_DOCS "Build Doxygen documentation" OFF)
if(BUILD_DOCS)
  find_package(Doxygen REQUIRED)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  add_custom_target(docs ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
endif()
