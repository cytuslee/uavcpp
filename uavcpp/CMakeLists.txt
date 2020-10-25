# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.
set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")
cmake_minimum_required(VERSION 3.1...3.15)

project( uavcpp )

if ( CMAKE_SYSTEM_NAME MATCHES "Windows" )
    set( my_std_pre "/std:" )
else()
    set( my_std_pre "-std=" )
endif()

set( basic_cxx17 "c++17" )
set( str_cxx17 "${my_std_pre}${basic_cxx17}" )

include( CheckCXXCompilerFlag )
check_cxx_compiler_flag( "${str_cxx17}" _cpp_17_flag_supported )
if ( _cpp_17_flag_supported )
    set( CMAKE_CXX_STANDARD 17 )
 endif()
# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
# GDAL and its components

find_package(GDAL REQUIRED)
if ( NOT GDAL_FOUND )

  message(STATUS "This project requires the GDAL library, and will not be compiled.")

  return()  

endif()
include_directories(${GDAL_INCLUDE_DIRS})

# include for local directory
# include for local package
include(${CGAL_USE_FILE})
# Creating entries for all C++ files with "main" routine
# ##########################################################
add_executable(test test.cpp)
target_link_libraries(test ${GDAL_LIBRARY} ${CGAL_LIBS})



