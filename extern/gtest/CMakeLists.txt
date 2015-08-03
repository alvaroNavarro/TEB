cmake_minimum_required(VERSION 2.8.12)
project(gtest_builder C CXX)
include(ExternalProject)

if (MSVC)
else (MSVC)
	set(CPP11_CXX_DEF -DCMAKE_CXX_FLAGS=-std=c++11)
	set(CUSTOM_BUILD_CMD BUILD_COMMAND make)
	set(CUSTOM_CFG_CMD CONFIGURE_COMMAND ${CMAKE_COMMAND} . ${CPP11_CXX_DEF})
endif (MSVC)

ExternalProject_Add(googletest
    SVN_REPOSITORY http://googletest.googlecode.com/svn/trunk
	SVN_REVISION -r 707
    TIMEOUT 10
    CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
               -DCMAKE_CXX_FLAGS=${MSVC_COMPILER_DEFS}
               -Dgtest_force_shared_crt=ONN
     #PREFIX "${CMAKE_SOURCE_DIR}/extern/gtest"
     PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
     ${CUSTOM_CFG_CMD}
     ${CUSTOM_BUILD_CMD}
     BUILD_IN_SOURCE 1
# Disable install step
    INSTALL_COMMAND ""
)

# Specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_DIRS ${source_dir}/include PARENT_SCOPE)


# Specify MainTest's link libraries
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBS_DIR ${binary_dir} PARENT_SCOPE)

# Specify libraries to link against
if(NOT WIN32)
	set(GTEST_LIB_DEBUG ${binary_dir}/libgtest.a PARENT_SCOPE)
	set(GTEST_LIB_MAIN_DEBUG ${binary_dir}/libgtest_main.a PARENT_SCOPE)
	set(GTEST_LIB_RELEASE ${binary_dir}/libgtest.a PARENT_SCOPE)
	set(GTEST_LIB_MAIN_RELEASE ${binary_dir}/libgtest_main.a PARENT_SCOPE)
else()
	set(GTEST_LIB_DEBUG ${binary_dir}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES} PARENT_SCOPE)
	set(GTEST_LIB_MAIN_DEBUG ${binary_dir}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES} PARENT_SCOPE)
   	set(GTEST_LIB_RELEASE ${binary_dir}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES} PARENT_SCOPE)
	set(GTEST_LIB_MAIN_RELEASE ${binary_dir}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES} PARENT_SCOPE)
endif()
