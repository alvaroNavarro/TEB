cmake_minimum_required(VERSION 2.6)

project(teb_package)

# ===== Build options ======
# Set to Release in order to speed up the program significantly.
# But set to Debug in order to use Debug functionalities (e.g. within your IDE).
# The Debug mode also enables much error checking stuff / assertions in the code.
set(CMAKE_BUILD_TYPE Release) # None, Debug, Release, RelWithDebInfo, MinSizeRel

# ===== Specify which examples should be build ======

OPTION(BUILD_EXAMPLES "Build all exmaples" True) # Set to False in order to disable building all examples
OPTION(BUILD_UNIT_TESTS "Build unit tests" False) # Set to True in order to build unit tests (googletest will be downloaded automatically)

OPTION(ENABLE_SOLVER_QPOASES "Enable solver qpOASES" True)

SET(EXAMPLE_NAMES # c++ example files (uncomment examples you do not want to build)
	#integrator_system/integrator_system1
	#integrator_system/integrator_system2
	#integrator_system/integrator_system_classic_mpc
	#linear_system/linear_system_ode
	#linear_system/linear_system_state_space
	#linear_system/linear_system_zeros
	#linear_system/linear_system_ode_ctrl_comparison
	#van_der_pol_system/van_der_pol_system
	#van_der_pol_system/van_der_pol_system_ctrl_comparison
	#rocket_system/rocket_system
	#overhead_crane/overhead_crane
	#motor_ecp220/motor_ecp220
	#mobile_robot_teb/mobile_robot_teb
	#mobile_rob_car_teb/mobile_robot_car_teb
	#car_like_system/car_like_system_main
	#car_like_system/car_like_system_obstacle_main
	#car_like_system/test
	#car_like_system/test_plotRobot
)

SET(MEX_EXAMPLE_NAMES # matlab mex example files (uncomment examples you do not want to build)
	#matlab_interface/integrator_system_mex	
)

SET(SFUN_EXAMPLE_NAMES # matlab simulink mex example files (uncomment examples you do not want to build)
	#matlab_interface/integrator_system_sfun
	#matlab_examples/timestwo
	#car_like_system/car_like_system_sfun
)


# ===== Build directories ======
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# ===== Compiler settings =====
include(${CMAKE_SOURCE_DIR}/cmake/macros/MiscMacros.cmake)

include(CheckCXXCompilerFlag)

# Windows only: Compile as static lib
#(make it compatible with Simulink's compiler)
#CHANGE_MD_TO_MT(CMAKE_CXX_FLAGS_DEBUG)
#CHANGE_MD_TO_MT(CMAKE_CXX_FLAGS_RELEASE)

# Activate C++11
if (NOT ${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  add_compile_options(-std=c++11)
endif()

IF (WIN32 AND MSVC_VERSION LESS 1800)
  message(FATAL_ERROR "Building with Microsoft compiler needs Visual Studio 2013 or later (Express version works as well)")
ENDIF(WIN32 AND MSVC_VERSION LESS 1800)
#IF (WIN32 AND MSVC_VERSION EQUAL 1800)
#  message(STATUS "Make sure that Microsoft Visual C++ Compiler Nov 2013 CTP ( http://www.microsoft.com/en-us/download/confirmation.aspx?id=41151 ) is installed.")
#  set(CMAKE_GENERATOR_TOOLSET "CTP_Nov2013" CACHE STRING "Platform Toolset" FORCE) 
#ENDIF(WIN32 AND MSVC_VERSION EQUAL 1800)

IF (WIN32 AND NOT CMAKE_CL_64)
	message(WARNING "Visual Studio 32 Bit mode selected. Be careful: we observered problems using Visual Studio 2013 32 Bit. Better switch to 64 Bit")
ENDIF(WIN32 AND NOT CMAKE_CL_64)

IF(MSVC)
	  # Copied from Eigen's CMakeLists to suppress some warnings:
	  # C4127 - conditional expression is constant
	  # C4714 - marked as __forceinline not inlined (I failed to deactivate it selectively)
	  #         We can disable this warning in the unit tests since it is clear that it occurs
	  #         because we are oftentimes returning objects that have a destructor or may
	  #         throw exceptions - in particular in the unit tests we are throwing extra many
	  #         exceptions to cover indexing errors.
	  # C4505 - unreferenced local function has been removed (impossible to deactive selectively)

	  # C4996 -  Disable warning for unsafe parameters that are passed to the transform function.
	  #			 We pass raw pointers that could be unsafe. But we ignore the warning and still use raw pointers...
	
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /wd4127 /wd4505 /wd4714 /wd4710 /wd4996 -D_CRT_SECURE_NO_WARNINGS")		

	 # Set compiler warning level to 2
	 # replace all /Wx by /W3
	string(REGEX REPLACE "/W[0-9]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
ELSE(MSVC)

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # Enable all compiler warnings
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g0 -O2")
ENDIF(MSVC)

# ==== DEFINITIONS =====
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # Mac OS X specific code
    add_definitions(-DAPPLE)
    add_definitions(-D__APPLE__) 
#SET(OperatingSystem "Mac OS X")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# ===== Include directories =====
INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include	
	${PROJECT_SOURCE_DIR}/extern
	${PROJECT_SOURCE_DIR}/extern/eigen3
)


# ===== Dependencies ===== 
FIND_PACKAGE(Doxygen)

FIND_PACKAGE(Gnuplot)
IF(GNUPLOT_FOUND)
    message(STATUS "Gnuplot found.")
    add_definitions( -DGNUPLOT_PATH="${GNUPLOT_EXECUTABLE}" )
ELSE(GNUPLOT_FOUND)
    message(WARNING "Gnuplot not found. Cannot use plot utilities")
ENDIF(GNUPLOT_FOUND)


#FIND_PACKAGE(Cygwin)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
FIND_PACKAGE(Matlab)
IF(MATLAB_FOUND)
  MESSAGE(STATUS "Matlab found. Compiling Mex activated.")
ELSE(MATLAB_FOUND)
  MESSAGE(STATUS "Matlab not found. Compiling Mex deactivated.")
ENDIF(MATLAB_FOUND)

FIND_PACKAGE(Cholmod)
IF(CHOLMOD_FOUND)
    message(STATUS "Cholmod found. Cholmod sparse solvers activated.")
    add_definitions( -DCHOLMOD )
    include_directories(${CHOLMOD_INCLUDE_DIR})
ELSE(CHOLMOD_FOUND)
    message(STATUS "Cholmod not found (optional for more dedicated sparse solvers).")
ENDIF(CHOLMOD_FOUND)

FIND_PACKAGE(Nlopt)
IF(NLOPT_FOUND)
    message(STATUS "NLOPT found. NLOPT solver support activated.")
    add_definitions( -DNLOPT )
    include_directories(${NLOPT_INCLUDE_DIRS})
ELSE(NLOPT_FOUND)
    message(STATUS "NLOPT not found (optional for more dedicated solvers).")
ENDIF(NLOPT_FOUND)

# Load qpOASES source
if (ENABLE_SOLVER_QPOASES)
	file(GLOB_RECURSE QPOASES_SRC ${PROJECT_SOURCE_DIR}/extern/qpOASES/src/*.cpp)
 # disable copyright information each time the constructor of the solver is called
	add_definitions(-D__NO_COPYRIGHT__)
	add_definitions(-DQPOASES)
	include_directories(${PROJECT_SOURCE_DIR}/extern/qpOASES/include)
endif (ENABLE_SOLVER_QPOASES)

# ===== compile teb libraries ===== 

# find all src files, drawback: if you add a cpp file, you have to manually regenerate cmake
file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

# find all header files, same drawback!
# Note: finding all header files is not necessary to compile your c++ project,
# it is only necessary that Visual Studio or XCODE knows, which header files are related to your project.
# Therefore is is just more comfortable
file(GLOB_RECURSE HEADER_HPP ${PROJECT_SOURCE_DIR}/include/*.hpp)
file(GLOB_RECURSE HEADER_H ${PROJECT_SOURCE_DIR}/include/*.h )

# Remove cpp files that refer to unavailable plugins and packages
IF (NOT NLOPT_FOUND)
	list(REMOVE_ITEM SRC ${PROJECT_SOURCE_DIR}/src/solver/solver_nlopt_package.cpp) # Remove NLOPT wrapper
ENDIF(NOT NLOPT_FOUND)

ADD_LIBRARY(teb_package STATIC ${SRC} ${QPOASES_SRC} ${HEADER_HPP} ${HEADER_H})

IF(CHOLMOD_FOUND)
	TARGET_LINK_LIBRARIES(teb_package ${CHOLMOD_LIBRARIES})
ENDIF(CHOLMOD_FOUND)

IF(NLOPT_FOUND)
	TARGET_LINK_LIBRARIES(teb_package ${NLOPT_LIBRARIES})
ENDIF(NLOPT_FOUND)


# ===== compile teb_package examples =====

include(${CMAKE_SOURCE_DIR}/cmake/macros/DirFileParser.cmake)

if (BUILD_EXAMPLES)
	FOREACH(ELEMENT ${EXAMPLE_NAMES})
	  # parse element string to get directory and name (only one subdirectory supported)
	  GET_DIR_FILE(${ELEMENT} EL_PATH EL_FILE)
	  ADD_EXECUTABLE(${EL_FILE} ${CMAKE_SOURCE_DIR}/examples/${EL_PATH}/${EL_FILE}.cpp)
	  TARGET_LINK_LIBRARIES(${EL_FILE} teb_package)
	  set_target_properties(${EL_FILE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${EL_PATH})
	ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})
endif (BUILD_EXAMPLES)


# INCLUDE car_like_system EXAMPLE

 add_subdirectory(${CMAKE_SOURCE_DIR}/examples/car_like_system)
# add_subdirectory(${CMAKE_SOURCE_DIR}/examples/diff_drive_system)
#add_subdirectory(${CMAKE_SOURCE_DIR}/examples/test_suite_mobile_robots)




# MY EXECUTABLES!!!!!!!
#ADD_EXECUTABLE(car
#${CMAKE_SOURCE_DIR}/examples/car_like_system/car_like_system_obstacle_main.cpp  ${HEADER_H})
#${CMAKE_SOURCE_DIR}/examples/carlike/carkul2.cpp
#${CMAKE_SOURCE_DIR}/examples/carlike/carkul3.cpp)
#TARGET_LINK_LIBRARIES(car teb_package)
#set_target_properties(car PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/car_like_system)





##########

# ===== Compile MEX examples ===== 

IF(MATLAB_FOUND)
	add_definitions(-DMEX)

	IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	  # POLICY CMP0042
	  cmake_policy(SET CMP0042 NEW)
	ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	  # activate fPIC for shared libraries
	  set(CMAKE_CXX_FLAGS "${CMAKE_CXX__FLAGS} -fPIC")
	  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC")
	  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fPIC")
	ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

	# FIX, if CMAKE_OSX_ARCHITECTURES variable is empty and system is a common mac
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin" AND "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "")
		IF(CMAKE_SIZEOF_VOID_P MATCHES "4") # 32Bit
			set(CMAKE_OSX_ARCHITECTURES i386)
		ELSE(CMAKE_SIZEOF_VOID_P MATCHES "4") # 64Bit
			set(CMAKE_OSX_ARCHITECTURES x86_64)
		ENDIF(CMAKE_SIZEOF_VOID_P MATCHES "4")
	ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin" AND "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "")

	include_directories(${MATLAB_INCLUDE_DIR})
	include(${CMAKE_SOURCE_DIR}/cmake/macros/MatlabMakeMacros.cmake)

	if (BUILD_EXAMPLES)
		FOREACH(ELEMENT ${MEX_EXAMPLE_NAMES})
			# parse element string to get directory and name (only one subdirectory supported)
			GET_DIR_FILE(${ELEMENT} EL_PATH EL_FILE)
			ADD_MEX_FILE(${EL_FILE} ${CMAKE_SOURCE_DIR}/examples/${EL_PATH}${EL_FILE}.cpp)
			TARGET_LINK_LIBRARIES(${EL_FILE} teb_package)
			set_target_properties(${EL_FILE} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}/${EL_PATH})
		ENDFOREACH(ELEMENT ${MEX_EXAMPLE_NAMES})
	endif (BUILD_EXAMPLES)
	
	# Compile S-FUNCTIONS
	IF(MATLAB_SIMULINK_FOUND)
	  message(STATUS "Simulink found. Compiling S-Function examples.")
	  include_directories(${MATLAB_SIMULINK_INCLUDE_DIR})
	  
	  add_definitions(-DMATLAB_MEX_FILE)
	  
	  if (BUILD_EXAMPLES)
		  FOREACH(ELEMENT ${SFUN_EXAMPLE_NAMES})
			# parse element string to get directory and name (only one subdirectory supported)
			GET_DIR_FILE(${ELEMENT} EL_PATH EL_FILE)
			ADD_MEX_FILE(${EL_FILE} ${CMAKE_SOURCE_DIR}/examples/${EL_PATH}${EL_FILE}.cpp)
			TARGET_LINK_LIBRARIES(${EL_FILE} teb_package)
			set_target_properties(${EL_FILE} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}/${EL_PATH})
		  ENDFOREACH(ELEMENT ${SFUN_EXAMPLE_NAMES})
	  endif (BUILD_EXAMPLES)
  		
	ELSE(MATLAB_SIMULINK_FOUND)
	  message(STATUS "Simulink not found. Skipping S-Function example.")
	ENDIF(MATLAB_SIMULINK_FOUND)
ENDIF(MATLAB_FOUND)



# ===== Building API documentation ===== 

IF( DOXYGEN_FOUND )
	ADD_CUSTOM_TARGET( doc
		${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/doxygen.config
		WORKING_DIRECTORY
			${PROJECT_SOURCE_DIR}/doc
		COMMENT
			"Generating API documentation with Doxygen" VERBATIM
	)
ENDIF( DOXYGEN_FOUND )



# ====== Unit testing =============

if (BUILD_UNIT_TESTS)
	find_package(Subversion)
	if(SUBVERSION_FOUND)
		# subversion will be checkout during the build process
		add_subdirectory(${PROJECT_SOURCE_DIR}/extern/gtest)
		
		enable_testing()
		if (WIN32)
			message(STATUS "Unit tests are enabled. Googletest Framework will be downloaded during the build process. Call tests by building RUN_TESTS in Visual Studio or by running the generated exe-file (verbose output).")
		else (WIN32)
			message(STATUS "Unit tests are enabled. Googletest Framework will be downloaded during the build process. Call tests using 'make test' inside the build folder, by running the generated binary or by using your IDE.")
		endif(WIN32)
		
		set(PROJECT_TEST_NAME teb_package_test)
		include_directories(${GTEST_INCLUDE_DIRS})
		file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp) # get all test source (cpp) files
		add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
		add_dependencies(${PROJECT_TEST_NAME} googletest)
		target_link_libraries(${PROJECT_TEST_NAME}
								teb_package
								debug ${GTEST_LIB_DEBUG}
								debug ${GTEST_LIB_MAIN_DEBUG}
								optimized ${GTEST_LIB_RELEASE}
								optimized ${GTEST_LIB_MAIN_RELEASE})

		add_test(NAME ${PROJECT_TEST_NAME} COMMAND ${PROJECT_TEST_NAME}) # we are using gtest for several tests, therefore add just a single test here	
	
		# On unix, googlecode is compiled using the pthread library. We need to link against it ...
		if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
			set(new_link_flags "-pthread")
			get_target_property(existing_link_flags ${PROJECT_TEST_NAME} LINK_FLAGS)
			if(existing_link_flags)
			    set(new_link_flags "${existing_link_flags} ${new_link_flags}")
			endif()
			set_target_properties(${PROJECT_TEST_NAME} PROPERTIES LINK_FLAGS ${new_link_flags})
		endif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")

	elseif (WIN32)
		message(WARNING "Unit tests are enabled, but cannot use gtest for unit testing since finding subversion executable failed. If you are using TortoiseSVN, make sure to install the command line tools. You can restart the TortoiseSVN installer and select it....")
	else (WIN32)
		message(WARNING "Unit tests are enabled, but cannot use gtest for unit testing since finding subversion executable failed.")
	endif(SUBVERSION_FOUND)
endif (BUILD_UNIT_TESTS)
