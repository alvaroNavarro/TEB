# include_directories(blabla)

# testing program
#add_executable(test 
 # testDiffDrive.cpp 
  #test_plotRobot.cpp
  #test_new_obstacles.cpp
#)
#target_link_libraries(test teb_package)
#set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/diff_drive_system)

# set(diff_source_files
#      controller_diff_drive.h
#      differential_drive_system.h
#      cost_edges_diff_drive.h    
# )
# 
# add_library(diff_drive_system ${diff_source_files})
# target_link_libraries(diff_drive_system teb_package)

# MATLAB INTERFACE
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
	
	
	 # ADD FILES HERE
	#ADD_MEX_FILE(car_like_system_sfun car_like_system_sfun.cpp)
	#TARGET_LINK_LIBRARIES(car_like_system_sfun teb_package)
	#set_target_properties(car_like_system_sfun PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/car_like_system)
	
	
	# Compile S-FUNCTIONS
	IF(MATLAB_SIMULINK_FOUND)
	  message(STATUS "Simulink found. Compiling S-Function examples.")
	  include_directories(${MATLAB_SIMULINK_INCLUDE_DIR})
	  
	  add_definitions(-DMATLAB_MEX_FILE)
	  
	  
        # ADD FILES HERE
	#ADD_MEX_FILE(carlike_system_sfun carlike_system_sfun.cpp)
	#TARGET_LINK_LIBRARIES(car_like_system_sfun teb_package)
	#set_target_properties(car_like_system_sfun PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/car_like_system)
	  
  		
	ELSE(MATLAB_SIMULINK_FOUND)
	  message(STATUS "Simulink not found. Skipping S-Function example.")
	ENDIF(MATLAB_SIMULINK_FOUND)
ENDIF(MATLAB_FOUND)
