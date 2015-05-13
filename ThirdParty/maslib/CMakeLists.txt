cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(maslib CXX)

# Compiler-specific C++11 activation.
if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7 or greater.")
    endif ()
    set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
elseif (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -stdlib=libc++")
endif ()

# find matlab
include(FindMatlabLatest.cmake CMAKE_FIND_MATLAB)

# define paths
set(SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(COMMONDIR ${SRCDIR}/common)
set(MEXDIR ${SRCDIR}/mex)
set(CMDDIR ${SRCDIR}/cmd)
set(TESTDIR ${SRCDIR}/test)

# default install directory
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "default install path" FORCE )
endif()

include_directories(${COMMONDIR})
include_directories(${MEXDIR})
include_directories(${MATLAB_INCLUDE_DIR})

# include source files
include(CMakeSources.txt RESULT_VARIABLE CMAKE_SOURCES_FILE)

# maslib
add_library(maslib SHARED ${COMMON_SOURCES})
install(TARGETS maslib DESTINATION bin)

# cmd files
foreach(CMD_SOURCE ${CMD_SOURCES})
    string(REGEX REPLACE ".*/\(.*\)_cmd.cxx" "\\1" CMD_TARGET ${CMD_SOURCE})
    add_executable(${CMD_TARGET} ${CMD_SOURCE})
    target_link_libraries(${CMD_TARGET} maslib)
    install(TARGETS ${CMD_TARGET} DESTINATION bin)
endforeach()

# mex files
foreach(MEX_SOURCE ${MEX_SOURCES})
    string(REGEX REPLACE ".*/\(.*\)_mex.cxx" "\\1" MEX_TARGET ${MEX_SOURCE})
    add_library(${MEX_TARGET} MODULE ${MEX_SOURCE})
    target_link_libraries(${MEX_TARGET} maslib)
    target_link_libraries(${MEX_TARGET} ${MATLAB_LIBRARIES})
    target_compile_definitions(${MEX_TARGET} PUBLIC "MX_COMPAT_32" "MATLAB_MEX_FILE")
    set_target_properties(${MEX_TARGET} PROPERTIES PREFIX "")
    set_target_properties(${MEX_TARGET} PROPERTIES SUFFIX ".${MATLAB_MEX_EXTENSION}")
    install(TARGETS ${MEX_TARGET} DESTINATION bin)
endforeach()

# m-files
install(FILES ${M_SOURCES} DESTINATION bin)
