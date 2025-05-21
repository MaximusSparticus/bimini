# -----------------------------------------------------------------------------
# Author:   Zachary Max Diekel
# Date:     May 2025
# Brief:    This is the CMake project file for Bimini with the primary purpose
#           of building the bimini library which is a BIM interface library,
#           used facilitate the interface between BIM and a simulation
#           environment
#
# Usage:
#           cmake -S . -B build && cmake --build build -j[N] --config Release
#               -DBIMINI_BUILD_TESTS=ON     ([ON]  to build unit tests)
#               -DBIMINI_BUILD_DOCS=OFF     ([OFF] to build documentation)
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(bimini VERSION 0.1.0 LANGUAGES CXX)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler verbosity
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
endif()

# Add options
option(BIMINI_BUILD_TESTS "Build the bimini test suite" ON)
option(BIMINI_BUILD_DOCS "Build documentation" OFF)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Library source files
file(GLOB_RECURSE BIMINI_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bimini/*.cpp
)

# Library header files
file(GLOB_RECURSE BIMINI_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bimini/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bimini/*.hpp
)

# Create the library
add_library(bimini ${BIMINI_SOURCES} ${BIMINI_HEADERS})
target_include_directories(bimini
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/ifcpp/IFC4X3/include
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/glm
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/Carve/include
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/Carve/src
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/Carve/src/include
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/Carve/src/common
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/Carve/build/src
	    ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/IfcPlusPlus/src/external/glm
)

# Handle IfcPlusPlus dependency and silence warnings
set(SAVED_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
endif()

# Force IfcPlusPlus options to be OFF
set(BUILD_CONSOLE_APPLICATION OFF CACHE BOOL "Build console application" FORCE)
set(BUILD_VIEWER_APPLICATION OFF CACHE BOOL "Build viewer application" FORCE)
ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_UNICODE)
ADD_DEFINITIONS(-DIFCQUERY_STATIC_LIB)

# Add submodule with build configuration
add_subdirectory(deps/ifcplusplus)
set(CMAKE_CXX_FLAGS ${SAVED_CMAKE_CXX_FLAGS})

# Check if IfcPlusPlusd target exists
if(TARGET IfcPlusPlusd)
    # Debug target exists, use conditional linking
    target_link_libraries(bimini PUBLIC
        $<$<CONFIG:Debug>:IfcPlusPlusd>
        $<$<NOT:$<CONFIG:Debug>>:IfcPlusPlus>
    )
else()
    # Debug target doesn't exist, always use IfcPlusPlus
    target_link_libraries(bimini PUBLIC IfcPlusPlus)
    message(STATUS "IfcPlusPlusd target not found, using IfcPlusPlus for all configurations")
endif()

# Setup installation
include(GNUInstallDirs)
install(TARGETS bimini
    EXPORT biminiTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/bimini
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Generate and install package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/biminiConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/biminiConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/biminiConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bimini
)
install(EXPORT biminiTargets
    FILE biminiTargets.cmake
    NAMESPACE bimini::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bimini
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/biminiConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/biminiConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bimini
)

# Testing
if(BIMINI_BUILD_TESTS)
    enable_testing()
    # Add googletest
    add_subdirectory(deps/googletest)
    
    # Find all test files
    file(GLOB_RECURSE TEST_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/test/bimini/*.cpp
    )
    
    # Create test executable
    add_executable(bimini-test ${TEST_SOURCES})
    target_link_libraries(bimini-test
        PRIVATE
            bimini
            GTest::gtest
            GTest::gtest_main
            GTest::gmock
    )

    # Copy example IFC file to the test binary directory
    add_custom_command(
        TARGET bimini-test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/ifcplusplus/examples/LoadFileExample/example.ifc
        $<TARGET_FILE_DIR:bimini-test>/test/test.ifc
        COMMENT "Copying example.ifc to test binary directory"
    )

    # Add tests to be run by CTest
    add_test(NAME bimini-test COMMAND bimini-test)
endif()

# Documentation
if(BIMINI_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        # Set up doxygen configuration
        set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        # Configure the Doxyfile
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        # Add custom target for documentation
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message(STATUS "Doxygen not found, documentation will not be built")
    endif()
endif()

# Export compile commands for tooling support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
