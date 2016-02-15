CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT( EFSLAM )

set(EFSLAM_MAJOR_VERSION 3)
set(EFSLAM_MINOR_VERSION 0)
set(EFSLAM_PATCH_VERSION 0)
set(EFSLAM_VERSION ${EFSLAM_MAJOR_VERSION}.${EFSLAM_MINOR_VERSION}.${EFSLAM_PATCH_VERSION})
set(EFSLAM_SOVERSION ${EFSLAM_MAJOR_VERSION}.${EFSLAM_MINOR_VERSION})

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# compiler settings and flags
include(CompilerSettings)


#-------------
# OpenMP
#-------------
set(EFSLAM_OMP TRUE CACHE BOOL "Enable/disable OpenMP parallelization")

if(DEFINED ENV{EFSLAM_OMP})
    set(EFSLAM_OMP $ENV{EFSLAM_OMP})
endif(DEFINED ENV{EFSLAM_OMP})

if(EFSLAM_OMP)
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(EFSLAM_OMP)


#-------------
# std::thread
#-------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


#-------------
# Qt4
#-------------
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 REQUIRED QtGui QtCore)
include( ${QT_USE_FILE} )


#-------------
# Eigen
#-------------
find_package(Eigen3)
add_definitions( -DHAVE_EIGEN )
if(NOT EIGEN3_FOUND)
	FIND_PACKAGE(PkgConfig)
	pkg_check_modules(EIGEN3 eigen3 REQUIRED)
endif()
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Please install Eigen3.")
endif()


#-------------
# Directories
#-------------
include_directories(
    ${INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


link_directories(
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

#--------------
# Installation
#--------------
set( BASE_DIR ${CMAKE_SOURCE_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

set(INSTALL_TARGETS_DEFAULT_ARGS
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

file(MAKE_DIRECTORY ${BASE_DIR}/results)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/efslam/
    DESTINATION include/efslam/
    FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/efslam/
    DESTINATION include/efslam/
    FILES_MATCHING PATTERN "*.hpp")


add_subdirectory( src )
 

# Finished
message("\n")
message(STATUS "Compile EFSLAM using: make")
message(STATUS "Install EFSLAM using: make install")
message(STATUS " (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
message("\n")
