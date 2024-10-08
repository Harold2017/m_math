cmake_minimum_required(VERSION 3.17)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE INTERNAL "" FORCE)
endif()

project(m_math)

# Set build type to DEBUG by default:
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type provided, default to DEBUG.")
    set(CMAKE_BUILD_TYPE "DEBUG")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(M_MATH_TEST      "Build tests" OFF)
option(M_MATH_BENCH     "Build benchmarks" OFF)
option(M_MATH_STATIC    "Builds a static library instead of a shared one" OFF)
option(M_MATH_SANITIZE  "Adds sanitive flags" OFF)

option(USE_OPEN3D      "USE_OPEN3D" OFF)
option(USE_TBBMALLOC   "USE_TBBMalloc" OFF)

if (M_MATH_STATIC)
    set(M_MATH_LIB_NAME m_math_static)
    set(M_MATH_LIB_TYPE STATIC)
else()
    set(M_MATH_LIB_NAME m_math)
    set(M_MATH_LIB_TYPE SHARED)
endif()

if(M_MATH_SANITIZE)
    set(CMAKE_CXX_FLAGS "-fsanitize=address -fsanitize=undefined ${CMAKE_CXX_FLAGS}")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules" ${CMAKE_MODULE_PATH})

find_package(Threads REQUIRED)

#find_package(GSL REQUIRED)

#find_package(FFTW REQUIRED)
#include_directories(${FFTW_INCLUDE_DIRS})

#find_package(CGAL REQUIRED)
#set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

#Eigen3
# system eigen3 may conflict with eigen header files in open3d erd party,
# so disable it when use open3d
if (USE_OPEN3D)
    find_package(Open3D REQUIRED)
    include_directories(${Open3D_INCLUDE_DIRS})
else()
    if (APPLE)
        find_package(Eigen3 REQUIRED)
        #message(${EIGEN3_INCLUDE_DIR})
    elseif(WIN32)
        find_path(EIGEN_INCLUDE_DIR NAMES Eigen/CORE)
        message(STATUS "Eigen include dir: ${EIGEN_INCLUDE_DIR}")
        include_directories(${EIGEN_INCLUDE_DIR})
    endif()
endif(USE_OPEN3D)

if (USE_TBBMALLOC)
	find_package(TBB REQUIRED COMPONENTS tbbmalloc tbbmalloc_proxy)
endif(USE_TBBMALLOC)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
        include
        utils)

if (M_MATH_TEST)
    add_subdirectory(tests)
endif()

if (M_MATH_BENCH)
    add_subdirectory(bench)
endif()

message(STATUS "")
message(STATUS "BUILD SUMMARY")
message(STATUS "  CMAKE_GENERATOR      : ${CMAKE_GENERATOR}")
message(STATUS "  Compiler ID          : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Library name         : ${M_MATH_LIB_NAME}")
message(STATUS "  Library type         : ${M_MATH_LIB_TYPE}")
message(STATUS "  Build tests          : ${M_MATH_TEST}")
message(STATUS "  Build benchmarks     : ${M_MATH_BENCH}")
message(STATUS "  Sanitize flags       : ${M_MATH_SANITIZE}")
message(STATUS "  Boost include dirs   : ${Boost_INCLUDE_DIRS}")
message(STATUS "  OpenMP found         : ${OPENMP_CXX_FOUND}")
message(STATUS "  Use Open3D           : ${USE_OPEN3D}")
message(STATUS "  Use TBBMALLOC        : ${USE_TBBMALLOC}")
message(STATUS "")

# examples
add_subdirectory(examples)

# drafts
add_subdirectory(drafts)

# building info
add_executable(opencv_building_info examples/opencv_building_info.cpp)
target_link_libraries(opencv_building_info ${OpenCV_LIBS})
