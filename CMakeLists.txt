cmake_minimum_required(VERSION 3.17)
project(m_math)

set(CMAKE_CXX_STANDARD 11)

find_package(GSL REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules" ${CMAKE_MODULE_PATH})
find_package(FFTW REQUIRED)
include_directories(${FFTW_INCLUDE_DIRS})

find_package(CGAL REQUIRED)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

include_directories(
        include
        utils)

# examples
add_executable(m_math examples/math.cpp)
add_executable(m_interp examples/interp.cpp)
add_executable(m_fft examples/fft.cpp)
add_executable(m_curvefit examples/curvefit.cpp)
add_executable(m_geo examples/geo.cpp)
add_executable(m_peaks examples/peaks.cpp)

target_link_libraries(m_interp GSL::gsl GSL::gslcblas)
target_link_libraries(m_fft ${FFTW_LIBRARIES})
target_link_libraries(m_curvefit GSL::gsl GSL::gslcblas)
target_link_libraries(m_geo CGAL::CGAL GSL::gsl GSL::gslcblas)
