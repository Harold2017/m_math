cmake_minimum_required(VERSION 3.17)
project(m_math)

set(CMAKE_CXX_STANDARD 11)

find_package(GSL REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules" ${CMAKE_MODULE_PATH})
find_package(FFTW REQUIRED)
include_directories(${FFTW_INCLUDE_DIRS})

find_package(CGAL REQUIRED)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

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
add_executable(m_curvature examples/curvature.cpp)
add_executable(m_filter examples/filter.cpp)
add_executable(m_peak examples/peak.cpp)
add_executable(m_p1d examples/p1d.cpp)
add_executable(m_integration examples/integration.cpp)
add_executable(m_fft_shift examples/fft_shift.cpp)
add_executable(m_autoc examples/autoc.cpp)
add_executable(m_hist examples/hist.cpp)
add_executable(m_fft_opencv examples/fft_opencv.cpp)
add_executable(m_p2d_opencv examples/p2d_opencv.cpp)
add_executable(m_area_opencv examples/area_opencv.cpp)
add_executable(m_distance_CGAL examples/distance.cpp)
add_executable(m_filters_opencv examples/filters_opencv.cpp)

target_link_libraries(m_interp GSL::gsl GSL::gslcblas)
target_link_libraries(m_fft ${FFTW_LIBRARIES})
target_link_libraries(m_fft_shift ${FFTW_LIBRARIES})
target_link_libraries(m_curvefit GSL::gsl GSL::gslcblas)
target_link_libraries(m_geo CGAL::CGAL GSL::gsl GSL::gslcblas)
target_link_libraries(m_curvature GSL::gsl GSL::gslcblas)
target_link_libraries(m_filter GSL::gsl GSL::gslcblas)
target_link_libraries(m_integration GSL::gsl GSL::gslcblas)
target_link_libraries(m_autoc ${FFTW_LIBRARIES})
target_link_libraries(m_hist GSL::gsl GSL::gslcblas)
target_link_libraries(m_fft_opencv ${OpenCV_LIBS})
target_link_libraries(m_p2d_opencv ${OpenCV_LIBS})
target_link_libraries(m_area_opencv ${OpenCV_LIBS})
target_link_libraries(m_distance_CGAL CGAL::CGAL)
target_link_libraries(m_filters_opencv ${OpenCV_LIBS})

# drafts
add_executable(d_fitler drafts/filter.cpp)
target_link_libraries(d_fitler ${OpenCV_LIBS})
