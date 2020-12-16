find_package(benchmark REQUIRED)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Suppressing benchmark's tests" FORCE)

include_directories(${benchmark_INCLUDE_DIRS})

set(bench_src
        run_all.cpp)

add_executable(bench_m_math ${bench_src})

target_link_libraries(bench_m_math
        ${benchmark_LIBRARIES} pthread)