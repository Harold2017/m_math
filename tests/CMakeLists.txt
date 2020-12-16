# gtest
enable_testing()
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

set(test_src
        run_all.cpp)

add_executable(test_m_math ${test_src})

target_link_libraries(test_m_math
        GTest::GTest
        GTest::Main)

add_test(
        NAME AllTests
        COMMAND test_m_math)