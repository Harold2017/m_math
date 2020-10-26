//
// Created by Harold on 2020/10/26.
//

#ifndef M_MATH_STOPWATCH_H
#define M_MATH_STOPWATCH_H

#include <chrono>
#include <iostream>

struct StopWatch
{
    std::string name;
    std::chrono::high_resolution_clock::time_point p;
    std::ostream &log;
    explicit StopWatch(std::string n, std::ostream& os = std::cout)
        : name(std::move(n)),
          p(std::chrono::high_resolution_clock::now()),
          log(os) { }
    ~StopWatch()
    {
        using dura_nano = std::chrono::nanoseconds;
        using dura_micro = std::chrono::microseconds;
        using dura_mili = std::chrono::milliseconds;
        auto d = std::chrono::high_resolution_clock::now() - p;
        log << name << ": "
            << std::chrono::duration_cast<dura_nano>(d).count()
            << " ns, "
            << std::chrono::duration_cast<dura_micro >(d).count()
            << " us, "
            << std::chrono::duration_cast<dura_mili >(d).count()
            << " ms"
            << std::endl;
    }
};

#define TIME_BLOCK(name) StopWatch _pfinstance(name)
#define TIME_BLOCK_WITH_LOG(name, log) StopWatch _pfinstance(name, log)

#endif //M_MATH_STOPWATCH_H
