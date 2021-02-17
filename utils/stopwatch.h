//
// Created by Harold on 2020/10/26.
//

#ifndef M_MATH_STOPWATCH_H
#define M_MATH_STOPWATCH_H

#include <chrono>
#include <iostream>
#include <sstream>

struct BlockWatch
{
    std::string name;
    std::chrono::high_resolution_clock::time_point start_time;
    std::ostream &log;
    explicit BlockWatch(std::string n, std::ostream& os = std::cout)
        : name(std::move(n)),
          start_time(std::chrono::high_resolution_clock::now()),
          log(os) { }
    ~BlockWatch()
    {
        auto ns = std::chrono::high_resolution_clock::now() - start_time;
        auto m = std::chrono::duration_cast<std::chrono::minutes>(ns);
        ns -= m;
        auto s = std::chrono::duration_cast<std::chrono::seconds>(ns);
        ns -= s;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(ns);
        ns -= ms;
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(ns);
        ns -= us;
        log << name << ": ";
        if (m.count())
            log << m.count() << " minutes ";
        if (s.count())
            log << s.count() << " s ";
        if (ms.count())
            log << ms.count() << " ms ";
        if (us.count())
            log << us.count() << " us ";
        log << ns.count() << " ns" << '\n';
    }
};

class StopWatch {
public:
    enum TimeFormat{ NANOSECONDS, MICROSECONDS, MILLISECONDS, SECONDS };

public:
    StopWatch(): name(), log(std::cout), fmt_(NANOSECONDS) {}
    explicit StopWatch(std::string n, TimeFormat fmt, std::ostream& os = std::cout)
        : name(std::move(n)),
          log(os),
          fmt_(fmt) { }

    void start() {
        start_time = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        auto duration = std::chrono::high_resolution_clock::now() - start_time;
        auto ns_count = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        log << name << ": " << get_ticks_str(ns_count) << '\n';
    }

    void stop(std::string const& n) {
        auto duration = std::chrono::high_resolution_clock::now() - start_time;
        auto ns_count = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        log << n << ": " << get_ticks_str(ns_count) << '\n';
    }

    void stop(std::string const& n, TimeFormat _fmt) {
        auto duration = std::chrono::high_resolution_clock::now() - start_time;
        auto ns_count = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        fmt_ = _fmt;
        log << n << ": " << get_ticks_str(ns_count) << '\n';
    }

    void stop(std::string const& n, TimeFormat _fmt, std::ostream& os) {
        auto duration = std::chrono::high_resolution_clock::now() - start_time;
        auto ns_count = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        fmt_ = _fmt;
        os << n << ": " << get_ticks_str(ns_count) << '\n';
    }

private:
    std::string name;
    std::ostream &log;
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_pt;
    time_pt start_time;
    TimeFormat fmt_;

    std::string get_ticks_str(long long ns_count) {
        std::stringstream buffer;
        switch(fmt_) {
            case TimeFormat::NANOSECONDS: {
                buffer << ns_count << " ns";
                break;
            }
            case TimeFormat::MICROSECONDS: {
                std::uint64_t up = ((ns_count / 100) % 10 >= 5) ? 1 : 0;
                const auto mus_count = (ns_count / 1000) + up;
                buffer << mus_count << " us";
                break;
            }
            case TimeFormat::MILLISECONDS: {
                std::uint64_t up = ((ns_count / 100000) % 10 >= 5) ? 1 : 0;
                const auto ms_count = (ns_count / 1000000) + up;
                buffer << ms_count << " ms";
                break;
            }
            case TimeFormat::SECONDS: {
                std::uint64_t up = ((ns_count / 100000000) % 10 >= 5) ? 1 : 0;
                const auto s_count = (ns_count / 1000000000) + up;
                buffer << s_count << " s";
                break;
            }
        }
        return buffer.str();
    }
};

#define TIME_BLOCK(name) BlockWatch _pfinstance(name)
#define TIME_BLOCK_WITH_LOG(name, log) BlockWatch _pfinstance(name, log)

#endif //M_MATH_STOPWATCH_H
