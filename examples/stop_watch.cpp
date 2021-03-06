//
// Created by Harold on 2020/10/26.
//

#include "stopwatch.h"

int main() {

    StopWatch sw("sw", StopWatch::NANOSECONDS, std::cerr);
    sw.start();
    int cnt = 0;
    for (auto i = 0; i < 10000; i++)
        cnt++;
    sw.stop();

    {
        TIME_BLOCK("test");
        cnt = 0;
        for (auto i = 0; i < 10000; i++)
            cnt++;
    }

    {
        TIME_BLOCK_WITH_LOG("test1", std::cerr);
        cnt = 0;
        for (auto i = 0; i < 10000; i++)
            cnt++;
    }

    sw.start();
    for (auto i = 0; i < 10000; i++)
        cnt++;
    sw.stop("another name");

    StopWatch sw1;
    sw1.start();
    sw1.stop();
    sw1.stop("any");
    sw1.stop("any", StopWatch::MICROSECONDS);
    sw1.stop("any", StopWatch::MICROSECONDS, std::cerr);
    return 0;
}
