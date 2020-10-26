//
// Created by Harold on 2020/10/26.
//

#include "stopwatch.h"

int main() {

    {
        TIME_BLOCK("test");
        int cnt = 0;
        for (auto i = 0; i < 10000; i++)
            cnt++;
    }

    {
        TIME_BLOCK_WITH_LOG("test1", std::cerr);
        int cnt = 0;
        for (auto i = 0; i < 10000; i++)
            cnt++;
    }

    return 0;
}
