#pragma once

#include <cpp_core/default.h>
#include <cpp_core/core/ChronoEntry.h>

class Chrono
{
public:
    static Chrono global_instance;

    Chrono() {}
    ~Chrono() {}

    std::string start();
    std::string start(std::string key);
    int64_t stop(std::string key);

private:
    std::map<std::string, ChronoEntry> entries;
};
