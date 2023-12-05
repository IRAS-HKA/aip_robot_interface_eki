#pragma once

#include <cpp_core/default.h>
#include <cpp_core/core/LogLevel.h>
#include <cpp_core/core/ChronoTime.h>

struct LogEntry
{
    std::string message;
    unsigned int hierarchy;
    LogLevel level;
    ChronoTime time;

    std::string to_string() const { return "[" + level.to_string() + " | " + time.to_string("%H:%M:%S") + "]: " + message; }
};
