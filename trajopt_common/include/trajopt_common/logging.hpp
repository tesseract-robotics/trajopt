#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cstdio>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
enum LogLevel
{
  LevelFatal = 0,
  LevelError = 1,
  LevelWarn = 2,
  LevelInfo = 3,
  LevelDebug = 4,
  LevelTrace = 5
};

extern TRAJOPT_API LogLevel gLogLevel;  // NOLINT

inline LogLevel GetLogLevel() { return gLogLevel; }
#define FATAL_PREFIX "\x1b[31m[FATAL] "
#define ERROR_PREFIX "\x1b[31m[ERROR] "
#define WARN_PREFIX "\x1b[33m[WARN] "
#define INFO_PREFIX "[INFO] "
#define DEBUG_PREFIX "\x1b[32m[DEBUG] "
#define TRACE_PREFIX "\x1b[34m[TRACE] "
#define LOG_SUFFIX "\x1b[0m\n"

#define LOG_FATAL(msg, ...)                                                                                            \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelFatal)                                                     \
  {                                                                                                                    \
    printf(FATAL_PREFIX);                                                                                              \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
#define LOG_ERROR(msg, ...)                                                                                            \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelError)                                                     \
  {                                                                                                                    \
    printf(ERROR_PREFIX);                                                                                              \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
#define LOG_WARN(msg, ...)                                                                                             \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelWarn)                                                      \
  {                                                                                                                    \
    printf(WARN_PREFIX);                                                                                               \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
#define LOG_INFO(msg, ...)                                                                                             \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelInfo)                                                      \
  {                                                                                                                    \
    printf(INFO_PREFIX);                                                                                               \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
#define LOG_DEBUG(msg, ...)                                                                                            \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelDebug)                                                     \
  {                                                                                                                    \
    printf(DEBUG_PREFIX);                                                                                              \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
#define LOG_TRACE(msg, ...)                                                                                            \
  if (trajopt_common::GetLogLevel() >= trajopt_common::LevelTrace)                                                     \
  {                                                                                                                    \
    printf(TRACE_PREFIX);                                                                                              \
    printf(msg, ##__VA_ARGS__);                                                                                        \
    printf(LOG_SUFFIX);                                                                                                \
  }
}  // namespace trajopt_common
