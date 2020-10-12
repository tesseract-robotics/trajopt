#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <chrono>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/clock.hpp>

namespace util
{
static thread_local std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

/*
 * Starts the clock!  Call this once at the beginning of the program.
 * Calling again will reset the clock to 0. It store the time in a thread local.
 */
// time in units of milliseconds since some time in the past
void StartClock() { start_time = std::chrono::high_resolution_clock::now(); }

/** @brief Returns the current time since the call to StartClock() in milliseconds */
double GetClock()
{
  auto current_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double, std::milli>(current_time - start_time).count();
}
}  // namespace util
