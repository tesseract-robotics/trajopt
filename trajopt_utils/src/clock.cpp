#include <sys/time.h>
#include <time.h>
#include <trajopt_utils/clock.hpp>

namespace util
{
static long unsigned int startTime = 0;

/*
 * Starts the clock!  Call this once at the beginning of the program.
 * Calling again will reset the clock to 0;  but doing so is not
 * thread-safe if other threads may be calling GetClock(); (which
 * is thread-safe since it only reads values and calls thread-safe
 * functions in the kernel).
 */
// time in units of seconds since some time in the past
void StartClock()
{
  // determine start time
  struct timeval startTimeStruct;
  gettimeofday(&startTimeStruct, NULL);
  startTime = startTimeStruct.tv_sec * (long unsigned int)(1e6) + startTimeStruct.tv_usec;
}

/*
 * Returns the current time since the call to StartClock();
 */
double GetClock()
{
  struct timeval startTimeStruct;
  unsigned long int curTime;
  gettimeofday(&startTimeStruct, NULL);
  curTime = startTimeStruct.tv_sec * (long unsigned int)(1e6) + startTimeStruct.tv_usec;
  return (1e-6) * (curTime - startTime);
}
}
