/*
 * TIMER.hpp
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#ifndef __timer_hpp__
#define __timer_hpp__

#include <inttypes.h>
#include <vector>
#include <string>

/**
 * @brief quick and dirty profiling tool.
 * @ingroup BotCoreTime
 *
 * inspired by the matlab tic/toc command
 *
 * call TIMER("description") to set the timer going
 * call it again with the same description to stop the timer
 *
 * Note: To get output, set the "FOVIS_TIMER" environment variable to something
 *
 * @{
 */

/**
 * Structure to keep track of timing information for each TIMER entry.
 */
struct tick_t
{
  int64_t t;
  int64_t totalT;
  int64_t ema;
  int64_t min;
  int64_t max;
  int numCalls;
  char flag;
  std::string description;
};

/**
 * TIMER:
 *
 * basic invocation, the second time its called, it returns the time difference in microseconds
 **/
int64_t
timer(const char *description);

/**
 * TIMER_sort_type_t:
 *
 * Different Options for sorting the printed results
 */
typedef enum
{
    TIMER_AVG,
    tick_tOTAL,
    TIMER_MIN,
    TIMER_MAX,
    TIMER_EMA,
    TIMER_ALPHABETICAL
} timer_sort_type_t;

/**
 * TIMER_print_stats:
 *
 * Print Out the stats from TIMER
 */
void timer_print_stats( timer_sort_type_t sortType = TIMER_AVG );

/**
 * Get TIMER entries.
 */
void timer_get_stats(std::vector<tick_t> *stats);

#endif
