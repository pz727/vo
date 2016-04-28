/*
 * TIMER.c
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include "timer.hpp"

//simple, quick and dirty profiling tool...

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static bool
_tick_t_avgTimeCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return (t1->totalT / t1->numCalls) < (t2->totalT / t2->numCalls);
}

static bool
_tick_t_totalTimeCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->totalT < t2->totalT;
}

static bool
_tick_t_maxTimeCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->max < t2->max;
}

static bool
_tick_t_minTimeCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->min > t2->min;
}

static bool
_tick_t_emaTimeCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->ema < t2->ema;
}

static bool
_tick_t_alphCompare(const tick_t* t1, const tick_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return strcmp(t1->description.c_str(), t2->description.c_str());
}

typedef std::map<std::string, tick_t> TimerMap;
static TimerMap _timer_map;


int64_t
timer(const char *description)
{
    int64_t ret = 0;

    int64_t TIMERtime = _timestamp_now();
    TimerMap::iterator eiter = _timer_map.find(description);
    if(eiter == _timer_map.end()) 
    {
        //first time around, allocate and set the timer goin...
        tick_t entry;
        entry.flag = 1;//设置为已调用
        entry.t = TIMERtime;
        entry.totalT = 0;
        entry.numCalls = 0;
        entry.max = -1e15;
        entry.min = 1e15;
        entry.ema = 0;
        entry.description = description;
        _timer_map[description] = entry;
        ret = TIMERtime;
    } 
    else if (eiter->second.flag == 0) 
    {
        eiter->second.flag = 1;
        eiter->second.t = TIMERtime;
        ret = TIMERtime;
    } 
    else 
    {
        eiter->second.flag = 0;//设置为未调用
        int64_t dt = TIMERtime - eiter->second.t;
        eiter->second.numCalls++;
        eiter->second.totalT += dt;
        if (dt < eiter->second.min)
            eiter->second.min = dt;
        if (dt > eiter->second.max)
            eiter->second.max = dt;
        // eiter->second.ema = (1.0 - ema_alpha) * eiter->second.ema + ema_alpha * dt;
        // if (ema != NULL)
        //     *ema = eiter->second.ema;
        eiter->second.ema = 0;
        ret = dt;
    }
    return ret;
}

void timer_get_stats(std::vector<tick_t> *stats)
{
    TimerMap::iterator iter = _timer_map.begin();
    TimerMap::iterator eiter = _timer_map.end();
    for(; iter != eiter; ++iter) {
      stats->push_back(iter->second);
    }
}

void timer_print_stats( timer_sort_type_t sortType )
{
  TimerMap::iterator iter = _timer_map.begin();
  TimerMap::iterator eiter = _timer_map.end();
  std::vector<const tick_t*> entries;
  for(; iter != eiter; ++iter) 
  {
    entries.push_back( &iter->second );
  }

    printf("\n--------------------------------------------\n");
    printf("timer Statistics, sorted by ");
    switch (sortType)
    {
      case TIMER_AVG:
        printf("average time\n");
        std::sort(entries.begin(), entries.end(), _tick_t_avgTimeCompare);
        break;
      case TIMER_MIN:
        printf("min time\n");
        std::sort(entries.begin(), entries.end(), _tick_t_minTimeCompare);
        break;
      case TIMER_MAX:
        printf("max time\n");
        std::sort(entries.begin(), entries.end(), _tick_t_maxTimeCompare);
        break;
      case TIMER_EMA:
        printf("EMA time\n");
        std::sort(entries.begin(), entries.end(), _tick_t_emaTimeCompare);
        break;
      case tick_tOTAL:
        printf("total time\n");
        std::sort(entries.begin(), entries.end(), _tick_t_totalTimeCompare);
        break;
      case TIMER_ALPHABETICAL:
        printf("alphabetically\n");
        std::sort(entries.begin(), entries.end(), _tick_t_alphCompare);
        break;
      default:
        fprintf(stderr, "WARNING: invalid sort type in TIMER, using AVG\n");
        std::sort(entries.begin(), entries.end(), _tick_t_avgTimeCompare);
        break;
    }
    printf("--------------------------------------------\n");
    for(std::vector<const tick_t*>::iterator iter=entries.begin();
        iter != entries.end(); ++iter) 
    {
      const tick_t* tt = *iter;
      if (tt->numCalls < 1)
        return;
      double totalT = (double) tt->totalT / 1.0e3;
      double avgT = ((double) tt->totalT / (double) tt->numCalls) / 1.0e3;
      double minT = (double) tt->min / 1.0e3;
      double maxT = (double) tt->max / 1.0e3;
      double emaT = (double) tt->ema / 1.0e3;
      printf("%15s ( ms ):\tnumCalls = %d \ttotalT=%.2f \tavgT=%.4f \tminT=%.4f \tmaxT=%.4f\n",
          tt->description.c_str(), tt->numCalls, totalT, avgT, minT, maxT);
    }
    printf("--------------------------------------------\n");
}

