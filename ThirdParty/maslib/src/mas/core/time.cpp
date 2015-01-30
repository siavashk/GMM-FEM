/*
 * Timer.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Antonio
 */

#include "mas/core/time.h"
#include<memory>

namespace mas {
namespace time {

Timer::Timer() {
   starttime = chrono_clock::now();
   elapsed = chrono_duration::zero();
   running = false;
}

void Timer::start() {
   starttime = chrono_clock::now();
   elapsed = chrono_duration::zero();
   running = true;
}

void Timer::resume() {
   if (!running) {
      chrono_time now = chrono_clock::now();
      starttime = now-elapsed;
      running = true;
   }
}

void Timer::stop() {
   if (running) {
      chrono_time end = chrono_clock::now();
      elapsed = end-starttime;
      running = false;
   }
}

void Timer::reset() {
   elapsed = chrono_duration::zero();
   starttime = chrono_clock::now();
}

double Timer::getMicroseconds() {
   if (running) {
      chrono_time now = chrono_clock::now();
      return std::chrono::duration_cast<std::chrono::duration<double,std::micro>>(now-starttime).count();
   } else {
      return std::chrono::duration_cast<std::chrono::duration<double,std::micro>>(elapsed).count();
   }
}

double Timer::getMilliseconds() {
   if (running) {
      chrono_time now = chrono_clock::now();
      return std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(now-starttime).count();
   } else {
      return std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(elapsed).count();
   }
}

double Timer::getSeconds() {
   if (running) {
      chrono_time now = chrono_clock::now();
      return std::chrono::duration_cast<std::chrono::duration<double>>(now-starttime).count();
   } else {
      return std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
   }
}

} /* namespace time */
} /* namespace mas */
