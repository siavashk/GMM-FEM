#ifndef MAS_TIME_H_
#define MAS_TIME_H_

#include <chrono>

namespace mas {
namespace time {

typedef std::chrono::high_resolution_clock chrono_clock;
typedef std::chrono::time_point<chrono_clock> chrono_time;
typedef chrono_clock::duration chrono_duration;

/**
 * @brief Simple timer.
 *
 * Simple timer, at least microsecond accuracy.
 */
class Timer {
private:
   chrono_time starttime;
   chrono_duration elapsed;
   bool running;

public:
   Timer();

   /**
    * @brief Starts the timer.
    *
    * Starts the timer.  If the timer is already running, resets
    * time to zero
    */
   void start();

   /**
    * @brief Resumes the timer.
    *
    * Resumes timing.  If the timer is already running, does nothing.
    */
   void resume();

   /**
    * @brief Stops the timer.
    *
    * Stops timing, remembering the current elapsed time.
    */
   void stop();

   /**
    * @brief Resets the timer.
    *
    * Sets the current elapsed time to zero.
    */
   void reset();

   /**
    * @brief Elapsed time in microseconds.
    *
    * Returns elapsed time in microseconds.
    */
   double getMicroseconds();

   /**
    * @brief Elapsed time in milliseconds.
    *
    * Returns elapsed time in milliseconds.
    */
   double getMilliseconds();

   /**
    * @brief Elapsed time in seconds.
    *
    * Returns elapsed time in seconds.
    */
   double getSeconds();
};

} /* namespace time */
} /* namespace mas */

#endif /* MAS_TIME_H_ */
