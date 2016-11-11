/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_STOPWATCH_HPP_
#define DART_COMMON_STOPWATCH_HPP_

#include <chrono>

namespace dart {
namespace common {

template <typename C>
class StopwatchT
{
public:
  StopwatchT();

  template <typename U>
  typename U::rep getElapsedTime() const;

  typename std::chrono::nanoseconds::rep getElapsedNanoSeconds() const;

  typename std::chrono::microseconds::rep getElapsedMicroSeconds() const;

  typename std::chrono::milliseconds::rep getElapsedMilliSeconds() const;

  typename std::chrono::seconds::rep getElapsedSeconds() const;

  void reset();

protected:
  std::chrono::time_point<C> mStart;
};

using Stopwatch = StopwatchT<std::chrono::high_resolution_clock>;

//==============================================================================
template <typename C>
StopwatchT<C>::StopwatchT() : mStart(C::now())
{
  // Do nothing
}

//==============================================================================
template <typename C>
void StopwatchT<C>::reset()
{
  mStart = C::now();
}

//==============================================================================
template <typename C>
template <typename U>
typename U::rep StopwatchT<C>::getElapsedTime() const
{
  return std::chrono::duration_cast<U>(C::now() - mStart).count();
}

//==============================================================================
template <typename C>
typename std::chrono::nanoseconds::rep
StopwatchT<C>::getElapsedNanoSeconds() const
{
  return getElapsedTime<std::chrono::nanoseconds>();
}

//==============================================================================
template <typename C>
typename std::chrono::microseconds::rep
StopwatchT<C>::getElapsedMicroSeconds() const
{
  return getElapsedTime<std::chrono::microseconds>();
}

//==============================================================================
template <typename C>
typename std::chrono::milliseconds::rep
StopwatchT<C>::getElapsedMilliSeconds() const
{
  return getElapsedTime<std::chrono::milliseconds>();
}

//==============================================================================
template <typename C>
typename std::chrono::seconds::rep StopwatchT<C>::getElapsedSeconds() const
{
  return getElapsedTime<std::chrono::seconds>();
}

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_TIMER_HPP_
