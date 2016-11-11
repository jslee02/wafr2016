//
// The MIT License (MIT)
//
// Copyright (c) 2016 Jeongseok Lee <jslee02@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef DART_COMMON_PROFILER_HPP_
#define DART_COMMON_PROFILER_HPP_

#include <string>
#include <iostream>

#include "dart/config.hpp"
#include "dart/common/Timer.hpp"
#include "dart/common/detail/Profiler.hpp"

// clang-format off

#if DART_ENABLE_PROFILING
  #define DART_PROFILE_NEW_FRAME            Profiler::beginNewFrame();
  #define DART_PROFILE_BEGIN_NODE(name)     Profiler::startProfilingBlock(name);
  #define DART_PROFILE_END_NODE             Profiler::stopProfilingBlock();
  #define DART_PROFILE_SCOPED_NODE(name)    ScopedProfileNode\
    scopedProfileNode##__FILE__##__LINE__(name)
  #define DART_PROFILE_PRINT_REPORT(stream) Profiler::printReport(stream);
#else
  #define DART_PROFILE_NEW_FRAME
  #define DART_PROFILE_BEGIN_NODE(name)
  #define DART_PROFILE_END_NODE
  #define DART_PROFILE_SCOPED_NODE(name)
  #define DART_PROFILE_PRINT_REPORT(stream)
#endif // #if DART_ENABLE_PROFILING

// clang-format on

#if DART_ENABLE_PROFILING

namespace dart {

class Profiler
{
public:
  /// Increment the frame counter.
  static void beginNewFrame();

  /// Return the number of frames so far.
  static std::size_t getNumFrames();

  /// Method called when we want to start profiling a block of code.
  static void startProfilingBlock(const std::string& name);

  /// Method called at the end of the scope where the startProfilingBlock()
  /// method has been called.
  static void stopProfilingBlock();

  /// Reset the timing data of the profiler.
  static void resetTime();

  /// Return the total elasped time since the start/reset of the profiling.
  template <typename U = std::chrono::milliseconds>
  static typename U::rep getElapsedTime();

  /// Return the total elasped time in micro seconds since the start/reset of
  /// the profiling.
  static typename std::chrono::seconds::rep getElapsedNanoSeconds();

  /// Return the total elasped time in milli seconds since the start/reset of
  /// the profiling.
  static typename std::chrono::milliseconds::rep getElapsedMilliSeconds();

  /// Return the total elasped time in seconds since the start/reset of the
  /// profiling.
  static typename std::chrono::seconds::rep getElapsedSeconds();

  /// Print the report of the profiler in a given output stream.
  static void printReport(std::ostream& outputStream = std::cout);

private:
  /// Constructor
  Profiler();

  /// Desctructor
  ~Profiler();

  /// Returns the singleton of profiler.
  static Profiler& theProfiler();

  /// Return an iterator over the profiler tree starting at the root.
  static detail::ProfileNodeIterator getNodeIterator();

  /// Recursively print the report of a given node of the profiler tree
  static void printRecursiveNodeReport(
      detail::ProfileNodeIterator iterator = Profiler::getNodeIterator(),
      int indent = 0,
      std::ostream& outputStream = std::cout);

  /// Root node of the profiler tree
  detail::ProfileNode mRootNode;

  /// Current node in the current execution
  detail::ProfileNode* mCurrentNode;

  /// Frame counter
  std::size_t mFrameCounter;

  /// Stopwatch to measure the total elapsed time.
  common::Stopwatch mStopwatch;
};

/// This class is used to represent a profile sample. It is constructed at the
/// beginning of a code block we want to profile and destructed at the end of
/// the scope to profile.
class ScopedProfileNode
{
public:
  /// Constructor
  ScopedProfileNode(const std::string& name);

  /// Destructor
  ~ScopedProfileNode();
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
inline void Profiler::startProfilingBlock(const std::string& name)
{
  // Look for the node in the tree that corresponds to the block of
  // code to profile
  if (name != theProfiler().mCurrentNode->getName())
    theProfiler().mCurrentNode =
        theProfiler().mCurrentNode->getOrCreateChildNode(name);

  // Start profile the node
  theProfiler().mCurrentNode->beginBlockOfCode();
}

//==============================================================================
inline void Profiler::stopProfilingBlock()
{
  // Go to the parent node unless if the current block
  // of code is recursing
  if (theProfiler().mCurrentNode->endBlockOfCode())
    theProfiler().mCurrentNode = theProfiler().mCurrentNode->getParentNode();
}

//==============================================================================
inline void Profiler::resetTime()
{
  theProfiler().mRootNode.reset();
  theProfiler().mRootNode.beginBlockOfCode();
  theProfiler().mStopwatch.reset();
  theProfiler().mFrameCounter = 0;
}

//==============================================================================
inline std::size_t Profiler::getNumFrames()
{
  return theProfiler().mFrameCounter;
}

//==============================================================================
template <typename U>
typename U::rep Profiler::getElapsedTime()
{
  return theProfiler().mStopwatch.getElapsedTime<U>();
}

//==============================================================================
inline typename std::chrono::milliseconds::rep
Profiler::getElapsedNanoSeconds()
{
  return theProfiler().mStopwatch.getElapsedNanoSeconds();
}

//==============================================================================
inline typename std::chrono::milliseconds::rep
Profiler::getElapsedMilliSeconds()
{
  return theProfiler().mStopwatch.getElapsedMilliSeconds();
}

//==============================================================================
inline typename std::chrono::milliseconds::rep Profiler::getElapsedSeconds()
{
  return theProfiler().mStopwatch.getElapsedSeconds();
}

//==============================================================================
inline void Profiler::beginNewFrame()
{
  theProfiler().mFrameCounter++;
}

//==============================================================================
inline detail::ProfileNodeIterator Profiler::getNodeIterator()
{
  return detail::ProfileNodeIterator(&(theProfiler().mRootNode));
}

//==============================================================================
inline void Profiler::printReport(std::ostream& outputStream)
{
  // Recursively print the report of each node of the profiler tree
  printRecursiveNodeReport(Profiler::getNodeIterator(), 0, outputStream);
}

//==============================================================================
inline Profiler::Profiler()
  : mRootNode{"Root", nullptr}, mCurrentNode{&mRootNode}, mFrameCounter{0},
    mStopwatch{common::Stopwatch()}
{
  // Do nothing
}

//==============================================================================
inline Profiler::~Profiler()
{
  // Do nothing
  printReport();
}

//==============================================================================
inline Profiler& Profiler::theProfiler()
{
  static Profiler theProfiler;
  return theProfiler;
}

//==============================================================================
inline void
Profiler::printRecursiveNodeReport(detail::ProfileNodeIterator iterator,
                                   int indent,
                                   std::ostream& outputStream)
{
  iterator.first();

  // If we are at the end of a branch in the profiler tree
  if (iterator.isEnd())
    return;

  auto parentTime = iterator.isRoot() ? getElapsedMilliSeconds()
                                      : iterator.getCurrentParentTotalTime();
  auto accumulatedTime = 0.0;
  auto numFrames = Profiler::getNumFrames();

  for (auto i = 0; i < indent; ++i)
    outputStream << " ";

  outputStream << "---------------" << std::endl;

  for (auto i = 0; i < indent; ++i)
    outputStream << " ";

  outputStream << "| Profiling : " << iterator.getCurrentParentName()
               << " (total running time : " << parentTime << " ms) ---"
               << std::endl;
  long double totalTime = 0.0;

  // Recurse over the children of the current node
  int numChildren = 0;
  for (auto i = 0; !iterator.isEnd(); ++i, iterator.next())
  {
    numChildren++;
    auto currentTotalTime = iterator.getCurrentTotalTime();
    accumulatedTime += currentTotalTime;
    auto fraction = parentTime > std::numeric_limits<long double>::epsilon()
                        ? (currentTotalTime / parentTime) * 100.0
                        : 0.0;
    for (auto j = 0; j < indent; ++j)
      outputStream << " ";
    outputStream << "|   " << i << " -- " << iterator.getCurrentName() << " : "
                 << fraction << " % | "
                 << (currentTotalTime / (long double)(numFrames))
                 << " nanosec/frame (" << iterator.getCurrentNbTotalCalls()
                 << " calls)" << std::endl;
    totalTime += currentTotalTime;
  }

  if (parentTime < accumulatedTime)
    outputStream << "Something is wrong !" << std::endl;

  for (auto i = 0; i < indent; ++i)
    outputStream << " ";

  auto percentage = parentTime > std::numeric_limits<long double>::epsilon()
                        ? ((parentTime - accumulatedTime) / parentTime) * 100.0
                        : 0.0;
  auto difference = parentTime - accumulatedTime;
  outputStream << "| Unaccounted : " << difference << " ms (" << percentage
               << " %)" << std::endl;

  for (auto i = 0; i < numChildren; ++i)
  {
    iterator.enterChild(i);
    printRecursiveNodeReport(iterator, indent + 3, outputStream);
    iterator.enterParent();
  }
}

//==============================================================================
inline ScopedProfileNode::ScopedProfileNode(const std::string& name)
{
  Profiler::startProfilingBlock(name);
}

//==============================================================================
inline ScopedProfileNode::~ScopedProfileNode()
{
  Profiler::stopProfilingBlock();
}

} // namespace hit

#endif // #if DART_ENABLE_PROFILING

#endif // #define DART_COMMON_PROFILER_HPP_
