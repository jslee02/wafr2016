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

#ifndef DART_COMMON_DETAIL_PROFILER_HPP_
#define DART_COMMON_DETAIL_PROFILER_HPP_

#include <cassert>
#include <memory>
#include <string>

#include "dart/config.hpp"
#include "dart/common/Stopwatch.hpp"

#if DART_ENABLE_PROFILING

namespace dart {
namespace detail {

class ProfileNode
{
public:
  /// Constructor
  ProfileNode(const std::string& name, ProfileNode* parentNode);

  /// Destructor
  ~ProfileNode();

  /// Return a pointer to a sub node
  ProfileNode* getOrCreateChildNode(const std::string& name);

  /// Return a pointer to the parent node
  ProfileNode* getParentNode() const;

  /// Return a pointer to a sibling node
  ProfileNode* getSiblingNode() const;

  /// Return a pointer to a child node
  ProfileNode* getChildNode() const;

  /// Return the name of the node
  const std::string& getName() const;

  /// Return the total number of call of the corresponding block of code
  std::size_t getNumTotalCalls() const;

  /// Return the total time spent in the block of code
  long double getTotalTime() const;

  /// Called when we enter the block of code corresponding to this profile node
  void beginBlockOfCode();

  /// Called when we exit the block of code corresponding to this profile node
  bool endBlockOfCode();

  /// Reset the profiling of the node
  void reset();

private:
  /// Name of the node
  const std::string mName;

  /// Stopwatch to measure the time spent by this node
  common::Stopwatch mStopwatch;

  /// Total number of calls of this node
  std::size_t mNumTotalCalls;

  /// Total time spent in the block of code
  long double mTotalTime;

  /// Recursion counter
  std::size_t mRecursionCounter;

  /// Pointer to the parent node
  ProfileNode* mParentNode;

  /// Pointer to a child node
  std::unique_ptr<ProfileNode> mChildNode;

  /// Pointer to a sibling node
  std::unique_ptr<ProfileNode> mSiblingNode;
};

class ProfileNodeIterator
{
public:
  /// Constructor
  ProfileNodeIterator(ProfileNode* startingNode);

  /// Go to the first node
  void first();

  /// Go to the next node
  void next();

  /// Enter a given child node
  void enterChild(int index);

  /// Enter a given parent node
  void enterParent();

  /// Return true if we are at the root of the profiler tree
  bool isRoot();

  /// Return true if we are at the end of a branch of the profiler tree
  bool isEnd();

  /// Return the name of the current node
  const std::string& getCurrentName();

  /// Return the total time of the current node
  long double getCurrentTotalTime();

  /// Return the total number of calls of the current node
  std::size_t getCurrentNbTotalCalls();

  /// Return the name of the current parent node
  const std::string& getCurrentParentName();

  /// Return the total time of the current parent node
  long double getCurrentParentTotalTime();

  /// Return the total number of calls of the current parent node
  std::size_t getCurrentParentNbTotalCalls();

private:
  /// Current parent node
  ProfileNode* mCurrentParentNode;

  /// Current child node
  ProfileNode* mCurrentChildNode;
};

//============================================================================//
//                                                                            //
//                              Implementations                               //
//                                                                            //
//============================================================================//

//==============================================================================
inline bool ProfileNodeIterator::isRoot()
{
  return (mCurrentParentNode->getParentNode() == nullptr);
}

//==============================================================================
inline bool ProfileNodeIterator::isEnd()
{
  return (mCurrentChildNode == nullptr);
}

//==============================================================================
inline const std::string& ProfileNodeIterator::getCurrentName()
{
  return mCurrentChildNode->getName();
}

//==============================================================================
inline long double ProfileNodeIterator::getCurrentTotalTime()
{
  return mCurrentChildNode->getTotalTime();
}

//==============================================================================
inline std::size_t ProfileNodeIterator::getCurrentNbTotalCalls()
{
  return mCurrentChildNode->getNumTotalCalls();
}

//==============================================================================
inline const std::string& ProfileNodeIterator::getCurrentParentName()
{
  return mCurrentParentNode->getName();
}

//==============================================================================
inline long double ProfileNodeIterator::getCurrentParentTotalTime()
{
  return mCurrentParentNode->getTotalTime();
}

//==============================================================================
inline std::size_t ProfileNodeIterator::getCurrentParentNbTotalCalls()
{
  return mCurrentParentNode->getNumTotalCalls();
}

//==============================================================================
inline ProfileNodeIterator::ProfileNodeIterator(ProfileNode* startingNode)
  : mCurrentParentNode(startingNode),
    mCurrentChildNode(mCurrentParentNode->getChildNode())
{
  assert(startingNode != nullptr);
}

//==============================================================================
inline void ProfileNodeIterator::first()
{
  mCurrentChildNode = mCurrentParentNode->getChildNode();
}

//==============================================================================
inline void ProfileNodeIterator::next()
{
  mCurrentChildNode = mCurrentChildNode->getSiblingNode();
}

//==============================================================================
inline void ProfileNodeIterator::enterChild(int index)
{
  mCurrentChildNode = mCurrentParentNode->getChildNode();
  while ((mCurrentChildNode != nullptr) && (index != 0))
  {
    index--;
    mCurrentChildNode = mCurrentChildNode->getSiblingNode();
  }

  if (mCurrentChildNode != nullptr)
  {
    mCurrentParentNode = mCurrentChildNode;
    mCurrentChildNode = mCurrentParentNode->getChildNode();
  }
}

//==============================================================================
inline void ProfileNodeIterator::enterParent()
{
  if (mCurrentParentNode->getParentNode() != nullptr)
    mCurrentParentNode = mCurrentParentNode->getParentNode();

  mCurrentChildNode = mCurrentParentNode->getChildNode();
}

//==============================================================================
inline ProfileNode::ProfileNode(const std::string& name,
                                ProfileNode* parentNode)
  : mName(name), mNumTotalCalls(0), mTotalTime(0), mRecursionCounter(0),
    mParentNode(parentNode), mChildNode(nullptr), mSiblingNode(nullptr)
{
  reset();
}

//==============================================================================
inline ProfileNode::~ProfileNode()
{
  // Do nothing
}

//==============================================================================
inline ProfileNode* ProfileNode::getOrCreateChildNode(const std::string& name)
{
  // Try to find the node among the child nodes
  auto* child = mChildNode.get();
  while (child != nullptr)
  {
    if (child->mName == name)
      return child;

    child = child->mSiblingNode.get();
  }

  // The nose has not been found. Therefore, we create it and add it to the
  // profiler tree
  auto* newNode = new ProfileNode(name, this);
  newNode->mSiblingNode.reset(mChildNode.release());
  mChildNode.reset(newNode);

  return newNode;
}

//==============================================================================
inline ProfileNode* ProfileNode::getParentNode() const
{
  return mParentNode;
}

//==============================================================================
inline ProfileNode* ProfileNode::getSiblingNode() const
{
  return mSiblingNode.get();
}

//==============================================================================
inline ProfileNode* ProfileNode::getChildNode() const
{
  return mChildNode.get();
}

//==============================================================================
inline const std::string& ProfileNode::getName() const
{
  return mName;
}

//==============================================================================
inline std::size_t ProfileNode::getNumTotalCalls() const
{
  return mNumTotalCalls;
}

//==============================================================================
inline long double ProfileNode::getTotalTime() const
{
  return mTotalTime;
}

//==============================================================================
inline void ProfileNode::beginBlockOfCode()
{
  mNumTotalCalls++;

  // If the current code is not called recursively
  if (mRecursionCounter == 0)
  {
    // Get the current system time to initialize the starting time of
    // the profiling of the current block of code
    mStopwatch.reset();
  }

  mRecursionCounter++;
}

//==============================================================================
inline bool ProfileNode::endBlockOfCode()
{
  assert(mRecursionCounter > 0);

  mRecursionCounter--;

  if (mRecursionCounter == 0 && mNumTotalCalls != 0)
  {
    // Increase the total elasped time in the current block of code
    mTotalTime += mStopwatch.getElapsedMilliSeconds();
  }

  // Return true if the current code is not recursing
  return (mRecursionCounter == 0);
}

//==============================================================================
inline void ProfileNode::reset()
{
  mNumTotalCalls = 0;
  mTotalTime = 0.0;

  // Reset the child node
  if (mChildNode != nullptr)
    mChildNode->reset();

  // Reset the sibling node
  if (mSiblingNode != nullptr)
    mSiblingNode->reset();
}

} // namespace detail
} // namespace hit

#endif // #if DART_ENABLE_PROFILING

#endif // #define DART_COMMON_DETAIL_PROFILER_HPP_
