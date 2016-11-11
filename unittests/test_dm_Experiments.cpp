/*
 * Copyright (c) 2014-2016, Georgia Tech Research Corporation
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

#include <iomanip>
#include <gtest/gtest.h>
#include "dart/dart.hpp"
#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
struct PerformanceTestSettings
{
  std::string algorithmName;
  std::shared_ptr<dynamics::Skeleton> model;
  double timeStep;
  std::size_t numSteps;
  std::size_t maxIter;
  double tol;
};

//==============================================================================
struct AccuracyTestResult
{
  PerformanceTestSettings settings;
  double errorRate;
  double executionTime;

  void print()
  {
    std::cout << "[" << settings.algorithmName << "]\n";
    std::cout << "Execution time (s): " << executionTime << "\n";
    std::cout << "Energy error (%)  : " << errorRate << "\n";
    std::cout << "\n";
  }
};

//==============================================================================
template <void (simulation::World::*PreStepFunction)() = nullptr,
          void (simulation::World::*StepFunction)(bool) = &simulation::World::step>
AccuracyTestResult testPendulum(const PerformanceTestSettings& settings)
{
  DART_PROFILE_SCOPED_NODE("testPendulum");

  common::Timer t;
  t.start();

  std::vector<double> totalEnergyRecord(settings.numSteps);

  auto& skel = settings.model;
  auto world = std::make_shared<simulation::World>();
  world->addSkeleton(skel);
//  world->setGravity(Vector3d::Zero());
  world->setTimeStep(settings.timeStep);

  const auto totalE0 = skel->getKineticEnergy() + skel->getPotentialEnergy();

  if (PreStepFunction)
    (world.get()->*PreStepFunction)();

  for (auto i = 0u; i < settings.numSteps; ++i)
  {
    (world.get()->*StepFunction)(true);

    auto totalE = skel->getKineticEnergy() + skel->getPotentialEnergy();
    totalEnergyRecord[i] = totalE;
  }

  const auto errorInPercentage = (totalEnergyRecord.back() - totalE0)
      / totalE0 * 100.0;

  t.stop();

  AccuracyTestResult result;
  result.executionTime = t.getLastElapsedTime();
  result.errorRate = errorInPercentage;
  result.settings = settings;

  return result;
}

//==============================================================================
TEST(DiscreteMechanics, Pendulums)
{
  PerformanceTestSettings settings;
  settings.timeStep = 1e-3;
  settings.numSteps = 1e+3;
  settings.maxIter = 15u;
  settings.tol = 1e-9;

  const auto numLinks = 25u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto originalModel = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = originalModel->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }

  std::vector<AccuracyTestResult> results;
  results.reserve(3);

//  settings.algorithmName = "Continuous time";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::prestep,
//      &simulation::World::step>(settings));

  settings.algorithmName = "DM JS Broyden";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_JS_Broyden,
      &simulation::World::dm_step_Broyden_DRNEA>(settings));

  settings.algorithmName = "VI linear(JS) residual impulse";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_RIQN_DRNEA,
      &simulation::World::dm_step_RIQN_DRNEA>(settings));

//  settings.algorithmName = "VI quadratic(Murphey) residual impulse";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_RIQN_SVI,
//      &simulation::World::dm_step_Murphey_RIQN>(settings));

//  settings.algorithmName = "VI Quasi Newton-Raphson (secant)";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_prestep,
//      &simulation::World::dm_stepQuasiNewtonSecant>(settings));

  for (auto& result : results)
    result.print();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

