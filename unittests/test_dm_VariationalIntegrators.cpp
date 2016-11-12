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

void setRandomState(const dynamics::SkeletonPtr& skel);

dynamics::SkeletonPtr createRandomSkeleton(std::size_t numBodies = 25u);
dynamics::SkeletonPtr createRandomSkeletonWithBallJoints(std::size_t numBodies = 25u);

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

  void print() const
  {
    std::cout << "[" << settings.algorithmName << "]\n";
    std::cout << "Execution time (s): " << executionTime << "\n";
    std::cout << "Energy error (%)  : " << errorRate << "\n";
    std::cout << "\n";
  }
};

//==============================================================================
template <void (simulation::World::*PreStepFunction)() = nullptr,
          void (simulation::World::*StepFunction)(double, std::size_t) = &simulation::World::step>
AccuracyTestResult testPendulum(const PerformanceTestSettings& settings)
{
  DART_PROFILE_SCOPED_NODE("testPendulum");

  common::Timer t;
  t.start();

  std::vector<double> totalEnergyRecord(settings.numSteps);

  auto& skel = settings.model;
  auto world = std::make_shared<simulation::World>();
  world->addSkeleton(skel);
  world->setGravity(Vector3d::Zero());
  world->setTimeStep(settings.timeStep);

  const auto totalE0 = skel->getKineticEnergy() + skel->getPotentialEnergy();

  if (PreStepFunction)
    (world.get()->*PreStepFunction)();

  for (auto i = 0u; i < settings.numSteps; ++i)
  {
    (world.get()->*StepFunction)(settings.tol, settings.maxIter);

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
TEST(VariationalIntegrators, EnergyBehavior)
{
  PerformanceTestSettings settings;
  settings.timeStep = 1e-3;
#ifdef NDEBUG
  settings.numSteps = 1e+3; // 1 sec
#else
  settings.numSteps = 1e+1; // 0.01 sec
#endif
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
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0,0,10);
  originalModel->getJoint(0)->setTransformFromParentBodyNode(T);
//  originalModel->setGravity(Eigen::Vector3d::Zero());

  std::vector<AccuracyTestResult> results;
  results.reserve(3);

  settings.algorithmName = "Newton + SVI";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_Newton_SVI,
      &simulation::World::dm_step_Newton_SVI>(settings));

  settings.algorithmName = "Newton + DRNEA";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_Newton_DRNEA,
      &simulation::World::dm_step_Newton_DRNEA>(settings));

  settings.algorithmName = "Secant + SVI";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_Secant_SVI,
      &simulation::World::dm_step_Secant_SVI>(settings));

  settings.algorithmName = "Secant + DRNEA";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_Secant_DRNEA,
      &simulation::World::dm_step_Secant_DRNEA>(settings));

//  settings.algorithmName = "Broyden + SVI";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_Broyden_SVI,
//      &simulation::World::dm_step_Broyden_SVI>(settings));

//  settings.algorithmName = "Broyden + DRNEA";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_Broyden_DRNEA,
//      &simulation::World::dm_step_Broyden_DRNEA>(settings));

  settings.algorithmName = "RIQN + SVI";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_RIQN_SVI,
      &simulation::World::dm_step_RIQN_SVI>(settings));

  settings.algorithmName = "RIQN + DRNEA";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_RIQN_DRNEA,
      &simulation::World::dm_step_RIQN_DRNEA>(settings));

  for (const auto& result : results)
  {
    EXPECT_LT(result.errorRate, 1.0);
    result.print();
  }
}

//==============================================================================
TEST(VariationalIntegrators, CompareVariationalIntegratorss)
{
#ifdef NDEBUG
  auto numTests = 1e+3;
#else
  auto numTests = 1e+1;
#endif

  const auto numBodies = 25u;

  auto skel = createRandomSkeleton(numBodies);
  ////
  skel->setGravity(Eigen::Vector3d::Zero());
  ////

  auto skelSVI = skel->clone();
  auto skelDRNEA = skel->clone();

  for (auto i = 0u; i < numTests; ++i)
  {
    Eigen::VectorXd randomPositions = Eigen::VectorXd::Random(numBodies);
    Eigen::VectorXd randomVelocities = Eigen::VectorXd::Random(numBodies);

    skelSVI->setPositions(randomPositions);
    skelDRNEA->setPositions(randomPositions);

    skelSVI->setVelocities(randomVelocities);
    skelDRNEA->setVelocities(randomVelocities);

    auto E0SVI = skelSVI->getTotalEnergy();
    auto E0DRNEA = skelDRNEA->getTotalEnergy();
    EXPECT_NEAR(E0SVI, E0DRNEA, 1e-9);

    const Eigen::VectorXd nextPositions = randomPositions;

    skelSVI->dm_initialize_RIQN_SVI();
    skelDRNEA->dm_initialize_RIQN_DRNEA();

    Eigen::VectorXd delSVI = skelSVI->dm_computeFDEL_SVI(nextPositions, true);
    Eigen::VectorXd delDRNEA = skelDRNEA->dm_computeFDEL_DRNEA(nextPositions);

    EXPECT_TRUE(delSVI.isApprox(delDRNEA, 1e-2));
    if (!delSVI.isApprox(delDRNEA, 1e-2))
    {
      std::cout << "delSVI  : " << delSVI.transpose() << std::endl;
      std::cout << "delDRNEA: " << delDRNEA.transpose() << std::endl;
    }

//    auto E1SVI = skelSVI->getTotalEnergy();
//    auto E1DRNEA = skelDRNEA->getTotalEnergy();
//    EXPECT_NEAR(E0SVI, E1SVI, 1e-4);
//    EXPECT_NEAR(E0DRNEA, E1DRNEA, 1e-4);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//==============================================================================
//
//                              Implementations
//
//==============================================================================

//==============================================================================
void setRandomState(const dynamics::SkeletonPtr& skel)
{
  const auto pi = math::constantsd::pi();
  const auto numDofs = skel->getNumDofs();
  const auto posLower = pi * -0.5;
  const auto posUpper = pi *  0.5;
  const auto velLower = pi * -0.5;
  const auto velUpper = pi *  0.5;

  for (auto i = 0u; i < numDofs; ++i)
  {
    auto dof = skel->getDof(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    dof->setPosition(pos);
    dof->setVelocity(vel);
  }
}

//==============================================================================
SkeletonPtr createRandomSkeleton(std::size_t numBodies)
{
  const auto l = 1.5;
  auto skel = createNLinkRobot(numBodies, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}

//==============================================================================
SkeletonPtr createRandomSkeletonWithBallJoints(std::size_t numBodies)
{
  const auto l = 1.5;
  auto skel = createNLinkRobot(numBodies, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}
