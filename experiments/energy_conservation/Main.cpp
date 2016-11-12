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

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include "dart/dart.hpp"
#include "../SkeletonGenerators.hpp"

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
  std::vector<double> energy;

  void print()
  {
    std::cout << "[" << settings.algorithmName << "]\n";
    std::cout << "Execution time (s): " << executionTime << "\n";
    std::cout << "Energy error (%)  : " << errorRate << "\n";
    std::cout << "Size of energy    : " << energy.size() << "\n";
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

//  std::cout << "Gravity: " << world->getGravity().transpose() << std::endl;

  const auto totalE0 = skel->getKineticEnergy() + skel->getPotentialEnergy();

  if (PreStepFunction)
    (world.get()->*PreStepFunction)();

  AccuracyTestResult result;

  for (auto i = 0u; i < settings.numSteps; ++i)
  {
    (world.get()->*StepFunction)(true);

    auto totalE = skel->getKineticEnergy() + skel->getPotentialEnergy();
    totalEnergyRecord[i] = totalE;
    result.energy.emplace_back(totalE);
  }

  const auto errorInPercentage = (totalEnergyRecord.back() - totalE0)
      / totalE0 * 100.0;

  t.stop();

  result.executionTime = t.getLastElapsedTime();
  result.errorRate = errorInPercentage;
  result.settings = settings;

  return result;
}

//==============================================================================
void writeCsv(
    const std::string& filename,
    const std::vector<std::vector<double>>& energyList)
{
  std::ofstream file;
  file.open(filename);

  for (auto i = 0u; i < energyList.size(); ++i)
  {
    auto& energy = energyList[i];

    for (auto j = 0u; j < energy.size(); ++j)
    {
      file << energy[j];

      if (j != energy.size() - 1)
        file << ",";
    }

    file << "\n";
  }

  file.close();
}

//==============================================================================
int main()
{
  PerformanceTestSettings settings;
  settings.timeStep = 1e-3;
  settings.numSteps = 1e+6; // 1000 sec
  settings.maxIter = 15u;
  settings.tol = 1e-9;

  const auto numLinks = 10u;
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

  std::vector<AccuracyTestResult> results;
  results.reserve(3);

  settings.algorithmName = "Continuous time";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::prestep,
      &simulation::World::step>(settings));

  settings.algorithmName = "RIQN";
  settings.model = originalModel->clone();
  settings.model->setPositions(originalModel->getPositions());
  settings.model->setVelocities(originalModel->getVelocities());
  results.emplace_back(testPendulum<
      &simulation::World::dm_initialize_RIQN_DRNEA,
      &simulation::World::dm_step_RIQN_DRNEA>(settings));

//  settings.algorithmName = "DRNEA + Newton";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_Newton_DRNEA,
//      &simulation::World::dm_step_Newton_DRNEA>(settings));

//  settings.algorithmName = "DRNEA + Secant";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_Secant_DRNEA,
//      &simulation::World::dm_step_Secant_DRNEA>(settings));

//  settings.algorithmName = "DRNEA + RIQN";
//  settings.model = originalModel->clone();
//  settings.model->setPositions(originalModel->getPositions());
//  settings.model->setVelocities(originalModel->getVelocities());
//  results.emplace_back(testPendulum<
//      &simulation::World::dm_initialize_RIQN_DRNEA,
//      &simulation::World::dm_step_RIQN_DRNEA>(settings));

////  settings.algorithmName = "SVI + Newton";
////  settings.model = originalModel->clone();
////  settings.model->setPositions(originalModel->getPositions());
////  settings.model->setVelocities(originalModel->getVelocities());
////  results.emplace_back(testPendulum<
////      &simulation::World::dm_initialize_Newton_SVI,
////      &simulation::World::dm_step_Newton_SVI>(settings));

////  settings.algorithmName = "SVI + Secant";
////  settings.model = originalModel->clone();
////  settings.model->setPositions(originalModel->getPositions());
////  settings.model->setVelocities(originalModel->getVelocities());
////  results.emplace_back(testPendulum<
////      &simulation::World::dm_initialize_Secant_SVI,
////      &simulation::World::dm_step_Secant_SVI>(settings));

////  settings.algorithmName = "SVI + RIQN";
////  settings.model = originalModel->clone();
////  settings.model->setPositions(originalModel->getPositions());
////  settings.model->setVelocities(originalModel->getVelocities());
////  results.emplace_back(testPendulum<
////      &simulation::World::dm_initialize_RIQN_SVI,
////      &simulation::World::dm_step_RIQN_SVI>(settings));

  std::vector<std::vector<double>> energyList;
  energyList.push_back(results[0].energy);
  energyList.push_back(results[1].energy);

  for (auto& result : results)
    result.print();

  writeCsv("accuracy.csv", energyList);

  std::cout << "Experiment is done." << std::endl;
}
