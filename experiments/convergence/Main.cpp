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
#include <gtest/gtest.h>
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
void writeCsv(
    const std::string& filename,
    const std::vector<std::vector<double>>& errorsList)
{
  std::ofstream file;
  file.open(filename);

  for (auto i = 0u; i < errorsList.size(); ++i)
  {
    auto& errors = errorsList[i];

    for (auto j = 0u; j < errors.size(); ++j)
    {
      file << errors[j];

      if (j != errors.size() - 1)
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
  settings.maxIter = 1e+3;
//  settings.tol = 1e-9;

  const auto numLinks = 50u;
  settings.tol = 1e-8;// * numLinks;

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

  std::vector<std::vector<double>> errorsList;

//  auto model_Newton_DRNEA = originalModel->clone();
//  model_Newton_DRNEA->setPositions(originalModel->getPositions());
//  model_Newton_DRNEA->setVelocities(originalModel->getVelocities());
//  model_Newton_DRNEA->dm_initialize_Newton_DRNEA();
//  errorsList.emplace_back(model_Newton_DRNEA->dm_integrate_with_report_Newton_DRNEA(
//        settings.tol, settings.maxIter, Skeleton::InitialGuessType::Zero));

  auto model_Secant_DRNEA = originalModel->clone();
  model_Secant_DRNEA->setPositions(originalModel->getPositions());
  model_Secant_DRNEA->setVelocities(originalModel->getVelocities());
  model_Secant_DRNEA->dm_initialize_Secant_DRNEA();
  errorsList.emplace_back(
        model_Secant_DRNEA->dm_integrate_with_report_Secant_DRNEA(
        settings.tol,
        settings.maxIter,
        Skeleton::InitialGuessType::Current));

  auto model_RIQN_DRNEA = originalModel->clone();
  model_RIQN_DRNEA->setPositions(originalModel->getPositions());
  model_RIQN_DRNEA->setVelocities(originalModel->getVelocities());
  model_RIQN_DRNEA->dm_initialize_RIQN_DRNEA();
  errorsList.emplace_back(
        model_RIQN_DRNEA->dm_integrate_with_report_RIQN_DRNEA(
        settings.tol,
        settings.maxIter,
        Skeleton::InitialGuessType::Current));

  writeCsv("convergence.csv", errorsList);
}
