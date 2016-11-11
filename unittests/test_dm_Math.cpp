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
TEST(DiscreteMechanics, dexp_inv)
{
  for (auto i = 0u; i < 1e+3; ++i)
  {
    Eigen::Vector6d V = Eigen::Vector6d::Random();
    Eigen::Vector6d W = Eigen::Vector6d::Random();

    Eigen::Vector6d dexp_inv1 = dexp_inv(V) * W;
    Eigen::Vector6d dexp_inv2 = dexp_inv(V, W);

    double dexp_inv_diff_norm = (dexp_inv1 - dexp_inv2).norm();

    EXPECT_LE(dexp_inv_diff_norm, 1e-12);

    Eigen::Vector6d dual_dexp_inv1 = dexp_inv(V).transpose() * W;
    Eigen::Vector6d dual_dexp_inv2 = dexp_inv_transpose(V, W);

    double dual_dexp_inv_diff_norm = (dual_dexp_inv1 - dual_dexp_inv2).norm();

    EXPECT_LE(dual_dexp_inv_diff_norm, 1e-12);
  }
}

//==============================================================================
TEST(DiscreteMechanics, dexp_inv_derivative)
{
  dynamics::Inertia inertia;
  inertia.setMass(2.0);
  inertia.setLocalCOM(Eigen::Vector3d(-2, 1.1, 4));
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity() * 2.0;
  inertia.setMoment(I);
  const Eigen::Matrix6d& G = inertia.getSpatialTensor();
  const double timeStep = 1e-3;

  for (auto i = 0u; i < 1; ++i)
  {
    Eigen::Vector6d V = Eigen::Vector6d::Random();

    Eigen::Matrix6d jacobian1 = computePartialMu(G, V, timeStep);
    Eigen::Matrix6d jacobian2 = computePartialMu_numerical(G, V, timeStep);

//    std::cout << "jacobian1:\n"
//              << jacobian1 << "\n\n";

//    std::cout << "jacobian2:\n"
//              << jacobian2 << "\n\n";

    EXPECT_TRUE(equals(jacobian1, jacobian2));
  }
}

//==============================================================================
TEST(DiscreteMechanics, CheckComponents)
{
  const auto numLinks = 5;
  const auto l = 1.5;
  auto skel = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  auto dofs = skel->getNumDofs();

  Eigen::VectorXd randomQNext = Eigen::VectorXd::Random(dofs);
  skel->setNextPositions(randomQNext);

  EXPECT_EQ(skel->getNextPositions(), randomQNext);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

