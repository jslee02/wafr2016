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
TEST(DMMurphey, BodyVelocityGradients)
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto skeleton = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skeleton->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }
  skeleton->dm_initialize_Secant_SVI();

  const auto nBodies = skeleton->getNumBodyNodes();

  std::vector<Eigen::Vector6d> bodyVelocityGradients_q_fd(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityGradients_dq_fd(nBodies);

  std::vector<Eigen::Vector6d> bodyVelocityGradients_q_analytical(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityGradients_dq_analytical(nBodies);

  skeleton->dm_updateGradientV_q_dq();

  for (auto i = 0u; i < skeleton->getNumDofs(); ++i)
  {
    bodyVelocityGradients_q_fd
        = skeleton->computeBodyVelocityGradientsUsingFiniteDifference_q(i, 1e-12);
    bodyVelocityGradients_dq_fd
        = skeleton->computeBodyVelocityGradientsUsingFiniteDifference_dq(i, 1e-12);

    for (auto j = 0u; j < skeleton->getNumDofs(); ++j)
    {
      bodyVelocityGradients_q_analytical[j]
          = skeleton->getBodyNode(j)->mDiscreteMurphey->mV_q[i];
      bodyVelocityGradients_dq_analytical[j]
          = skeleton->getBodyNode(j)->mDiscreteMurphey->mV_dq[i];

      EXPECT_TRUE(bodyVelocityGradients_q_fd[j].isApprox(
                    bodyVelocityGradients_q_analytical[j], 1e-1));
      EXPECT_TRUE(bodyVelocityGradients_dq_fd[j].isApprox(
                    bodyVelocityGradients_dq_analytical[j], 1e-1));

//      std::cout << "fd        : " << bodyVelocityGradients_q_fd[j].transpose() << std::endl;
//      std::cout << "analytical: " << bodyVelocityGradients_q_analytical[j].transpose() << std::endl;
    }
  }
}

//==============================================================================
TEST(DMMurphey, BodyVelocityHessians)
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto skeleton = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skeleton->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }
  skeleton->dm_initialize_Newton_SVI();

  const auto nBodies = skeleton->getNumBodyNodes();

  std::vector<Eigen::Vector6d> bodyVelocityHessians_q_q_fd(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_q_dq_fd(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_dq_q_fd(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_dq_dq_fd(nBodies);

  std::vector<Eigen::Vector6d> bodyVelocityHessians_q_q_analytical(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_q_dq_analytical(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_dq_q_analytical(nBodies);
  std::vector<Eigen::Vector6d> bodyVelocityHessians_dq_dq_analytical(nBodies);

  skeleton->dm_updateGradientV_q_dq();
  skeleton->dm_updateHessianV_q_dq();

  for (auto i = 0u; i < skeleton->getNumDofs(); ++i)
  {
    for (auto j = 0u; j < skeleton->getNumDofs(); ++j)
    {
      bodyVelocityHessians_q_q_fd
          = skeleton->computeBodyVelocityHessiansUsingFiniteDifference_q_q(
            i, j, 1e-12);
      bodyVelocityHessians_q_dq_fd
          = skeleton->computeBodyVelocityHessiansUsingFiniteDifference_q_dq(
            i, j, 1e-12);
      bodyVelocityHessians_dq_q_fd
          = skeleton->computeBodyVelocityHessiansUsingFiniteDifference_dq_q(
            i, j, 1e-12);
      bodyVelocityHessians_dq_dq_fd
          = skeleton->computeBodyVelocityHessiansUsingFiniteDifference_dq_dq(
            i, j, 1e-12);

      for (auto k = 0u; k < skeleton->getNumDofs(); ++k)
      {
        bodyVelocityHessians_q_q_analytical[k]
            = skeleton->getBodyNode(k)->mDiscreteMurphey->mV_q_q[i][j];
        bodyVelocityHessians_q_dq_analytical[k]
            = skeleton->getBodyNode(k)->mDiscreteMurphey->mV_q_dq[i][j];
        bodyVelocityHessians_dq_q_analytical[k]
            = skeleton->getBodyNode(k)->mDiscreteMurphey->mV_q_dq[j][i];
        bodyVelocityHessians_dq_dq_analytical[k]
            = skeleton->getBodyNode(k)->mDiscreteMurphey->mV_dq_dq[i][j];

        EXPECT_TRUE(bodyVelocityHessians_q_q_fd[k].isApprox(
                      bodyVelocityHessians_q_q_analytical[k], 1e-2));
        EXPECT_TRUE(bodyVelocityHessians_q_dq_fd[k].isApprox(
                      bodyVelocityHessians_q_dq_analytical[k], 1e-2));
        EXPECT_TRUE(bodyVelocityHessians_dq_q_fd[k].isApprox(
                      bodyVelocityHessians_dq_q_analytical[k], 1e-2));
        EXPECT_TRUE(bodyVelocityHessians_dq_dq_fd[k].isApprox(
                      bodyVelocityHessians_dq_dq_analytical[k], 1e-2));

        if (!bodyVelocityHessians_q_q_fd[k].isApprox(
              bodyVelocityHessians_q_q_analytical[k], 1e-2))
        {
          std::cout << "q_q_fd        : "
                    << bodyVelocityHessians_q_q_fd[k].transpose()
                    << std::endl;
          std::cout << "q_q_analytical: "
                    << bodyVelocityHessians_q_q_analytical[k].transpose()
                    << std::endl;
        }

        if (!bodyVelocityHessians_q_dq_fd[k].isApprox(
              bodyVelocityHessians_q_dq_analytical[k], 1e-2))
        {
          std::cout << "q_dq_fd        : "
                    << bodyVelocityHessians_q_dq_fd[k].transpose()
                    << std::endl;
          std::cout << "q_dq_analytical: "
                    << bodyVelocityHessians_q_dq_analytical[k].transpose()
                    << std::endl;
        }

        if (!bodyVelocityHessians_dq_q_fd[k].isApprox(
              bodyVelocityHessians_dq_q_analytical[k], 1e-2))
        {
          std::cout << "dq_q_fd        : "
                    << bodyVelocityHessians_dq_q_fd[k].transpose()
                    << std::endl;
          std::cout << "dq_q_analytical: "
                    << bodyVelocityHessians_dq_q_analytical[k].transpose()
                    << std::endl;
        }

        if (!bodyVelocityHessians_q_q_fd[k].isApprox(
              bodyVelocityHessians_q_q_analytical[k], 1e-2))
        {
          std::cout << "dq_dq_fd        : "
                    << bodyVelocityHessians_dq_dq_fd[k].transpose()
                    << std::endl;
          std::cout << "dq_dq_analytical: "
                    << bodyVelocityHessians_dq_dq_analytical[k].transpose()
                    << std::endl;
        }
      }
    }
  }
}

//==============================================================================
TEST(DMMurphey, LagrangianGradients)
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto skeleton = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skeleton->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }
  skeleton->dm_initialize_Secant_SVI();

  auto gradFd_q = skeleton->computeLagrangianGradientUsingFiniteDifference_q(1e-9);
  auto gradFd_dq = skeleton->computeLagrangianGradientUsingFiniteDifference_dq(1e-9);

  skeleton->dm_updateGradientLagrangian_q_dq();
  auto gradAnalytical_q = skeleton->mDiscreteMurphey->m_dm_GradientOfLagrangian_q;
  auto gradAnalytical_dq = skeleton->mDiscreteMurphey->m_dm_GradientOfLagrangian_dq;

  EXPECT_TRUE(gradFd_q.isApprox(gradAnalytical_q, 1e-3));
  EXPECT_TRUE(gradFd_dq.isApprox(gradAnalytical_dq, 1e-3));

//  std::cout << (gradAnalytical_q - gradFd_q).transpose() << std::endl;
//  std::cout << (gradAnalytical_dq - gradFd_dq).transpose() << std::endl;
}

//==============================================================================
TEST(DMMurphey, LagrangianHessians)
{
  const auto numLinks = 10u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto skeleton = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skeleton->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }
  skeleton->dm_initialize_Newton_SVI();

  auto hessianFd_q_q_mark1 = skeleton->computeLagrangianHessianUsingFiniteDifference_q_q_mark1(1e-6);
  auto hessianFd_q_q_mark2 = skeleton->computeLagrangianHessianUsingFiniteDifference_q_q_mark2(1e-6);
  auto hessianFd_q_dq = skeleton->computeLagrangianHessianUsingFiniteDifference_q_dq(1e-6);
  auto hessianFd_dq_dq = skeleton->computeLagrangianHessianUsingFiniteDifference_dq_dq(1e-6);

  skeleton->dm_updateGradientLagrangian_q_dq();
  skeleton->dm_updateHessianLagrangian_q_dq();
  auto hessianAnalytical_q_q = skeleton->mDiscreteMurphey->m_dm_HessianOfLagrangian_q_q;
  auto hessianAnalytical_q_dq = skeleton->mDiscreteMurphey->m_dm_HessianOfLagrangian_q_dq;
  auto hessianAnalytical_dq_dq = skeleton->mDiscreteMurphey->m_dm_HessianOfLagrangian_dq_dq;

  EXPECT_TRUE(hessianFd_q_q_mark1.isApprox(hessianAnalytical_q_q, 1e-2));
  EXPECT_TRUE(hessianFd_q_q_mark2.isApprox(hessianAnalytical_q_q, 1e-2));
  EXPECT_TRUE(hessianFd_q_dq.isApprox(hessianAnalytical_q_dq, 1e-2));
  EXPECT_TRUE(hessianFd_dq_dq.isApprox(hessianAnalytical_dq_dq, 1e-2));

//  std::cout << "finite difference: \n" << (hessianFd_q_q_mark1) << std::endl << std::endl;
//  std::cout << "finite difference: \n" << (hessianFd_q_q_mark2) << std::endl << std::endl;
//  std::cout << "analytical       : \n" << (hessianAnalytical_q_q) << std::endl << std::endl;

//  std::cout << "finite difference: \n" << (hessianFd_q_dq) << std::endl << std::endl;
//  std::cout << "analytical       : \n" << (hessianAnalytical_q_dq) << std::endl << std::endl;

//  std::cout << "finite difference: \n" << (hessianFd_dq_dq) << std::endl << std::endl;
//  std::cout << "analytical       : \n" << (hessianAnalytical_dq_dq) << std::endl << std::endl;
}

//==============================================================================
TEST(DMMurphey, CompareDelEquationOfJSAndMurpheyMethods)
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  const auto posLower = math::constantsd::pi() * -0.5;
  const auto posUpper = math::constantsd::pi() *  0.5;
  const auto velLower = math::constantsd::pi() * -0.5;
  const auto velUpper = math::constantsd::pi() *  0.5;
  auto skeleton1 = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skeleton1->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    joint->setPosition(0u, pos);
    joint->setVelocity(0u, vel);
  }
  const Eigen::VectorXd randomQ
      = skeleton1->getPositions()
      + Eigen::VectorXd::Constant(skeleton1->getNumDofs(), 0.01);

  auto skeleton2 = skeleton1->clone();
  skeleton2->setPositions(skeleton1->getPositions());
  skeleton2->setVelocities(skeleton1->getVelocities());
  skeleton2->setAccelerations(skeleton1->getAccelerations());

  EXPECT_EQ(skeleton1->getPositions(), skeleton2->getPositions());
  EXPECT_EQ(skeleton1->getVelocities(), skeleton2->getVelocities());

//  std::cout << skeleton1->getPositions().transpose() << std::endl;
//  std::cout << skeleton2->getPositions().transpose() << std::endl;

  skeleton1->dm_DRNEA_impulse_initialize();
  skeleton2->dm_initialize_Secant_SVI();

  EXPECT_TRUE(skeleton1->dm_computeFDEL_DRNEA(randomQ).isApprox(
                skeleton2->dm_computeFDEL_SVI(randomQ, 0.5)));

  std::cout << skeleton1->dm_computeFDEL_DRNEA(randomQ).transpose() << std::endl;
  std::cout << skeleton2->dm_computeFDEL_SVI(randomQ, 0.5).transpose() << std::endl;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
