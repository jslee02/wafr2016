/*
 * Copyright (c) 2013-2016, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_PRISMATICJOINT_HPP_
#define DART_DYNAMICS_PRISMATICJOINT_HPP_

#include "dart/dynamics/detail/PrismaticJointAspect.hpp"

namespace dart {
namespace dynamics {

/// class RevoluteJoint
class PrismaticJoint : public detail::PrismaticJointBase
{
public:

  friend class Skeleton;
  using Base = detail::PrismaticJointBase;
  using UniqueProperties = detail::PrismaticJointUniqueProperties;
  using Properties = detail::PrismaticJointProperties;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, PrismaticJointAspect)

  PrismaticJoint(const PrismaticJoint&) = delete;

  /// Destructor
  virtual ~PrismaticJoint();

  /// Set the Properties of this PrismaticJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this PrismaticJoint
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectProperties of this PrismaticJoint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this PrismaticJoint
  Properties getPrismaticJointProperties() const;

  /// Copy the Properties of another PrismaticJoint
  void copy(const PrismaticJoint& _otherJoint);

  /// Copy the Properties of another PrismaticJoint
  void copy(const PrismaticJoint* _otherJoint);

  /// Same as copy(const PrismaticJoint&)
  PrismaticJoint& operator=(const PrismaticJoint& _otherJoint);
  
  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  ///
  void setAxis(const Eigen::Vector3d& _axis);

  ///
  const Eigen::Vector3d& getAxis() const;

  // Documentation inherited
  const GeometricJoint<math::R1Space>::JacobianMatrix getLocalJacobianStatic(
      const GeometricJoint<math::R1Space>::Vector& positions) const override;

protected:

  /// Constructor called by Skeleton class
  PrismaticJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateLocalTransform() const override;

  // Documentation inherited
  void updateLocalJacobian(bool _mandatory=true) const override;

  // Documentation inherited
  void updateLocalJacobianTimeDeriv() const override;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_PRISMATICJOINT_HPP_
