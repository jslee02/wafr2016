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

#include "dart/dynamics/ScrewJoint.hpp"

#include <string>

#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
ScrewJoint::~ScrewJoint()
{
  // Do nothing
}

//==============================================================================
void ScrewJoint::setProperties(const Properties& _properties)
{
  GeometricJoint<math::R1Space>::setProperties(
        static_cast<const GeometricJoint<math::R1Space>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void ScrewJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void ScrewJoint::setAspectProperties(const AspectProperties& properties)
{
  setAxis(properties.mAxis);
  setPitch(properties.mPitch);
}

//==============================================================================
ScrewJoint::Properties ScrewJoint::getScrewJointProperties() const
{
  return Properties(getGeometricJointProperties(), mAspectProperties);
}

//==============================================================================
void ScrewJoint::copy(const ScrewJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getScrewJointProperties());
}

//==============================================================================
void ScrewJoint::copy(const ScrewJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
ScrewJoint& ScrewJoint::operator=(const ScrewJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& ScrewJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ScrewJoint::getStaticType()
{
  static const std::string name = "ScrewJoint";
  return name;
}

//==============================================================================
bool ScrewJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
void ScrewJoint::setAxis(const Eigen::Vector3d& _axis)
{
  if(_axis == mAspectProperties.mAxis)
    return;

  mAspectProperties.mAxis = _axis.normalized();
  Joint::notifyPositionUpdate();
  updateLocalJacobian();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& ScrewJoint::getAxis() const
{
  return mAspectProperties.mAxis;
}

//==============================================================================
void ScrewJoint::setPitch(double _pitch)
{
  if(_pitch == mAspectProperties.mPitch)
    return;

  mAspectProperties.mPitch = _pitch;
  Joint::notifyPositionUpdate();
  updateLocalJacobian();
  Joint::incrementVersion();
}

//==============================================================================
double ScrewJoint::getPitch() const
{
  return mAspectProperties.mPitch;
}

//==============================================================================
const GeometricJoint<math::R1Space>::JacobianMatrix
ScrewJoint::getLocalJacobianStatic(
    const GeometricJoint<math::R1Space>::Vector& /*positions*/) const
{
  using namespace dart::math::suffixes;

  Eigen::Vector6d S = Eigen::Vector6d::Zero();
  S.head<3>() = getAxis();
  S.tail<3>() = getAxis() * getPitch() * 0.5_pi;

  GeometricJoint<math::R1Space>::JacobianMatrix jacobian
      = math::AdT(Joint::mAspectProperties.mT_ChildBodyToJoint, S);

  assert(!math::isNan(jacobian));

  return jacobian;
}

//==============================================================================
ScrewJoint::ScrewJoint(const Properties& properties)
  : detail::ScrewJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createScrewJointAspect(properties);
  createGeometricJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* ScrewJoint::clone() const
{
  return new ScrewJoint(getScrewJointProperties());
}

//==============================================================================
void ScrewJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName, false);
}

//==============================================================================
void ScrewJoint::updateLocalTransform() const
{
  using namespace dart::math::suffixes;

  Eigen::Vector6d S = Eigen::Vector6d::Zero();
  S.head<3>() = getAxis();
  S.tail<3>() = getAxis()*getPitch()*0.5_pi;
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * math::expMap(S * getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
  assert(math::verifyTransform(mT));
}

//==============================================================================
void ScrewJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
    mJacobian = getLocalJacobianStatic(getPositionsStatic());
}

//==============================================================================
void ScrewJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of screw joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
