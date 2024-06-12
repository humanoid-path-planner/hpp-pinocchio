//
// Copyright (c) 2016 CNRS
// Author: NMansard
//
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <boost/serialization/vector.hpp>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/serialization.hh>
#include <limits>
#include <pinocchio/algorithm/jacobian.hpp>

#include "joint/bound.hh"

#define CALL_JOINT(method) model().joints[jointIndex].method()

namespace hpp {
namespace pinocchio {
using ::pinocchio::LOCAL;
using ::pinocchio::WORLD;

JointPtr_t Joint::create(DeviceWkPtr_t device, JointIndex indexInJointList) {
  if (indexInJointList == 0)
    return JointPtr_t();
  else
    return JointPtr_t(new Joint(device, indexInJointList));
}

Joint::Joint(DeviceWkPtr_t device, JointIndex indexInJointList)
    : maximalDistanceToParent_(-1),
      devicePtr(device),
      jointIndex(indexInJointList) {
  assert(devicePtr.lock());
  assert(robot()->modelPtr());
  assert(std::size_t(jointIndex) < model().joints.size());
  setChildList();
}

void Joint::setChildList() {
  assert(robot()->modelPtr());
  assert(robot()->dataPtr());
  const Data& data = robot()->data();
  children.clear();
  for (JointIndex child = jointIndex + 1;
       int(child) <= data.lastChild[jointIndex]; ++child)
    if (model().parents[child] == jointIndex) children.push_back(child);
}

inline void Joint::selfAssert() const {
  DevicePtr_t device = devicePtr.lock();
  assert(device);
  assert(device->modelPtr());
  assert(device->dataPtr());
  assert(device->model().joints.size() > std::size_t(jointIndex));
  assert(jointIndex > 0);
}

Model& Joint::model() {
  selfAssert();
  return robot()->model();
}
const Model& Joint::model() const {
  selfAssert();
  return robot()->model();
}

JointPtr_t Joint::parentJoint() const {
  selfAssert();
  JointIndex idParent = model().parents[jointIndex];
  if (idParent == 0)
    return JointPtr_t();
  else
    return JointPtr_t(new Joint(devicePtr, idParent));
}

const std::string& Joint::name() const {
  selfAssert();
  return model().names[jointIndex];
}

const Transform3f& Joint::currentTransformation(const DeviceData& d) const {
  selfAssert();
  return d.data_->oMi[jointIndex];
}

size_type Joint::numberDof() const {
  selfAssert();
  return CALL_JOINT(nv);
}

size_type Joint::configSize() const {
  selfAssert();
  return CALL_JOINT(nq);
}

size_type Joint::rankInConfiguration() const {
  selfAssert();
  return CALL_JOINT(idx_q);
}

size_type Joint::rankInVelocity() const {
  selfAssert();
  return CALL_JOINT(idx_v);
}

std::size_t Joint::numberChildJoints() const { return children.size(); }

JointPtr_t Joint::childJoint(std::size_t rank) const {
  selfAssert();
  assert(rank < children.size());
  return JointPtr_t(new Joint(devicePtr, children[rank]));
}

const Transform3f& Joint::positionInParentFrame() const {
  selfAssert();
  return model().jointPlacements[jointIndex];
}

void Joint::isBounded(size_type rank, bool bounded) {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  if (!bounded) {
    const value_type& inf = std::numeric_limits<value_type>::infinity();
    model().lowerPositionLimit[idx] = -inf;
    model().upperPositionLimit[idx] = inf;
  } else {
    assert(false &&
           "This function can only unset bounds. "
           "Use lowerBound and upperBound to set the bounds.");
  }
}
bool Joint::isBounded(size_type rank) const {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  return !std::isinf(model().lowerPositionLimit[idx]) &&
         !std::isinf(model().upperPositionLimit[idx]);
}
value_type Joint::lowerBound(size_type rank) const {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  return model().lowerPositionLimit[idx];
}
value_type Joint::upperBound(size_type rank) const {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  return model().upperPositionLimit[idx];
}
void Joint::lowerBound(size_type rank, value_type lowerBound) {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  model().lowerPositionLimit[idx] = lowerBound;
}
void Joint::upperBound(size_type rank, value_type upperBound) {
  const size_type idx = rankInConfiguration() + rank;
  assert(rank < configSize());
  model().upperPositionLimit[idx] = upperBound;
}
void Joint::lowerBounds(vectorIn_t lowerBounds) {
  selfAssert();
  SetBoundStep::run(jointModel(), SetBoundStep::ArgsType(
                                      lowerBounds, model().lowerPositionLimit));
}
void Joint::upperBounds(vectorIn_t upperBounds) {
  selfAssert();
  SetBoundStep::run(jointModel(), SetBoundStep::ArgsType(
                                      upperBounds, model().upperPositionLimit));
}

/* --- MAX DISTANCE ------------------------------------------------------*/
/* --- MAX DISTANCE ------------------------------------------------------*/
/* --- MAX DISTANCE ------------------------------------------------------*/

template <bool X, bool Y, bool Z, typename Derived>
value_type computeMaximalDistanceToParentForAlignedTranslation(
    const Eigen::MatrixBase<Derived>& lower,
    const Eigen::MatrixBase<Derived>& upper, const SE3& placement) {
  if (!lower.allFinite() || !upper.allFinite())
    return std::numeric_limits<value_type>::infinity();

  value_type d = 0;
  const size_type iX = 0;
  const size_type iY = (X ? 1 : 0);
  const size_type iZ = iY + (Y ? 1 : 0);
  vector3_t p(vector3_t::Zero());
  for (size_type i = 0; i < (X ? 1 : 2); ++i) {
    if (X) p[0] = (i == 0 ? lower[iX] : upper[iX]);
    for (size_type j = 0; j < (Y ? 1 : 2); ++j) {
      if (Y) p[1] = (j == 0 ? lower[iY] : upper[iY]);
      for (size_type k = 0; k < (Z ? 1 : 2); ++k) {
        if (Z) p[1] = (k == 0 ? lower[iZ] : upper[iZ]);
        d = std::max(d, placement.act(p).norm());
      }
    }
  }
  return d;
}

template <typename Joint>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/, const ::pinocchio::JointModelBase<Joint>&,
    const SE3& /*jointPlacement*/) {
  assert(false &&
         "The function <maximalDistance> as not been implemented for this "
         "class of joint");
  return 0.0;
}

value_type computeMaximalDistanceToParent(
    const Model& model, const ::pinocchio::JointModelFreeFlyer& jmodel,
    const SE3& jointPlacement) {
  const size_type& i = jmodel.idx_q();
  return computeMaximalDistanceToParentForAlignedTranslation<true, true, true>(
      model.lowerPositionLimit.segment< ::pinocchio::JointModelTranslation::NQ>(
          i),
      model.upperPositionLimit.segment< ::pinocchio::JointModelTranslation::NQ>(
          i),
      jointPlacement);
}

template <typename Scalar, int Options, int Axis>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/,
    const ::pinocchio::JointModelRevoluteTpl<Scalar, Options, Axis>&,
    const SE3& jointPlacement) {
  return jointPlacement.translation().norm();
}

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/,
    const ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>&,
    const SE3& jointPlacement) {
  return jointPlacement.translation().norm();
}

template <typename Scalar, int Options, int Axis>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/,
    const ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, Axis>&,
    const SE3& jointPlacement) {
  return jointPlacement.translation().norm();
}

template <typename Scalar, int Options, int Axis>
value_type computeMaximalDistanceToParent(
    const Model& model,
    const ::pinocchio::JointModelPrismaticTpl<Scalar, Options, Axis>& jmodel,
    const SE3& jointPlacement) {
  return computeMaximalDistanceToParentForAlignedTranslation<
      Axis == 0, Axis == 1, Axis == 2>(
      model.lowerPositionLimit.segment<1>(jmodel.idx_q()),
      model.upperPositionLimit.segment<1>(jmodel.idx_q()), jointPlacement);
}

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& model,
    const ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options>& jmodel,
    const SE3& jointPlacement) {
  if (std::isinf(model.lowerPositionLimit[jmodel.idx_q()]) ||
      std::isinf(model.upperPositionLimit[jmodel.idx_q()]))
    return std::numeric_limits<value_type>::infinity();

  Eigen::Vector3d pmin = jmodel.axis * model.lowerPositionLimit[jmodel.idx_q()];
  Eigen::Vector3d pmax = jmodel.axis * model.upperPositionLimit[jmodel.idx_q()];

  return std::max(jointPlacement.act(pmin).norm(),
                  jointPlacement.act(pmax).norm());
}

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/,
    const ::pinocchio::JointModelSphericalTpl<Scalar, Options>&,
    const SE3& jointPlacement) {
  return jointPlacement.translation().norm();
}

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& /*model*/,
    const ::pinocchio::JointModelSphericalZYXTpl<Scalar, Options>&,
    const SE3& jointPlacement) {
  return jointPlacement.translation().norm();
}

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& model,
    const ::pinocchio::JointModelTranslationTpl<Scalar, Options>& jmodel,
    const SE3& jointPlacement) {
  const size_type& i = jmodel.idx_q();
  return computeMaximalDistanceToParentForAlignedTranslation<true, true, true>(
      model.lowerPositionLimit.segment< ::pinocchio::JointModelTranslation::NQ>(
          i),
      model.upperPositionLimit.segment< ::pinocchio::JointModelTranslation::NQ>(
          i),
      jointPlacement);
}
// TODO (really?): handle the case where the translation is bounded.

template <typename Scalar, int Options>
value_type computeMaximalDistanceToParent(
    const Model& model,
    const ::pinocchio::JointModelPlanarTpl<Scalar, Options>& jmodel,
    const SE3& jointPlacement) {
  const size_type& i = jmodel.idx_q();
  return computeMaximalDistanceToParentForAlignedTranslation<true, true, false>(
      model.lowerPositionLimit.segment<2>(i),
      model.upperPositionLimit.segment<2>(i), jointPlacement);
}

struct VisitMaximalDistanceToParent : public boost::static_visitor<value_type> {
  const Model& model;
  const SE3& jointPlacement;
  VisitMaximalDistanceToParent(const Model& model, const SE3& jointPlacement)
      : model(model), jointPlacement(jointPlacement) {}

  template <typename Joint>
  value_type operator()(const ::pinocchio::JointModelBase<Joint>& jmodel) {
    return computeMaximalDistanceToParent(model, jmodel.derived(),
                                          jointPlacement);
  }
};

void Joint::computeMaximalDistanceToParent() {
  selfAssert();
  VisitMaximalDistanceToParent visitor(model(),
                                       model().jointPlacements[jointIndex]);
  maximalDistanceToParent_ = boost::apply_visitor(visitor, jointModel());
}

/* --- MAX VEL -----------------------------------------------------------*/
/* --- MAX VEL -----------------------------------------------------------*/
/* --- MAX VEL -----------------------------------------------------------*/

/* --- LINEAR VELOCITY ---------------------------------------------------*/
template <typename D>
value_type upperBoundLinearVelocity(const ::pinocchio::JointModelBase<D>&) {
  assert(false &&
         "The function <upperBoundLinearVel> as not been implemented for this "
         "class of joint");
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelFreeFlyerTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelRevoluteTpl<Scalar, Options, AXIS>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>&) {
  return 0.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, AXIS>&) {
  return 0.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelPrismaticTpl<Scalar, Options, AXIS>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelSphericalTpl<Scalar, Options>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelSphericalZYXTpl<Scalar, Options>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelTranslationTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundLinearVelocity(
    const ::pinocchio::JointModelPlanarTpl<Scalar, Options>&) {
  return 1.0;
}

struct VisitUpperBoundLinearVelocity
    : public boost::static_visitor<value_type> {
  template <typename Joint>
  value_type operator()(
      const ::pinocchio::JointModelBase<Joint>& jmodel) const {
    return upperBoundLinearVelocity(jmodel.derived());
  }
};

value_type Joint::upperBoundLinearVelocity() const {
  selfAssert();
  VisitUpperBoundLinearVelocity visitor;
  return boost::apply_visitor(visitor, jointModel());
}

/* --- ANGULAR VELOCITY -------------------------------------------------- */
template <typename D>
value_type upperBoundAngularVelocity(const ::pinocchio::JointModelBase<D>&) {
  assert(false &&
         "The function <upperBoundAngularVel> as not been implemented for this "
         "class of joint");
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelFreeFlyerTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelRevoluteTpl<Scalar, Options, AXIS>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, AXIS>&) {
  return 1.0;
}
template <typename Scalar, int Options, int AXIS>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelPrismaticTpl<Scalar, Options, AXIS>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelSphericalTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelSphericalZYXTpl<Scalar, Options>&) {
  return 1.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelTranslationTpl<Scalar, Options>&) {
  return 0.0;
}
template <typename Scalar, int Options>
value_type upperBoundAngularVelocity(
    const ::pinocchio::JointModelPlanarTpl<Scalar, Options>&) {
  return 1.0;
}

struct VisitUpperBoundAngularVelocity
    : public boost::static_visitor<value_type> {
  template <typename Joint>
  value_type operator()(
      const ::pinocchio::JointModelBase<Joint>& jmodel) const {
    return upperBoundAngularVelocity(jmodel.derived());
  }
};

value_type Joint::upperBoundAngularVelocity() const {
  selfAssert();
  VisitUpperBoundAngularVelocity visitor;
  return boost::apply_visitor(visitor, jointModel());
}

JointJacobian_t& Joint::jacobian(DeviceData& d, const bool local) const {
  selfAssert();
  if (robot()->computationFlag() & JACOBIAN) {
    std::logic_error(
        "Robot computation flag should contain JACOBIAN in "
        "order to retrieve the joint jacobians");
  }
  assert(jointIndex > 0 && jointIndex - 1 < d.jointJacobians_.size());
  JointJacobian_t& jacobian(d.jointJacobians_[jointIndex - 1]);
  if (jacobian.cols() != model().nv)
    jacobian = JointJacobian_t::Zero(6, model().nv);
  if (local)
    ::pinocchio::getJointJacobian(model(), *d.data_, jointIndex, LOCAL,
                                  jacobian);
  else
    ::pinocchio::getJointJacobian(model(), *d.data_, jointIndex, WORLD,
                                  jacobian);
  return jacobian;
}

BodyPtr_t Joint::linkedBody() const {
  return BodyPtr_t(new Body(devicePtr.lock(), jointIndex));
}

std::ostream& Joint::display(std::ostream& os) const {
  os << "Joint " << jointIndex << "(nq=" << configSize()
     << ",nv=" << numberDof() << ")" << std::endl;

  os << "\"" << name() << "\""
     << "[shape=box label=\"" << name() << "\\n";
  if (configSize() != 0)
    os << "Rank in configuration: " << rankInConfiguration() << "\\n";
  else
    os << "Anchor joint\\n";
  os << "Current transformation: " << currentTransformation();
  os << "\\n";
  os << "\"]" << std::endl;
  for (unsigned int iChild = 0; iChild < numberChildJoints(); iChild++) {
    // for (hpp::model::JointVector_t::const_iterator it =
    //    children_.begin (); it != children_.end (); ++it) {

    // write edges to children joints
    os << "\"" << name() << "\"->\"" << childJoint(iChild)->name() << "\""
       << std::endl;
  }
  return os;
}

template <typename LieGroupMap_t>
struct ConfigSpaceVisitor : public ::pinocchio::fusion::JointUnaryVisitorBase<
                                ConfigSpaceVisitor<LieGroupMap_t> > {
  typedef boost::fusion::vector<LiegroupSpace&> ArgsType;

  template <typename JointModel>
  static void algo(const ::pinocchio::JointModelBase<JointModel>& jmodel,
                   LiegroupSpace& space) {
    _algo(jmodel.derived(), space);
  }

  template <typename JointModel>
  static void _algo(const ::pinocchio::JointModelBase<JointModel>&,
                    LiegroupSpace& space) {
    typedef typename LieGroupMap_t::template operation<JointModel>::type LG_t;
    space *= LiegroupSpace::create(LG_t());
  }

  static void _algo(const JointModelComposite& jmodel, LiegroupSpace& space) {
    ::pinocchio::details::Dispatch<ConfigSpaceVisitor>::run(
        jmodel, ConfigSpaceVisitor::ArgsType(space));
  }
};

LiegroupSpacePtr_t Joint::configurationSpace() const {
  LiegroupSpacePtr_t res = LiegroupSpace::empty();
  ConfigSpaceVisitor<DefaultLieGroupMap>::ArgsType args(*res);
  ConfigSpaceVisitor<DefaultLieGroupMap>::run(jointModel(), args);
  return res;
}

LiegroupSpacePtr_t Joint::RnxSOnConfigurationSpace() const {
  LiegroupSpacePtr_t res = LiegroupSpace::empty();
  ConfigSpaceVisitor<RnxSOnLieGroupMap>::ArgsType args(*res);
  ConfigSpaceVisitor<RnxSOnLieGroupMap>::run(jointModel(), args);
  return res;
}

const JointModel& Joint::jointModel() const {
  selfAssert();
  return model().joints[index()];
}

DeviceData& Joint::data() const { return devicePtr.lock()->d(); }

template <class Archive>
void Joint::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  ar& BOOST_SERIALIZATION_NVP(devicePtr);
  ar& BOOST_SERIALIZATION_NVP(jointIndex);
  if (!Archive::is_saving::value) {
    maximalDistanceToParent_ = -1;
    setChildList();
  }
}

HPP_SERIALIZATION_IMPLEMENT(Joint);
}  // namespace pinocchio
}  // namespace hpp
