// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/liegroup.hh>
#include <hpp/pinocchio/util.hh>
#include <hpp/util/indent.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/liegroup/liegroup.hpp>

namespace hpp {
namespace pinocchio {
void saturate(const DevicePtr_t& robot, ConfigurationOut_t configuration) {
  const Model& model = robot->model();
  configuration.head(model.nq) =
      model.upperPositionLimit.cwiseMin(configuration.head(model.nq));
  configuration.head(model.nq) =
      model.lowerPositionLimit.cwiseMax(configuration.head(model.nq));

  const ExtraConfigSpace& ecs = robot->extraConfigSpace();
  const size_type& d = ecs.dimension();
  configuration.tail(d) = ecs.upper().cwiseMin(configuration.tail(d));
  configuration.tail(d) = ecs.lower().cwiseMax(configuration.tail(d));
}

bool saturate(const DevicePtr_t& robot, ConfigurationOut_t configuration,
              ArrayXb& saturation) {
  bool ret = false;
  const Model& model = robot->model();

  for (std::size_t i = 1; i < model.joints.size(); ++i) {
    const size_type nq = model.joints[i].nq();
    const size_type nv = model.joints[i].nv();
    const size_type idx_q = model.joints[i].idx_q();
    const size_type idx_v = model.joints[i].idx_v();
    for (size_type j = 0; j < nq; ++j) {
      const size_type iq = idx_q + j;
      const size_type iv = idx_v + std::min(j, nv - 1);
      if (configuration[iq] > model.upperPositionLimit[iq]) {
        saturation[iv] = true;
        configuration[iq] = model.upperPositionLimit[iq];
        ret = true;
      } else if (configuration[iq] < model.lowerPositionLimit[iq]) {
        saturation[iv] = true;
        configuration[iq] = model.lowerPositionLimit[iq];
        ret = true;
      } else
        saturation[iv] = false;
    }
  }

  const ExtraConfigSpace& ecs = robot->extraConfigSpace();
  const size_type& d = ecs.dimension();

  for (size_type k = 0; k < d; ++k) {
    const size_type iq = model.nq + k;
    const size_type iv = model.nv + k;
    if (configuration[iq] > ecs.upper(k)) {
      saturation[iv] = true;
      configuration[iq] = ecs.upper(k);
      ret = true;
    } else if (configuration[iq] < ecs.lower(k)) {
      saturation[iv] = true;
      configuration[iq] = ecs.lower(k);
      ret = true;
    } else
      saturation[iv] = false;
  }
  return ret;
}

template <bool saturateConfig, typename LieGroup>
void integrate(const DevicePtr_t& robot, ConfigurationIn_t configuration,
               vectorIn_t velocity, ConfigurationOut_t result) {
  const Model& model = robot->model();
  result.head(model.nq) =
      ::pinocchio::integrate<LieGroup>(model, configuration, velocity);
  const size_type& dim = robot->extraConfigSpace().dimension();
  result.tail(dim) = configuration.tail(dim) + velocity.tail(dim);
  if (saturateConfig) saturate(robot, result);
}

template void integrate<true, RnxSOnLieGroupMap>(
    const DevicePtr_t& robot, ConfigurationIn_t configuration,
    vectorIn_t velocity, ConfigurationOut_t result);
template void integrate<false, RnxSOnLieGroupMap>(
    const DevicePtr_t& robot, ConfigurationIn_t configuration,
    vectorIn_t velocity, ConfigurationOut_t result);
template void integrate<true, DefaultLieGroupMap>(
    const DevicePtr_t& robot, ConfigurationIn_t configuration,
    vectorIn_t velocity, ConfigurationOut_t result);
template void integrate<false, DefaultLieGroupMap>(
    const DevicePtr_t& robot, ConfigurationIn_t configuration,
    vectorIn_t velocity, ConfigurationOut_t result);

void integrate(const DevicePtr_t& robot, ConfigurationIn_t configuration,
               vectorIn_t velocity, ConfigurationOut_t result) {
  integrate<true, DefaultLieGroupMap>(robot, configuration, velocity, result);
}

template <typename LieGroup>
void interpolate(const DevicePtr_t& robot, ConfigurationIn_t q0,
                 ConfigurationIn_t q1, const value_type& u,
                 ConfigurationOut_t result) {
  const Model& model = robot->model();
  result.head(model.nq) = ::pinocchio::interpolate<LieGroup>(model, q0, q1, u);
  const size_type& dim = robot->extraConfigSpace().dimension();
  result.tail(dim) = u * q1.tail(dim) + (1 - u) * q0.tail(dim);
}

template void interpolate<DefaultLieGroupMap>(const DevicePtr_t& robot,
                                              ConfigurationIn_t q0,
                                              ConfigurationIn_t q1,
                                              const value_type& u,
                                              ConfigurationOut_t result);
template void interpolate<RnxSOnLieGroupMap>(const DevicePtr_t& robot,
                                             ConfigurationIn_t q0,
                                             ConfigurationIn_t q1,
                                             const value_type& u,
                                             ConfigurationOut_t result);

// TODO remove me. This is kept for backward compatibility
template void interpolate< ::pinocchio::LieGroupMap>(const DevicePtr_t& robot,
                                                     ConfigurationIn_t q0,
                                                     ConfigurationIn_t q1,
                                                     const value_type& u,
                                                     ConfigurationOut_t result);

void interpolate(const DevicePtr_t& robot, ConfigurationIn_t q0,
                 ConfigurationIn_t q1, const value_type& u,
                 ConfigurationOut_t result) {
  interpolate<RnxSOnLieGroupMap>(robot, q0, q1, u, result);
}

template <typename LieGroup>
void difference(const DevicePtr_t& robot, ConfigurationIn_t q1,
                ConfigurationIn_t q2, vectorOut_t result) {
  const Model& model = robot->model();
  result.head(model.nv) = ::pinocchio::difference<LieGroup>(model, q2, q1);
  const size_type& dim = robot->extraConfigSpace().dimension();
  result.tail(dim) = q1.tail(dim) - q2.tail(dim);
}

template void difference<DefaultLieGroupMap>(const DevicePtr_t& robot,
                                             ConfigurationIn_t q1,
                                             ConfigurationIn_t q2,
                                             vectorOut_t result);
// TODO remove me. This is kept for backward compatibility
template void difference< ::pinocchio::LieGroupMap>(const DevicePtr_t& robot,
                                                    ConfigurationIn_t q1,
                                                    ConfigurationIn_t q2,
                                                    vectorOut_t result);

template void difference<RnxSOnLieGroupMap>(const DevicePtr_t& robot,
                                            ConfigurationIn_t q1,
                                            ConfigurationIn_t q2,
                                            vectorOut_t result);

void difference(const DevicePtr_t& robot, ConfigurationIn_t q1,
                ConfigurationIn_t q2, vectorOut_t result) {
  difference<RnxSOnLieGroupMap>(robot, q1, q2, result);
}

bool isApprox(const DevicePtr_t& robot, ConfigurationIn_t q1,
              ConfigurationIn_t q2, value_type eps) {
  if (!::pinocchio::isSameConfiguration< ::pinocchio::LieGroupMap>(
          robot->model(), q1, q2, eps))
    return false;
  const size_type& dim = robot->extraConfigSpace().dimension();
  return q2.tail(dim).isApprox(q1.tail(dim), eps);
}

value_type distance(const DevicePtr_t& robot, ConfigurationIn_t q1,
                    ConfigurationIn_t q2) {
  vector_t dist = ::pinocchio::squaredDistance< ::pinocchio::LieGroupMap>(
      robot->model(), q1, q2);
  const size_type& dim = robot->extraConfigSpace().dimension();
  if (dim == 0)
    return sqrt(dist.sum());
  else
    return sqrt(dist.sum() + (q2.tail(dim) - q1.tail(dim)).squaredNorm());
}

void normalize(const DevicePtr_t& robot, Configuration_t& q) {
  ::pinocchio::normalize(robot->model(), q);
}

struct IsNormalizedStep
    : public ::pinocchio::fusion::JointUnaryVisitorBase<IsNormalizedStep> {
  typedef boost::fusion::vector<ConfigurationIn_t, const value_type&, bool&>
      ArgsType;

  template <typename JointModel>
  static void algo(const ::pinocchio::JointModelBase<JointModel>& jmodel,
                   ConfigurationIn_t q, const value_type& eps, bool& ret) {
    typedef typename RnxSOnLieGroupMap::operation<JointModel>::type LG_t;
    ret = ret && LG_t::isNormalized(jmodel.jointConfigSelector(q), eps);
  }
};

template <>
void IsNormalizedStep::algo<JointModelComposite>(
    const ::pinocchio::JointModelBase<JointModelComposite>& jmodel,
    ConfigurationIn_t q, const value_type& eps, bool& ret) {
  ::pinocchio::details::Dispatch<IsNormalizedStep>::run(
      jmodel.derived(), IsNormalizedStep::ArgsType(q, eps, ret));
}

bool isNormalized(const DevicePtr_t& robot, ConfigurationIn_t q,
                  const value_type& eps) {
  bool ret = true;
  const Model& model = robot->model();
  for (std::size_t i = 1; i < (std::size_t)model.njoints; ++i) {
    IsNormalizedStep::run(model.joints[i],
                          IsNormalizedStep::ArgsType(q, eps, ret));
    if (!ret) return false;
  }
  return true;
}

std::ostream& display(std::ostream& os, const SE3& m) {
  return os << pretty_print(m);
}
}  // namespace pinocchio
}  // namespace hpp
