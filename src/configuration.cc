// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
// hpp-pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/pinocchio/configuration.hh>

#include <hpp/util/indent.hh>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/liegroup/liegroup.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup.hh>

namespace hpp {
  namespace pinocchio {
    void saturate (const DevicePtr_t& robot,
			  ConfigurationOut_t configuration)
    {
      const se3::Model& model = robot->model();
      configuration.head(model.nq) = model.upperPositionLimit.cwiseMin(configuration.head(model.nq));
      configuration.head(model.nq) = model.lowerPositionLimit.cwiseMax(configuration.head(model.nq));

      const ExtraConfigSpace& ecs = robot->extraConfigSpace();
      const size_type& d = ecs.dimension();
      configuration.tail(d) = ecs.upper().cwiseMin(configuration.tail(d));
      configuration.tail(d) = ecs.lower().cwiseMax(configuration.tail(d));
    }

    template<bool saturateConfig, typename LieGroup>
    void integrate (const DevicePtr_t& robot,
                    ConfigurationIn_t configuration,
                    vectorIn_t velocity, ConfigurationOut_t result)
    {
      const se3::Model& model = robot->model();
      result.head(model.nq) = se3::integrate<LieGroup>(model, configuration, velocity);
      const size_type& dim = robot->extraConfigSpace().dimension();
      result.tail (dim) = configuration.tail (dim) + velocity.tail (dim);
      if (saturateConfig) saturate(robot, result);
    }

    template void integrate<true,  LieGroupTpl>
                                  (const DevicePtr_t& robot,
                                   ConfigurationIn_t configuration,
                                   vectorIn_t velocity, ConfigurationOut_t result);
    template void integrate<false, LieGroupTpl>
                                  (const DevicePtr_t& robot,
                                   ConfigurationIn_t configuration,
                                   vectorIn_t velocity, ConfigurationOut_t result);
    template void integrate<true,  se3::LieGroupTpl>
                                  (const DevicePtr_t& robot,
                                   ConfigurationIn_t configuration,
                                   vectorIn_t velocity, ConfigurationOut_t result);
    template void integrate<false, se3::LieGroupTpl>
                                  (const DevicePtr_t& robot,
                                   ConfigurationIn_t configuration,
                                   vectorIn_t velocity, ConfigurationOut_t result);

    void integrate (const DevicePtr_t& robot,
                           ConfigurationIn_t configuration,
                           vectorIn_t velocity, ConfigurationOut_t result)
    {
      integrate<true, se3::LieGroupTpl> (robot, configuration, velocity, result);
    }

    template <typename LieGroup>
    void interpolate (const DevicePtr_t& robot,
                      ConfigurationIn_t q0,
                      ConfigurationIn_t q1,
                      const value_type& u,
                      ConfigurationOut_t result)
    {
      result = se3::interpolate<LieGroup>(robot->model(), q0, q1, u);
      const size_type& dim = robot->extraConfigSpace().dimension();
      result.tail (dim) = u * q1.tail (dim) + (1-u) * q0.tail (dim);
    }

    template void interpolate<se3::LieGroupTpl> (const DevicePtr_t& robot,
                                                 ConfigurationIn_t q0,
                                                 ConfigurationIn_t q1,
                                                 const value_type& u,
                                                 ConfigurationOut_t result);

    void interpolate (const DevicePtr_t& robot,
                      ConfigurationIn_t q0,
                      ConfigurationIn_t q1,
                      const value_type& u,
                      ConfigurationOut_t result)
    {
      interpolate<LieGroupTpl> (robot, q0, q1, u, result);
    }

    template <typename LieGroup>
    void difference (const DevicePtr_t& robot, ConfigurationIn_t q1,
                     ConfigurationIn_t q2, vectorOut_t result)
    {
      result = se3::differentiate<LieGroup> (robot->model(), q2, q1);
      const size_type& dim = robot->extraConfigSpace().dimension();
      result.tail (dim) = q1.tail (dim) - q2.tail (dim);
    }

    template void difference <se3::LieGroupTpl> (const DevicePtr_t& robot,
						 ConfigurationIn_t q1,
						 ConfigurationIn_t q2,
						 vectorOut_t result);

    void difference (const DevicePtr_t& robot, ConfigurationIn_t q1,
                     ConfigurationIn_t q2, vectorOut_t result)
    {
      difference <LieGroupTpl> (robot, q1, q2, result);
    }

    bool isApprox (const DevicePtr_t& robot, ConfigurationIn_t q1,
			  ConfigurationIn_t q2, value_type eps)
    {
      if (!se3::isSameConfiguration<se3::LieGroupTpl>(robot->model(), q1, q2, eps)) return false;
      const size_type& dim = robot->extraConfigSpace().dimension();
      return q2.tail (dim).isApprox (q1.tail (dim), eps);
    }

    value_type distance (const DevicePtr_t& robot, ConfigurationIn_t q1,
                         ConfigurationIn_t q2)
    {
      vector_t dist = se3::squaredDistance<se3::LieGroupTpl>(robot->model(), q1, q2);
      const size_type& dim = robot->extraConfigSpace().dimension();
      if (dim == 0) return sqrt(dist.sum());
      else return sqrt (dist.sum() + (q2.tail (dim) - q1.tail (dim)).squaredNorm ());
    }

    void normalize (const DevicePtr_t& robot, Configuration_t& q)
    {
      se3::normalize(robot->model(), q);
    }

    struct IsNormalizedStep : public se3::fusion::JointModelVisitor<IsNormalizedStep>
    {
      typedef boost::fusion::vector<ConfigurationIn_t,
                                    const value_type &,
                                    bool &> ArgsType;

      JOINT_MODEL_VISITOR_INIT(IsNormalizedStep);

      template<typename JointModel>
      static void algo(const se3::JointModelBase<JointModel> & jmodel,
                       ConfigurationIn_t q,
                       const value_type & eps,
                       bool & ret)
      {
        typedef typename LieGroupTpl::operation<JointModel>::type LG_t;
        ret = ret && LG_t::isNormalized(jmodel.jointConfigSelector(q), eps);
      }
    };

    template<>
    void IsNormalizedStep::algo<se3::JointModelComposite>(const se3::JointModelBase<se3::JointModelComposite> & jmodel,
                     ConfigurationIn_t q,
                     const value_type & eps,
                     bool & ret)
    {
      se3::details::Dispatch<IsNormalizedStep>::run(jmodel, IsNormalizedStep::ArgsType(q, eps, ret));
    }

    bool isNormalized (const DevicePtr_t& robot, ConfigurationIn_t q, const value_type& eps)
    {
      bool ret = true;
      const se3::Model& model = robot->model();
      for (std::size_t i = 1; i < (std::size_t)model.njoints; ++i) {
        IsNormalizedStep::run(model.joints[i],
                         IsNormalizedStep::ArgsType(q, eps, ret));
        if (!ret) return false;
      }
      return true;
    }

    std::ostream& display (std::ostream& os, const se3::SE3& m)
    {
      os <<         "R = " << m.rotation().row(0)
        << iendl << "    " << m.rotation().row(1)
        << iendl << "    " << m.rotation().row(2)
        << iendl << "p = " << m.translation().transpose();
      return os;
    }
  } // namespace pinocchio
} // namespace hpp
