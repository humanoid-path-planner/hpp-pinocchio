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

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

namespace se3
{

  // Compute the vector of joint index of the subtree.
  inline void
  subtreeIndexes(const Model & model, SubtreeModel & stModel)
  {
    JointIndex root = stModel.root;
    const size_t shift = root;
    std::vector<bool> shouldInclude (model.nbody-shift, true);
    for( Model::JointIndex i= (Model::JointIndex) (model.nbody-1);i>root;--i )
    {
      if (!shouldInclude[i - shift]) continue;
      Model::JointIndex j = i;
      while (j > root) j = model.parents[j];
      if (j == root) continue;
      j = i;
      while (j > root) {
        shouldInclude[j - shift] = false;
        j = model.parents[j];
      }
    }
    stModel.joints.clear();
    for(int i=(int)shouldInclude.size()-1; i>=0;--i)
      if (shouldInclude[i]) stModel.joints.push_back(i+shift);
  }


  struct SubtreeJacobianCenterOfMassBackwardStep
  : public fusion::JointVisitor<SubtreeJacobianCenterOfMassBackwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  JointIndex
                                  > ArgsType;

    JOINT_VISITOR_INIT(SubtreeJacobianCenterOfMassBackwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model& model,
                     se3::Data& data,
                     const JointIndex& r )
    {
      const Model::JointIndex & i      = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];

      typedef Data::Matrix6x Matrix6x;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;

      ColBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S());

      if( JointModel::NV==1 )
        data.Jcom.col(jmodel.idx_v())
        = data.mass[r] * Jcols.template topLeftCorner<3,1>()
        - data.com[r].cross(Jcols.template bottomLeftCorner<3,1>());
      else
        jmodel.jointCols(data.Jcom)
        = data.mass[r] * Jcols.template topRows<3>()
        - skew(data.com[r]) * Jcols.template bottomRows<3>();
    }

  };

  inline const Data::Matrix3x &
  subtreeJacobianCenterOfMass(const Model & model, Data & data,
                              const SubtreeModel & stModel)
  {
    Model::JointIndex r = stModel.root;
    assert (r == stModel.joints.back());
    data.Jcom.setZero();

    for(std::size_t j = 0; j < stModel.joints.size(); ++j)
    {
      Model::JointIndex i = stModel.joints[j];

      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();

      data.mass[i] = mass;
      data.com[i] = mass*data.oMi[i].act(lever);
    }

    // Backward step
    for(std::size_t j = 0; j < stModel.joints.size(); ++j)
    {
      Model::JointIndex i = stModel.joints[j];
      JacobianCenterOfMassBackwardStep
      ::run(model.joints[i],data.joints[i],
            JacobianCenterOfMassBackwardStep::ArgsType(model,data, true));
    }

    for (Model::JointIndex i = model.parents[r]; i != 0; i = model.parents[i])
    {
      SubtreeJacobianCenterOfMassBackwardStep
      ::run(model.joints[i],data.joints[i],
            SubtreeJacobianCenterOfMassBackwardStep::ArgsType(model,data, r));
    }

    data.Jcom /=  data.mass[r];

    return data.Jcom;
  }

} // namespace se3
