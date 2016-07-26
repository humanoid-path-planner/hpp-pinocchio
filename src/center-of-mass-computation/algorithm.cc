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
  // inline void
  // subtreeIndexes(const Model & model, SubtreeModel & stModel)
  // {
  //   JointIndex root = stModel.root;
  //   const size_t shift = root;
  //   std::vector<bool> shouldInclude (model.nbody-shift, true);
  //   for( Model::JointIndex i= (Model::JointIndex) (model.nbody-1);i>root;--i )
  //   {
  //     if (!shouldInclude[i - shift]) continue;
  //     Model::JointIndex j = i;
  //     while (j > root) j = model.parents[j];
  //     if (j == root) continue;
  //     j = i;
  //     while (j > root) {
  //       shouldInclude[j - shift] = false;
  //       j = model.parents[j];
  //     }
  //   }
  //   stModel.joints.clear();
  //   for(int i=(int)shouldInclude.size()-1; i>=0;--i)
  //     if (shouldInclude[i]) stModel.joints.push_back(i+shift);
  // }


  // struct SubtreeJacobianCenterOfMassBackwardStep
  // : public fusion::JointVisitor<SubtreeJacobianCenterOfMassBackwardStep>
  // {
  //   typedef boost::fusion::vector<const se3::Model &,
  //                                 se3::Data &,
  //                                 JointIndex
  //                                 > ArgsType;

  //   JOINT_VISITOR_INIT(SubtreeJacobianCenterOfMassBackwardStep);

  //   template<typename JointModel>
  //   static void algo(const se3::JointModelBase<JointModel> & jmodel,
  //                    se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
  //                    const se3::Model& model,
  //                    se3::Data& data,
  //                    const JointIndex& r )
  //   {
  //     const Model::JointIndex & i      = (Model::JointIndex) jmodel.id();
  //     const Model::JointIndex & parent = model.parents[i];

  //     if (i != r) {
  //       data.com[parent]  += data.com[i];
  //       data.mass[parent] += data.mass[i];
  //     }

  //     typedef Data::Matrix6x Matrix6x;
  //     typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;

  //     ColBlock Jcols = jmodel.jointCols(data.J);
  //     Jcols = data.oMi[i].act(jdata.S());

  //     if( JointModel::NV==1 )
  //       data.Jcom.col(jmodel.idx_v())
  //       = data.mass[i] * Jcols.template topLeftCorner<3,1>()
  //       - data.com[i].cross(Jcols.template bottomLeftCorner<3,1>());
  //     else
  //       jmodel.jointCols(data.Jcom)
  //       = data.mass[i] * Jcols.template topRows<3>()
  //       - skew(data.com[i]) * Jcols.template bottomRows<3>();

  //     data.com[i] /= data.mass[i];
  //   }

  // }    ;

  inline void
  subtreeCenterOfMass(const Model & model, Data & data,
                              const std::vector <JointIndex>& roots)
  {
    data.com[0].setZero ();
    data.mass[0] = 0;
    
    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.nbody);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      data.mass[i] = mass;
      data.com[i] = mass*data.oMi[i].act(lever);
    }

    // Assume root is sorted from largest id.
    // Nasty cast below, from (u-long) size_t to int.
    for( int root=int(roots.size()-1); root>=0; --root )
      {
        se3::JointIndex rootId = roots[root];

        // Backward loop on descendents
        for( se3::JointIndex jid = data.lastChild[rootId];jid>=rootId;--jid )
          {
            const Model::JointIndex & parent = model.parents[jid];
            data.com [parent] += data.com [jid];
            data.mass[parent] += data.mass[jid];
          } // end for jid

        // Backward loop on ancestors
        se3::JointIndex jid = model.parents[rootId];
        rootId = (root>0) ? roots[root-1] : 0;
        while (jid>rootId) // stopping when meeting the next subtree
          {
            const Model::JointIndex & parent = model.parents[jid];
            data.com [parent] += data.com [jid];
            data.mass[parent] += data.mass[jid];
            jid = parent;
          } // end while
      } // end for i

    data.com[0]  /= data.mass[0];
    data.Jcom    /= data.mass[0];
  }

  inline void
  subtreeJacobianCenterOfMass(const Model & model, Data & data,
                              const std::vector <JointIndex>& roots)
  {
    data.com[0].setZero ();
    data.mass[0] = 0;
    
    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.nbody);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      data.mass[i] = mass;
      data.com[i] = mass*data.oMi[i].act(lever);
    }

    // Assume root is sorted from largest id.
    // Nasty cast below, from (u-long) size_t to int.
    for( int root=int(roots.size()-1); root>=0; --root )
      {
        se3::JointIndex rootId = roots[root];

        // Backward loop on descendents
        for( se3::JointIndex jid = data.lastChild[rootId];jid>=rootId;--jid )
          {
            std::cout << "Bwd1 " << jid << std::endl;
            se3::JacobianCenterOfMassBackwardStep
              ::run(model.joints[jid],data.joints[jid],
                    JacobianCenterOfMassBackwardStep::ArgsType(model,data,false));
            std::cout << data.oMi [jid] << std::endl;
            std::cout << data.mass[jid] << std::endl;
            std::cout << data.com [jid].transpose() << std::endl;
          } // end for jid

        // Backward loop on ancestors
        se3::JointIndex jid = model.parents[rootId];
        rootId = (root>0) ? roots[root-1] : 0;
        while (jid>rootId) // stopping when meeting the next subtree
          {
            std::cout << "Bwd2 " << jid << std::endl;
            se3::JacobianCenterOfMassBackwardStep
              ::run(model.joints[jid],data.joints[jid],
                    JacobianCenterOfMassBackwardStep::ArgsType(model,data,false));
            
            jid = model.parents[jid];
          } // end while
      } // end for i

    data.com[0]  /= data.mass[0];
    data.Jcom    /= data.mass[0];
  }

} // namespace se3
