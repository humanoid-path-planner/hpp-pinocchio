//
// Copyright (c) 2016 CNRS
// Author: NMansard, Joseph Mirabel from Florent Lamiraux
//
//
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_PINOCCHIO_FWD_HH
# define HPP_PINOCCHIO_FWD_HH

# include <vector>
# include <list>
# include <map>
# include <Eigen/Core>
# include <hpp/util/pointer.hh>
# include <hpp/fcl/fwd.hh>
# include <hpp/fcl/math/matrix_3f.h>
# include <pinocchio/multibody/model.hpp>

# include <hpp/pinocchio/deprecated.hh>

namespace se3
{
  struct Model;
  struct Data;
  struct GeometryModel;
  struct GeometryData;
  struct DistanceResult;
}

/*# define HPP_PREDEF_CLASS_AND_POINTERS(NAME)                          \
  HPP_PREDEF_CLASS(NAME);                                               \
  typedef boost::shared_ptr<NAME> NAME##Ptr_t;                          \
  typedef boost::shared_ptr<const NAME> NAME##ConstPtr_t;               \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n
*/

namespace hpp {
  namespace pinocchio {
    HPP_PREDEF_CLASS (Body);
    HPP_PREDEF_CLASS (CollisionObject);
    HPP_PREDEF_CLASS (Device);
    //DEPREC HPP_PREDEF_CLASS (DistanceResult);
    HPP_PREDEF_CLASS (HumanoidRobot);
    HPP_PREDEF_CLASS (Joint);
    HPP_PREDEF_CLASS (JointConfiguration);
    HPP_PREDEF_CLASS (Gripper);
    HPP_PREDEF_CLASS (CenterOfMassComputation);
    enum Request_t {COLLISION, DISTANCE};

    typedef double value_type;
    typedef Eigen::Matrix <value_type, Eigen::Dynamic, 1> vector_t;
    typedef vector_t Configuration_t;
    typedef Eigen::Ref <const Configuration_t> ConfigurationIn_t;
    typedef Eigen::Ref <Configuration_t> ConfigurationOut_t;
    typedef boost::shared_ptr <Configuration_t> ConfigurationPtr_t;
    typedef Eigen::Ref <const vector_t> vectorIn_t;
    typedef Eigen::Ref <vector_t> vectorOut_t;
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> matrix_t;
    typedef Eigen::Ref <matrix_t> matrixOut_t;
    typedef matrix_t::Index size_type;
    typedef Eigen::Matrix<value_type, 3, 3> matrix3_t;
    typedef Eigen::Matrix<value_type, 3, 1> vector3_t;
    typedef se3::Data::Matrix6x JointJacobian_t;
    typedef se3::Data::Matrix3x ComJacobian_t;
    typedef Eigen::Block <JointJacobian_t, 3, Eigen::Dynamic> HalfJointJacobian_t;

    struct JointVector;
    typedef JointVector JointVector_t;
    typedef boost::shared_ptr<Body> BodyPtr_t;
    typedef boost::shared_ptr<const Body> BodyConstPtr_t;
    //typedef std::vector<BodyPtr_t> BodyVector_t;
    typedef fcl::CollisionObject * fclCollisionObjectPtr_t;
    typedef const fcl::CollisionObject * fclConstCollisionObjectPtr_t;
    typedef boost::shared_ptr<CollisionObject> CollisionObjectPtr_t;
    typedef boost::shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;
    //DEPREC typedef std::list <CollisionObjectPtr_t> ObjectVector_t;
    typedef boost::shared_ptr <Device> DevicePtr_t;
    typedef boost::shared_ptr <const Device> DeviceConstPtr_t;
    typedef std::vector <se3::DistanceResult> DistanceResults_t;
    typedef boost::shared_ptr <HumanoidRobot> HumanoidRobotPtr_t;
    typedef boost::shared_ptr <CenterOfMassComputation> CenterOfMassComputationPtr_t;
    typedef boost::shared_ptr<Joint> JointPtr_t;
    typedef boost::shared_ptr<const Joint> JointConstPtr_t;
    typedef boost::shared_ptr <Gripper> GripperPtr_t;
    typedef std::vector <GripperPtr_t> Grippers_t;
    //DEPREC typedef fcl::Transform3f Transform3f;
    typedef se3::SE3 Transform3f;

    typedef boost::shared_ptr<se3::Model>               ModelPtr_t;
    typedef boost::shared_ptr<const se3::Model>         ModelConstPtr_t;
    typedef boost::shared_ptr<se3::Data>                DataPtr_t;
    typedef boost::shared_ptr<const se3::Data>          DataConstPtr_t;

    typedef boost::shared_ptr<se3::GeometryModel>       GeomModelPtr_t;
    typedef boost::shared_ptr<const se3::GeometryModel> GeomModelConstPtr_t;
    typedef boost::shared_ptr<se3::GeometryData>        GeomDataPtr_t;
    typedef boost::shared_ptr<const se3::GeometryData>  GeomDataConstPtr_t;

  } // namespace pinocchio
} // namespace hpp
#endif //HPP_MODEL_FWD_HH
