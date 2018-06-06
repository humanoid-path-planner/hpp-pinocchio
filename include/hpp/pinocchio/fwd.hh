//
// Copyright (c) 2016 CNRS
// Author: NMansard, Joseph Mirabel from Florent Lamiraux
//
//
// This file is part of hpp-pinocchio
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
// hpp-pinocchio  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_PINOCCHIO_FWD_HH
# define HPP_PINOCCHIO_FWD_HH

# include <vector>
# include <Eigen/Core>

# include <hpp/util/pointer.hh>
# include <hpp/fcl/fwd.hh>              // Needs boost/shared_ptr.hpp
# include <hpp/fcl/collision_data.h>    // DistanceResult

# include <pinocchio/spatial/fwd.hpp>
# include <pinocchio/multibody/fwd.hpp>
# include <pinocchio/multibody/joint/fwd.hpp>

namespace hpp {
  namespace pinocchio {
    HPP_PREDEF_CLASS (Body);
    HPP_PREDEF_CLASS (CollisionObject);
    HPP_PREDEF_CLASS (Device);
    HPP_PREDEF_CLASS (HumanoidRobot);
    HPP_PREDEF_CLASS (Joint);
    HPP_PREDEF_CLASS (JointConfiguration);
    HPP_PREDEF_CLASS (Gripper);
    HPP_PREDEF_CLASS (CenterOfMassComputation);
    class Frame;

    enum Request_t {COLLISION, DISTANCE};
    enum InOutType { INNER, OUTER };

    // Pinocchio typedefs
    typedef se3::JointIndex     JointIndex;
    typedef se3::FrameIndex     FrameIndex;
    typedef se3::GeomIndex      GeomIndex;
    typedef se3::Model          Model;
    typedef se3::Data           Data;
    typedef se3::GeometryModel  GeomModel;
    typedef se3::GeometryData   GeomData;
    typedef se3::SE3            Transform3f;
    typedef se3::JointModel     JointModel;

    typedef Eigen::Array <bool, Eigen::Dynamic, 1> ArrayXb;

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
    typedef Eigen::Matrix<value_type, 4, 1> vector4_t;
    typedef Eigen::Matrix<value_type, 6, Eigen::Dynamic> JointJacobian_t;
    typedef Eigen::Matrix<value_type, 3, Eigen::Dynamic> ComJacobian_t;
    typedef Eigen::Block <JointJacobian_t, 3, Eigen::Dynamic> HalfJointJacobian_t;

    struct JointVector;
    typedef JointVector JointVector_t;
    struct ObjectVector;
    typedef ObjectVector ObjectVector_t;
    typedef boost::shared_ptr<Body> BodyPtr_t;
    typedef boost::shared_ptr<const Body> BodyConstPtr_t;
    //typedef std::vector<BodyPtr_t> BodyVector_t;
    typedef       fcl::CollisionObject   FclCollisionObject;
    typedef       fcl::CollisionObject * FclCollisionObjectPtr_t;
    typedef const fcl::CollisionObject * FclConstCollisionObjectPtr_t;
    typedef boost::shared_ptr<CollisionObject> CollisionObjectPtr_t;
    typedef boost::shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;
    typedef boost::shared_ptr <Device> DevicePtr_t;
    typedef boost::shared_ptr <const Device> DeviceConstPtr_t;
    typedef std::vector <fcl::DistanceResult> DistanceResults_t;
    typedef boost::shared_ptr <HumanoidRobot> HumanoidRobotPtr_t;
    typedef boost::shared_ptr <CenterOfMassComputation> CenterOfMassComputationPtr_t;
    typedef boost::shared_ptr<Joint> JointPtr_t;
    typedef boost::shared_ptr<const Joint> JointConstPtr_t;
    typedef boost::shared_ptr <Gripper> GripperPtr_t;
    typedef std::vector <GripperPtr_t> Grippers_t;

    typedef boost::shared_ptr<Model>           ModelPtr_t;
    typedef boost::shared_ptr<const Model>     ModelConstPtr_t;
    typedef boost::shared_ptr<Data>            DataPtr_t;
    typedef boost::shared_ptr<const Data>      DataConstPtr_t;

    typedef boost::shared_ptr<GeomModel>       GeomModelPtr_t;
    typedef boost::shared_ptr<const GeomModel> GeomModelConstPtr_t;
    typedef boost::shared_ptr<GeomData>        GeomDataPtr_t;
    typedef boost::shared_ptr<const GeomData>  GeomDataConstPtr_t;

    template <typename vector_type> class LiegroupNonconstElementBase;
    typedef LiegroupNonconstElementBase<   vector_t> LiegroupElement;
    HPP_PREDEF_CLASS (LiegroupSpace);
    typedef boost::shared_ptr <LiegroupSpace> LiegroupSpacePtr_t;
    typedef boost::shared_ptr <const LiegroupSpace> LiegroupSpaceConstPtr_t;
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_FWD_HH
