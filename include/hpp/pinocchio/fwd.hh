//
// Copyright (c) 2016 CNRS
// Author: NMansard, Joseph Mirabel from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_FWD_HH
#define HPP_PINOCCHIO_FWD_HH

#ifndef PINOCCHIO_WITH_HPP_FCL
#error "hpp-fcl support in Pinocchio is mandatory."
#endif

#include <hpp/fcl/collision_data.h>  // DistanceResult

#include <Eigen/Core>
#include <hpp/fcl/fwd.hh>
#include <hpp/util/pointer.hh>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <vector>

namespace hpp {
namespace pinocchio {
typedef double value_type;

HPP_PREDEF_CLASS(Body);
HPP_PREDEF_CLASS(CollisionObject);
HPP_PREDEF_CLASS(Device);
HPP_PREDEF_CLASS(HumanoidRobot);
HPP_PREDEF_CLASS(Joint);
HPP_PREDEF_CLASS(JointConfiguration);
HPP_PREDEF_CLASS(Gripper);
HPP_PREDEF_CLASS(CenterOfMassComputation);
class Frame;
class AbstractDevice;
class DeviceSync;
struct DeviceData;

enum Request_t { COLLISION, DISTANCE };
enum InOutType { INNER, OUTER };

// Pinocchio typedefs
template <typename _Scalar, int _Options>
struct JointCollectionTpl;
typedef JointCollectionTpl<value_type, 0> JointCollection;

typedef ::pinocchio::JointIndex JointIndex;
typedef ::pinocchio::FrameIndex FrameIndex;
typedef ::pinocchio::GeomIndex GeomIndex;
typedef ::pinocchio::ModelTpl<value_type, 0, JointCollectionTpl> Model;
typedef ::pinocchio::DataTpl<value_type, 0, JointCollectionTpl> Data;
typedef ::pinocchio::GeometryModel GeomModel;
typedef ::pinocchio::GeometryData GeomData;
typedef ::pinocchio::SE3 Transform3f;
typedef ::pinocchio::SE3 SE3;
typedef ::pinocchio::JointModelTpl<value_type, 0, JointCollectionTpl>
    JointModel;

typedef Eigen::Array<bool, Eigen::Dynamic, 1> ArrayXb;

typedef Eigen::Matrix<value_type, Eigen::Dynamic, 1> vector_t;
typedef vector_t Configuration_t;
typedef Eigen::Ref<const Configuration_t> ConfigurationIn_t;
typedef Eigen::Ref<Configuration_t> ConfigurationOut_t;
typedef shared_ptr<Configuration_t> ConfigurationPtr_t;
typedef Eigen::Ref<const vector_t> vectorIn_t;
typedef Eigen::Ref<vector_t> vectorOut_t;
typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> matrix_t;
typedef Eigen::Ref<matrix_t> matrixOut_t;
typedef matrix_t::Index size_type;
typedef Eigen::Matrix<value_type, 3, 3> matrix3_t;
typedef Eigen::Matrix<value_type, 3, 1> vector3_t;
typedef Eigen::Matrix<value_type, 4, 1> vector4_t;
typedef Eigen::Matrix<value_type, 6, Eigen::Dynamic> JointJacobian_t;
typedef Eigen::Matrix<value_type, 3, Eigen::Dynamic> ComJacobian_t;
typedef Eigen::Block<JointJacobian_t, 3, Eigen::Dynamic> HalfJointJacobian_t;

struct JointVector;
typedef JointVector JointVector_t;
struct ObjectVector;
typedef ObjectVector ObjectVector_t;
typedef shared_ptr<Body> BodyPtr_t;
typedef shared_ptr<const Body> BodyConstPtr_t;
using fcl::CollisionGeometry;
typedef shared_ptr<CollisionGeometry> CollisionGeometryPtr_t;
typedef fcl::CollisionObject FclCollisionObject;
typedef fcl::CollisionObject* FclCollisionObjectPtr_t;
typedef const fcl::CollisionObject* FclConstCollisionObjectPtr_t;
typedef shared_ptr<CollisionObject> CollisionObjectPtr_t;
typedef shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;
typedef shared_ptr<Device> DevicePtr_t;
typedef shared_ptr<const Device> DeviceConstPtr_t;
typedef std::vector<fcl::DistanceResult> DistanceResults_t;
typedef shared_ptr<HumanoidRobot> HumanoidRobotPtr_t;
typedef shared_ptr<CenterOfMassComputation> CenterOfMassComputationPtr_t;
typedef shared_ptr<Joint> JointPtr_t;
typedef shared_ptr<const Joint> JointConstPtr_t;
typedef shared_ptr<Gripper> GripperPtr_t;
typedef std::vector<GripperPtr_t> Grippers_t;

typedef shared_ptr<Model> ModelPtr_t;
typedef shared_ptr<const Model> ModelConstPtr_t;
typedef shared_ptr<Data> DataPtr_t;
typedef shared_ptr<const Data> DataConstPtr_t;

typedef shared_ptr<GeomModel> GeomModelPtr_t;
typedef shared_ptr<const GeomModel> GeomModelConstPtr_t;
typedef shared_ptr<GeomData> GeomDataPtr_t;
typedef shared_ptr<const GeomData> GeomDataConstPtr_t;

template <typename vector_type>
class LiegroupElementConstBase;
template <typename vector_type>
class LiegroupElementBase;
/// Const reference to a \ref LiegroupElement
typedef LiegroupElementConstBase<vectorIn_t> LiegroupElementConstRef;
/// Element of a Lie group
typedef LiegroupElementBase<vector_t> LiegroupElement;
/// Writable reference to a \ref LiegroupElement
typedef LiegroupElementBase<vectorOut_t> LiegroupElementRef;

HPP_PREDEF_CLASS(LiegroupSpace);
typedef shared_ptr<LiegroupSpace> LiegroupSpacePtr_t;
typedef shared_ptr<const LiegroupSpace> LiegroupSpaceConstPtr_t;
}  // namespace pinocchio
}  // namespace hpp
#endif  // HPP_PINOCCHIO_FWD_HH
