// Copyright (C) 2016 by Joseph Mirabel
//
// This file is part of the hpp-pinocchio.
//
// hpp-pinocchio is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-pinocchio.  If not, see
// <http://www.gnu.org/licenses/>.


/// \brief Utility functions.

#ifndef HPP_PINOCCHIO_URDF_UTIL
# define HPP_PINOCCHIO_URDF_UTIL

#include <hpp/pinocchio/fwd.hh>

namespace hpp
{
  namespace pinocchio
  {
    namespace urdf
    {
      /// Load robot model by name
      ///
      /// \param robot Empty robot created before calling the function.
      ///        Users can pass an instance of a class deriving from Device.
      /// \param baseJoint joint to which the joint tree is added.
      /// \param prefix string to insert before all names
      ///               (joint, link, body names)
      /// \param rootJointType type of root joint among "anchor", "freeflyer",
      /// "planar",
      /// \param package ros package containing the model
      /// \param modelName robot model name
      /// \param urdfSuffix suffix for urdf file
      /// \param srdfSuffix suffix for srdf file

      /// \note This function reads the following files:
      /// \li
      /// package://${package}/urdf/${modelName}${urdfSuffix}.urdf
      /// \li
      /// package://${package}/srdf/${modelName}${srdfSuffix}.srdf
      void loadRobotModel (const DevicePtr_t& robot,
                           const JointPtr_t&  baseJoint,
			   const std::string& prefix,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
			   const std::string& srdfSuffix);
      void loadRobotModel (const DevicePtr_t& robot,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
			   const std::string& srdfSuffix);

      /// Load humanoid robot model by name
      ///
      /// \param robot Empty robot created before calling the function.
      ///        Users can pass an instance of a class deriving from
      ///        HumanoidRobot.
      /// \param baseJoint joint to which the joint tree is added.
      /// \param prefix string to insert before all names
      ///               (joint, link, body names)
      /// \param rootJointType type of root joint among "anchor", "freeflyer",
      /// "planar",
      /// \param package ros package containing the model
      /// \param modelName robot model name
      /// \param urdfSuffix suffix for urdf file
      /// \param srdfSuffix suffix for srdf file

      /// \note This function reads the following files:
      /// \li
      /// package://${package}/urdf/${modelName}${urdfSuffix}.urdf
      /// \li
      /// package://${package}/srdf/${modelName}${srdfSuffix}.srdf
      void loadHumanoidModel (const HumanoidRobotPtr_t& robot,
                              const JointPtr_t&  baseJoint,
                              const std::string& prefix,
			      const std::string& rootJointType,
			      const std::string& package,
			      const std::string& modelName,
			      const std::string& urdfSuffix,
			      const std::string& srdfSuffix);
      void loadHumanoidModel (const HumanoidRobotPtr_t& robot,
			      const std::string& rootJointType,
			      const std::string& package,
			      const std::string& modelName,
			      const std::string& urdfSuffix,
			      const std::string& srdfSuffix);

      /// Load only urdf model file
      ///
      /// \param robot Empty robot created before calling the function.
      ///        Users can pass an instance of a class deriving from Device.
      /// \param baseJoint joint to which the joint tree is added.
      /// \param prefix string to insert before all names
      ///               (joint, link, body names)
      /// \param rootJointType type of root joint among "anchor", "freeflyer",
      /// "planar",
      /// \param package ros package containing the model
      /// \param filename name of the file containing the model.

      /// \note This function reads the following file:
      /// \li
      /// package://${package}/urdf/${filename}.urdf
      void loadUrdfModel (const DevicePtr_t& robot,
                          const JointPtr_t&  baseJoint,
                          const std::string& prefix,
			  const std::string& rootJointType,
			  const std::string& package,
			  const std::string& filename);
      void loadUrdfModel (const DevicePtr_t& robot,
			  const std::string& rootJointType,
			  const std::string& package,
			  const std::string& filename);

    } // end of namespace urdf.
  } // end of namespace pinocchio.
} // end of namespace hpp.

#endif // HPP_PINOCCHIO_URDF_PARSER
