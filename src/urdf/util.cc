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

#include <hpp/pinocchio/urdf/util.hh>

#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>

namespace hpp {
  namespace pinocchio {
    namespace urdf {
      namespace {
#ifdef HPP_DEBUG
        const bool verbose = false;
#else
        const bool verbose = true;
#endif

        JointPtr_t findSpecialJoint (const HumanoidRobotPtr_t& robot, const std::string& linkName)
        {
          return robot->getJointByBodyName (linkName);
        }

        void setSpecialJoints (const HumanoidRobotPtr_t& robot) {
          try {
            robot->waist (findSpecialJoint (robot, "root_joint"));
          } catch (const std::exception&) {
            hppDout (notice, "No waist joint found");
          }
          try {
            robot->chest (findSpecialJoint (robot, "chest"));
          } catch (const std::exception&) {
            hppDout (notice, "No chest joint found");
          }
          try {
            robot->leftWrist (findSpecialJoint (robot, "l_wrist"));
          } catch (const std::exception&) {
            hppDout (notice, "No left wrist joint found");
          }
          try {
            robot->rightWrist (findSpecialJoint (robot, "r_wrist"));
          } catch (const std::exception&) {
            hppDout (notice, "No right wrist joint found");
          }
          try {
            robot->leftAnkle (findSpecialJoint (robot, "l_ankle"));
          } catch (const std::exception&) {
            hppDout (notice, "No left ankle joint found");
          }
          try {
            robot->rightAnkle (findSpecialJoint (robot, "r_ankle"));
          } catch (const std::exception&) {
            hppDout (notice, "No right ankle joint found");
          }
          try {
            robot->gazeJoint (findSpecialJoint (robot, "gaze"));
          } catch (const std::exception&) {
            hppDout (notice, "No gaze joint found");
          }
        }

        void fillGaze (const HumanoidRobotPtr_t robot)
        {
          vector3_t dir, origin;
          // Gaze direction is defined by the gaze joint local
          // orientation.
          dir[0] = 1;
          dir[1] = 0;
          dir[2] = 0;
          // Gaze position should is defined by the gaze joint local
          // origin.
          origin[0] = 0;
          origin[1] = 0;
          origin[2] = 0;
          robot->gaze (dir, origin);
        }

        se3::JointModelVariant buildJoint (const std::string& type)
        {
          if (type == "freeflyer")   return se3::JointModelFreeFlyer();
          else if (type == "planar") return se3::JointModelPlanar();
          else                       throw  std::invalid_argument
            ("Root joint type is currently not available.");
        }
      }

      void loadRobotModel (const DevicePtr_t& robot,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
                           const std::string& srdfSuffix)
      {
        loadUrdfModel (robot, rootJointType, package, modelName + urdfSuffix);
        loadSrdfModel (robot, package, modelName + srdfSuffix);
      }

      void loadRobotModel (const DevicePtr_t& robot,
                           const JointPtr_t&  baseJoint,
			   const std::string& prefix,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
                           const std::string& srdfSuffix)
      {
        assert ("Not implemented yet because the prefix/baseJoint not yet available in Pinocchio.");
      }

      void loadHumanoidModel (const HumanoidRobotPtr_t& robot,
			      const std::string& rootJointType,
			      const std::string& package,
			      const std::string& modelName,
			      const std::string& urdfSuffix,
			      const std::string& srdfSuffix)
      {
        loadRobotModel(robot, rootJointType, package, modelName, urdfSuffix, srdfSuffix);

	// Look for special joints and attach them to the model.
	setSpecialJoints (robot);
	// Fill gaze position and direction.
	fillGaze (robot);

      }

      void loadHumanoidModel (const HumanoidRobotPtr_t& robot,
                              const JointPtr_t&  baseJoint,
			      const std::string& prefix,
			      const std::string& rootJointType,
			      const std::string& package,
			      const std::string& modelName,
			      const std::string& urdfSuffix,
			      const std::string& srdfSuffix)
      {
        assert ("Not implemented yet because the prefix/baseJoint not yet available in Pinocchio.");
      }

      void loadUrdfModel (const DevicePtr_t& robot,
			  const std::string& rootType,
			  const std::string& package,
			  const std::string& filename)
      {
	std::string urdfPath = "package://" + package + "/urdf/"
	  + filename + ".urdf";
        std::vector<std::string> baseDirs = se3::rosPaths();

        std::string urdfName = se3::retrieveResourcePath(urdfPath, baseDirs);
        if (rootType == "anchor")
          se3::urdf::buildModel(urdfName, robot->model(), verbose);
        else
          se3::urdf::buildModel(urdfName, buildJoint(rootType), robot->model(), verbose);
        robot->createData();

        se3::urdf::buildGeom(robot->model(), urdfName, se3::COLLISION,
            robot->geomModel(), baseDirs);
        // TODO the collision pairs are reinitialized while they should not be.
        // We should keep the information from the previous GeometryModel.
        robot->geomModel().addAllCollisionPairs();
        robot->createGeomData();
	hppDout (notice, "Finished parsing URDF file.");
      }

      void loadUrdfModel (const DevicePtr_t& robot,
                          const JointPtr_t&  baseJoint,
			  const std::string& prefix,
			  const std::string& rootJointType,
			  const std::string& package,
			  const std::string& filename)
      {
        assert ("Not implemented yet because the prefix/baseJoint not yet available in Pinocchio.");
      }

      void loadSrdfModel (const DevicePtr_t& robot,
			  const std::string& package,
			  const std::string& filename)
      {
	std::string srdfPath = "package://" + package + "/srdf/"
	  + filename + ".srdf";
        std::vector<std::string> baseDirs = se3::rosPaths();

        const std::size_t nColPair = robot->geomModel().collisionPairs.size();
        std::string srdfName = se3::retrieveResourcePath(srdfPath, baseDirs);
        se3::srdf::removeCollisionPairsFromSrdf
          (robot->model(), robot->geomModel(), srdfName, verbose);
	hppDout (notice, "Finished parsing SRDF file. Removed "
            << nColPair - robot->geomModel().collisionPairs.size() <<
            " collision pairs.");
      }
    } // end of namespace urdf.
  } // end of namespace pinocchio.
} // end of namespace  hpp.
