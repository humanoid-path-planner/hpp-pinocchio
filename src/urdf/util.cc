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
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>

namespace hpp {
  namespace pinocchio {
    namespace urdf {
      using se3::FrameIndex;

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

        void setSpecialJoints (const HumanoidRobotPtr_t& robot, const std::string& prefix) {
          try {
            robot->waist (robot->getJointByName(prefix + "root_joint"));
          } catch (const std::exception&) {
            hppDout (notice, "No waist joint found");
          }
          try {
            robot->chest (findSpecialJoint (robot, prefix + "chest"));
          } catch (const std::exception&) {
            hppDout (notice, "No chest joint found");
          }
          try {
            robot->leftWrist (findSpecialJoint (robot, prefix + "l_wrist"));
          } catch (const std::exception&) {
            hppDout (notice, "No left wrist joint found");
          }
          try {
            robot->rightWrist (findSpecialJoint (robot, prefix + "r_wrist"));
          } catch (const std::exception&) {
            hppDout (notice, "No right wrist joint found");
          }
          try {
            robot->leftAnkle (findSpecialJoint (robot, prefix + "l_ankle"));
          } catch (const std::exception&) {
            hppDout (notice, "No left ankle joint found");
          }
          try {
            robot->rightAnkle (findSpecialJoint (robot, prefix + "r_ankle"));
          } catch (const std::exception&) {
            hppDout (notice, "No right ankle joint found");
          }
          try {
            robot->gazeJoint (findSpecialJoint (robot, prefix + "gaze"));
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

        void setPrefix (const std::string& prefix,
            Model& model, GeomModel& geomModel,
            const JointIndex& idFirstJoint,
            const FrameIndex& idFirstFrame)
        {
          for (JointIndex i = idFirstJoint; i < model.joints.size(); ++i) {
            model.names[i] = prefix + model.names[i];
          }
          for (FrameIndex i = idFirstFrame; i < model.frames.size(); ++i) {
            se3::Frame& f = model.frames[i];
            f.name = prefix + f.name;
          }
          BOOST_FOREACH(se3::GeometryObject& go, geomModel.geometryObjects) {
            go.name = prefix + go.name;
          }
        }

        template <bool LoadSRDF> void loadModel
          (const DevicePtr_t& robot,
           const JointIndex&  baseJoint,
           const std::string& prefix,
           const std::string& rootType,
           const std::string& package,
           const std::string& urdfName,
           const std::string& srdfName)
          {
            if (baseJoint != 0)
              throw std::invalid_argument ("Only appending robots at the world is supported.");
            std::vector<std::string> baseDirs = se3::rosPaths();

            std::string urdfPath =
              "package://" + package + "/urdf/" + urdfName + ".urdf";
            std::string urdfFileName = se3::retrieveResourcePath(urdfPath, baseDirs);

            Model& model = robot->model();
            const JointIndex idFirstJoint = model.joints.size();
            const FrameIndex idFirstFrame = model.frames.size();
            if (rootType == "anchor")
              se3::urdf::buildModel(urdfFileName, robot->model(), verbose);
            else
              se3::urdf::buildModel(urdfFileName, buildJoint(rootType), robot->model(), verbose);
            robot->createData();

            hppDout (notice, "Finished parsing URDF file.");

            GeomModel geomModel;

            se3::urdf::buildGeom(robot->model(), urdfFileName, se3::COLLISION, geomModel, baseDirs);
            geomModel.addAllCollisionPairs();

            if (LoadSRDF) {
              std::string srdfPath =
                "package://" + package + "/srdf/" + srdfName + ".srdf";
              std::string srdfFileName = se3::retrieveResourcePath(srdfPath, baseDirs);
              se3::srdf::removeCollisionPairsFromSrdf
                (robot->model(), geomModel, srdfFileName, verbose);
            }

            if (!prefix.empty()) setPrefix(prefix, robot->model(), geomModel, idFirstJoint, idFirstFrame);

            se3::appendGeometryModel(robot->geomModel(), geomModel);
            robot->createGeomData();

            hppDout (notice, "Finished parsing SRDF file.");
          }
      }

      void loadRobotModel (const DevicePtr_t& robot,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
                           const std::string& srdfSuffix)
      {
        loadModel <true> (robot, 0, "", rootJointType,
            package, modelName + urdfSuffix, modelName + srdfSuffix);
      }

      void loadRobotModel (const DevicePtr_t& robot,
                           const JointIndex&  baseJoint,
			   const std::string& prefix,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
                           const std::string& srdfSuffix)
      {
        loadModel <true> (robot, baseJoint, 
            (prefix.empty() ? "" : prefix + "/"),
            rootJointType,
            package, modelName + urdfSuffix, modelName + srdfSuffix);
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
	setSpecialJoints (robot, "");
	// Fill gaze position and direction.
	fillGaze (robot);
      }

      void loadHumanoidModel (const HumanoidRobotPtr_t& robot,
                              const JointIndex&  baseJoint,
			      const std::string& prefix,
			      const std::string& rootJointType,
			      const std::string& package,
			      const std::string& modelName,
			      const std::string& urdfSuffix,
			      const std::string& srdfSuffix)
      {
        loadRobotModel(robot, baseJoint, prefix, rootJointType, package, modelName, urdfSuffix, srdfSuffix);

	// Look for special joints and attach them to the model.
	setSpecialJoints (robot, prefix);
	// Fill gaze position and direction.
	fillGaze (robot);
      }

      void loadUrdfModel (const DevicePtr_t& robot,
                          const JointIndex&  baseJoint,
                          const std::string& prefix,
			  const std::string& rootJointType,
			  const std::string& package,
			  const std::string& filename)
      {
        loadModel<false> (robot, baseJoint,
            (prefix.empty() ? "" : prefix + "/"),
            rootJointType, package, filename, "");
      }

      void loadUrdfModel (const DevicePtr_t& robot,
			  const std::string& rootType,
			  const std::string& package,
			  const std::string& filename)
      {
        loadModel<false> (robot, 0, "", rootType, package, filename, "");
      }
    } // end of namespace urdf.
  } // end of namespace pinocchio.
} // end of namespace  hpp.
