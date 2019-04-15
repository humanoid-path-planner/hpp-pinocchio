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

#include <urdf_parser/urdf_parser.h>

#include <hpp/fcl/mesh_loader/loader.h>

#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>

namespace hpp {
  namespace pinocchio {
    namespace urdf {
      namespace {
#ifdef HPP_DEBUG
        const bool verbose = true;
#else
        const bool verbose = false;
#endif

        JointPtr_t findSpecialJoint (const HumanoidRobotPtr_t& robot, const std::string& linkName)
        {
          return robot->getJointByBodyName (linkName);
        }

        void setSpecialJoints (const HumanoidRobotPtr_t& robot, std::string prefix)
        {
          if (!prefix.empty() && *prefix.rbegin() != '/') prefix += "/";
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

        JointModelVariant buildJoint (const std::string& type)
        {
          if (type == "freeflyer")          return JointCollection::JointModelFreeFlyer();
          else if (type == "planar")        return JointCollection::JointModelPlanar();
          else if (type == "prismatic_x")   return JointCollection::JointModelPX();
          else if (type == "prismatic_y")   return JointCollection::JointModelPY();
          else if (type == "translation3d") return JointCollection::JointModelTranslation();
          else                          throw  std::invalid_argument
            ("Root joint type \"" + type + "\" is currently not available.");
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
            ::pinocchio::Frame& f = model.frames[i];
            f.name = prefix + f.name;
          }
          BOOST_FOREACH(::pinocchio::GeometryObject& go, geomModel.geometryObjects) {
            go.name = prefix + go.name;
          }
        }

        void setRootJointBounds(Model& model,
            const JointIndex& rtIdx,
            const std::string& rootType)
        {
          value_type b = std::numeric_limits<value_type>::infinity();
          if (rootType == "freeflyer") {
            const std::size_t idx = model.joints[rtIdx].idx_q();
            model.upperPositionLimit.segment<3>(idx).setConstant(+b);
            model.lowerPositionLimit.segment<3>(idx).setConstant(-b);
            // Quaternion bounds
            b = 1.01;
            const size_type quat_idx = idx + 3;
            model.upperPositionLimit.segment<4>(quat_idx).setConstant(+b);
            model.lowerPositionLimit.segment<4>(quat_idx).setConstant(-b);
          } else if (rootType == "planar") {
            const std::size_t idx = model.joints[rtIdx].idx_q();
            model.upperPositionLimit.segment<2>(idx).setConstant(+b);
            model.lowerPositionLimit.segment<2>(idx).setConstant(-b);
            // Unit complex bounds
            b = 1.01;
            const size_type cplx_idx = idx + 2;
            model.upperPositionLimit.segment<2>(cplx_idx).setConstant(+b);
            model.lowerPositionLimit.segment<2>(cplx_idx).setConstant(-b);
          }
        }

        std::string makeModelPath (const std::string& package,
                                   const std::string& type,
                                   const std::string& modelName,
                                   const std::string& suffix = "")
        {
          std::stringstream ss;
          ss << "package://" << package << "/" << type << "/" << modelName << suffix << "." << type;
          return ss.str();
        }

        template <bool XmlString>
        void _removeCollisionPairs (
            const Model& model,
            GeomModel& geomModel,
            const std::string& srdf,
            bool verbose)
        {
          ::pinocchio::srdf::removeCollisionPairsFromXML
            (model, geomModel, srdf, verbose);
        }

        template <>
        void _removeCollisionPairs<false> (
            const Model& model,
            GeomModel& geomModel,
            const std::string& srdf,
            bool verbose)
        {
          ::pinocchio::srdf::removeCollisionPairs
            (model, geomModel, srdf, verbose);
        }

        template <bool srdfAsXmlString>
        void _loadModel (const DevicePtr_t& robot,
                        const FrameIndex&  baseFrame,
                        std::string prefix,
                        const std::string& rootType,
                        const ::urdf::ModelInterfaceSharedPtr urdfTree,
                        const std::istream& urdfStream,
                        const std::string& srdf)
        {
          if (!urdfTree)
            throw std::invalid_argument ("Failed to parse URDF. Use check_urdf command to know what's wrong.");

          ModelPtr_t model = (baseFrame==0 ? robot->modelPtr() : ModelPtr_t(new Model));
          const JointIndex idFirstJoint = model->joints.size();
          const FrameIndex idFirstFrame = model->frames.size();
          if (rootType == "anchor")
            ::pinocchio::urdf::buildModel(urdfTree, *model, verbose);
          else
            ::pinocchio::urdf::buildModel(urdfTree, buildJoint(rootType), *model, verbose);

          hppDout (notice, "Finished parsing URDF file.");

          GeomModel geomModel;

          std::vector<std::string> baseDirs = ::pinocchio::rosPaths();
          static fcl::MeshLoaderPtr loader (new fcl::CachedMeshLoader (fcl::BV_OBB));
          ::pinocchio::urdf::buildGeom(*model, urdfStream, ::pinocchio::COLLISION, geomModel, baseDirs, loader);
          geomModel.addAllCollisionPairs();

          if (!srdf.empty()) {
            _removeCollisionPairs<srdfAsXmlString>
              (*model, geomModel, srdf, verbose);
            if(!srdfAsXmlString)
              ::pinocchio::srdf::loadReferenceConfigurations(*model,srdf,verbose);
            else{
              hppDout(warning,"Neutral configuration won't be extracted from SRDF string.");
              //TODO : A method getNeutralConfigurationFromSrdfString must be added in Pinocchio,
              // similarly to removeCollisionPairsFromSrdf / removeCollisionPairsFromSrdfString
            }
          }

          if (!prefix.empty()) {
            if (*prefix.rbegin() != '/') prefix += "/";
            setPrefix(prefix, *model, geomModel, idFirstJoint, idFirstFrame);
          }

          // Update root joint bounds
          assert((rootType == "anchor")
              || (model->names[idFirstJoint] == prefix + "root_joint"));
          setRootJointBounds(*model, idFirstJoint, rootType);

          if (baseFrame == 0)
            ::pinocchio::appendGeometryModel(robot->geomModel(), geomModel);
          else {
            ModelPtr_t m (new Model);
            GeomModelPtr_t gm (new GeomModel);
            ::pinocchio::appendModel(robot->model(), *model,
                robot->geomModel(), geomModel,
                baseFrame, Transform3f::Identity(),
                *m, *gm);
          }

          robot->createData();
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
        loadModel (robot, 0, "", rootJointType,
            makeModelPath(package, "urdf", modelName, urdfSuffix),
            makeModelPath(package, "srdf", modelName, srdfSuffix));
      }

      void loadRobotModel (const DevicePtr_t& robot,
                           const FrameIndex&  baseFrame,
			   const std::string& prefix,
			   const std::string& rootJointType,
			   const std::string& package,
			   const std::string& modelName,
			   const std::string& urdfSuffix,
                           const std::string& srdfSuffix)
      {
        loadModel (robot, baseFrame,
            (prefix.empty() ? "" : prefix + "/"),
            rootJointType,
            makeModelPath(package, "urdf", modelName, urdfSuffix),
            makeModelPath(package, "srdf", modelName, srdfSuffix));
      }

      void setupHumanoidRobot (const HumanoidRobotPtr_t& robot,
          const std::string& prefix)
      {
	// Look for special joints and attach them to the model.
	setSpecialJoints (robot, prefix);
	// Fill gaze position and direction.
	fillGaze (robot);
      }

      void loadUrdfModel (const DevicePtr_t& robot,
                          const FrameIndex&  baseFrame,
                          const std::string& prefix,
			  const std::string& rootJointType,
			  const std::string& package,
			  const std::string& filename)
      {
        loadModel (robot, baseFrame,
            (prefix.empty() ? "" : prefix + "/"),
            rootJointType,
            makeModelPath(package, "urdf", filename), "");
      }

      void loadUrdfModel (const DevicePtr_t& robot,
			  const std::string& rootType,
			  const std::string& package,
			  const std::string& filename)
      {
        loadModel (robot, 0, "", rootType,
            makeModelPath(package, "urdf", filename), "");
      }

        void loadModel (const DevicePtr_t& robot,
                        const FrameIndex&  baseFrame,
                        const std::string& prefix,
                        const std::string& rootType,
                        const std::string& urdfPath,
                        const std::string& srdfPath)
        {
          std::vector<std::string> baseDirs = ::pinocchio::rosPaths();

          std::string urdfFileName = ::pinocchio::retrieveResourcePath(urdfPath, baseDirs);
          if (urdfFileName == "") {
            throw std::invalid_argument (std::string ("Unable to retrieve ") +
                urdfPath);
          }

          std::string srdfFileName;
          if (!srdfPath.empty()) {
            srdfFileName = ::pinocchio::retrieveResourcePath(srdfPath, baseDirs);
            if (srdfFileName == "") {
              throw std::invalid_argument (std::string ("Unable to retrieve ") +
                  srdfPath);
            }
          }

          ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFileName);
          std::ifstream urdfStream (urdfFileName.c_str());
          _loadModel <false> (robot, baseFrame, prefix, rootType,
              urdfTree, urdfStream, srdfFileName);
        }

        void loadModelFromString (const DevicePtr_t& robot,
                                  const FrameIndex&  baseFrame,
                                  const std::string& prefix,
                                  const std::string& rootType,
                                  const std::string& urdfString,
                                  const std::string& srdfString)
        {
          ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(urdfString);
          std::istringstream urdfStream (urdfString);
          _loadModel <true> (robot, baseFrame, prefix, rootType,
              urdfTree, urdfStream, srdfString);
        }
    } // end of namespace urdf.
  } // end of namespace pinocchio.
} // end of namespace  hpp.
