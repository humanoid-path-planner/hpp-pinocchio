// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/fcl/mesh_loader/loader.h>
#include <urdf_parser/urdf_parser.h>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/util/debug.hh>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>

namespace hpp {
namespace pinocchio {
namespace srdf {
void removeCollisionPairs(const Model& model, GeomModel& geom_model,
                          const std::string& prefix, std::istream& stream) {
  // Read xml stream
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(stream, pt);

  // Iterate over collision pairs
  auto last = geom_model.collisionPairs.end();
  for (const ptree::value_type& v : pt.get_child("robot")) {
    if (v.first == "disable_collisions") {
      const std::string link1 =
          prefix + v.second.get<std::string>("<xmlattr>.link1");
      const std::string link2 =
          prefix + v.second.get<std::string>("<xmlattr>.link2");

      // Check first if the two bodies exist in model
      if (!model.existBodyName(link1)) {
        hppDout(error, link1 << ", " << link2
                             << ". Link1 does not exist in model. Skip");
        continue;
      }
      if (!model.existBodyName(link2)) {
        hppDout(error, link1 << ", " << link2
                             << ". Link2 does not exist in model. Skip");
        continue;
      }

      const typename Model::FrameIndex frame_id1 = model.getBodyId(link1);
      const typename Model::FrameIndex frame_id2 = model.getBodyId(link2);

      // Malformed SRDF
      if (frame_id1 == frame_id2) {
        hppDout(info, "Cannot disable collision between " << link1 << " and "
                                                          << link2);
        continue;
      }

      auto nlast = std::remove_if(
          geom_model.collisionPairs.begin(), last, [&](const auto& colPair) {
            const auto &ga = geom_model.geometryObjects[colPair.first],
                       &gb = geom_model.geometryObjects[colPair.second];
            return ((ga.parentFrame == frame_id1) &&
                    (gb.parentFrame == frame_id2)) ||
                   ((gb.parentFrame == frame_id1) &&
                    (ga.parentFrame == frame_id2));
          });
      if (last != nlast) {
        hppDout(info,
                "Remove collision pair (" << link1 << "," << link2 << ")");
      }
      last = nlast;
    }
  }
  if (last != geom_model.collisionPairs.end()) {
    hppDout(info, "Removing "
                      << std::distance(last, geom_model.collisionPairs.end())
                      << " collision pairs.");
    geom_model.collisionPairs.erase(last, geom_model.collisionPairs.end());
  }
}

void removeCollisionPairs(const Model& model, GeomModel& geom_model,
                          const std::string& prefix,
                          const std::string& filename) {
  // Check extension
  const std::string extension = filename.substr(filename.find_last_of('.') + 1);
  if (extension != "srdf") {
    const std::string exception_message(filename +
                                        " does not have the right extension.");
    throw std::invalid_argument(exception_message);
  }

  // Open file
  std::ifstream srdf_stream(filename.c_str());
  if (!srdf_stream.is_open()) {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }

  removeCollisionPairs(model, geom_model, prefix, srdf_stream);
}

void removeCollisionPairsFromXML(const Model& model, GeomModel& geom_model,
                                 const std::string& prefix,
                                 const std::string& xmlString) {
  std::istringstream srdf_stream(xmlString);
  removeCollisionPairs(model, geom_model, prefix, srdf_stream);
}
}  // namespace srdf

namespace urdf {
namespace {
#ifdef HPP_DEBUG
const bool verbose = true;
#else
const bool verbose = false;
#endif

JointPtr_t findSpecialJoint(const HumanoidRobotPtr_t& robot,
                            const std::string& linkName) {
  return robot->getJointByBodyName(linkName);
}

void setSpecialJoints(const HumanoidRobotPtr_t& robot, std::string prefix) {
  if (!prefix.empty() && *prefix.rbegin() != '/') prefix += "/";
  try {
    robot->waist(robot->getJointByName(prefix + "root_joint"));
  } catch (const std::exception&) {
    hppDout(notice, "No waist joint found");
  }
  try {
    robot->chest(findSpecialJoint(robot, prefix + "chest"));
  } catch (const std::exception&) {
    hppDout(notice, "No chest joint found");
  }
  try {
    robot->leftWrist(findSpecialJoint(robot, prefix + "l_wrist"));
  } catch (const std::exception&) {
    hppDout(notice, "No left wrist joint found");
  }
  try {
    robot->rightWrist(findSpecialJoint(robot, prefix + "r_wrist"));
  } catch (const std::exception&) {
    hppDout(notice, "No right wrist joint found");
  }
  try {
    robot->leftAnkle(findSpecialJoint(robot, prefix + "l_ankle"));
  } catch (const std::exception&) {
    hppDout(notice, "No left ankle joint found");
  }
  try {
    robot->rightAnkle(findSpecialJoint(robot, prefix + "r_ankle"));
  } catch (const std::exception&) {
    hppDout(notice, "No right ankle joint found");
  }
  try {
    robot->gazeJoint(findSpecialJoint(robot, prefix + "gaze"));
  } catch (const std::exception&) {
    hppDout(notice, "No gaze joint found");
  }
}

void fillGaze(const HumanoidRobotPtr_t robot) {
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
  robot->gaze(dir, origin);
}

JointModelVariant buildJoint(const std::string& type) {
  if (type == "freeflyer")
    return JointCollection::JointModelFreeFlyer();
  else if (type == "planar")
    return JointCollection::JointModelPlanar();
  else if (type == "prismatic_x")
    return JointCollection::JointModelPX();
  else if (type == "prismatic_y")
    return JointCollection::JointModelPY();
  else if (type == "translation3d")
    return JointCollection::JointModelTranslation();
  else
    throw std::invalid_argument("Root joint type \"" + type +
                                "\" is currently not available.");
}

void setPrefix(const std::string& prefix, Model& model, GeomModel& geomModel,
               const JointIndex& idFirstJoint, const FrameIndex& idFirstFrame) {
  for (JointIndex i = idFirstJoint; i < model.joints.size(); ++i) {
    model.names[i] = prefix + model.names[i];
  }
  for (FrameIndex i = idFirstFrame; i < model.frames.size(); ++i) {
    ::pinocchio::Frame& f = model.frames[i];
    f.name = prefix + f.name;
  }
  for (::pinocchio::GeometryObject& go : geomModel.geometryObjects)
    go.name = prefix + go.name;
}

void setRootJointBounds(Model& model, const JointIndex& rtIdx,
                        const std::string& rootType) {
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

std::string makeModelPath(const std::string& package, const std::string& type,
                          const std::string& modelName,
                          const std::string& suffix = "") {
  std::stringstream ss;
  ss << "package://" << package << "/" << type << "/" << modelName << suffix
     << "." << type;
  return ss.str();
}

template <bool XmlString>
void _removeCollisionPairs(const Model& model, GeomModel& geomModel,
                           const std::string& prefix, const std::string& srdf) {
  hppDout(info, "parsing SRDF string:\n" << srdf);
  srdf::removeCollisionPairsFromXML(model, geomModel, prefix, srdf);
}

template <>
void _removeCollisionPairs<false>(const Model& model, GeomModel& geomModel,
                                  const std::string& prefix,
                                  const std::string& srdf) {
  hppDout(info, "parsing SRDF file: " << srdf);
  srdf::removeCollisionPairs(model, geomModel, prefix, srdf);
}

/// JointPtrType will be a boost or std shared_ptr depending on the
/// version of urdfdom.
template <typename JointPtrType>
void addMimicJoints(const std::map<std::string, JointPtrType>& joints,
                    const std::string& prefix, const DevicePtr_t& robot) {
  typedef std::map<std::string, JointPtrType> UrdfJointMap_t;
  for (typename UrdfJointMap_t::const_iterator _joint = joints.begin();
       _joint != joints.end(); ++_joint) {
    const JointPtrType& joint = _joint->second;
    if (joint && joint->type != ::urdf::Joint::FIXED && joint->mimic) {
      Device::JointLinearConstraint constraint;
      constraint.joint = robot->getJointByName(prefix + joint->name);
      constraint.reference =
          robot->getJointByName(prefix + joint->mimic->joint_name);
      constraint.multiplier = joint->mimic->multiplier;
      constraint.offset = joint->mimic->offset;
      robot->addJointConstraint(constraint);
    }
  }
}

template <bool srdfAsXmlString>
void _loadModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                const SE3& bMr, std::string prefix, const std::string& rootType,
                const ::urdf::ModelInterfaceSharedPtr urdfTree,
                const std::istream& urdfStream, const std::string& srdf) {
  if (!urdfTree)
    throw std::invalid_argument(
        "Failed to parse URDF. Use check_urdf command to know what's wrong.");

  ModelPtr_t model = ModelPtr_t(new Model);
  const JointIndex idFirstJoint = model->joints.size();
  const FrameIndex idFirstFrame = model->frames.size();
  if (rootType == "anchor")
    ::pinocchio::urdf::buildModel(urdfTree, *model, verbose);
  else
    ::pinocchio::urdf::buildModel(urdfTree, buildJoint(rootType), *model,
                                  verbose);

  hppDout(notice, "Finished parsing URDF file.");

  GeomModel geomModel;

  std::vector<std::string> baseDirs = ::pinocchio::rosPaths();
  static fcl::MeshLoaderPtr loader(new fcl::CachedMeshLoader(fcl::BV_OBBRSS));
  ::pinocchio::urdf::buildGeom(*model, urdfStream, ::pinocchio::COLLISION,
                               geomModel, baseDirs, loader);
  geomModel.addAllCollisionPairs();

  if (!srdf.empty()) {
    if (!srdfAsXmlString)
      ::pinocchio::srdf::loadReferenceConfigurations(*model, srdf, verbose);
    else {
      hppDout(warning,
              "Neutral configuration won't be extracted from SRDF string.");
      // TODO : A method getNeutralConfigurationFromSrdfString must be added in
      // Pinocchio,
      //  similarly to removeCollisionPairsFromSrdf /
      //  removeCollisionPairsFromSrdfString
    }
  }

  if (!prefix.empty()) {
    if (*prefix.rbegin() != '/') prefix += "/";
    setPrefix(prefix, *model, geomModel, idFirstJoint, idFirstFrame);
  }

  // Update root joint bounds
  assert((rootType == "anchor") ||
         (model->names[idFirstJoint] == prefix + "root_joint"));
  setRootJointBounds(*model, idFirstJoint, rootType);

  ModelPtr_t m(new Model);
  GeomModelPtr_t gm(new GeomModel);
  ::pinocchio::appendModel(robot->model(), *model, robot->geomModel(),
                           geomModel, baseFrame, bMr, *m, *gm);
  robot->setModel(m);
  robot->setGeomModel(gm);

  if (!srdf.empty()) {
    _removeCollisionPairs<srdfAsXmlString>(robot->model(), robot->geomModel(),
                                           prefix, srdf);
  }

  robot->createData();
  robot->createGeomData();

  // Build mimic joint table.
  addMimicJoints(urdfTree->joints_, prefix, robot);

  hppDout(notice, "Finished parsing SRDF file.");
}

}  // namespace

void loadRobotModel(const DevicePtr_t& robot, const std::string& rootJointType,
                    const std::string& package, const std::string& modelName,
                    const std::string& urdfSuffix,
                    const std::string& srdfSuffix) {
  loadModel(robot, 0, "", rootJointType,
            makeModelPath(package, "urdf", modelName, urdfSuffix),
            makeModelPath(package, "srdf", modelName, srdfSuffix));
}

void loadRobotModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                    const std::string& prefix, const std::string& rootJointType,
                    const std::string& package, const std::string& modelName,
                    const std::string& urdfSuffix,
                    const std::string& srdfSuffix, const SE3& bMr) {
  loadModel(robot, baseFrame, (prefix.empty() ? "" : prefix + "/"),
            rootJointType,
            makeModelPath(package, "urdf", modelName, urdfSuffix),
            makeModelPath(package, "srdf", modelName, srdfSuffix), bMr);
}

void setupHumanoidRobot(const HumanoidRobotPtr_t& robot,
                        const std::string& prefix) {
  // Look for special joints and attach them to the model.
  setSpecialJoints(robot, prefix);
  // Fill gaze position and direction.
  fillGaze(robot);
}

void loadUrdfModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                   const std::string& prefix, const std::string& rootJointType,
                   const std::string& package, const std::string& filename,
                   const SE3& bMr) {
  loadModel(robot, baseFrame, (prefix.empty() ? "" : prefix + "/"),
            rootJointType, makeModelPath(package, "urdf", filename), "", bMr);
}

void loadUrdfModel(const DevicePtr_t& robot, const std::string& rootType,
                   const std::string& package, const std::string& filename) {
  loadModel(robot, 0, "", rootType, makeModelPath(package, "urdf", filename),
            "");
}

void loadModel(const DevicePtr_t& robot, const FrameIndex& baseFrame,
               const std::string& prefix, const std::string& rootType,
               const std::string& urdfPath, const std::string& srdfPath,
               const SE3& bMr) {
  std::vector<std::string> baseDirs = ::pinocchio::rosPaths();
  baseDirs.push_back(EXAMPLE_ROBOT_DATA_MODEL_DIR);
  baseDirs.push_back(EXAMPLE_ROBOT_DATA_MODEL_DIR "/../..");

  std::string urdfFileName =
      ::pinocchio::retrieveResourcePath(urdfPath, baseDirs);
  if (urdfFileName == "") {
    throw std::invalid_argument(std::string("Unable to retrieve ") + urdfPath);
  }

  std::string srdfFileName;
  if (!srdfPath.empty()) {
    srdfFileName = ::pinocchio::retrieveResourcePath(srdfPath, baseDirs);
    if (srdfFileName == "") {
      throw std::invalid_argument(std::string("Unable to retrieve ") +
                                  srdfPath);
    }
  }

  ::urdf::ModelInterfaceSharedPtr urdfTree =
      ::urdf::parseURDFFile(urdfFileName);
  std::ifstream urdfStream(urdfFileName.c_str());
  _loadModel<false>(robot, baseFrame, bMr, prefix, rootType, urdfTree,
                    urdfStream, srdfFileName);
}

void loadModelFromString(const DevicePtr_t& robot, const FrameIndex& baseFrame,
                         const std::string& prefix, const std::string& rootType,
                         const std::string& urdfString,
                         const std::string& srdfString, const SE3& bMr) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(urdfString);
  std::istringstream urdfStream(urdfString);
  _loadModel<true>(robot, baseFrame, bMr, prefix, rootType, urdfTree,
                   urdfStream, srdfString);
}

void loadSRDFModel(const DevicePtr_t& robot, std::string prefix,
                   const std::string& srdfPath) {
  std::vector<std::string> baseDirs = ::pinocchio::rosPaths();
  std::string srdfFileName =
      ::pinocchio::retrieveResourcePath(srdfPath, baseDirs);
  if (srdfFileName == "") {
    throw std::invalid_argument(std::string("Unable to retrieve ") + srdfPath);
  }
  if (!prefix.empty() && *prefix.rbegin() != '/') prefix += "/";

  _removeCollisionPairs<false>(robot->model(), robot->geomModel(), prefix,
                               srdfFileName);
  ::pinocchio::srdf::loadReferenceConfigurations(robot->model(), srdfFileName,
                                                 verbose);
  robot->createGeomData();
}

void loadSRDFModelFromString(const DevicePtr_t& robot, std::string prefix,
                             const std::string& srdfString) {
  if (!prefix.empty() && *prefix.rbegin() != '/') prefix += "/";
  _removeCollisionPairs<true>(robot->model(), robot->geomModel(), prefix,
                              srdfString);
  robot->createGeomData();
}
}  // end of namespace urdf.
}  // end of namespace pinocchio.
}  // end of namespace  hpp.
