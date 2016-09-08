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

#include <hpp/pinocchio/hpp-model/model-loader.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>

/* Default path of the urdf file describing the robot to parse. */
const std::string urdfDefaultFilename =
       ROMEO_MODEL_DIR "/romeo_description/urdf/romeo_small.urdf";
const std::string srdfDefaultFilename =
       ROMEO_MODEL_DIR "/romeo_description/srdf/romeo_small.srdf";

/* Build a hpp::pinocchio::Device from urdf path. */
hpp::pinocchio::DevicePtr_t hppPinocchio( bool withGeoms, bool withSrdf,
    const std::string urdfFilename, const std::string srdfFilename)
{
  assert (!withSrdf || (withSrdf && withGeoms));
  hpp::pinocchio::DevicePtr_t pinocchio = hpp::pinocchio::Device::create(urdfFilename);
  se3::urdf::buildModel(urdfFilename,se3::JointModelFreeFlyer(),pinocchio->model());
  pinocchio->createData();

  if( withGeoms )
    {
      std::vector<std::string> baseDirs; baseDirs.push_back(ROMEO_MODEL_DIR);
      se3::urdf::buildGeom(pinocchio->model(),pinocchio->name(),se3::COLLISION,pinocchio->geomModel(),baseDirs);
      pinocchio->geomModel().addAllCollisionPairs();

      if (withSrdf)
        se3::srdf::removeCollisionPairsFromSrdf
          (pinocchio->model(), pinocchio->geomModel(), srdfFilename, false);

      pinocchio->createGeomData();
    }

  return pinocchio;
}

