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

#include <hpp/model/urdf/util.hh>
#include <pinocchio/parsers/urdf.hpp>

#include <hpp/model/device.hh>
#include <hpp/pinocchio/device.hh>


namespace hpp {
  namespace model {
    DevicePtr_t
    robotFromUrdf( const std::string filename, const std::string rootType="freeflyer" )
    {
      // Create the robot object.
      DevicePtr_t robot  = hpp::model::Device::create(filename);
      // Build robot model from URDF.
      urdf::Parser urdfParser (rootType, robot);
      urdfParser.parse (std::string("file://")+filename);
      hppDout (notice, "Finished parsing URDF file.");
      return robot;
    }

    DevicePtr_t
    robotFromUrdfSrdf( const std::string urdf, const std::string srdf, const std::string rootType="freeflyer" )
    {
      // Create the robot object.
      DevicePtr_t robot  = hpp::model::Device::create(urdf);
      // Build robot model from URDF.
      urdf::Parser urdfParser (rootType, robot);
      urdfParser.parse (std::string("file://")+urdf);
      hppDout (notice, "Finished parsing URDF file.");
      srdf::Parser srdfParser (&urdfParser);
      srdfParser.parse (std::string("file://")+srdf, robot);
      hppDout (notice, "Finished parsing SRDF file.");
      return robot;
    }
  } // namespace hpp
} // namespace model

/* Default path of the urdf file describing the robot to parse. */
extern const std::string urdfDefaultFilename;
extern const std::string srdfDefaultFilename;

/* Build a hpp::model::Device from urdf path. */  

hpp::model::DevicePtr_t hppModel( bool withSrdf = false, const std::string urdfFilename = urdfDefaultFilename , const std::string srdfFilename = srdfDefaultFilename)
{ 
  if (withSrdf) return hpp::model::robotFromUrdfSrdf(urdfFilename, srdfFilename);
  else          return hpp::model::robotFromUrdf    (urdfFilename);
}

/* Build a hpp::pinocchio::Device from urdf path. */
hpp::pinocchio::DevicePtr_t hppPinocchio( bool withGeoms = false,
                                          bool withSrdf = false,
                                          const std::string urdfFilename = urdfDefaultFilename,
                                          const std::string srdfFilename = srdfDefaultFilename);
