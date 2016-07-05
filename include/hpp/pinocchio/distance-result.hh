//
// Copyright (c) 2016 CNRS
// Author: NMansard from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_DISTANCE_RESULT_HH
# define HPP_PINOCCHIO_DISTANCE_RESULT_HH

# include <hpp/fcl/collision_data.h>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    /// Result of distance computation between two CollisionObject.
    struct HPP_PINOCCHIO_DLLAPI DistanceResult {
      /// Get distance between objects
      const value_type& distance () const
      {
	return fcl.min_distance;
      }
      /// Get closest point on inner object in global frame,
      const fcl::Vec3f& closestPointInner () const
      {
	return fcl.nearest_points [0];
      }
      /// Get closest point on outer object in global frame,
      const fcl::Vec3f& closestPointOuter () const
      {
	return fcl.nearest_points [1];
      }
      fcl::DistanceResult fcl;
      CollisionObjectPtr_t innerObject;
      CollisionObjectPtr_t outerObject;
    }; // struct DistanceResult
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_DISTANCE_RESULT_HH
