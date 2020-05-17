// Copyright (c) 2020, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_LIEGROUP_SERIALIZATION_HH
#define HPP_PINOCCHIO_LIEGROUP_SERIALIZATION_HH

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>

#include <hpp/pinocchio/liegroup.hh>

namespace boost {
namespace serialization {

template<class Archive, int Size, bool rot>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot>& lg,
    const unsigned int version)
{
  (void) version;
  if (Size == Eigen::Dynamic) {
    int size = static_cast<int>(lg.nq());
    ar & make_nvp("size", size);
    if (!Archive::is_saving::value) {
      // TODO VectorSpaceOperation should provide a resize operation.
      lg = hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot>(size);
    }
  }
}

template<class Archive, typename LieGroup1, typename LieGroup2>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::CartesianProductOperation<LieGroup1, LieGroup2>& lg,
    const unsigned int version)
{
  (void) version;
#if PINOCCHIO_VERSION_AT_LEAST(2,4,5)
  ar & make_nvp("lg1", lg.lg1);
  ar & make_nvp("lg2", lg.lg2);
#else
  (void) ar; (void) lg;
  throw std::logic_error("Pinocchio version is too low for serializing "
      "hpp::pinocchio::liegroup::CartesianProductOperation");
#endif
}

template<class Archive, int N>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::SpecialOrthogonalOperation<N>& lg,
    const unsigned int version)
{
  (void) ar;
  (void) lg;
  (void) version;
}

template<class Archive, int N>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::SpecialEuclideanOperation<N>& lg,
    const unsigned int version)
{
  (void) ar;
  (void) lg;
  (void) version;
}
} // namespace serialization
} // namespace boost

#endif // HPP_PINOCCHIO_LIEGROUP_SERIALIZATION_HH
