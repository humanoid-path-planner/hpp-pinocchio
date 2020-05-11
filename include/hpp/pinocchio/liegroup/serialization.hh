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
inline void load(Archive & ar,
    hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot>& lg,
    const unsigned int version)
{
  (void) version;
  if (Size == Eigen::Dynamic) {
    ::pinocchio::Index size;
    ar & make_nvp("size", size);
    lg = hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot> (size);
  }
}

template<class Archive, int Size, bool rot>
inline void save(Archive & ar,
    const hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot>& lg,
    const unsigned int version)
{
  (void) version;
  if (Size == Eigen::Dynamic) {
    ::pinocchio::Index size (lg.nq());
    ar & make_nvp("size", size);
  }
}

template<class Archive, int Size, bool rot>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::VectorSpaceOperation<Size,rot>& lg,
    const unsigned int version)
{
  split_free(ar, lg, version);
}

template<class Archive, typename LieGroup1, typename LieGroup2>
inline void serialize(Archive & ar,
    hpp::pinocchio::liegroup::CartesianProductOperation<LieGroup1, LieGroup2>& lg,
    const unsigned int version)
{
  (void) version;
  ar & make_nvp("lg1_", lg.lg1_);
  ar & make_nvp("lg2_", lg.lg2_);
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
