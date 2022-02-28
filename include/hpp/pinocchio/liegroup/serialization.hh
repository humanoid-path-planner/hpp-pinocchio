// Copyright (c) 2020, Joseph Mirabel
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
