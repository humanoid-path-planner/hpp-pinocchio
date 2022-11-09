// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_LIEGROUP_HH
#define HPP_PINOCCHIO_LIEGROUP_HH

#include <pinocchio/multibody/joint/fwd.hpp>
// #include <pinocchio/multibody/liegroup/liegroup.hpp>

#include <hpp/pinocchio/deprecated.hh>
#include <hpp/pinocchio/liegroup/cartesian-product.hh>
#include <hpp/pinocchio/liegroup/special-euclidean.hh>
#include <hpp/pinocchio/liegroup/special-orthogonal.hh>
#include <hpp/pinocchio/liegroup/vector-space.hh>

namespace hpp {
namespace pinocchio {
typedef ::pinocchio::JointModelCompositeTpl<value_type, 0, JointCollectionTpl>
    JointModelComposite;

/// This class maps at compile time a joint type to a lie group type.
///
/// JointModelPlanar (JointModelFreeFlyer resp.) maps to
/// \f$ R^2 \times SO(2) \f$ (resp. \f$ R^3 \times SO(3) \f$)
///
/// Unknown joint types map to an empty operation (compile time failure).
struct RnxSOnLieGroupMap {
  template <typename JointModel>
  struct operation {};
};
/// \deprecated Use RnxSOnLieGroupMap
HPP_PINOCCHIO_DEPRECATED typedef RnxSOnLieGroupMap LieGroupTpl;

/// This class maps at compile time a joint type to a lie group type.
///
/// JointModelPlanar (JointModelFreeFlyer resp.) maps to
/// \f$ SE(2) \f$ (resp. \f$ SE(3) \f$).
///
/// Unknown joint types map to an empty operation (compile time failure).
struct DefaultLieGroupMap {
  template <typename JointModel>
  struct operation {};
};

/// \cond
//---------------- RnxSOnLieGroupMap -------------------------------//
// JointModelRevolute, JointModelRevoluteUnbounded, JointModelRevoluteUnaligned,
// JointModelRevoluteUnboundedUnaligned
template <typename Scalar, int Options, int Axis>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteTpl<Scalar, Options, Axis> > {
  typedef liegroup::VectorSpaceOperation<1, true> type;
};
template <typename Scalar, int Options, int Axis>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, Axis> > {
  typedef liegroup::SpecialOrthogonalOperation<2> type;
};
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<1, true> type;
};
#if PINOCCHIO_VERSION_AT_LEAST(2, 1, 5)
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> > {
  typedef liegroup::SpecialOrthogonalOperation<2> type;
};
#endif

// JointModelPrismaticTpl, JointModelPrismaticUnaligned, JointModelTranslation
template <typename Scalar, int Options, int Axis>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelPrismaticTpl<Scalar, Options, Axis> > {
  typedef liegroup::VectorSpaceOperation<1, false> type;
};
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<1, false> type;
};
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelTranslationTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<3, false> type;
};

// JointModelSpherical, JointModelSphericalZYX,
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelSphericalTpl<Scalar, Options> > {
  typedef liegroup::SpecialOrthogonalOperation<3> type;
};
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelSphericalZYXTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<3, true> type;
};

// JointModelFreeFlyer, JointModelPlanar
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelFreeFlyerTpl<Scalar, Options> > {
  typedef liegroup::CartesianProductOperation<
      liegroup::VectorSpaceOperation<3, false>,
      liegroup::SpecialOrthogonalOperation<3> >
      type;
};
template <typename Scalar, int Options>
struct RnxSOnLieGroupMap::operation<
    ::pinocchio::JointModelPlanarTpl<Scalar, Options> > {
  typedef liegroup::CartesianProductOperation<
      liegroup::VectorSpaceOperation<2, false>,
      liegroup::SpecialOrthogonalOperation<2> >
      type;
};

//---------------- DefaultLieGroupMap ------------------------------------//

// JointModelRevolute, JointModelRevoluteUnbounded, JointModelRevoluteUnaligned,
// JointModelRevoluteUnboundedUnaligned
template <typename Scalar, int Options, int Axis>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteTpl<Scalar, Options, Axis> > {
  typedef liegroup::VectorSpaceOperation<1, true> type;
};
template <typename Scalar, int Options, int Axis>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar, Options, Axis> > {
  typedef liegroup::SpecialOrthogonalOperation<2> type;
};
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<1, true> type;
};
#if PINOCCHIO_VERSION_AT_LEAST(2, 1, 5)
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> > {
  typedef liegroup::SpecialOrthogonalOperation<2> type;
};
#endif

// JointModelPrismaticTpl, JointModelPrismaticUnaligned, JointModelTranslation
template <typename Scalar, int Options, int Axis>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelPrismaticTpl<Scalar, Options, Axis> > {
  typedef liegroup::VectorSpaceOperation<1, false> type;
};
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<1, false> type;
};
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelTranslationTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<3, false> type;
};

// JointModelSpherical, JointModelSphericalZYX,
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelSphericalTpl<Scalar, Options> > {
  typedef liegroup::SpecialOrthogonalOperation<3> type;
};
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelSphericalZYXTpl<Scalar, Options> > {
  typedef liegroup::VectorSpaceOperation<3, true> type;
};

// JointModelFreeFlyer, JointModelPlanar
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelFreeFlyerTpl<Scalar, Options> > {
  typedef liegroup::SpecialEuclideanOperation<3> type;
};
template <typename Scalar, int Options>
struct DefaultLieGroupMap::operation<
    ::pinocchio::JointModelPlanarTpl<Scalar, Options> > {
  typedef liegroup::SpecialEuclideanOperation<2> type;
};
/// \endcond
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_LIEGROUP_HH
