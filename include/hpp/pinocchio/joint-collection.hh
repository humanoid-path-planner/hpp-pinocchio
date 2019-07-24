//
// Copyright (c) 2018 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_JOINT_COLLECTION_HH
#define HPP_PINOCCHIO_JOINT_COLLECTION_HH

#include <boost/variant.hpp>
#include <boost/variant/recursive_wrapper.hpp>

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/multibody/joint/joint-planar.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"
#if PINOCCHIO_VERSION_AT_LEAST(2,1,5)
# include "pinocchio/multibody/joint/joint-revolute-unbounded-unaligned.hpp"
#endif
#include "pinocchio/multibody/joint/joint-prismatic-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unbounded.hpp"
//#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"
//#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"

namespace hpp {
  namespace pinocchio {
    
    template<typename _Scalar, int _Options>
    struct JointCollectionTpl
    {
      typedef _Scalar Scalar;
      enum { Options = _Options };
      
      // Joint Revolute
      typedef ::pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
      typedef ::pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
      typedef ::pinocchio::JointModelRevoluteTpl<Scalar,Options,2> JointModelRZ;
      
      // Joint Revolute Unaligned
      typedef ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModelRevoluteUnaligned;
      
      // Joint Revolute UBounded
      typedef ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,0> JointModelRUBX;
      typedef ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,1> JointModelRUBY;
      typedef ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,2> JointModelRUBZ;

#if PINOCCHIO_VERSION_AT_LEAST(2,1,5)
      // Joint Revolute Unbounded Unaligned
      typedef ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModelRevoluteUnboundedUnaligned;
#endif

      // Joint Prismatic
      typedef ::pinocchio::JointModelPrismaticTpl<Scalar,Options,0> JointModelPX;
      typedef ::pinocchio::JointModelPrismaticTpl<Scalar,Options,1> JointModelPY;
      typedef ::pinocchio::JointModelPrismaticTpl<Scalar,Options,2> JointModelPZ;
      
      // Joint Prismatic Unaligned
      typedef ::pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModelPrismaticUnaligned;
      
      // Joint Spherical
      typedef ::pinocchio::JointModelSphericalTpl<Scalar,Options> JointModelSpherical;
      
      // Joint Spherical ZYX
      typedef ::pinocchio::JointModelSphericalZYXTpl<Scalar,Options> JointModelSphericalZYX;
      
      // Joint Translation
      typedef ::pinocchio::JointModelTranslationTpl<Scalar,Options> JointModelTranslation;
      
      // Joint FreeFlyer
      typedef ::pinocchio::JointModelFreeFlyerTpl<Scalar,Options> JointModelFreeFlyer;
      
      // Joint Planar
      typedef ::pinocchio::JointModelPlanarTpl<Scalar,Options> JointModelPlanar;
      
      // Joint Composite
      typedef ::pinocchio::JointModelCompositeTpl<Scalar,Options,pinocchio::JointCollectionTpl> JointModelComposite;
      
      typedef boost::variant<
  //    JointModelVoid,
      JointModelRX, JointModelRY, JointModelRZ
      , JointModelFreeFlyer, JointModelPlanar
      , JointModelRevoluteUnaligned
#if PINOCCHIO_VERSION_AT_LEAST(2,1,5)
      , JointModelRevoluteUnboundedUnaligned
#endif
      , JointModelPX, JointModelPY, JointModelPZ
      , JointModelPrismaticUnaligned
      , JointModelTranslation
      , JointModelRUBX, JointModelRUBY, JointModelRUBZ
      > JointModelVariant;
     
      
      // Joint Revolute
      typedef ::pinocchio::JointDataRevoluteTpl<Scalar,Options,0> JointDataRX;
      typedef ::pinocchio::JointDataRevoluteTpl<Scalar,Options,1> JointDataRY;
      typedef ::pinocchio::JointDataRevoluteTpl<Scalar,Options,2> JointDataRZ;
      
      // Joint Revolute Unaligned
      typedef ::pinocchio::JointDataRevoluteUnalignedTpl<Scalar,Options> JointDataRevoluteUnaligned;
      
      // Joint Revolute UBounded
      typedef ::pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,0> JointDataRUBX;
      typedef ::pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,1> JointDataRUBY;
      typedef ::pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,2> JointDataRUBZ;
    
#if PINOCCHIO_VERSION_AT_LEAST(2,1,5)
      // Joint Revolute Unbounded Unaligned
      typedef ::pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> JointDataRevoluteUnboundedUnaligned;
#endif
      
      // Joint Prismatic
      typedef ::pinocchio::JointDataPrismaticTpl<Scalar,Options,0> JointDataPX;
      typedef ::pinocchio::JointDataPrismaticTpl<Scalar,Options,1> JointDataPY;
      typedef ::pinocchio::JointDataPrismaticTpl<Scalar,Options,2> JointDataPZ;
      
      // Joint Prismatic Unaligned
      typedef ::pinocchio::JointDataPrismaticUnalignedTpl<Scalar,Options> JointDataPrismaticUnaligned;
      
      // Joint Spherical
      typedef ::pinocchio::JointDataSphericalTpl<Scalar,Options> JointDataSpherical;
      
      // Joint Spherical ZYX
      typedef ::pinocchio::JointDataSphericalZYXTpl<Scalar,Options> JointDataSphericalZYX;
      
      // Joint Translation
      typedef ::pinocchio::JointDataTranslationTpl<Scalar,Options> JointDataTranslation;
      
      // Joint FreeFlyer
      typedef ::pinocchio::JointDataFreeFlyerTpl<Scalar,Options> JointDataFreeFlyer;
      
      // Joint Planar
      typedef ::pinocchio::JointDataPlanarTpl<Scalar,Options> JointDataPlanar;
      
      // Joint Composite
      typedef ::pinocchio::JointDataCompositeTpl<Scalar,Options,pinocchio::JointCollectionTpl> JointDataComposite;
      
      typedef boost::variant<
  //    JointDataVoid
      JointDataRX, JointDataRY, JointDataRZ
      , JointDataFreeFlyer, JointDataPlanar
      , JointDataRevoluteUnaligned
#if PINOCCHIO_VERSION_AT_LEAST(2,1,5)
      , JointDataRevoluteUnboundedUnaligned
#endif
      , JointDataPX, JointDataPY, JointDataPZ
      , JointDataPrismaticUnaligned
      , JointDataTranslation
      , JointDataRUBX, JointDataRUBY, JointDataRUBZ
      > JointDataVariant;

    };
    
    typedef JointCollection::JointModelVariant JointModelVariant;
    typedef JointCollection::JointDataVariant JointDataVariant;
    
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_JOINT_COLLECTION_HH
