// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_LIEGROUP_SPACE_HH
# define HPP_PINOCCHIO_LIEGROUP_SPACE_HH

# include <vector>
# include <boost/variant.hpp>
# include <pinocchio/multibody/liegroup/special-euclidean.hpp>
# include <pinocchio/multibody/liegroup/special-orthogonal.hpp>
# include <pinocchio/multibody/liegroup/vector-space.hpp>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    /// \addtogroup liegroup
    /// \{

    /// Elementary Lie groups
    typedef boost::variant <se3::VectorSpaceOperation <Eigen::Dynamic>,
                            se3::VectorSpaceOperation <1>,
                            se3::VectorSpaceOperation <2>,
                            se3::VectorSpaceOperation <3>,
                            se3::SpecialEuclideanOperation<2>,
                            se3::SpecialEuclideanOperation<3>,
                            se3::SpecialOrthogonalOperation <2>,
                            se3::SpecialOrthogonalOperation <3> > LiegroupType;

    /// Cartesian product of elementary Lie groups
    ///
    /// Some values produced and manipulated by functions belong to Lie groups
    /// For instance rotations, rigid-body motions are element of Lie groups.
    ///
    /// Elements of Lie groups are usually applied common operations, like
    /// \li integrating a velocity from a given element during unit time,
    /// \li computing the constant velocity that moves from one element to
    ///     another one in unit time.
    ///
    /// By analogy with vector spaces that are a particular type of Lie group,
    /// the above operations are implemented as operators + and - respectively
    /// acting on LiegroupElement instances.
    ///
    /// This class represents a Lie group as the cartesian product of elementaty
    /// Lie groups. Those elementary Lie groups are gathered in a variant called
    /// LiegroupType.
    ///
    /// Elements of a Lie group are represented by class LiegroupElement.
    class LiegroupSpace
    {
    public:
      friend LiegroupSpace operator* (const LiegroupSpace& sp1,
                                      const LiegroupSpace& sp2);
      /// \name Elementary Lie groups
      /// \{

      /// Return \f$\mathbf{R}^n\f$ as a Lie group
      /// \param n dimension of vector space
      static LiegroupSpace Rn (const size_type& n)
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back
          (se3::VectorSpaceOperation <Eigen::Dynamic> ((int)n));
        S.nq_ = S.nv_ = n;
        S.neutral_.resize (S.nq_); S.neutral_.setZero ();
        return S;
      }

      /// Return \f$\mathbf{R}\f$ as a Lie group
      static LiegroupSpace R1 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::VectorSpaceOperation <1> ());
        S.nq_ = S.nv_ = 1;
        S.neutral_.resize (S.nq_); S.neutral_.setZero ();
        return S;
      }

      /// Return \f$\mathbf{R}^2\f$ as a Lie group
      static LiegroupSpace R2 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::VectorSpaceOperation <2> ());
        S.nq_ = S.nv_ = 2;
        S.neutral_.resize (S.nq_); S.neutral_.setZero ();
        return S;
      }

      /// Return \f$\mathbf{R}^3\f$ as a Lie group
      static LiegroupSpace R3 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::VectorSpaceOperation <3> ());
        S.nq_ = S.nv_ = 3;
        S.neutral_.resize (S.nq_); S.neutral_.setZero ();
        return S;
      }

      /// Return \f$SE(2)\f$
      static LiegroupSpace SE2 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::SpecialEuclideanOperation<2> ());
        S.nq_ = se3::SpecialEuclideanOperation<2>::NQ;
        S.nv_ = se3::SpecialEuclideanOperation<2>::NV;
        S.neutral_.resize (S.nq_); S.neutral_.setZero (); S.neutral_ [2] = 1;
        return S;
      }

      /// Return \f$SE(3)\f$
      static LiegroupSpace SE3 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::SpecialEuclideanOperation<3> ());
        S.nq_ = se3::SpecialEuclideanOperation<3>::NQ;
        S.nv_ = se3::SpecialEuclideanOperation<3>::NV;
        S.neutral_.resize (S.nq_); S.neutral_.setZero (); S.neutral_ [6] = 1;
        return S;
      }

      /// Return \f$SO(2)\f$
      static LiegroupSpace SO2 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::SpecialOrthogonalOperation<2> ());
        S.nq_ = se3::SpecialOrthogonalOperation<2>::NQ;
        S.nv_ = se3::SpecialOrthogonalOperation<2>::NV;
        S.neutral_.resize (S.nq_); S.neutral_.setZero (); S.neutral_ [0] = 1;
        return S;
      }

      /// Return \f$SO(3)\f$
      static LiegroupSpace SO3 ()
      {
        LiegroupSpace S;
        S.liegroupTypes_.push_back (se3::SpecialOrthogonalOperation<3> ());
        S.nq_ = se3::SpecialOrthogonalOperation<3>::NQ;
        S.nv_ = se3::SpecialOrthogonalOperation<3>::NV;
        S.neutral_.resize (S.nq_); S.neutral_.setZero (); S.neutral_ [3] = 1;
        return S;
      }

      /// Return empty Lie group
      static LiegroupSpace empty ()
      {
        LiegroupSpace S;
        S.nq_ = S.nv_ = 0;
        S.neutral_.resize (S.nq_);
        return S;
      }

      /// \}

      /// Constructor of vector space of given size
      LiegroupSpace (const size_type& size) : nq_ (size), nv_ (size),
                                              neutral_ (size)
      {
        nq_ = nv_ = size;
        liegroupTypes_.push_back
          (se3::VectorSpaceOperation <Eigen::Dynamic> ((int) nq_));
        neutral_.setZero ();
      }
      LiegroupSpace (const LiegroupSpace& other) :
        liegroupTypes_ (other.liegroupTypes_), nq_ (other.nq_), nv_ (other.nv_),
        neutral_ (other.neutral_)
      {
      }
      /// Dimension of the vector representation
      size_type nq () const
      {
        return nq_;
      }
      /// Dimension of the Lie group tangent space
      size_type nv () const
      {
        return nv_;
      }

      /// Get reference to vector of elementary types
      const std::vector <LiegroupType>& liegroupTypes () const
      {
        return liegroupTypes_;
      }

      /// Return an element
      ///
      /// Vector representation is allocated but not initialized
      LiegroupElement element () const;

      /// Return the neutral element as a vector
      vector_t neutral () const;
    private:
      /// Private constructor
      ///
      /// dimensions not initialized.
      LiegroupSpace () : liegroupTypes_ (), neutral_ ()
      {
      }
      std::vector <LiegroupType> liegroupTypes_;
      /// Size of vector representation and of Lie group tangent space
      size_type nq_, nv_;
      /// Neutral element of the Lie group
      vector_t neutral_;
    }; // class LiegroupSpace

    /// Cartesian product between Lie groups
    LiegroupSpace operator*
    (const LiegroupSpace& sp1, const LiegroupSpace& sp2);
    /// \}
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_SPACE_HH
