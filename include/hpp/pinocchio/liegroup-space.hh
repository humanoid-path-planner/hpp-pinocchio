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
# include <string>
# include <boost/variant.hpp>
# include <pinocchio/multibody/liegroup/special-euclidean.hpp>
# include <pinocchio/multibody/liegroup/special-orthogonal.hpp>
# include <pinocchio/multibody/liegroup/vector-space.hpp>
# include <hpp/pinocchio/liegroup.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    /// \addtogroup liegroup
    /// \{

    /// Elementary Lie groups
    typedef boost::variant <liegroup::VectorSpaceOperation
                            <Eigen::Dynamic, false>,
                            liegroup::VectorSpaceOperation <1, true>,
                            liegroup::VectorSpaceOperation <1, false>,
                            liegroup::VectorSpaceOperation <2, false>,
                            liegroup::VectorSpaceOperation <3, false>,
                            liegroup::VectorSpaceOperation <3, true>,
                            liegroup::CartesianProductOperation<
                              liegroup::VectorSpaceOperation<3, false>,
                              liegroup::SpecialOrthogonalOperation<3> >,
                            liegroup::CartesianProductOperation<
                              liegroup::VectorSpaceOperation<2, false>,
                              liegroup::SpecialOrthogonalOperation<2> >,
                            liegroup::SpecialOrthogonalOperation <2>,
                            liegroup::SpecialOrthogonalOperation <3>,
                            se3::SpecialEuclideanOperation <2>,
                            se3::SpecialEuclideanOperation <3> >
    LiegroupType;

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
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \name Elementary Lie groups
      /// \{

      /// Return \f$\mathbf{R}^n\f$ as a Lie group
      /// \param n dimension of vector space
      static LiegroupSpacePtr_t Rn (const size_type& n);
      /// Return \f$\mathbf{R}\f$ as a Lie group
      static LiegroupSpacePtr_t R1 ();
      /// Return \f$\mathbf{R}^2\f$ as a Lie group
      static LiegroupSpacePtr_t R2 ();
      /// Return \f$\mathbf{R}^3\f$ as a Lie group
      static LiegroupSpacePtr_t R3 ();
      /// Return \f$SE(2)\f$
      static LiegroupSpacePtr_t SE2 ();
      /// Return \f$SE(3)\f$
      static LiegroupSpacePtr_t SE3 ();
      /// Return \f$SO(2)\f$
      static LiegroupSpacePtr_t R2xSO2 ();
      /// Return \f$SO(3)\f$
      static LiegroupSpacePtr_t R3xSO3 ();
      /// Return empty Lie group
      static LiegroupSpacePtr_t empty ();
      /// \}

      /// Create instance of empty space
      static LiegroupSpacePtr_t create ()
      {
        LiegroupSpace* ptr (new LiegroupSpace ());
        LiegroupSpacePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create instance of vector space of given size
      static LiegroupSpacePtr_t create (const size_type& size)
      {
        LiegroupSpace* ptr (new LiegroupSpace (size));
        LiegroupSpacePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create copy
      static LiegroupSpacePtr_t createCopy
      (const LiegroupSpaceConstPtr_t& other)
      {
        LiegroupSpace* ptr (new LiegroupSpace (*other));
        LiegroupSpacePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create instance with one Elementary Lie group
      static LiegroupSpacePtr_t create (const LiegroupType& type)
      {
        LiegroupSpace* ptr (new LiegroupSpace (type));
        LiegroupSpacePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
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
      /// Dimension of elementary Liegroup at given rank
      size_type nq (const std::size_t& rank) const
      {
        return nqs_ [rank];
      }
      /// Dimension of elementary Liegroup tangent space at given rank
      size_type nv (const std::size_t& rank) const
      {
        return nvs_ [rank];
      }

      /// Get reference to vector of elementary types
      const std::vector <LiegroupType>& liegroupTypes () const
      {
        return liegroupTypes_;
      }

      /// Return the neutral element as a vector
      LiegroupElement neutral () const;

      /// Return exponential of a tangent vector
      LiegroupElement exp (vectorIn_t v) const;

      /// Compute the Jacobian of the integration operation.
      /// Given \f$ y = x + v \f$,
      ///
      /// \param[in] J the Jacobian of x
      /// \param[out] J the Jacobian of y
      void Jintegrate (vectorIn_t v, matrixOut_t J) const;

      /// Return name of Lie group
      std::string name () const;

      void mergeVectorSpaces ();

      bool operator== (const LiegroupSpace& other) const;
      bool operator!= (const LiegroupSpace& other) const;

      LiegroupSpacePtr_t operator*= (const LiegroupSpaceConstPtr_t& other);

    protected:

      /// Constructor of empty space
      LiegroupSpace ();
      /// Constructor of vector space of given size
      LiegroupSpace (const size_type& size);
      LiegroupSpace (const LiegroupSpace& other);
      LiegroupSpace (const LiegroupType& type);

    private:
      /// Initialize weak pointer to itself
      void init (const LiegroupSpaceWkPtr_t weak);
      /// Compute size of space
      void computeSize ();
      /// Compute neutral element as a vector
      void computeNeutral ();
      typedef std::vector <LiegroupType> LiegroupTypes;
      LiegroupTypes liegroupTypes_;
      /// Size of vector representation and of Lie group tangent space
      size_type nq_, nv_;
      /// Sizes of elementary Lie group
      std::vector <size_type> nqs_, nvs_;
      /// Neutral element of the Lie group
      vector_t neutral_;
      /// weak pointer to itself
      LiegroupSpaceWkPtr_t weak_;
    }; // class LiegroupSpace
    /// Writing in a stream
    inline std::ostream& operator<< (std::ostream& os,
                                     const LiegroupSpace& space)
    {
      os << space.name (); return os;
    }

    /// \}
  } // namespace pinocchio
} // namespace hpp

namespace boost {
  /// \addtogroup liegroup
  /// \{

  /// Cartesian product between Lie groups
  hpp::pinocchio::LiegroupSpacePtr_t operator* (
      const hpp::pinocchio::LiegroupSpaceConstPtr_t& sp1,
      const hpp::pinocchio::LiegroupSpaceConstPtr_t& sp2);
  /// \}
} // namespace boost

#endif // HPP_PINOCCHIO_LIEGROUP_SPACE_HH
