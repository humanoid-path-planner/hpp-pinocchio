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
# include <pinocchio/fwd.hpp>
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

#ifdef HPP_PINOCCHIO_PARSED_BY_DOXYGEN
    /// Elementary Lie groups
    /// A boost variant with the following classes:
    /// \li \f$\mathbf{R}^n\f$, where \f$n\f$ is either 1, 2, 3 or dynamic,
    /// \li \f$\mathbf{R}^n \times SO(n) \f$, where \f$n\f$ is either 2 or 3,
    /// \li \f$SO(n) \f$, where \f$n\f$ is either 2 or 3,
    /// \li \f$SE(n) \f$, where \f$n\f$ is either 2 or 3.
    /// \sa hpp::pinocchio::liegroup::VectorSpaceOperation,
    ///     hpp::pinocchio::liegroup::CartesianProductOperation,
    ///     hpp::pinocchio::liegroup::SpecialOrthogonalOperation,
    ///     hpp::pinocchio::liegroup::SpecialEuclideanOperation,
    typedef ABoostVariant LiegroupType;
#else
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
                            liegroup::SpecialEuclideanOperation <2>,
                            liegroup::SpecialEuclideanOperation <3> >
    LiegroupType;
#endif

    enum DerivativeProduct {
      DerivativeTimesInput,
      InputTimesDerivative
    };

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
      static LiegroupSpacePtr_t SO2 ();
      /// Return \f$SO(3)\f$
      static LiegroupSpacePtr_t SO3 ();
      /// Return \f$\mathbf{R}^2 \times SO(2)\f$
      static LiegroupSpacePtr_t R2xSO2 ();
      /// Return \f$\mathbf{R}^3 \times SO(3)\f$
      static LiegroupSpacePtr_t R3xSO3 ();
      /// Return empty Lie group
      static LiegroupSpacePtr_t empty ();
      /// \}

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
      size_type nq (const std::size_t& rank) const;
      /// Dimension of elementary Liegroup tangent space at given rank
      size_type nv (const std::size_t& rank) const;

      /// Get reference to vector of elementary types
      const std::vector <LiegroupType>& liegroupTypes () const
      {
        return liegroupTypes_;
      }

      /// Return the neutral element as a vector
      LiegroupElement neutral () const;

      /// Create a LiegroupElement from a configuration.
      LiegroupElement element (vectorIn_t q) const;

      /// Create a LiegroupElementRef from a configuration.
      LiegroupElementRef elementRef (vectorOut_t q) const;

      /// Create a LiegroupElementRef from a configuration.
      LiegroupElementConstRef elementConstRef (vectorIn_t q) const;

      /// Return exponential of a tangent vector
      LiegroupElement exp (vectorIn_t v) const;

      /// Compute the Jacobian of the integration operation with respect to q.
      ///
      /// Given \f$ \mathbf{p} = \mathbf{q} + \mathbf{v} \f$,
      /// compute \f$J_{\mathbf{q}}\f$ such that
      ///
      /// \f{equation}
      /// \dot{\mathbf{p}} = J_{\mathbf{q}}\dot{\mathbf{q}}
      /// \f}
      /// for constant \f$\mathbf{v}\f$
      ///
      /// \param q the configuration,
      /// \param v the velocity vector,
      /// \retval Jq the Jacobian (initialized as identity)
      ///
      /// \note For each elementary Lie group in q.space (), ranging
      ///       over indices \f$[iq, iq+nq-1]\f$, the Jacobian
      ///       \f$J_{Lg} (q [iq:iq+nq])\f$ is computed by method
      ///       ::pinocchio::LieGroupBase::dIntegrate_dq.
      /// lines \f$[iq:iq+nq]\f$ of Jq are then left multiplied by
      /// \f$J_{Lg} (q [iq:iq+nq])\f$.
      template <DerivativeProduct side>
      void dIntegrate_dq (LiegroupElementConstRef q, vectorIn_t v, matrixOut_t Jq) const;

      /// Compute the Jacobian of the integration operation with respect to v.
      ///
      /// Given \f$ \mathbf{p} = \mathbf{q} + \mathbf{v} \f$,
      /// compute \f$J_{\mathbf{v}}\f$ such that
      ///
      /// \f{equation}
      /// \dot{\mathbf{p}} = J_{\mathbf{v}}\dot{\mathbf{v}}
      /// \f}
      /// for constant \f$\mathbf{q}\f$
      ///
      /// \param q the configuration,
      /// \param v the velocity vector,
      /// \retval Jv the Jacobian (initialized to identity)
      /// \note For each elementary Lie group in q.space (), ranging
      ///       over indices \f$[iv, iv+nv-1]\f$, the Jacobian
      ///       \f$J_{Lg} (q [iv:iv+nv])\f$ is computed by method
      ///       ::pinocchio::LieGroupBase::dIntegrate_dq.
      /// lines \f$[iv:iv+nv]\f$ of Jv are then left multiplied by
      /// \f$J_{Lg} (q [iv:iv+nv])\f$.
      template <DerivativeProduct side>
      void dIntegrate_dv (LiegroupElementConstRef q, vectorIn_t v, matrixOut_t Jv) const;

      /// \deprecated Use dDifference_dq0 and dDifference_dq1
      template <bool ApplyOnTheLeft>
      void Jdifference (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0, matrixOut_t J1) const;

      /// Compute the Jacobian matrices of the difference operation.
      /// Given \f$ \mathbf{v} = \mathbf{q}_1 - \mathbf{q}_0 \f$,
      ///
      /// Compute matrices \f$J_{0}\f$ and \f$J_{1}\f$ such that
      /// \f{equation}
      /// \dot{\mathbf{v}} = J_{0}\dot{\mathbf{q}_0} + J_{1}\dot{\mathbf{q}_1}
      /// \f}
      /// \param[in] q0,q1 Lie group elements,
      /// \param[out] J0 the Jacobian of v with respect to q0.
      template <DerivativeProduct side>
      void dDifference_dq0 (vectorIn_t q0, vectorIn_t q1, matrixOut_t J0) const;

      /// Compute the Jacobian matrices of the difference operation.
      /// Given \f$ \mathbf{v} = \mathbf{q}_1 - \mathbf{q}_0 \f$,
      ///
      /// Compute matrices \f$J_{0}\f$ and \f$J_{1}\f$ such that
      /// \f{equation}
      /// \dot{\mathbf{v}} = J_{0}\dot{\mathbf{q}_0} + J_{1}\dot{\mathbf{q}_1}
      /// \f}
      /// \param[in] q0,q1 Lie group elements,
      /// \param[out] J1 the Jacobian of v with respect to q1.
      template <DerivativeProduct side>
      void dDifference_dq1 (vectorIn_t q0, vectorIn_t q1, matrixOut_t J1) const;

      /// Return name of Lie group
      std::string name () const;

      void mergeVectorSpaces ();

      LiegroupSpacePtr_t vectorSpacesMerged () const;

      bool isVectorSpace () const;

      bool operator== (const LiegroupSpace& other) const;
      bool operator!= (const LiegroupSpace& other) const;

      LiegroupSpacePtr_t operator*= (const LiegroupSpaceConstPtr_t& other);

    protected:

      /// Constructor of vector space of given size
      LiegroupSpace (const size_type& size);
      LiegroupSpace (const LiegroupSpace& other);
      LiegroupSpace (const LiegroupType& type);

    private:
      /// Constructor of empty space
      LiegroupSpace ();
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
  /// Cartesian power by an integer
  hpp::pinocchio::LiegroupSpacePtr_t operator^
  (const hpp::pinocchio::LiegroupSpaceConstPtr_t& sp,
   hpp::pinocchio::size_type n);
  /// \}
} // namespace boost

#endif // HPP_PINOCCHIO_LIEGROUP_SPACE_HH
