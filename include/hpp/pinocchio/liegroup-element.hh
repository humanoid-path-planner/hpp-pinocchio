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

#ifndef HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
# define HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH

# include <hpp/pinocchio/liegroup-space.hh>
# include <hpp/pinocchio/deprecated.hh>

namespace hpp {
  namespace pinocchio {
    /// \addtogroup liegroup
    /// \{

    template <typename vector_type>
    class LiegroupConstElementBase
    {
    public:
      /// Constructor
      /// \param value vector representation,
      /// \param liegroupSpace space the element belongs to.
      template <typename Derived>
      LiegroupConstElementBase (const Eigen::EigenBase<Derived>& value,
                           const LiegroupSpacePtr_t& liegroupSpace) :
        value_ (value.derived()), space_ (liegroupSpace)
      {
        check();
      }

      /// Constructor
      /// \param value vector representation,
      ///
      /// By default the space containing the value is a vector space.
      template <typename Derived>
      explicit LiegroupConstElementBase (const Eigen::EigenBase<Derived>& value) :
        value_ (value.derived()),
        space_ (LiegroupSpace::create (value.derived().size ())) {}

      /// Constructor to allow *casting* LiegroupElement and LiegroupElementRef
      /// into a LiegroupConstElementRef
      template <typename vector_type2>
      LiegroupConstElementBase (const LiegroupConstElementBase<vector_type2>& other):
        value_ (other.vector()),
        space_ (other.space ())
      {}

      /// get reference to vector of Lie groups
      const LiegroupSpacePtr_t& space () const
      {
        return space_;
      }

      /// Const vector representation
      const vector_type& vector () const
      {
        return value_;
      }

      /// Size of the vector representation
      size_type size () const
      {
        return value_.size ();
      }

      /// Check that size of vector fits size of space
      /// \note only in debug mode
      void check () const
      {
        assert (value_.size () == space_->nq ());
      }

    protected:
      template <typename Derived>
      LiegroupConstElementBase (const Eigen::EigenBase<Derived>& value,
          const LiegroupSpacePtr_t& space,
          void* /*to enable this constructor*/):
        value_ (const_cast<Derived&>(value.derived())),
        space_ (space)
      {}

      vector_type value_;
      LiegroupSpacePtr_t space_;

      template <typename vector_type2> friend class LiegroupConstElementBase;
      template <typename vector_type2> friend class LiegroupElementBase;
    };

    typedef LiegroupConstElementBase<vectorIn_t> LiegroupConstElementRef;

    template <typename vector_type>
    class LiegroupElementBase : public LiegroupConstElementBase<vector_type>
    {
      public:
        typedef LiegroupConstElementBase<vector_type> Base;

        /// Constructor
        /// \param value vector representation,
        /// \param liegroupSpace space the element belongs to.
        LiegroupElementBase (const vector_type& value,
                             const LiegroupSpacePtr_t& space) :
          Base (value, space, NULL) {}

        /// Constructor
        /// \param value vector representation,
        ///
        /// By default the space containing the value is a vector space.
        LiegroupElementBase (const LiegroupSpacePtr_t& space) :
          Base (vector_t (space->nq ()), space) {}

        /// Constructor
        /// \param value vector representation,
        ///
        /// By default the space containing the value is a vector space.
        explicit LiegroupElementBase (const vector_type& value) :
          Base (value, LiegroupSpace::create (value.size ()), NULL) {}

        /// Copy constructor. Handles the following case:
        /// \li LiegroupElement    <- LiegroupConstElementRef
        template <typename vector_type2>
        LiegroupElementBase (const LiegroupConstElementBase<vector_type2>& other):
          Base (other.value_, other.space()) {}

        /// Copy constructor. Handles the following cases:
        /// \li LiegroupElement    <- LiegroupElement
        /// \li LiegroupElementRef <- LiegroupElementRef
        /// \li LiegroupElement    <- LiegroupElementRef
        template <typename vector_type2>
        LiegroupElementBase (const LiegroupElementBase<vector_type2>& other):
          Base (other.value_, other.space()) {}

        /// *Casting* operator from LiegroupElement to LiegroupElementRef
        template <typename vector_type2>
        LiegroupElementBase (LiegroupElementBase<vector_type2>& other):
          Base (other.value_, other.space(), NULL) {}

        /// Constructor of trivial element
        LiegroupElementBase () : Base (vector_t(), LiegroupSpace::empty ()) {}

        /// Const vector representation
        const vector_type& vector () const
        {
          return Base::vector();
        }

        /// Modifiable vector representation
        vector_type& vector ()
        {
          return this->value_;
        }

        /// Set element to neutral element
        void setNeutral ()
        {
          this->value_ = this->space_->neutral ().vector ();
        }

        /// Inplace integration of a velocity vector
        LiegroupElementBase& operator+= (vectorIn_t v);
    };

    /// Element of a Lie group
    ///
    /// See class LiegroupSpace.
    typedef LiegroupElementBase<   vector_t> LiegroupElement;
    typedef LiegroupElementBase<vectorOut_t> LiegroupElementRef;

    /// Integration of a velocity vector from a configuration
    ///
    /// \param e element of the Lie group,
    /// \param v element of the tangent space of the Lie group.
    /// \return the element of the Lie group resulting from the integration
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type>
    LiegroupElement operator+ (const LiegroupConstElementBase<vector_type>& e, vectorIn_t v);

    /// Difference between two configurations
    ///
    /// \param e1, e2 elements of the Lie group,
    /// \return the velocity that integrated from e2 yiels e1
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type1, typename vector_type2>
    vector_t operator- (const LiegroupConstElementBase<vector_type1>& e1, const LiegroupConstElementBase<vector_type2>& e2);
    /// \}

    /// Compute the log as a tangent vector of a Lie group element
    template <typename vector_type>
    vector_t log (const LiegroupConstElementBase<vector_type>& lge);

    template <typename vector_type>
    inline std::ostream& operator<< (std::ostream& os, const LiegroupConstElementBase<vector_type>& e)
    {
      os << "Lie group element in " << *(e.space ())
         << " represented by vector (" << e. vector ().transpose () << ")";
      return os;
    }
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
