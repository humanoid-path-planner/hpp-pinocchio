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
    class LiegroupElementBase
    {
    public:
      /// Constructor
      /// \param value vector representation,
      /// \param liegroupSpace space the element belongs to.
      template <typename Derived>
      LiegroupElementBase (const Eigen::EigenBase<Derived>& value,
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
      explicit LiegroupElementBase (const Eigen::EigenBase<Derived>& value) :
        value_ (value.derived()),
        space_ (LiegroupSpace::create (value.derived().size ())) {}

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
      vector_type value_;
      LiegroupSpacePtr_t space_;
    };

    typedef LiegroupElementBase<vectorIn_t> LiegroupConstElementRef;

    template <typename vector_type>
    class LiegroupNonconstElementBase : public LiegroupElementBase<vector_type>
    {
      public:
        typedef LiegroupElementBase<vector_type> Base;

        /// Constructor
        /// \param value vector representation,
        /// \param liegroupSpace space the element belongs to.
        LiegroupNonconstElementBase (const vector_type& value,
                                     const LiegroupSpacePtr_t& space)
          : Base (value, space) {}

        /// Constructor
        /// \param value vector representation,
        ///
        /// By default the space containing the value is a vector space.
        LiegroupNonconstElementBase (const LiegroupSpacePtr_t& space) :
            Base (vector_t (space->nq ()), space) {}

        /// Constructor
        /// \param value vector representation,
        ///
        /// By default the space containing the value is a vector space.
        explicit LiegroupNonconstElementBase (const vector_type& value)
          : Base (value) {}

        /// Copy constructor
        template <typename vector_type2>
        LiegroupNonconstElementBase (const LiegroupElementBase<vector_type2>& other)
          : Base (other.vector(), other.space()) {}

        /// Constructor of trivial element
        LiegroupNonconstElementBase () : Base (vector_t(), LiegroupSpace::empty ()) {}

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
        LiegroupNonconstElementBase& operator+= (vectorIn_t v);
    };

    /// Element of a Lie group
    ///
    /// See class LiegroupSpace.
    typedef LiegroupNonconstElementBase<   vector_t> LiegroupElement;
    typedef LiegroupNonconstElementBase<vectorOut_t> LiegroupElementRef;

    /// Integration of a velocity vector from a configuration
    ///
    /// \param e element of the Lie group,
    /// \param v element of the tangent space of the Lie group.
    /// \return the element of the Lie group resulting from the integration
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type>
    LiegroupElement operator+ (const LiegroupElementBase<vector_type>& e, vectorIn_t v);

    /// Difference between two configurations
    ///
    /// \param e1, e2 elements of the Lie group,
    /// \return the velocity that integrated from e2 yiels e1
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type1, typename vector_type2>
    vector_t operator- (const LiegroupElementBase<vector_type1>& e1, const LiegroupElementBase<vector_type2>& e2);
    /// \}

    /// Compute the log as a tangent vector of a Lie group element
    template <typename vector_type>
    vector_t log (const LiegroupElementBase<vector_type>& lge);

    template <typename vector_type>
    inline std::ostream& operator<< (std::ostream& os, const LiegroupElementBase<vector_type>& e)
    {
      os << "Lie group element in " << *(e.space ())
         << " represented by vector (" << e. vector ().transpose () << ")";
      return os;
    }
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
