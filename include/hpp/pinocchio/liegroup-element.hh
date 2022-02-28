// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
# define HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH

# include <hpp/pinocchio/liegroup-space.hh>
# include <hpp/pinocchio/deprecated.hh>

namespace hpp {
  namespace pinocchio {
    /// \addtogroup liegroup
    /// \{

    /// Const reference to a \ref LiegroupElement
    ///
    /// \sa LiegroupSpace, LiegroupElementConstRef
    template <typename vector_type>
    class LiegroupElementConstBase
    {
    public:
      /// Constructor
      /// \param value vector representation,
      /// \param liegroupSpace space the element belongs to.
      template <typename Derived>
      LiegroupElementConstBase (const Eigen::EigenBase<Derived>& value,
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
      explicit LiegroupElementConstBase (const Eigen::EigenBase<Derived>& value) :
        value_ (value.derived()),
        space_ (LiegroupSpace::create (value.derived().size ())) {}

      /// Constructor to allow *casting* LiegroupElement and LiegroupElementRef
      /// into a LiegroupElementConstRef
      template <typename vector_type2>
      LiegroupElementConstBase (const LiegroupElementConstBase<vector_type2>& other):
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

      /// Equality operator
      template <typename vector_type2>
      bool operator==(const LiegroupElementConstBase<vector_type2>& other)
      {
	return ((*space_ == *(other.space_)) && (value_ == other.value_));
      }

      /// Equality operator
      template <typename vector_type2>
      bool operator!=(const LiegroupElementConstBase<vector_type2>& other)
      {
	return ((*space_ != *(other.space_)) || (value_ != other.value_));
      }

    protected:
      template <typename Derived>
      LiegroupElementConstBase (const Eigen::EigenBase<Derived>& value,
          const LiegroupSpacePtr_t& space,
          void* /*to enable this constructor*/):
        value_ (const_cast<Derived&>(value.derived())),
        space_ (space)
      {}

      vector_type value_;
      LiegroupSpacePtr_t space_;

      template <typename vector_type2> friend class LiegroupElementConstBase;
      template <typename vector_type2> friend class LiegroupElementBase;
    };

    /// Writable element of a Lie group
    ///
    /// \sa LiegroupSpace, LiegroupElement, LiegroupElementRef
    template <typename vector_type>
    class LiegroupElementBase : public LiegroupElementConstBase<vector_type>
    {
      public:
        typedef LiegroupElementConstBase<vector_type> Base;

        /// Constructor
        /// \param value vector representation,
        /// \param space space the element belongs to.
        LiegroupElementBase (const vector_type& value,
                             const LiegroupSpacePtr_t& space) :
          Base (value, space, NULL) {}

        /// Constructor
        /// \param space space the element belongs to.
        LiegroupElementBase (const LiegroupSpacePtr_t& space) :
          Base (vector_t (space->nq ()), space) {}

        /// Constructor
        /// \param value vector representation,
        ///
        /// By default the space containing the value is a vector space.
        explicit LiegroupElementBase (const vector_type& value) :
          Base (value, LiegroupSpace::create (value.size ()), NULL) {}

        /// Copy constructor. Handles the following case:
        /// \li LiegroupElement    <- LiegroupElementConstRef
        template <typename vector_type2>
        LiegroupElementBase (const LiegroupElementConstBase<vector_type2>& other):
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

        /// Assignment from another LiegroupElement
        template <typename vector_type2>
        LiegroupElementBase& operator= (const LiegroupElementConstBase<vector_type2>& other)
        {
          this->space_ = other.space();
          this->value_ = other.vector();
          return *this;
        }

        /// Assignment from a vector
        template <typename Vector>
        LiegroupElementBase& operator= (const Eigen::MatrixBase<Vector>& v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector);
          assert (this->space_->nq() == v.derived().size());
          this->value_.noalias() = v.derived();
          return *this;
        }
    };

    /// Integration of a velocity vector from a configuration
    ///
    /// \param e element of the Lie group,
    /// \param v element of the tangent space of the Lie group.
    /// \return the element of the Lie group resulting from the integration
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type>
    LiegroupElement operator+ (const LiegroupElementConstBase<vector_type>& e, vectorIn_t v);

    /// Difference between two configurations
    ///
    /// \param e1, e2 elements of the Lie group,
    /// \return the velocity that integrated from e2 yiels e1
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    template <typename vector_type1, typename vector_type2>
    vector_t operator- (const LiegroupElementConstBase<vector_type1>& e1, const LiegroupElementConstBase<vector_type2>& e2);
    /// \}

    /// Compute the log as a tangent vector of a Lie group element
    template <typename vector_type>
    vector_t log (const LiegroupElementConstBase<vector_type>& lge);

    template <typename vector_type>
    inline std::ostream& operator<< (std::ostream& os, const LiegroupElementConstBase<vector_type>& e)
    {
      os << "Lie group element in " << *(e.space ())
         << " represented by vector (" << e. vector ().transpose () << ")";
      return os;
    }
  } // namespace pinocchio
} // namespace hpp

HPP_SERIALIZABLE_SPLIT_FREE(hpp::pinocchio::LiegroupElement)

#endif // HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
