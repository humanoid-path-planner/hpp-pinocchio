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

# include <vector>
# include <boost/variant.hpp>
# include <pinocchio/multibody/liegroup/special-euclidean.hpp>
# include <pinocchio/multibody/liegroup/special-orthogonal.hpp>
# include <pinocchio/multibody/liegroup/vector-space.hpp>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    typedef boost::variant <se3::VectorSpaceOperation <Eigen::Dynamic>,
                            se3::VectorSpaceOperation <1>,
                            se3::VectorSpaceOperation <2>,
                            se3::VectorSpaceOperation <3>,
                            se3::SpecialEuclideanOperation<2>,
                            se3::SpecialEuclideanOperation<3>,
                            se3::SpecialOrthogonalOperation <2>,
                            se3::SpecialOrthogonalOperation <3> > LiegroupType;

    class LiegroupElement
    {
    public:
      friend LiegroupElement operator+
      (const LiegroupElement& e, const vector_t& v);
      friend vector_t operator-
      (const LiegroupElement& e1, const LiegroupElement& e2);

      typedef std::vector <LiegroupType> LiegroupTypes;
      /// Constructor
      /// \param value vector representation,
      /// \param liegroupTypes vector of types of Lie groups defining the
      ///        the space the value belongs to as a Cartesian product of Lie
      ///        groups.
      LiegroupElement (const vector_t& value,
                       const LiegroupTypes& liegroupTypes) :
        value_ (value), liegroupTypes_ (liegroupTypes)
      {
        computeSize ();
      }
      /// Constructor
      /// \param value vector representation,
      ///
      /// By default the space containing the value is a vector space.
      LiegroupElement (const vector_t& value) :
        value_ (value), liegroupTypes_ ()
      {
        nq_ = nv_ = value_.size ();
        liegroupTypes_.push_back
          (se3::VectorSpaceOperation <Eigen::Dynamic> ((int) nq_));
      }
      /// get reference to vector of Lie groups
      const LiegroupTypes& liegroupTypes () const
      {
        return liegroupTypes_;
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
      /// Vector representation
      const vector_t& value () const
      {
        return value_;
      }
    private:
      void computeSize ();
      vector_t value_;
      LiegroupTypes liegroupTypes_;
      // Size of vector representation and of Lie group tangent space
      size_type nq_, nv_;
    }; // class LiegroupElement

    /// Integration of a velocity vector from a configuration
    ///
    /// \param e element of the Lie group,
    /// \param v element of the tangent space of the Lie group.
    /// \return the element of the Lie group resulting from the integration
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    LiegroupElement operator+ (const LiegroupElement& e, const vector_t& v);

    /// Difference between two configurations
    ///
    /// \param e1, e2 elements of the Lie group,
    /// \return the velocity that integrated from e2 yiels e1
    ///
    /// By extension of the vector space case, we represent the integration
    /// of a constant velocity during unit time by an addition
    vector_t operator- (const LiegroupElement& e1, const LiegroupElement& e2);
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_LIEGROUP_ELEMENT_HH
