//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH
#define HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH

# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    /// Extra degrees of freedom to store internal values in configurations
    ///
    /// In some applications, it is useful to store extra variables
    /// with the configuration vector of a robot. For instance, when
    /// planning motions in state space using roadmap based methods,
    /// the velocity of the robot is stored in the nodes of the
    /// roadmap.
    class ExtraConfigSpace
    {
    public:
      ExtraConfigSpace () : dimension_ (0), lowerBounds_ (), upperBounds_ ()
      {
	lowerBounds_.resize (0);
	upperBounds_.resize (0);
      }
      value_type& lower (const size_type& index)
      {
	return lowerBounds_ [index];
      }
      value_type& upper (const size_type& index)
      {
	return upperBounds_ [index];
      }
      const value_type& lower (const size_type& index) const
      {
	return lowerBounds_ [index];
      }
      const value_type& upper (const size_type& index) const
      {
	return upperBounds_ [index];
      }
      const vector_t& lower () const
      {
        return lowerBounds_;
      }
      const vector_t& upper () const
      {
        return upperBounds_;
      }
      /// Get dimension
      size_type dimension () const
      {
	return dimension_;
      }	
    private:
      /// Set dimension of extra configuration space
      ///
      /// resize lowerBounds and upperBounds, set bounds to -infinity, +infinity
      void setDimension (const size_type& dimension)
      {
	dimension_ = dimension;
	lowerBounds_.resize (dimension);
	upperBounds_.resize (dimension);
	lowerBounds_.setConstant (-std::numeric_limits<value_type>::infinity());
	upperBounds_.setConstant (+std::numeric_limits<value_type>::infinity());
      }
      size_type dimension_;
      vector_t lowerBounds_;
      vector_t upperBounds_;
      friend class Device;
    }; // class ExtraConfigSpace
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_EXTRA_CONFIG_SPACE_HH
