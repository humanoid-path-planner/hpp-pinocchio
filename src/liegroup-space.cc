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

#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/liegroup-space.hh>

namespace hpp {
  namespace pinocchio {
    LiegroupElement LiegroupSpace::element () const
    {
      vector_t value (nq_);
      return LiegroupElement (value, *this);
    }

    vector_t LiegroupSpace::neutral () const
    {
      return neutral_;
    }

    LiegroupSpace operator*
    (const LiegroupSpace& sp1, const LiegroupSpace& sp2)
    {
      LiegroupSpace res (sp1);
      res.liegroupTypes_.insert (res.liegroupTypes_.end (),
                                 sp2.liegroupTypes_.begin (),
                                 sp2.liegroupTypes_.end ());
      res.nq_ = sp1.nq_ + sp2.nq_;
      res.nv_ = sp1.nv_ + sp2.nv_;
      res.neutral_.resize (res.nq_);
      res.neutral_.head (sp1.nq ()) = sp1.neutral ();
      res.neutral_.tail (sp2.nq ()) = sp2.neutral ();
      return res;
    }

  } // namespace pinocchio
} // namespace hpp
