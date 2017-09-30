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

#ifndef HPP_PINOCCHIO_SRC_LOG_VISITOR_HH
# define HPP_PINOCCHIO_SRC_LOG_VISITOR_HH

# include <Eigen/Geometry>

namespace hpp {
  namespace pinocchio {
    namespace liegroupType {

      typedef Eigen::Quaternion <value_type> quaternion_t;

      /// Visitor to compute log of a LiegroupType instance
      struct LogVisitor : public boost::static_visitor <>
      {
        LogVisitor (vectorIn_t v, vectorOut_t log) : v_ (v), log (log)
        {
        }
        template <typename LiegroupType> void operator ()
        (const LiegroupType& lg);
        vectorIn_t v_;
        vectorOut_t log;
      }; // struct LogVisitor

      template <> inline void LogVisitor::operator ()
        <liegroup::CartesianProductOperation<
           liegroup::VectorSpaceOperation<3, false>,
           liegroup::SpecialOrthogonalOperation<3> > >
        (const liegroup::CartesianProductOperation<
         liegroup::VectorSpaceOperation<3, false>,
         liegroup::SpecialOrthogonalOperation<3> >&)
      {
        log.head <3> () = v_.head <3> ();
        quaternion_t q (v_.tail <4> ());
        Eigen::AngleAxis <value_type> u (q);
        if (u.angle() > M_PI) {
          log.tail <3> () = -(2*M_PI - u.angle()) * u.axis();
        }
        else {
          log.tail <3> () = u.axis() * u.angle();
        }
      }

      template <> inline void LogVisitor::operator ()
        <liegroup::CartesianProductOperation<
           liegroup::VectorSpaceOperation<2, false>,
        liegroup::SpecialOrthogonalOperation<2> > >
        (const liegroup::CartesianProductOperation<
         liegroup::VectorSpaceOperation<2, false>,
         liegroup::SpecialOrthogonalOperation<2> >&)
      {
        log.head <2> () = v_.head <2> ();
        value_type c = v_ [2], s = v_ [3];
        log [2] = atan2 (s, c);
      }

      template <> inline void LogVisitor::operator ()
        <liegroup::SpecialOrthogonalOperation <2> >
        (const liegroup::SpecialOrthogonalOperation <2>&)
      {
        value_type c = v_ [0], s = v_ [1];
        log [0] = atan2 (s, c);
      }

      template <> inline void LogVisitor::operator ()
        <liegroup::SpecialOrthogonalOperation <3> >
        (const liegroup::SpecialOrthogonalOperation <3>&)
      {
        vector4_t tmp (v_);
        quaternion_t q (tmp);
        Eigen::AngleAxis <value_type> u (q);
        if (u.angle() > M_PI) {
          log = -(2*M_PI - u.angle()) * u.axis();
        }
        else {
          log = u.axis() * u.angle();
        }
      }

      template <typename LiegroupType> void LogVisitor::operator ()
        (const LiegroupType&)
      {
        log = v_;
      } 
    } // namespace liegroupType
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SRC_LOG_VISITOR_HH
