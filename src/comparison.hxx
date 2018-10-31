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

namespace hpp {
  namespace pinocchio {
    namespace liegroup {

      namespace level1 {
        IsEqualVisitor::IsEqualVisitor (const LiegroupType& lg2) : lg2_ (lg2)
        {
        }

        template <typename LgT1> void IsEqualVisitor::operator ()
          (const LgT1& lg1)
        {
          hppDout (info,
                   "level1::isEqual::operator ()");
          result = isEqual (lg1, lg2_);
        }

        template <typename LgT1> bool isEqual
        (const LgT1& lgt1, const LiegroupType& lgt2)
        {
          hppDout (info, "level1::isEqual");
          level2::IsEqualVisitor<LgT1> visitor (lgt1);
          boost::apply_visitor (visitor, lgt2);
          return visitor.result;
        }
      } // namespace level1

      namespace level2 {

        template <typename LgT1>
        IsEqualVisitor <LgT1>::IsEqualVisitor (const LgT1& lg1) :
        lg1_ (lg1)
        {
        }

        template <typename LgT1>
        template <typename LgT2>
        void IsEqualVisitor <LgT1>::operator () (const LgT2& lg2)
        {
          hppDout (info, "level2::isEqual::operator ()");
          Comparison <LgT1, LgT2> comp;
          result = comp (lg1_, lg2);
        }
      } // namespace level2

      // Specialization for vector spaces
      template <bool rot>
      bool Comparison <VectorSpaceOperation <Eigen::Dynamic, rot>,
                       VectorSpaceOperation <Eigen::Dynamic, rot> >::operator ()
        (const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt1,
         const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt2)
      {
        hppDout (info, "Comparison::operator () for vector spaces");
        return (lgt1.nq () == lgt2.nq ());
      }

      template <int Size, bool rot>
      bool Comparison <VectorSpaceOperation <Eigen::Dynamic, rot>,
                       VectorSpaceOperation <Size, rot> >::operator ()
        (const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt1,
         const VectorSpaceOperation <Size, rot>& lgt2)
      {
        hppDout (info, "Comparison::operator () for vector spaces");
        return (lgt1.nq () == lgt2.nq ());
      }

      template <int Size, bool rot>
      bool Comparison <VectorSpaceOperation <Size, rot>,
                       VectorSpaceOperation <Eigen::Dynamic, rot> >::operator ()
        (const VectorSpaceOperation <Size, rot>& lgt1,
         const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt2)
      {
        hppDout (info, "Comparison::operator () for vector spaces");
        return (lgt1.nq () == lgt2.nq ());
      }

      // Specialization for same types
      template <typename LgT>
      bool Comparison <LgT, LgT>::operator () (const LgT&, const LgT&)
      {
        hppDout (info, "Comparison::operator () for same types");
        return true;
      }

      template <typename LgT1, typename LgT2>
      bool Comparison <LgT1, LgT2>::operator ()
        (const LgT1&, const LgT2&)
      {
        hppDout (info, "Default Comparison::operator ()");
        return false;
      }

    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp
