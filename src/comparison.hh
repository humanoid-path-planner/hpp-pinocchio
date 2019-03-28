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

#ifndef HPP_PINOCCHIO_SRC_COMPARISON_HH
# define HPP_PINOCCHIO_SRC_COMPARISON_HH

# include <hpp/util/debug.hh>

namespace hpp {
  namespace pinocchio {
    namespace liegroup {

      namespace level1 {
        template <typename LgT1> bool isEqual
        (const LgT1& lgt1, const LiegroupType& lgt2);

        struct IsEqualVisitor : public boost::static_visitor <bool>
        {
          inline IsEqualVisitor (const LiegroupType& lg2);
          template <typename LgT1> inline bool operator () (const LgT1& lg1);
        private:
          const LiegroupType& lg2_;
        }; // struct IsEqualVisitor
      } // namespace level1

      namespace level2 {
        template <typename LgT1>
        struct IsEqualVisitor : public boost::static_visitor <bool>
        {
          inline IsEqualVisitor (const LgT1& lg1);
          template <typename LgT2> inline bool operator () (const LgT2& lg2);
        private:
          const LgT1& lg1_;
        }; // struct IsEqualVisitor
      } // namespace level2

      /// Default implementation: for pairs of different types
      template <typename LgT1, typename LgT2>
      struct Comparison
      {
        inline bool operator () (const LgT1&, const LgT2&);
      }; // class Comparison

      // Specialization for two instances of same type
      template <typename LgT> struct Comparison <LgT, LgT>
      {
        inline bool operator () (const LgT&, const LgT&);
      };

      // Specialization for vector spaces
      template <bool rot>
      struct Comparison <VectorSpaceOperation <Eigen::Dynamic, rot>,
                         VectorSpaceOperation <Eigen::Dynamic, rot> >
      {
        inline bool operator ()
          (const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt1,
           const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt2);
      };

      template <int Size, bool rot>
      struct Comparison <VectorSpaceOperation <Eigen::Dynamic, rot>,
                         VectorSpaceOperation <Size, rot> >
      {
        inline bool operator ()
          (const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt1,
           const VectorSpaceOperation <Size, rot>& lgt2);
      };

      template <int Size, bool rot>
      struct Comparison <VectorSpaceOperation <Size, rot>,
                         VectorSpaceOperation <Eigen::Dynamic, rot> >
      {
        inline bool operator ()
          (const VectorSpaceOperation <Size, rot>& lgt1,
           const VectorSpaceOperation <Eigen::Dynamic, rot>& lgt2);
      };

    } // namespace liegroup
  } // namespace pinocchio
} // namespace hpp

# include "../src/comparison.hxx"

#endif
