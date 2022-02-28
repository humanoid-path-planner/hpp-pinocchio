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

namespace hpp {
  namespace pinocchio {
    namespace liegroup {

      namespace level1 {
        IsEqualVisitor::IsEqualVisitor (const LiegroupType& lg2) : lg2_ (lg2)
        {
        }

        template <typename LgT1> bool IsEqualVisitor::operator ()
          (const LgT1& lg1)
        {
          hppDout (info,
                   "level1::isEqual::operator ()");
          return isEqual (lg1, lg2_);
        }

        template <typename LgT1> bool isEqual
        (const LgT1& lgt1, const LiegroupType& lgt2)
        {
          hppDout (info, "level1::isEqual");
          level2::IsEqualVisitor<LgT1> visitor (lgt1);
          return boost::apply_visitor (visitor, lgt2);
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
        bool IsEqualVisitor <LgT1>::operator () (const LgT2& lg2)
        {
          hppDout (info, "level2::isEqual::operator ()");
          Comparison <LgT1, LgT2> comp;
          return comp (lg1_, lg2);
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
