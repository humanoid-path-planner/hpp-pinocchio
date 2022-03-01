//
// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
//
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

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <hpp/pinocchio/liegroup.hh>

namespace hpp {
  namespace pinocchio {
      struct SetBoundStep : public ::pinocchio::fusion::JointUnaryVisitorBase<SetBoundStep>
      {
        typedef boost::fusion::vector<ConfigurationIn_t,
                Configuration_t &> ArgsType;

        template<typename JointModel>
        static void algo(const ::pinocchio::JointModelBase<JointModel> & jmodel,
            ConfigurationIn_t bounds,
            Configuration_t& out)
        {
          ::hpp::pinocchio::RnxSOnLieGroupMap::template operation<JointModel>::type
            ::setBound(bounds,
                       jmodel.jointConfigSelector(out));
        }
      };

      template <>
      void SetBoundStep::algo<JointModelComposite>(
          const ::pinocchio::JointModelBase<JointModelComposite> & jmodel,
          ConfigurationIn_t bounds,
          Configuration_t & out)
      {
        ::pinocchio::details::Dispatch<SetBoundStep>::run(
            jmodel.derived(),
            ArgsType(bounds, out));
      }
  } // namespace pinocchio
} // namespace hpp
