// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_PINOCCHIO_SIMPLE_DEVICE_HH
# define HPP_PINOCCHIO_SIMPLE_DEVICE_HH

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace pinocchio {
    DevicePtr_t humanoidSimple(
        const std::string& name = "humanoidSimple",
        bool usingFF = true,
        Computation_t compFlags = (Computation_t) (JOINT_POSITION | JACOBIAN)
        );

    DevicePtr_t humanoidSimple(
        const std::string& name = "humanoidSimple",
        Computation_t compFlags = (Computation_t) (JOINT_POSITION | JACOBIAN)
        );

    namespace unittest {
      enum TestDeviceType {
        HumanoidRomeo,
        HumanoidSimple,
        CarLike,
        ManipulatorArm2
      };

      DevicePtr_t makeDevice (TestDeviceType type,
                              const std::string& prefix = "");
    }
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_SIMPLE_DEVICE_HH
