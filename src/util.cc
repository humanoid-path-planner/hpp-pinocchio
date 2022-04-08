// Copyright (c) 2017-2018, Joseph Mirabel
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

#include <hpp/pinocchio/util.hh>
#include <iostream>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
long& getpythonformat(std::ostream& o) {
  // The slot to store the current python format type.
  static const int pythonformat_index = std::ios::xalloc();
  return o.iword(pythonformat_index);
}
std::ostream& setpyformat(std::ostream& o) {
  getpythonformat(o) = 1;
  return o;
}
std::ostream& unsetpyformat(std::ostream& o) {
  getpythonformat(o) = 0;
  return o;
}

// ::pinocchio::SE3
template <int Option>
struct HPP_PINOCCHIO_DLLAPI prettyPrint<pinocchio::SE3, Option> {
  static std::ostream& run(std::ostream& os, const ::pinocchio::SE3& M) {
    enum {
      OneLine = ((Option & OutputFormatBits) == OneLineOutput),
      Condensed = ((Option & OutputFormatBits) == CondensedOutput)
    };
    static const Eigen::IOFormat mfmt_py = eigen_format < OneLine || Condensed,
                                 true, false > ::run();
    static const Eigen::IOFormat vfmt_py = eigen_format < OneLine || Condensed,
                                 true, true > ::run();
    static const Eigen::IOFormat mfmt_raw = eigen_format < OneLine || Condensed,
                                 false, false > ::run();
    static const Eigen::IOFormat vfmt_raw = eigen_format < OneLine || Condensed,
                                 false, true > ::run();
    bool use_py_fmt = (getpythonformat(os) != 0);
    const Eigen::IOFormat& vfmt = (use_py_fmt ? vfmt_py : vfmt_raw);

    switch (Option & OutputFormatBits) {
      case OneLineOutput:
        return os << "q = "
                  << one_line(::pinocchio::SE3::Quaternion(M.rotation()))
                  << ", p = " << M.translation().transpose().format(vfmt);
      case CondensedOutput:
        return os << "q = "
                  << one_line(::pinocchio::SE3::Quaternion(M.rotation()))
                  << iendl
                  << "p = " << M.translation().transpose().format(vfmt);
      case PrettyOutput:
      default:
        char po[] = "( ", pc[] = " )", sp[] = "  ";
        if (!use_py_fmt) {
          po[0] = ' ';
          pc[1] = ' ';
        }
        return os << "R = " << po << M.rotation().row(0).format(vfmt) << iendl
                  << "    " << sp << M.rotation().row(1).format(vfmt) << iendl
                  << "    " << sp << M.rotation().row(2).format(vfmt) << pc
                  << iendl
                  << "p = " << M.translation().transpose().format(vfmt);
    }
  }
};

template struct HPP_PINOCCHIO_DLLAPI prettyPrint<pinocchio::SE3, PrettyOutput>;
template struct HPP_PINOCCHIO_DLLAPI
    prettyPrint<pinocchio::SE3, CondensedOutput>;
template struct HPP_PINOCCHIO_DLLAPI prettyPrint<pinocchio::SE3, OneLineOutput>;
}  // namespace hpp
