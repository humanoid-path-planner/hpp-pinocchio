// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/pinocchio/util.hh>

#include <iostream>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
  namespace pinocchio {
    long& getpythonformat (std::ostream& o)
    {
      // The slot to store the current python format type.
      static const int pythonformat_index = std::ios::xalloc ();
      return o.iword (pythonformat_index);
    }
    std::ostream&   setpyformat (std::ostream& o) { getpythonformat(o) = 1; return o; }
    std::ostream& unsetpyformat (std::ostream& o) { getpythonformat(o) = 0; return o; }

    template <bool OneLine, bool PythonStyle, bool Vector> const Eigen::IOFormat&
      eigen_format<OneLine, PythonStyle, Vector>::run() {
        static const Eigen::IOFormat fmt(
            (PythonStyle ? Eigen::FullPrecision : Eigen::StreamPrecision),
            0,
            ", ",                                 // Coeff separator
            (PythonStyle                          // Row separator
             ? (OneLine ? ", ": ",\n" )
             : (OneLine ? "; ": "\n"  )),
            (PythonStyle ? "("  : ""),            // row prefix
            (PythonStyle ? ",)" : ""),            // row suffix
            (PythonStyle && !Vector ? "( " : ""), // mat prefix
            (PythonStyle && !Vector ? ", )" : "") // mat suffix
            );
        return fmt;
      }

    // se3::SE3
    template <int Option> struct HPP_PINOCCHIO_DLLAPI prettyPrint <se3::SE3, Option> {
      static std::ostream& run (std::ostream& os, const se3::SE3& M)
      {
        enum {
          OneLine = ((Option & OutputFormatBits) == OneLineOutput),
          Condensed = ((Option & OutputFormatBits) == CondensedOutput)
        };
        static const Eigen::IOFormat& mfmt_py  = eigen_format< OneLine || Condensed, true , false>::run();
        static const Eigen::IOFormat& vfmt_py  = eigen_format< OneLine || Condensed, true , true >::run();
        static const Eigen::IOFormat& mfmt_raw = eigen_format< OneLine || Condensed, false, false>::run();
        static const Eigen::IOFormat& vfmt_raw = eigen_format< OneLine || Condensed, false, true >::run();
        bool use_py_fmt = (getpythonformat(os) != 0);
        const Eigen::IOFormat& mfmt = (use_py_fmt ? mfmt_py : mfmt_raw);
        const Eigen::IOFormat& vfmt = (use_py_fmt ? vfmt_py : vfmt_raw);

        switch (Option & OutputFormatBits) {
          case OneLineOutput:
            return os <<  "R = " << M.rotation().format(mfmt)
              << ", p = " << M.translation().transpose().format(vfmt);
          case CondensedOutput:
            return os <<  "R = " << M.rotation().format(mfmt)
              << iendl << "p = " << M.translation().transpose().format(vfmt);
          case PrettyOutput:
          default:
            char po[] = "( ", pc[] = " )", sp[] = "  ";
            if (!use_py_fmt) { po[0] = ' '; pc[1] = ' '; }
            return os <<  "R = " << po << M.rotation().row(0).format(vfmt)
              << iendl << "    " << sp << M.rotation().row(1).format(vfmt)
              << iendl << "    " << sp << M.rotation().row(2).format(vfmt) << pc
              << iendl << "p = " << M.translation().transpose().format(vfmt);
        }
      }
    };

    template struct HPP_PINOCCHIO_DLLAPI prettyPrint <se3::SE3, PrettyOutput    >;
    template struct HPP_PINOCCHIO_DLLAPI prettyPrint <se3::SE3, CondensedOutput >;
    template struct HPP_PINOCCHIO_DLLAPI prettyPrint <se3::SE3, OneLineOutput   >;
  } // namespace pinocchio
} // namespace hpp
