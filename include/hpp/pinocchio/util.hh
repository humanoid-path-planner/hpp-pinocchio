//
// Copyright (c) 2017-2018 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_UTIL_HH
# define HPP_PINOCCHIO_UTIL_HH

# include <hpp/util/indent.hh>

# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
    /// \addtogroup to_output_stream Printing to output stream
    /// \{

    /// This function must be specialized for the type you want to print.
    template <typename T, int Option> struct HPP_PINOCCHIO_DLLAPI prettyPrint { static std::ostream& run (std::ostream& os, const T& pp); };

    /// The printing options, currently only contains the output format
    enum {
      OutputFormatBits = 3,

      OneLineOutput = 0,
      CondensedOutput = 1,
      PrettyOutput = 2
    };

    /// \cond
    HPP_PINOCCHIO_DLLAPI long& getpythonformat (std::ostream& o);

    template <bool OneLine, bool PythonStyle, bool Vector>
    struct HPP_PINOCCHIO_DLLAPI eigen_format { static const Eigen::IOFormat run(); };
  
  // Default implementation
  template <bool OneLine, bool PythonStyle, bool Vector> const Eigen::IOFormat
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

    template <typename T, int Option> struct PrettyPrint {
      const T& value;
      inline explicit PrettyPrint (const T& t) : value (t) {}
    };

    template <typename T, int Option>
      std::ostream& operator<< (std::ostream& os, const PrettyPrint<T, Option> pp)
      {
        return prettyPrint<T, Option>::run (os, pp.value);
      }

    /// Generic implementation for Eigen objects
    template <typename Derived, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrintEigen {
        static inline std::ostream& run (std::ostream& os, const Derived& M)
        {
          enum { Condensed = ((Option & OutputFormatBits) == OneLineOutput) || ((Option & OutputFormatBits) == CondensedOutput) };
          static const Eigen::IOFormat mfmt_py  = eigen_format< Condensed, true , false>::run();
          static const Eigen::IOFormat vfmt_py  = eigen_format< Condensed, true , true >::run();
          static const Eigen::IOFormat mfmt_raw = eigen_format< Condensed, false, false>::run();
          static const Eigen::IOFormat vfmt_raw = eigen_format< Condensed, false, true >::run();
          bool use_py_fmt = (getpythonformat(os) != 0);
          const Eigen::IOFormat& fmt =
            (Derived::IsVectorAtCompileTime
             ? (use_py_fmt ? vfmt_py : vfmt_raw)
             : (use_py_fmt ? mfmt_py : mfmt_raw));
          bool transpose = (Derived::ColsAtCompileTime == 1);

          if (transpose) return os << M.transpose().format(fmt);
          else           return os << M.format(fmt);
        }
      };
    /// FIXME All eigen object must be manually specialized as follow...

    /// Pretty printer for Eigen::Matrix
    template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrint <Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >, Option>
      : prettyPrintEigen <Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >, Option> {};

    /// Pretty printer for Eigen::VectorBlock
    template<typename OtherDerived, int Size, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrint <Eigen::VectorBlock< OtherDerived, Size >, Option>
      : prettyPrintEigen <Eigen::VectorBlock< OtherDerived, Size >, Option > {};

    /// Pretty printer for Eigen::Block
    template<typename XprType, int BlockRows, int BlockCols, bool InnerPanel, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrint <Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>, Option>
      : prettyPrintEigen <Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>, Option > {};

    /// Pretty printer for Eigen::Ref
    template<typename _PlainObjectType, int _Options, typename _StrideType, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrint <Eigen::Ref< _PlainObjectType, _Options, _StrideType>, Option>
      : prettyPrintEigen <Eigen::Ref< _PlainObjectType, _Options, _StrideType>, Option> {};

    /// Pretty printer for Eigen::Quaternion
    template<typename _Scalar, int _Options, int Option>
      struct HPP_PINOCCHIO_DLLAPI prettyPrint <Eigen::Quaternion< _Scalar, _Options>, Option>
      {
        typedef Eigen::Quaternion< _Scalar, _Options> Derived;
        typedef typename Eigen::internal::traits<Derived>::Coefficients Coefficients;
        static inline std::ostream& run (std::ostream& os, const Derived& M)
        {
          return prettyPrint<Coefficients, Option>::run (os, M.coeffs());
        }
      };
    /// \endcond

    // Set python formatting of vector and matrices
    HPP_PINOCCHIO_DLLAPI std::ostream& setpyformat (std::ostream& o);

    // Unset python formatting of vector and matrices
    HPP_PINOCCHIO_DLLAPI std::ostream& unsetpyformat (std::ostream& o);

    /// Pretty printing
    template <typename T> inline PrettyPrint<T, PrettyOutput    > pretty_print (const T& t) { return PrettyPrint<T, PrettyOutput    >(t); }
    /// Condensed printing
    template <typename T> inline PrettyPrint<T, CondensedOutput > condensed    (const T& t) { return PrettyPrint<T, CondensedOutput >(t); }
    /// Print on one line
    template <typename T> inline PrettyPrint<T, OneLineOutput   > one_line     (const T& t) { return PrettyPrint<T, OneLineOutput   >(t); }

    /// \}
} // namespace hpp
#endif // HPP_PINOCCHIO_UTIL_HH
