//
// Copyright (c) 2017-2018 CNRS
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

#ifndef HPP_PINOCCHIO_UTIL_HH
#define HPP_PINOCCHIO_UTIL_HH

#include <hpp/pinocchio/config.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/util/indent.hh>

namespace hpp {
/// \addtogroup to_output_stream Printing to output stream
///
/// Some tools to ease pretty printing of objects in HPP.
/// To print vectors, you have the following options:
/// \snippet tests/print.cc Example usage
/// which should output something like
/// \code
///    R =   1, 0, 0
///          0, 1, 0
///          0, 0, 1
///    p = 0, 0, 0
///    q = 0, 0, 0, 1
///    p = 0, 0, 0
///    q = 0, 0, 0, 1, p = 0, 0, 0
///
///  R = ( (1, 0, 0,)
///        (0, 1, 0,)
///        (0, 0, 1,) )
///  p = (0, 0, 0,)
///  q = (0, 0, 0, 1,)
///  p = (0, 0, 0,)
///  q = (0, 0, 0, 1,), p = (0, 0, 0,)
///
///  1, 1
///  1, 1
///  1, 1
///  1, 1
///  1, 1
///
///  ( (1, 0, 0,),
///     (0, 1, 0,),
///     (0, 0, 1,), )
///  ( (1, 0, 0,),    (0, 1, 0,),    (0, 0, 1,), )
///  ( (1, 0,),    (0, 1,), )
/// \endcode
/// \{

/// This function must be specialized for the type you want to print.
template <typename T, int Option>
struct HPP_PINOCCHIO_DLLAPI prettyPrint {
  static std::ostream& run(std::ostream& os, const T& pp);
};

/// The printing options, currently only contains the output format
enum {
  OutputFormatBits = 3,

  OneLineOutput = 0,
  CondensedOutput = 1,
  PrettyOutput = 2
};

/// \cond
HPP_PINOCCHIO_DLLAPI long& getpythonformat(std::ostream& o);

template <bool OneLine, bool PythonStyle, bool Vector>
struct HPP_PINOCCHIO_DLLAPI eigen_format {
  static const Eigen::IOFormat run();
};

// Default implementation
template <bool OneLine, bool PythonStyle, bool Vector>
const Eigen::IOFormat eigen_format<OneLine, PythonStyle, Vector>::run() {
  static const Eigen::IOFormat fmt(
      (PythonStyle ? Eigen::FullPrecision : Eigen::StreamPrecision), 0,
      ", ",         // Coeff separator
      (PythonStyle  // Row separator
           ? (OneLine ? ", " : ",\n")
           : (OneLine ? "; " : "\n")),
      (PythonStyle ? "(" : ""),              // row prefix
      (PythonStyle ? ",)" : ""),             // row suffix
      (PythonStyle && !Vector ? "( " : ""),  // mat prefix
      (PythonStyle && !Vector ? ", )" : "")  // mat suffix
  );
  return fmt;
}

template <typename T, int Option>
struct PrettyPrint {
  const T& value;
  inline explicit PrettyPrint(const T& t) : value(t) {}
};

template <typename T, int Option>
std::ostream& operator<<(std::ostream& os, const PrettyPrint<T, Option> pp) {
  return prettyPrint<T, Option>::run(os, pp.value);
}

/// Generic implementation for Eigen objects
template <typename Derived, int Option>
struct HPP_PINOCCHIO_DLLAPI prettyPrintEigen {
  static inline std::ostream& run(std::ostream& os, const Derived& M) {
    enum {
      Condensed = ((Option & OutputFormatBits) == OneLineOutput) ||
                  ((Option & OutputFormatBits) == CondensedOutput)
    };
    static const Eigen::IOFormat mfmt_py =
        eigen_format<Condensed, true, false>::run();
    static const Eigen::IOFormat vfmt_py =
        eigen_format<Condensed, true, true>::run();
    static const Eigen::IOFormat mfmt_raw =
        eigen_format<Condensed, false, false>::run();
    static const Eigen::IOFormat vfmt_raw =
        eigen_format<Condensed, false, true>::run();
    bool use_py_fmt = (getpythonformat(os) != 0);
    const Eigen::IOFormat& fmt =
        (Derived::IsVectorAtCompileTime ? (use_py_fmt ? vfmt_py : vfmt_raw)
                                        : (use_py_fmt ? mfmt_py : mfmt_raw));
    bool transpose = (Derived::ColsAtCompileTime == 1);

    if (transpose)
      return os << M.transpose().format(fmt);
    else
      return os << M.format(fmt);
  }
};
/// FIXME All eigen object must be manually specialized as follow...

/// Pretty printer for Eigen::Matrix
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
          int _MaxCols, int Option>
struct HPP_PINOCCHIO_DLLAPI prettyPrint<
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, Option>
    : prettyPrintEigen<
          Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>,
          Option> {};

/// Pretty printer for Eigen::VectorBlock
template <typename OtherDerived, int Size, int Option>
struct HPP_PINOCCHIO_DLLAPI
    prettyPrint<Eigen::VectorBlock<OtherDerived, Size>, Option>
    : prettyPrintEigen<Eigen::VectorBlock<OtherDerived, Size>, Option> {};

/// Pretty printer for Eigen::Block
template <typename XprType, int BlockRows, int BlockCols, bool InnerPanel,
          int Option>
struct HPP_PINOCCHIO_DLLAPI
    prettyPrint<Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>, Option>
    : prettyPrintEigen<Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>,
                       Option> {};

/// Pretty printer for Eigen::Ref
template <typename _PlainObjectType, int _Options, typename _StrideType,
          int Option>
struct HPP_PINOCCHIO_DLLAPI
    prettyPrint<Eigen::Ref<_PlainObjectType, _Options, _StrideType>, Option>
    : prettyPrintEigen<Eigen::Ref<_PlainObjectType, _Options, _StrideType>,
                       Option> {};

/// Pretty printer for Eigen::Quaternion
template <typename _Scalar, int _Options, int Option>
struct HPP_PINOCCHIO_DLLAPI
    prettyPrint<Eigen::Quaternion<_Scalar, _Options>, Option> {
  typedef Eigen::Quaternion<_Scalar, _Options> Derived;
  typedef typename Eigen::internal::traits<Derived>::Coefficients Coefficients;
  static inline std::ostream& run(std::ostream& os, const Derived& M) {
    return prettyPrint<Coefficients, Option>::run(os, M.coeffs());
  }
};
/// \endcond

// Set python formatting of vector and matrices
HPP_PINOCCHIO_DLLAPI std::ostream& setpyformat(std::ostream& o);

// Unset python formatting of vector and matrices
HPP_PINOCCHIO_DLLAPI std::ostream& unsetpyformat(std::ostream& o);

/// Pretty printing
template <typename T>
inline PrettyPrint<T, PrettyOutput> pretty_print(const T& t) {
  return PrettyPrint<T, PrettyOutput>(t);
}
/// Condensed printing
template <typename T>
inline PrettyPrint<T, CondensedOutput> condensed(const T& t) {
  return PrettyPrint<T, CondensedOutput>(t);
}
/// Print on one line
template <typename T>
inline PrettyPrint<T, OneLineOutput> one_line(const T& t) {
  return PrettyPrint<T, OneLineOutput>(t);
}

/// \}
}  // namespace hpp
#endif  // HPP_PINOCCHIO_UTIL_HH
