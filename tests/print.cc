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

#include <iostream>

#include <hpp/pinocchio/util.hh>

#include <hpp/pinocchio/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

using namespace hpp;
using hpp::pinocchio::SE3;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::matrix_t;
using hpp::pinocchio::vectorOut_t;
using hpp::pinocchio::vectorIn_t;

int main ()
{
//! [Example usage]
  std::cout << incindent
    << iendl << pretty_print (SE3::Identity())
    << iendl << condensed (SE3::Identity())
    << iendl << one_line (SE3::Identity())
    << decindent << iendl;

  std::cout << setpyformat
    << iendl << pretty_print (SE3::Identity())
    << iendl << condensed (SE3::Identity())
    << iendl << one_line (SE3::Identity())
    << iendl;

  vector_t v = vector_t::Ones(2);
  matrix_t m = matrix_t::Identity(3,3);

  std::cout << unsetpyformat
    << iendl << pretty_print (v)
    << iendl << condensed (v)
    << iendl << one_line (v.segment(0,2))
    << iendl << pretty_print (vectorOut_t(v))
    << iendl << condensed (vectorIn_t(v))
    << iendl;

  std::cout << setpyformat
    << iendl << pretty_print (m)
    << iendl << condensed (m)
    << iendl << one_line (m.block(1,1,2,2))
    << iendl;
//! [Example usage]

  return 0;
}
