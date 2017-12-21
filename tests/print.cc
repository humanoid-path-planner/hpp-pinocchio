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

#include <hpp/pinocchio/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

using namespace hpp::pinocchio;

int main ()
{
  std::cout << hpp::incindent
    << hpp::iendl << pretty_print (se3::SE3::Identity())
    << hpp::iendl << condensed (se3::SE3::Identity())
    << hpp::iendl << one_line (se3::SE3::Identity())
    << hpp::decindent << hpp::iendl;

  std::cout << setpyformat
    << hpp::iendl << pretty_print (se3::SE3::Identity())
    << hpp::iendl << condensed (se3::SE3::Identity())
    << hpp::iendl << one_line (se3::SE3::Identity())
    << hpp::iendl;

  vector_t v = vector_t::Ones(2);
  matrix_t m = matrix_t::Identity(3,3);

  std::cout << unsetpyformat
    << hpp::iendl << pretty_print (v)
    << hpp::iendl << condensed (v)
    << hpp::iendl << one_line (v)
    << hpp::iendl;

  std::cout << setpyformat
    << hpp::iendl << pretty_print (m)
    << hpp::iendl << condensed (m)
    << hpp::iendl << one_line (m)
    << hpp::iendl;

  return 0;
}
