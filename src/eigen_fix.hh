// Copyright (c) 2018, CNRS
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

// Eigen 3.2.0 has no fixed size segment<>, moddleRows<>, middleCols<> with 2
// arguments. This is not documented in recent eigen either.
// This was introduced in Eigen 3.2.1. See
// - http://eigen.tuxfamily.org/index.php?title=ChangeLog#Eigen_3.2.1 (bug 579)
// - http://eigen.tuxfamily.org/bz/show_bug.cgi?id=579
#if EIGEN_VERSION_AT_LEAST(3,2,1)
# define _BLOCK_ACCESSOR_EIGEN(var,method,tplSize,arg,size)                    \
    var.method<tplSize>(arg,size)
#else
# define _BLOCK_ACCESSOR_EIGEN(var,method,tplSize,arg,size)                    \
    var.method         (arg,size)
#endif
