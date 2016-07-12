//
// Copyright (c) 2016 CNRS
// Author: NMansard
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

#define BOOST_TEST_MODULE tdevice

#include <boost/test/unit_test.hpp>


#include <hpp/pinocchio/fake-container.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace pinocchio {


    struct FakeJointVector
      : public FakeContainer<int *,const int *>
    {
      FakeJointVector(DevicePtr_t ptr) : FakeContainer<int*,const int*>(ptr) {}

      virtual ~FakeJointVector() {}
      virtual int* at(const size_type i) { std::cout << i << std::endl; return new int(); }
      virtual const int* at(const size_type i) const { std::cout << i << std::endl; return new int(); }
      virtual size_type size() const { return 10; }
    };
  } // namespace pinocchio
} // namespace hpp

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (fake)
{
  using namespace hpp::pinocchio;
  DevicePtr_t device = Device::create("");

  FakeJointVector vec(device);
  for( FakeJointVector::iterator it=vec.begin();it!=vec.end();++it )
    *it;

  for( FakeJointVector::iterator it=vec.rbegin();it!=vec.rend();--it )
    *it;

}
