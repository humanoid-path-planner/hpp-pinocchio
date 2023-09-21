// Copyright (c) 2017, Joseph Mirabel
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

#define BOOST_TEST_MODULE tdevice - sync

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/test/unit_test.hpp>
#include <ctime>
#include <pinocchio/fwd.hpp>

namespace bpt = boost::posix_time;

#include <hpp/pinocchio/device-sync.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/util.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>

using namespace hpp;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_CASE(single_thread) {
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  BOOST_REQUIRE(robot);

  robot->numberDeviceData(2);
  DeviceSync *d1, *d2;

  DataPtr_t data1, data2;
  d1 = new DeviceSync(robot);
  d2 = new DeviceSync(robot);
  data1 = d1->dataPtr();
  data2 = d2->dataPtr();
  // Delete in same order of creation: It should swap the internal datas.
  delete d1;
  delete d2;
  d1 = new DeviceSync(robot);
  d2 = new DeviceSync(robot);
  BOOST_CHECK_EQUAL(data1, d2->dataPtr());
  BOOST_CHECK_EQUAL(data2, d1->dataPtr());
  delete d1;
  delete d2;

  size_type nCfg = robot->configSize();
  robot->setDimensionExtraConfigSpace(3);
  BOOST_CHECK_EQUAL(robot->configSize(), nCfg + 3);
  d1 = new DeviceSync(robot);
  d2 = new DeviceSync(robot);
  BOOST_CHECK_EQUAL(d1->d().currentConfiguration_.size(), nCfg + 3);
  BOOST_CHECK_EQUAL(d2->d().currentConfiguration_.size(), nCfg + 3);
  delete d1;
  delete d2;
}

typedef ::pinocchio::container::aligned_vector<SE3> SE3Vector_t;

void compute_forward_kinematics(DevicePtr_t& device, const Configuration_t& q,
                                SE3Vector_t& res) {
  device->currentConfiguration(q);
  device->computeForwardKinematics(JOINT_POSITION);

  res = device->data().oMi;
}

void compute_forward_kinematics_thread_safe(DevicePtr_t& device,
                                            const Configuration_t& q,
                                            SE3Vector_t& res) {
  DeviceSync sync(device);
  sync.currentConfiguration(q);
  sync.computeForwardKinematics(JOINT_POSITION);

  res = sync.data().oMi;
}

BOOST_AUTO_TEST_CASE(check_synchronization) {
  DevicePtr_t robot = unittest::makeDevice(unittest::CarLike);
  BOOST_REQUIRE(robot);

  const size_type Nthreads = 4;
  robot->numberDeviceData(4);

  const std::size_t N = 1000;
  // Generate N configurations
  std::vector<Configuration_t> qs(N);
  for (std::size_t i = 0; i < qs.size(); ++i) {
    qs[i] = ::pinocchio::randomConfiguration(robot->model());
    // BOOST_CHECK (qs[i].isFinite());
    BOOST_CHECK(!qs[i].hasNaN());
  }

  typedef std::vector<SE3Vector_t> Results_t;
  Results_t res_single(qs.size()), res_multi(qs.size());

  bpt::ptime t0 = bpt::microsec_clock::local_time();

  // In a single threaded fashion, compute the joint positions
  for (std::size_t i = 0; i < qs.size(); ++i)
    compute_forward_kinematics(robot, qs[i], res_single[i]);

  bpt::ptime t1 = bpt::microsec_clock::local_time();
  BOOST_TEST_MESSAGE("1 thread:  " << (t1 - t0).total_milliseconds() << "ms");

  t0 = bpt::microsec_clock::local_time();
  // In a multi threaded fashion, compute the joint positions
#pragma omp parallel for
  for (std::size_t i = 0; i < qs.size(); ++i)
    compute_forward_kinematics_thread_safe(robot, qs[i], res_multi[i]);

  t1 = bpt::microsec_clock::local_time();
  BOOST_TEST_MESSAGE(Nthreads << " threads: " << (t1 - t0).total_milliseconds()
                              << "ms");

  // Check the results
  for (std::size_t i = 0; i < qs.size(); ++i) {
    BOOST_CHECK_EQUAL(res_single[i].size(), res_multi[i].size());
    for (std::size_t j = 0; j < res_single[i].size(); ++j) {
      BOOST_CHECK_EQUAL(res_single[i][j], res_multi[i][j]);
    }
  }
}
