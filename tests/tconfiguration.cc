///
/// Copyright (c) 2011 CNRS
/// Author: Florent Lamiraux
///
///
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

// This test
//   - builds a robot with various types of joints,
//   - randomly samples pairs of configurations,
//   - test consistency between hpp::pinocchio::difference and
//     hpp::pinocchio::integrate functions.

#define BOOST_TEST_MODULE tdevice

#include <boost/test/unit_test.hpp>

#include <Eigen/Geometry>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include "../tests/utils.hh"

/// This is a mere copy of the test in hpp-model.
/// Not testing against hpp-model outputs.

using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::vector_t;
using hpp::pinocchio::vectorIn_t;
using hpp::pinocchio::value_type;
using hpp::pinocchio::size_type;
typedef Eigen::AngleAxis <value_type> AngleAxis_t;
typedef Eigen::Quaternion <value_type> Quaternion_t;

vector_t slerp (vectorIn_t v0, vectorIn_t v1, const value_type t) {
  Quaternion_t q0(v0[3], v0[0], v0[1], v0[2]);
  Quaternion_t q1(v1[3], v1[0], v1[1], v1[2]);
  Quaternion_t q = q0.slerp (t, q1);
  vector_t res(4);
  res[0] = q.x(); res[1] = q.y(); res[2] = q.z(); res[3] = q.w();
  return res;
}

AngleAxis_t aa (vectorIn_t v0, vectorIn_t v1) {
  Quaternion_t q0(v0[3], v0[0], v0[1], v0[2]);
  Quaternion_t q1(v1[3], v1[0], v1[1], v1[2]);
  return AngleAxis_t (q0 * q1.conjugate ());
}

vector_t interpolation (hpp::pinocchio::DevicePtr_t robot, vectorIn_t q0, vectorIn_t q1, int n)
{
  const size_type rso3 = 3;
  bool print = false;

  vector_t q2 (q0);
  value_type u = 0;
  value_type step = (value_type)1 / (value_type)(n + 2);
  vector_t angle1 (n+2), angle2(n+2);

  if (print) std::cout << "HPP  : ";
  for (int i = 0; i < n + 2; ++i) {
    u = 0 + i * step;
    hpp::pinocchio::interpolate (robot, q0, q1, u, q2);
    if (print) std::cout << q2.segment<4>(rso3).transpose() << "\n";
    AngleAxis_t aa1 (aa (q0.segment<4>(rso3),q2.segment<4>(rso3)));
    angle1[i] = aa1.angle ();
    if (print) std::cout << aa1.angle () << ", ";
  }
  if (print) std::cout << "\nEigen: ";
  for (int i = 0; i < n + 2; ++i) {
    u = 0 + i * step;
    vector_t eigen_slerp = slerp (q0.segment<4>(rso3), q1.segment<4>(rso3), u);
    if (print) std::cout << eigen_slerp.transpose() << "\n";
    AngleAxis_t aa2 (aa (q0.segment<4>(rso3),eigen_slerp));
    angle2[i] = aa2.angle ();
    if (print) std::cout << aa2.angle () << ", ";
  }
  if (print) std::cout << "\n";
  return angle1 - angle2;
}

BOOST_AUTO_TEST_CASE(difference_and_integrate)
{
  hpp::pinocchio::DevicePtr_t robot = hppPinocchio();
  robot->model().lowerPositionLimit.head<3>().setConstant(-1);
  robot->model().upperPositionLimit.head<3>().setOnes();

  Configuration_t q0; q0.resize (robot->configSize ());
  Configuration_t q1; q1.resize (robot->configSize ());
  Configuration_t q2; q2.resize (robot->configSize ());
  vector_t q1_minus_q0; q1_minus_q0.resize (robot->numberDof ());
  const value_type eps_dist = robot->numberDof() * sqrt(Eigen::NumTraits<value_type>::epsilon());
  for (size_type i=0; i<10000; ++i) {
    q0 = se3::randomConfiguration (robot->model());
    q1 = se3::randomConfiguration (robot->model());

    hpp::pinocchio::difference (robot, q1, q0, q1_minus_q0);
    hpp::pinocchio::integrate (robot, q0, q1_minus_q0, q2);

    // Check that the distance is the norm of the difference
    value_type distance = hpp::pinocchio::distance (robot, q0, q1);
    BOOST_CHECK_MESSAGE (distance - q1_minus_q0.norm () < Eigen::NumTraits<value_type>::dummy_precision(),
        "\nThe distance is not the norm of the difference\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "distance=" << distance  << "\n" <<
        "(q1 - q0).norm () = " << q1_minus_q0.norm ()
        );

    // Check that distance (q0 + (q1 - q0), q1) is zero
    distance = hpp::pinocchio::distance (robot, q1, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + (q1 - q0)) is not equivalent to q1\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );
  }
}

BOOST_AUTO_TEST_CASE(interpolate)
{
  hpp::pinocchio::DevicePtr_t robot = hppPinocchio();
  robot->model().lowerPositionLimit.head<3>().setConstant(-1);
  robot->model().upperPositionLimit.head<3>().setOnes();

  Configuration_t q0; q0.resize (robot->configSize ());
  Configuration_t q1; q1.resize (robot->configSize ());
  Configuration_t q2; q2.resize (robot->configSize ());
  vector_t q1_minus_q0; q1_minus_q0.resize (robot->numberDof ());
  const value_type eps_dist = robot->numberDof() * sqrt(Eigen::NumTraits<value_type>::epsilon());
  value_type distance;
  for (size_type i=0; i<10000; ++i) {
    q0 = se3::randomConfiguration (robot->model());
    q1 = se3::randomConfiguration (robot->model());

    hpp::pinocchio::interpolate (robot, q0, q1, 0, q2);
    distance = hpp::pinocchio::distance (robot, q0, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + 0 * (q1 - q0)) is not equivalent to q0\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );

    vector_t errors = interpolation (robot, q0, q1, 4);
    BOOST_CHECK_MESSAGE (errors.isZero (),
        "The interpolation computed by HPP does not match the Eigen SLERP"
        );

    hpp::pinocchio::interpolate (robot, q0, q1, 1, q2);
    distance = hpp::pinocchio::distance (robot, q1, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + 1 * (q1 - q0)) is not equivalent to q1\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );
  }
}
