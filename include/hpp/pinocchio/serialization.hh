//
// Copyright (c) 2020 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_SERIALIZATION_HH
# define HPP_PINOCCHIO_SERIALIZATION_HH

# include <boost/serialization/split_free.hpp>
# include <boost/serialization/shared_ptr.hpp>
# include <boost/serialization/weak_ptr.hpp>

# include <hpp/pinocchio/fwd.hh>

namespace hpp {
namespace serialization {
struct archive_device_wrapper {
  pinocchio::DevicePtr_t device;
  virtual ~archive_device_wrapper() {}
};
} // namespace pinocchio
} // namespace hpp


BOOST_SERIALIZATION_SPLIT_FREE(hpp::pinocchio::DevicePtr_t)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::pinocchio::DeviceWkPtr_t)

namespace boost {
namespace serialization {
template<class Archive>
inline void load (Archive& ar, hpp::pinocchio::DevicePtr_t& d, const unsigned int version)
{
  load<Archive, hpp::pinocchio::Device> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = adw->device;
}
template<class Archive>
inline void load (Archive& ar, hpp::pinocchio::DeviceWkPtr_t& d, const unsigned int version)
{
  load<Archive, hpp::pinocchio::Device> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = adw->device;
}
template<class Archive>
inline void load (Archive& ar, hpp::pinocchio::HumanoidRobotPtr_t& d, const unsigned int version)
{
  load<Archive, hpp::pinocchio::HumanoidRobot> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = boost::dynamic_pointer_cast<hpp::pinocchio::HumanoidRobot>(adw->device);
}
template<class Archive>
inline void load (Archive& ar, hpp::pinocchio::HumanoidRobotWkPtr_t& d, const unsigned int version)
{
  load<Archive, hpp::pinocchio::HumanoidRobot> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = boost::dynamic_pointer_cast<hpp::pinocchio::HumanoidRobot>(adw->device);
}

template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(Archive & ar, Eigen::Array<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version)
{
  (void) version;
  Eigen::DenseIndex rows(m.rows()), cols(m.cols());
  ar & BOOST_SERIALIZATION_NVP(rows);
  ar & BOOST_SERIALIZATION_NVP(cols);
  if (!Archive::is_saving::value)
    m.resize(rows,cols);
  if(m.size() > 0)
    ar & make_nvp("data",make_array(m.data(), (size_t)m.size()));
}

} // namespace serialization
} // namespace boost

#endif // HPP_PINOCCHIO_SERIALIZATION_HH
