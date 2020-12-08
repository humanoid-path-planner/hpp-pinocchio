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

# include <set>
# include <type_traits>

# include <boost/serialization/split_free.hpp>
# include <boost/serialization/shared_ptr.hpp>
# include <boost/serialization/weak_ptr.hpp>

# include <hpp/pinocchio/fwd.hh>
# include <pinocchio/serialization/eigen.hpp>

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

namespace hpp {
namespace serialization {
namespace remove_duplicate {
template<typename Key, typename Compare = std::less<Key> >
struct ptr_less : Compare {
  inline bool operator() (Key const* t1, Key const* t2) { return Compare::operator() (*t1, *t2); }
};

template<typename Derived>
struct eigen_compare {
  bool operator() (const Eigen::DenseBase<Derived>& a,
                   const Eigen::DenseBase<Derived>& b)
  {
    if (a.size() < b.size()) return true;
    if (a.size() > b.size()) return false;
    for(Eigen::Index i = 0; i < a.size(); ++i) {
      if (a.derived().data()[i] < b.derived().data()[i]) return true;
      if (a.derived().data()[i] > b.derived().data()[i]) return false;
    }
    return false;
  }
};

template<typename Key, typename Compare = std::less<Key> >
struct archive {
  typedef Compare compare_type;
  typedef ptr_less<Key, Compare> ptr_compare_type;
  std::set<Key const*, ptr_compare_type > datas;
};

typedef archive<::hpp::pinocchio::vector_t, eigen_compare<::hpp::pinocchio::vector_t> > vector_archive;

template<class Archive, typename Key>
inline void load_or_save_no_remove_duplicate_check (Archive& ar,
    const char* name,
    Key& key,
    const unsigned int version)
{
  (void) version;
  Key* value = &key;
  ar & boost::serialization::make_nvp(name, value);
  if (!Archive::is_saving::value) key = *value;
}

template<class Archive, typename Key>
inline void save_impl (Archive& ar,
    const char* name,
    const Key& key,
    const unsigned int version)
{
  (void) version;
  Key const* value = &key;
  ar << boost::serialization::make_nvp(name, value);
}

template<class Archive, typename Key, typename Compare = std::less<Key>>
inline void save_impl (Archive& ar,
    std::set<Key const*, ptr_less<Key,Compare> >& set,
    const char* name,
    const Key& key,
    const unsigned int version)
{
  (void) version;
  if(!Archive::is_saving::value)
    throw std::logic_error("HPP serialization: cannot load into a const element. This should never happen.");
  auto result = set.insert(&key);
  bool inserted = result.second;
  Key const* k = (inserted ? &key : *result.first);
  ar & boost::serialization::make_nvp(name, k);
}

template<class Archive, typename Key, typename Compare = std::less<Key>>
inline void serialize (Archive& ar,
    std::set<Key const*, ptr_less<Key,Compare> >& set,
    const char* name,
    Key& key,
    const unsigned int version)
{
  if (Archive::is_saving::value) {
    save_impl(ar, set, name, key, version);
  } else {
    load_or_save_no_remove_duplicate_check(ar, name, key, version);
  }
}

template <bool is_base>
struct serialiaze_impl {
template<typename Archive, typename Key, typename Compare = std::less<Key> >
static inline void run (Archive& ar,
    const char* name,
    Key& key,
    const unsigned int version)
{
  serialize(ar, dynamic_cast<archive<Key, Compare>&>(ar).datas, name, key, version);
}
template<typename Archive, typename Key, typename Compare = std::less<Key> >
static inline void save (Archive& ar,
    const char* name,
    const Key& key,
    const unsigned int version)
{
  save_impl(ar, dynamic_cast<archive<Key, Compare>&>(ar).datas, name, key, version);
}
};

template <>
struct serialiaze_impl<false> {
template<typename Archive, typename Key, typename Compare = std::less<Key> >
static inline void run (Archive& ar,
    const char* name,
    Key& key,
    const unsigned int version)
{
  if (dynamic_cast<archive<Key, Compare>*>(&ar) != NULL)
    serialiaze_impl<true>::run<Archive, Key, Compare>(ar, name, key, version);
  else
    load_or_save_no_remove_duplicate_check(ar, name, key, version);
}
template<typename Archive, typename Key, typename Compare = std::less<Key> >
static inline void save (Archive& ar,
    const char* name,
    const Key& key,
    const unsigned int version)
{
  if (dynamic_cast<archive<Key, Compare>*>(&ar) != NULL)
    serialiaze_impl<true>::save<Archive, Key, Compare>(ar, name, key, version);
  else
    save_impl(ar, name, key, version);
}
};

template<typename Archive, typename Key, typename Compare = std::less<Key>,
  bool is_base = std::is_base_of<archive<Key, Compare>, Archive>::value >
inline void serialize (Archive& ar,
    const char* name,
    Key& key,
    const unsigned int version)
{
  serialiaze_impl<is_base>::template run<Archive, Key, Compare>(ar, name, key, version);
}

template<typename Archive, typename Key, typename Compare = std::less<Key>,
  bool is_base = std::is_base_of<archive<Key, Compare>, Archive>::value >
inline void save (Archive& ar,
    const char* name,
    const Key& key,
    const unsigned int version)
{
  serialiaze_impl<is_base>::template save<Archive, Key, Compare>(ar, name, key, version);
}

template<typename Archive>
inline void serialize_vector (Archive& ar,
    const char* name,
    ::hpp::pinocchio::vector_t& key,
    const unsigned int version)
{
  serialize<Archive, ::hpp::pinocchio::vector_t, vector_archive::compare_type>
    (ar, name, key, version);
}

template<typename Archive>
inline void save_vector (Archive& ar,
    const char* name,
    const ::hpp::pinocchio::vector_t& key,
    const unsigned int version)
{
  save<Archive, ::hpp::pinocchio::vector_t, vector_archive::compare_type>
    (ar, name, key, version);
}

} // namespace remove_duplicate
} // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_SERIALIZATION_HH
