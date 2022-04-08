//
// Copyright (c) 2020 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_SERIALIZATION_HH
#define HPP_PINOCCHIO_SERIALIZATION_HH

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/util/serialization.hh>
#include <pinocchio/fwd.hpp>
#include <pinocchio/serialization/eigen.hpp>
#include <set>
#include <type_traits>

BOOST_SERIALIZATION_SPLIT_FREE(hpp::pinocchio::DevicePtr_t)
BOOST_SERIALIZATION_SPLIT_FREE(hpp::pinocchio::DeviceWkPtr_t)

namespace boost {
namespace serialization {
template <class Archive>
inline void load(Archive& ar, hpp::pinocchio::DevicePtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::pinocchio::Device>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  if (d && har && har->contains(d->name()))
    d = har->template get<hpp::pinocchio::Device>(d->name(), true)->self();
}
template <class Archive>
inline void load(Archive& ar, hpp::pinocchio::DeviceWkPtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::pinocchio::Device>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  std::string name(d.lock()->name());
  if (d.lock() && har && har->contains(name))
    d = har->template get<hpp::pinocchio::Device>(name, true)->self();
}
template <class Archive>
inline void load(Archive& ar, hpp::pinocchio::HumanoidRobotPtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::pinocchio::HumanoidRobot>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  if (d && har && har->contains(d->name()))
    d = har->template getChildClass<hpp::pinocchio::Device,
                                    hpp::pinocchio::HumanoidRobot>(d->name(),
                                                                   true)
            ->self();
}
template <class Archive>
inline void load(Archive& ar, hpp::pinocchio::HumanoidRobotWkPtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::pinocchio::HumanoidRobot>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  std::string name(d.lock()->name());
  if (d.lock() && har && har->contains(name))
    d = har->template getChildClass<hpp::pinocchio::Device,
                                    hpp::pinocchio::HumanoidRobot>(name, true)
            ->self();
}

template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline void serialize(
    Archive& ar,
    Eigen::Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m,
    const unsigned int version) {
  (void)version;
  Eigen::DenseIndex rows(m.rows()), cols(m.cols());
  ar& BOOST_SERIALIZATION_NVP(rows);
  ar& BOOST_SERIALIZATION_NVP(cols);
  if (!Archive::is_saving::value) m.resize(rows, cols);
  if (m.size() > 0)
    ar& make_nvp("data", make_array(m.data(), (size_t)m.size()));
}

}  // namespace serialization
}  // namespace boost

namespace hpp {
namespace serialization {
namespace remove_duplicate {
template <typename Key, typename Compare = std::less<Key>>
struct ptr_less : Compare {
  inline bool operator()(Key const* t1, Key const* t2) const {
    return Compare::operator()(*t1, *t2);
  }
};

template <typename Derived>
struct eigen_compare {
  bool operator()(const Eigen::PlainObjectBase<Derived>& a,
                  const Eigen::PlainObjectBase<Derived>& b) const {
    if (a.size() < b.size()) return true;
    if (a.size() > b.size()) return false;
    for (Eigen::Index i = 0; i < a.size(); ++i) {
      if (a.derived().data()[i] < b.derived().data()[i]) return true;
      if (a.derived().data()[i] > b.derived().data()[i]) return false;
    }
    return false;
  }
};

template <typename Key, typename Compare = std::less<Key>>
struct archive {
  typedef Compare compare_type;
  typedef ptr_less<Key, Compare> ptr_compare_type;
  std::set<Key const*, ptr_compare_type> datas;
  int hitcount;

  archive() : hitcount(0) {}
};

typedef archive<::hpp::pinocchio::vector_t,
                eigen_compare<::hpp::pinocchio::vector_t>>
    vector_archive;

template <class Archive, typename Key>
inline void load_or_save_no_remove_duplicate_check(Archive& ar,
                                                   const char* name, Key& key,
                                                   const unsigned int version) {
  (void)version;
  Key* value = &key;
  ar& boost::serialization::make_nvp(name, value);
  if (!Archive::is_saving::value) key = *value;
}

template <class Archive, typename Key>
inline void save_impl(Archive& ar, const char* name, const Key& key,
                      const unsigned int version) {
  (void)version;
  Key const* value = &key;
  ar << boost::serialization::make_nvp(name, value);
}

template <class Archive, typename Key, typename Compare = std::less<Key>>
inline void save_impl(Archive& ar,
                      std::set<Key const*, ptr_less<Key, Compare>>& set,
                      int& hitcount, const char* name, const Key& key,
                      const unsigned int version) {
  (void)version;
  if (!Archive::is_saving::value)
    throw std::logic_error(
        "HPP serialization: cannot load into a const element. This should "
        "never happen.");
  auto result = set.insert(&key);
  bool inserted = result.second;
  Key const* k = (inserted ? &key : *result.first);
  ar& boost::serialization::make_nvp(name, k);
  if (!inserted) hitcount++;
}

template <class Archive, typename Key, typename Compare = std::less<Key>>
inline void serialize(Archive& ar,
                      std::set<Key const*, ptr_less<Key, Compare>>& set,
                      int& hitcount, const char* name, Key& key,
                      const unsigned int version) {
  if (Archive::is_saving::value) {
    save_impl(ar, set, hitcount, name, key, version);
  } else {
    load_or_save_no_remove_duplicate_check(ar, name, key, version);
  }
}

template <bool is_base>
struct serialiaze_impl {
  template <typename Archive, typename Key, typename Compare = std::less<Key>>
  static inline void run(Archive& ar, const char* name, Key& key,
                         const unsigned int version) {
    archive<Key, Compare>& rda = dynamic_cast<archive<Key, Compare>&>(ar);
    serialize(ar, rda.datas, rda.hitcount, name, key, version);
  }
  template <typename Archive, typename Key, typename Compare = std::less<Key>>
  static inline void save(Archive& ar, const char* name, const Key& key,
                          const unsigned int version) {
    archive<Key, Compare>& rda = dynamic_cast<archive<Key, Compare>&>(ar);
    save_impl(ar, rda.datas, rda.hitcount, name, key, version);
  }
};

template <>
struct serialiaze_impl<false> {
  template <typename Archive, typename Key, typename Compare = std::less<Key>>
  static inline void run(Archive& ar, const char* name, Key& key,
                         const unsigned int version) {
    if (dynamic_cast<archive<Key, Compare>*>(&ar) != NULL)
      serialiaze_impl<true>::run<Archive, Key, Compare>(ar, name, key, version);
    else
      load_or_save_no_remove_duplicate_check(ar, name, key, version);
  }
  template <typename Archive, typename Key, typename Compare = std::less<Key>>
  static inline void save(Archive& ar, const char* name, const Key& key,
                          const unsigned int version) {
    if (dynamic_cast<archive<Key, Compare>*>(&ar) != NULL)
      serialiaze_impl<true>::save<Archive, Key, Compare>(ar, name, key,
                                                         version);
    else
      save_impl(ar, name, key, version);
  }
};

template <typename Archive, typename Key, typename Compare = std::less<Key>,
          bool is_base = std::is_base_of<archive<Key, Compare>, Archive>::value>
inline void serialize(Archive& ar, const char* name, Key& key,
                      const unsigned int version) {
  serialiaze_impl<is_base>::template run<Archive, Key, Compare>(ar, name, key,
                                                                version);
}

template <typename Archive, typename Key, typename Compare = std::less<Key>,
          bool is_base = std::is_base_of<archive<Key, Compare>, Archive>::value>
inline void save(Archive& ar, const char* name, const Key& key,
                 const unsigned int version) {
  serialiaze_impl<is_base>::template save<Archive, Key, Compare>(ar, name, key,
                                                                 version);
}

template <typename Archive>
inline void serialize_vector(Archive& ar, const char* name,
                             ::hpp::pinocchio::vector_t& key,
                             const unsigned int version) {
  serialize<Archive, ::hpp::pinocchio::vector_t, vector_archive::compare_type>(
      ar, name, key, version);
}

template <typename Archive>
inline void save_vector(Archive& ar, const char* name,
                        const ::hpp::pinocchio::vector_t& key,
                        const unsigned int version) {
  save<Archive, ::hpp::pinocchio::vector_t, vector_archive::compare_type>(
      ar, name, key, version);
}

}  // namespace remove_duplicate
}  // namespace serialization
}  // namespace hpp

#endif  // HPP_PINOCCHIO_SERIALIZATION_HH
