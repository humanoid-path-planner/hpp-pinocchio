//
// Copyright (c) 2018 CNRS
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

#ifndef HPP_PINOCCHIO_POOL_HH
#define HPP_PINOCCHIO_POOL_HH

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <hpp/pinocchio/config.hh>
#include <vector>

namespace hpp {
namespace pinocchio {

/// \brief Pool of objects
///
/// Usage:
/// \code
/// Pool<Foo> pool;
/// std::vector<Foo*> temporary_foos;
/// // Fill temporary_foos new objects.
/// // The ownership is passed to the pool.
/// pool.push_back (foo.begin(), foo.end());
///
/// Foo* foo = pool.acquire();
/// // Access to foo is thread safe.
/// pool.release(foo);
/// \endcode
template <typename T>
class HPP_PINOCCHIO_DLLAPI Pool {
 public:
  /// Get an object from the pool.
  /// If the pool is empty, wait until one element becomes available.
  T* acquire() {
    boost::mutex::scoped_lock lock(mutex_);
    condVariable_.wait(lock, OneIsAvailable(*this));
    T* ret = NULL;
    std::swap(ret, Ts_[lastFree_]);
    lastFree_++;
    return ret;
  }

  /// Release a previously acquired object.
  /// \warning There is no check that the object was actually one returned
  /// by Pool::acquire
  void release(T* t) {
    boost::mutex::scoped_lock lock(mutex_);
    // At least one T* is not free.
    assert(lastFree_ > 0);
    assert(Ts_[lastFree_ - 1] == NULL);
    Ts_[lastFree_ - 1] = t;
    lastFree_--;
    condVariable_.notify_one();
  }

  /// Returns true is at least one object is not locked.
  bool available() const { return OneIsAvailable(*this)(); }

  std::size_t size() const { return Ts_.size(); }

  /// Deletes all internal objects.
  void clear() {
    boost::mutex::scoped_lock lock(mutex_);
    if (lastFree_ > 0)
      throw std::logic_error("Cannot free pool when some objects are in use.");
    for (std::size_t i = 0; i < size(); ++i) delete Ts_[i];
    Ts_.resize(0);
  }

  /// Adds an object to the pool
  /// The pool takes ownership of the object
  void push_back(T* t) {
    boost::mutex::scoped_lock lock(mutex_);
    Ts_.push_back(t);
  }

  /// Add objects to the pool
  /// The pool takes ownership of the object
  template <class InputIterator>
  void push_back(InputIterator first, InputIterator last) {
    boost::mutex::scoped_lock lock(mutex_);
    Ts_.insert(Ts_.end(), first, last);
  }

  /// Destructor
  /// Calls Pool::clear()
  ~Pool() { clear(); }

  /// Constructor
  Pool() : lastFree_(0) {}

 private:
  struct OneIsAvailable {
    const Pool& p;
    OneIsAvailable(const Pool& pool) : p(pool) {}
    bool operator()() { return p.lastFree_ < p.size(); }
  };

  boost::mutex mutex_;
  boost::condition_variable condVariable_;
  std::vector<T*> Ts_;
  std::size_t lastFree_;
};  // class Pool
}  // namespace pinocchio
}  // namespace hpp

#endif  // HPP_PINOCCHIO_DEVICE_HH
