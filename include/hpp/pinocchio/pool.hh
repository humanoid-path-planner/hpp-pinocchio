//
// Copyright (c) 2018 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_PINOCCHIO_POOL_HH
#define HPP_PINOCCHIO_POOL_HH

# include <vector>

# include <boost/thread/mutex.hpp>
# include <boost/thread/condition_variable.hpp>

# include <hpp/pinocchio/config.hh>

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
    class HPP_PINOCCHIO_DLLAPI Pool
    {
      public:
        /// Get an object from the pool.
        /// If the pool is empty, wait until one element becomes available.
        T* acquire ()
        {
          boost::mutex::scoped_lock lock (mutex_);
          condVariable_.wait (lock, OneIsAvailable(*this));
          T* ret = NULL;
          std::swap (ret, Ts_[lastFree_]);
          lastFree_++;
          return ret;
        }

        /// Release a previously acquired object.
        /// \warning There is no check that the object was actually one returned
        /// by Pool::acquire
        void release (T* t)
        {
          boost::mutex::scoped_lock lock (mutex_);
          // At least one T* is not free.
          assert (lastFree_ > 0);
          assert (Ts_[lastFree_-1] == NULL);
          Ts_[lastFree_-1] = t;
          lastFree_--;
          condVariable_.notify_one ();
        }

        /// Returns true is at least one object is not locked.
        bool available () const
        {
          return OneIsAvailable (*this) ();
        }

        std::size_t size () const
        {
          return Ts_.size();
        }

        /// Deletes all internal objects.
        void clear ()
        {
          boost::mutex::scoped_lock lock (mutex_);
          if (lastFree_ > 0)
            throw std::logic_error ("Cannot free pool when some objects are in use.");
          for (std::size_t i = 0; i < size(); ++i) delete Ts_[i];
          Ts_.resize(0);
        }

        /// Adds an object to the pool
        /// The pool takes ownership of the object
        void push_back (T* t)
        {
          boost::mutex::scoped_lock lock (mutex_);
          Ts_.push_back(t);
        }

        /// Add objects to the pool
        /// The pool takes ownership of the object
        template <class InputIterator>
        void push_back (InputIterator first, InputIterator last)
        {
          boost::mutex::scoped_lock lock (mutex_);
          Ts_.insert(Ts_.end(), first, last);
        }

        /// Destructor
        /// Calls Pool::clear()
        ~Pool ()
        {
          clear();
        }

        /// Constructor
        Pool () : lastFree_ (0) {}

      private:
        struct OneIsAvailable
        {
          const Pool& p;
          OneIsAvailable (const Pool& pool) : p (pool) {}
          bool operator() () { return p.lastFree_ < p.size(); }
        };

        boost::mutex mutex_;
        boost::condition_variable condVariable_;
        std::vector<T*> Ts_;
        std::size_t lastFree_;
    }; // class Pool
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_DEVICE_HH
