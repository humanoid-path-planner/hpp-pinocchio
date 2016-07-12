//
// Copyright (c) 2016 CNRS
// Author: NMansard from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_FAKECONTAINER_HH
# define HPP_PINOCCHIO_FAKECONTAINER_HH

# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {

    template< typename T_,typename Tconst_ >
    struct HPP_PINOCCHIO_DLLAPI FakeContainer
    {
      typedef T_ T;
      typedef Tconst_ Tconst;

      virtual ~FakeContainer() {}
      virtual T at(const size_type) = 0;
      virtual Tconst at(const size_type) const = 0;
      virtual size_type size() const = 0;
      virtual size_type ibegin() const { return 0; }
      virtual size_type iend  () const { return size(); }

      DevicePtr_t device;
      FakeContainer( DevicePtr_t device ) : device(device) {}
      FakeContainer() : device() {}

      struct iterator
      {
        FakeContainer & ref;
        size_type idx;
        
        iterator(FakeContainer & ref,size_type idx) : ref(ref),idx(idx) {}

        iterator&  operator++ ()    { ++idx; return *this; }
        iterator   operator++ (int) { iterator copy = *this; idx++; return copy; }
        iterator&  operator-- ()    { --idx; return *this; }
        iterator   operator-- (int) { iterator copy = *this; idx--; return copy; }
        T          operator*  ()    { return ref.at(idx); }
        bool       operator== (const iterator & i2) { return idx==i2.idx; }
        bool       operator!= (const iterator & i2) { return idx!=i2.idx; }; 
      };

      struct const_iterator
      {
        const FakeContainer & ref;
        size_type idx;
        
        const_iterator(const FakeContainer & ref,size_type idx) : ref(ref),idx(idx) {}
        const_iterator(const iterator & it) : ref(it.ref),idx(it.idx) {}

        const_iterator&  operator++ ()    { ++idx; return *this; }
        const_iterator   operator++ (int) { const_iterator copy = *this; idx++; return copy; }
        const_iterator&  operator-- ()    { --idx; return *this; }
        const_iterator   operator-- (int) { const_iterator copy = *this; idx--; return copy; }
        Tconst           operator*  ()    { return ref.at(idx); }
        bool             operator== (const const_iterator & i2){ return idx==i2.idx; }
        bool             operator!= (const const_iterator & i2){ return idx!=i2.idx; }
      };

      iterator        begin()       { assert(device); return iterator      (*this,ibegin()  ); }
      iterator        end()         { assert(device); return iterator      (*this,iend()    ); }
      iterator       rbegin()       { assert(device); return iterator      (*this,iend()-1  ); }
      iterator       rend()         { assert(device); return iterator      (*this,ibegin()-1); }

      const_iterator  begin() const { assert(device); return const_iterator(*this,ibegin()  ); }
      const_iterator  end()   const { assert(device); return const_iterator(*this,iend()    ); }
      const_iterator rbegin() const { assert(device); return const_iterator(*this,iend()-1  ); }
      const_iterator rend()   const { assert(device); return const_iterator(*this,ibegin()-1); }

      T               operator[](const int idx)       { return at(idx); }
      Tconst          operator[](const int idx) const { return at(idx); }
    };

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_FAKECONTAINER_HH
