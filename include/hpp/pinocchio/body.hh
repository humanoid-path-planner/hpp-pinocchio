//
// Copyright (c) 2016 CNRS
// Author: NMansard from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_BODY_HH
# define HPP_PINOCCHIO_BODY_HH

# include <hpp/util/pointer.hh>

# include <pinocchio/multibody/fwd.hpp>

# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {

    /// Geometry associated to a Joint
    ///
    /// A body is a geometry container attached to a joint.
    /// The body contains objects (CollisionObject) that move with the joint and
    /// called <em>inner objects</em>.
    ///
    /// Collision and distance computation is performed against other objects
    /// that can be obstacles or objects attached to other joints. These object
    /// are called <em>outer objects</em> for the body.
    class HPP_PINOCCHIO_DLLAPI Body
    {
    public:
      /// \name Construction and copy and destruction
      /// @{
      /// Constructor
      Body (DeviceWkPtr_t device, JointIndex joint);

      virtual ~Body () {}
      /// @}

      /// \name Name
      /// \{

      /// Get name
      const std::string & name () const;
      /// \}

      /// Get joint holding the body
      JointPtr_t joint () const;

      /// \name Inner/outer objects
      /// \{

      /// Number of inner objects.
      size_type nbInnerObjects () const;

      /// Access to an inner object.
      CollisionObjectPtr_t innerObjectAt (const size_type& i) const;

      /// Get radius
      ///
      /// Radius is defined as an upper-bound to the distance of all points of
      /// the body to the origin of the joint that holds the body.
      value_type radius () const;

      /// Number of outer objects.
      size_type nbOuterObjects () const;

      /// Access to an outer object.
      CollisionObjectPtr_t outerObjectAt (const size_type& i) const;
      /// \}

      /// \name Inertial information
      /// @{
      /// Get position of center of mass in joint local reference frame.
      const vector3_t& localCenterOfMass () const;
      /// Get Intertia matrix expressed in joint local reference frame.
      matrix3_t inertiaMatrix() const;
      /// Get mass.
      value_type mass() const;

      ///  @}
    private:
      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;
      /// If frameIndex==-1 (after init), search in pinocchio frame list the proper index.
      void searchFrameIndex() const; 

      const Model&  model() const ;
      Model&        model() ;
      const ::pinocchio::Frame & frame() const ;
      ::pinocchio::Frame &       frame() ;

      DeviceWkPtr_t devicePtr;
      JointIndex jointIndex;
      mutable FrameIndex frameIndex; // In pinocchio, bodies are stored as frames of type BODY.
      mutable bool       frameIndexSet;
    }; // class Body
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_BODY_HH
