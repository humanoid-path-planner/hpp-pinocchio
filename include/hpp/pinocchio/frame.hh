//
// Copyright (c) 2017 CNRS
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

#ifndef HPP_PINOCCHIO_FRAME_HH
# define HPP_PINOCCHIO_FRAME_HH

# include <cstddef>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/deprecated.hh>

namespace hpp {
  namespace pinocchio {
    /// Robot frame
    class HPP_PINOCCHIO_DLLAPI Frame {
    public:
      /// \name Construction and copy and destruction
      /// \{

      /// Constructor
      /// \param device pointer on the device the frame is belonging to.
      /// \param indexInFrameList index of the frame, i.e. frame = device.model.frames[index]
      Frame (DeviceWkPtr_t device, FrameIndex indexInFrameList );

      ~Frame() {}
      /// \}
      // -----------------------------------------------------------------------
      /// \name Name
      /// \{

      /// Get name
      const std::string& name() const;

      /// \}
      // -----------------------------------------------------------------------
      /// \name Position
      /// \{

      /// Frame transformation
      Transform3f currentTransformation () const;

      /// Frame transformation
      Transform3f currentTransformation (const DeviceData& data) const;

      /// Get const reference to Jacobian
      ///
      /// The jacobian (6d) is expressed in the local frame.
      /// the linear part corresponds to the velocity of the center of the frame.
      JointJacobian_t jacobian () const { return jacobian (data()); }

      JointJacobian_t jacobian (const DeviceData& data) const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Kinematic chain
      /// \{

      /// Returns true if the frame type is ::pinocchio::FIXED_JOINT
      bool isFixed () const;

      /// Returns the joint associated to this frame
      JointPtr_t joint () const;

      /// Get the parent frame (if any).
      /// \warning the parent joint of the universe is the universe itself.
      Frame parentFrame () const;

      /// Returns true if this frame is the universe frame.
      bool isRootFrame () const;

      const std::vector<FrameIndex>& children ()
      {
        setChildList();
        return children_;
      }

      /// Get (constant) placement of frame in parent joint
      const Transform3f& positionInParentJoint () const;

      /// Get (constant) placement of frame in parent frame
      Transform3f positionInParentFrame () const;

      /// Set position of frame in parent frame
      void positionInParentFrame (const Transform3f& p);
      ///\}

    public:

      // -----------------------------------------------------------------------

      /// Access robot owning the object
      DeviceConstPtr_t robot () const { selfAssert();  return devicePtr_.lock();}
      /// Access robot owning the object
      DevicePtr_t robot () { selfAssert(); return devicePtr_.lock();}

      /// Display frame
      virtual std::ostream& display (std::ostream& os) const;

      /// \name Pinocchio API
      /// \{

      const FrameIndex& index () const
      {
        return frameIndex_;
      }

      const ::pinocchio::Frame& pinocchio() const;

      /// \}

    private:
      DeviceWkPtr_t devicePtr_;
      FrameIndex frameIndex_;
      std::vector<FrameIndex> children_;

      ::pinocchio::Frame& pinocchio();

      /// Store list of childrens.
      void setChildList();
      Model&        model() ;
      const Model&  model() const ;
      DeviceData& data() const;

      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;
    }; // class Frame

    inline std::ostream& operator<< (std::ostream& os, const Frame& frame) { return frame.display(os); }

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_FRAME_HH
