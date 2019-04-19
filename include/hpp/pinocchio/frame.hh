//
// Copyright (c) 2017 CNRS
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

      const std::vector<FrameIndex>& children () const
      {
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
