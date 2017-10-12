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
      Frame (DevicePtr_t device, FrameIndex indexInFrameList );

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

      /// Get const reference to Jacobian
      /// \param localFrame if true, compute the jacobian (6d) in the local frame, 
      /// whose linear part corresponds to the velocity of the center of the frame.
      /// If false, the jacobian is expressed in the global frame and its linear part
      /// corresponds to the value of the velocity vector field at the center of the world.
      JointJacobian_t jacobian (const bool localFrame=true) const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Kinematic chain
      /// \{

      /// Returns true if the frame type is se3::FIXED_JOINT
      bool isFixed () const;

      /// Returns the joint associated to this frame
      // /// \throws std::logic_error is isFixed() returns true.
      Joint joint () const;

      /// Get the parent frame (if any).
      /// \warning the parent joint of the universe is the universe itself.
      Frame parentFrame () const;

      /// Returns true if this frame is the universe frame.
      bool isRootFrame () const;

      /// Number of child frames
      // std::size_t numberChildFrames () const;

      /// Get child frame
      // FramePtr_t childFrame (std::size_t rank) const;

      const std::vector<FrameIndex>& children () const
      {
        return children_;
      }

      /// Get (constant) placement of frame in parent frame
      Transform3f positionInParentFrame () const;

      /// Set position of frame in parent frame
      void positionInParentFrame (const Transform3f& p);
      ///\}

    public:

      // -----------------------------------------------------------------------

      /// Access robot owning the object
      DeviceConstPtr_t robot () const { selfAssert();  return devicePtr_;}
      /// Access robot owning the object
      DevicePtr_t robot () { selfAssert(); return devicePtr_;}

      /*
      /// \name Body linked to the frame
      /// \{
      /// Get linked body
      BodyPtr_t linkedBody () const;
      /// \}
      */

      /// Display frame
      virtual std::ostream& display (std::ostream& os) const;

      /// \name Pinocchio API
      /// \{

      const FrameIndex& index () const
      {
        return frameIndex_;
      }

      const se3::Frame& pinocchio() const;

      /// \}

    private:
      DevicePtr_t devicePtr_;
      FrameIndex frameIndex_;
      std::vector<FrameIndex> children_;

      se3::Frame& pinocchio();

      /// Store list of childrens.
      void setChildList();
      Model&        model() ;
      const Model&  model() const ;
      Data &        data()  ;
      const Data &  data()  const ;

      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;
    }; // class Frame

    inline std::ostream& operator<< (std::ostream& os, const Frame& frame) { return frame.display(os); }

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_FRAME_HH
