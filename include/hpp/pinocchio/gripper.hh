//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel from Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_GRIPPER_HH
# define HPP_PINOCCHIO_GRIPPER_HH

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>

namespace hpp {
  namespace pinocchio {
    /// Definition of a robot gripper
    ///
    /// This class represent a robot gripper as a frame attached to the joint
    /// of the robot that holds the gripper.
    ///
    /// \image html figures/figure-gripper.png
    ///
    /// To graps a box-shaped object with small lengths along x and y, the
    /// gripper frame should coincide with the object frame.
    class HPP_PINOCCHIO_DLLAPI Gripper
    {
      public:
        /// Return a shared pointer to new instance
        /// \param name of the gripper in the device,
        /// \param device
        static GripperPtr_t create (const std::string& name,
            const DeviceWkPtr_t& device)
        {
          Gripper* ptr = new Gripper (name, device);
          GripperPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

        static GripperPtr_t createCopy (const GripperPtr_t& gripper,
            const DeviceWkPtr_t& otherDevice)
        {
          Gripper* ptr = new Gripper (gripper->name(), otherDevice);
          ptr->clearance(gripper->clearance());
          GripperPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

        /// Get joint to which the gripper is attached
        const JointPtr_t& joint () const
        {
          return joint_;
        }

        /// Get the frame Id of the gripper in the vector of frame of the Model
        const FrameIndex& frameId () const
        {
          return fid_;
        }

        /// Get handle position in the the Grippering joint
        const Transform3f& objectPositionInJoint () const;

        ///get name
        const std::string& name () const
        {
          return name_;
        }

        /// Get the clearance
        ///
        /// The clearance is a distance, from the center of the gripper and along
        /// the x-aixs, that "ensures" an object being at that distance is not
        /// colliding with this gripper.
        /// It also gives an order of magnitude of the size of the gripper.
        value_type clearance () const
        {
          return clearance_;
        }

        /// Set the clearance
        /// \sa clearance()
        void clearance (const value_type& clearance)
        {
          clearance_ = clearance;
        }

        GripperPtr_t clone () const;

        std::ostream& print (std::ostream& os) const;

      protected:
        /// Constructor
        /// \param name of the gripper in the device,
        /// \param device
        /// \todo device should be of type DeviceConstPtr_t but the constructor of
        /// JointPtr_t needs a DevicePtr_t.
        Gripper (const std::string& name, const DeviceWkPtr_t& device);

        void init (GripperWkPtr_t weakPtr)
        {
          weakPtr_ = weakPtr;
        }

      private:
        inline DevicePtr_t device() const;

        std::string name_;
        DeviceWkPtr_t device_;
        /// Joint of the robot that holds handles.
        JointPtr_t joint_;
        FrameIndex fid_;
        /// Clearance
        value_type clearance_;
        /// Weak pointer to itself
        GripperWkPtr_t weakPtr_;
    }; // class Gripper

    inline std::ostream& operator<< (std::ostream& os, const Gripper& gripper)
    {
      return gripper.print (os);
    }
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_GRIPPER_HH
