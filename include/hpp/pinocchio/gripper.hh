//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel from Florent Lamiraux
//
//
// This file is part of hpp-pinocchio.
// hpp-pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-pinocchio. If not, see
// <http://www.gnu.org/licenses/>.

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
