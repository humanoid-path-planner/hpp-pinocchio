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

# include <hpp/model/fwd.hh>
# include <hpp/model/joint.hh>

namespace hpp {
  namespace model {
    /// Definition of a robot gripper
    ///
    /// This class represent a robot gripper as a frame attached to the joint
    /// of the robot that holds the gripper.
    ///
    /// \image html figures/figure-gripper.png
    ///
    /// To graps a box-shaped object with small lengths along x and y, the
    /// gripper frame should coincide with the object frame.
    class HPP_MODEL_DLLAPI Gripper
    {
      public:
        /// Return a shared pointer to new instance
        /// \param joint joint of the robot that will hold handles,
        /// \param objectPositionInJoint object position in the the grasping
        ///        joint.
        static GripperPtr_t create (const std::string& name,
            const DevicePtr_t& device);
        {
          Gripper* ptr = new Gripper (name, device);
          GripperPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

        /// Get joint that grip
        const JointPtr_t& joint () const
        {
          return joint_;
        }

        /// Get handle position in the the Grippering joint
        const Transform3f& objectPositionInJoint () const
        {
          return device_->model()->getFramePlacement(fid_);
        }
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

        virtual std::ostream& print (std::ostream& os) const;

      protected:
        /// Constructor
        /// \param name of the gripper in the device,
        /// \param device
        /// \todo device should be of type DeviceConstPtr_t but the constructor of
        /// JointPtr_t needs a DevicePtr_t.
        Gripper (const std::string& name, const DevicePtr_t& device) :
          name_ (name),
          device_ (device),
          clearance_ (0)
        {
          fid_ = device->model()->getFrameId (name);
          joint_ = JointPtr_t (
              new Joint(device, device->model()->getFrameParent (fid)));
        }

        void init (GripperWkPtr_t weakPtr)
        {
          weakPtr_ = weakPtr;
        }

      private:
        std::string name_;
        DeviceConstPtr_t device_;
        /// Joint of the robot that holds handles.
        JointPtr_t joint_;
        se3::FrameIndex fid_;
        /// Clearance
        value_type clearance_;
        /// Weak pointer to itself
        GripperWkPtr_t weakPtr_;
    }; // class Gripper
    std::ostream& operator<< (std::ostream& os, const Gripper& gripper);
  } // namespace model
} // namespace hpp

#endif // HPP_PINOCCHIO_GRIPPER_HH
