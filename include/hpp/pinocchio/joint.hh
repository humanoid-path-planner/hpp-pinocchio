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

#ifndef HPP_PINOCCHIO_JOINT_HH
# define HPP_PINOCCHIO_JOINT_HH

# include <cstddef>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>

namespace hpp {
  namespace pinocchio {
    /// Robot joint
    ///
    /// A joint maps an input vector to a transformation of SE(3) from the
    /// parent frame to the joint frame.
    ///
    /// The input vector is provided through the configuration vector of the
    /// robot the joint belongs to. The joint input vector is composed of the
    /// components of the robot configuration starting at
    /// Joint::rankInConfiguration.
    ///
    /// The joint input vector represents a element of a Lie group, either
    /// \li a vector space for JointTranslation, and bounded JointRotation,
    /// \li the unit circle for non-bounded JointRotation joints,
    /// \li an element of SO(3) for JointSO3, represented by a unit quaternion.
    ///
    /// Operations specific to joints (uniform sampling of input space, straight
    /// interpolation, distance, ...) are performed by a JointConfiguration
    /// instance that has the same class hierarchy as Joint.
    class HPP_PINOCCHIO_DLLAPI Joint {
    public:
      /// \name Construction and copy and destruction
      /// \{

      /// Create a new joint
      /// Returns a null pointer if indexInJointList is 0.
      static JointPtr_t create (DeviceWkPtr_t device, JointIndex indexInJointList );

      /// Constructor
      /// \param device pointer on the device the joint is belonging to.
      /// \param indexInJointList index of the joint, i.e. joint = device.model.joints[index]
      Joint (DeviceWkPtr_t device, JointIndex indexInJointList );

      ~Joint() {}
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

      /// Joint transformation
      const Transform3f& currentTransformation () const { return currentTransformation(data()); }

      /// Joint transformation
      const Transform3f& currentTransformation (const DeviceData& data) const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Size and rank
      ///\{

      /// Return number of degrees of freedom
      size_type numberDof () const;

      /// Return number of degrees of freedom
      size_type configSize () const;

      /// Return rank of the joint in the configuration vector
      size_type rankInConfiguration () const;

      /// Return rank of the joint in the velocity vector
      size_type rankInVelocity () const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Kinematic chain
      /// \{

      /// Get a pointer to the parent joint (if any).
      JointPtr_t parentJoint () const;

      /// Number of child joints
      std::size_t numberChildJoints () const;

      /// Get child joint
      JointPtr_t childJoint (std::size_t rank) const;

      /// Get (constant) placement of joint in parent frame, i.e. model.jointPlacement[idx]
      const Transform3f& positionInParentFrame () const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Bounds
      /// \{

      /// Set whether given degree of freedom is bounded
      /// \warning Joint always has bounds. When `bounded == false`,
      /// the bounds are `-infinity` and `infinity`.
      void isBounded (size_type rank, bool bounded);
      /// Get whether given degree of freedom is bounded
      bool isBounded (size_type rank) const;
      /// Get lower bound of given degree of freedom
      value_type lowerBound (size_type rank) const;
      /// Get upper bound of given degree of freedom
      value_type upperBound (size_type rank) const;
      /// Set lower bound of given degree of freedom
      void lowerBound (size_type rank, value_type lowerBound);
      /// Set upper bound of given degree of freedom
      void upperBound (size_type rank, value_type upperBound);
      /// Set lower bounds
      void lowerBounds (vectorIn_t lowerBounds);
      /// Set upper bounds
      void upperBounds (vectorIn_t upperBounds);

      /// Get upper bound on linear velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\mathbf {v}\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\mathbf{v} = J(\mathbf{q})*\mathbf{\dot{q}} \f$ is the linear velocity of the joint frame.
      value_type upperBoundLinearVelocity () const;

      /// Get upper bound on angular velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\omega\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\omega = J(\mathbf{q})*\mathbf{\dot{q}}\f$ is the angular velocity of the joint frame.
      value_type upperBoundAngularVelocity () const;

      /// Maximal distance of joint origin to parent origin
      const value_type& maximalDistanceToParent () const { return maximalDistanceToParent_; }

      /// \}
    protected:
      /// Compute the maximal distance. \sa maximalDistanceToParent
      void computeMaximalDistanceToParent ();
    public:
      // -----------------------------------------------------------------------
      /// \name Jacobian
      /// \{

      /// Get non const reference to Jacobian
      /// \param localFrame if true, compute the jacobian (6d) in the local frame, 
      /// whose linear part corresponds to the velocity of the center of the frame.
      /// If false, the jacobian is expressed in the global frame and its linear part
      /// corresponds to the value of the velocity vector field at the center of the world.
      JointJacobian_t& jacobian (const bool localFrame=true) const { return jacobian (data(), localFrame); }

      /// Get reference to Jacobian
      /// \param localFrame if true, compute the jacobian (6d) in the local frame, 
      /// whose linear part corresponds to the velocity of the center of the frame.
      /// If false, the jacobian is expressed in the global frame and its linear part
      /// corresponds to the value of the velocity vector field at the center of the world.
      JointJacobian_t& jacobian (DeviceData& data, const bool localFrame=true) const;

      /// \}
      // -----------------------------------------------------------------------

      /// Access robot owning the object
      DeviceConstPtr_t robot () const { return devicePtr.lock();}
      /// Access robot owning the object
      DevicePtr_t robot () { return devicePtr.lock();}

      /// \name Body linked to the joint
      /// \{

      /// Get linked body
      BodyPtr_t linkedBody () const;

      /// \}

      /// Display joint
      std::ostream& display (std::ostream& os) const;

      /// Get configuration space of joint
      LiegroupSpacePtr_t configurationSpace () const;

      /// Get configuration space of joint.
      /// Use R^n x SO(n) instead of SE(n).
      LiegroupSpacePtr_t RnxSOnConfigurationSpace () const;

      /// \name Pinocchio API
      /// \{

      const JointIndex& index () const
      {
        return jointIndex;
      }

      const JointModel& jointModel() const;

      /// \}

    protected:
      value_type maximalDistanceToParent_;
      DeviceWkPtr_t devicePtr;
      JointIndex jointIndex;
      std::vector<JointIndex> children;

      /// Store list of childrens.
      void setChildList();
      Model&        model() ;      
      const Model&  model() const ;
      DeviceData& data() const;

      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;

      friend class Device;
    }; // class Joint

    inline std::ostream& operator<< (std::ostream& os, const Joint& joint) { return joint.display(os); }

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_JOINT_HH
