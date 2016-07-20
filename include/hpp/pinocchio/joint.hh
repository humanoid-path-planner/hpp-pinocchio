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
# include <hpp/fcl/math/transform.h>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/fake-container.hh>

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
      typedef se3::Index Index;

      /// \name Construction and copy and destruction
      /// \{

      /// Constructor
      /// \param device pointer on the device the joint is belonging to.
      /// \param indexInJointList index of the joint, i.e. joint = device.model.joints[index]
      Joint (DevicePtr_t device, Index indexInJointList );

      //DEPREC /// Constructor
      //DEPREC /// \param initialPosition position of the joint before being inserted
      //DEPREC ///        in a kinematic chain,
      //DEPREC /// \param configSize dimension of the configuration vector,
      //DEPREC /// \param numberDof dimension of the velocity vector.
      //DEPREC Joint (const Transform3f& initialPosition, size_type configSize,
      //DEPREC	     size_type numberDof);
      //DEPREC /// Copy constructor
      //DEPREC ///
      //DEPREC /// Clone body and therefore inner and outer objects (see Body::clone).
      //DEPREC Joint (const Joint& joint);
      //DEPREC /// Return pointer to copy of this
      //DEPREC ///
      //DEPREC /// Clone body and therefore inner and outer objects (see Body::clone).
      //DEPREC  virtual JointPtr_t clone () const = 0;
      //DEPREC virtual ~Joint ();

      ~Joint() {}
      /// \}
      // -----------------------------------------------------------------------
      /// \name Name
      /// \{

      //DEPREC /// Set name
      //DEPREC virtual inline void name(const std::string& name)

      /// Get name
      const std::string& name() const;

      /// \}
      // -----------------------------------------------------------------------
      /// \name Position
      /// \{

      //DEPREC /// Joint initial position (when robot is in zero configuration)
      //DEPREC const Transform3f& initialPosition () const;

      /// Joint transformation
      const Transform3f& currentTransformation () const;

      //DEPREC /// Compute position of joint
      //DEPREC /// \param configuration the configuration of the robot,
      //DEPREC /// \param parentPosition position of parent joint,
      //DEPREC /// \retval position position of this joint.
      //DEPREC virtual void computePosition (ConfigurationIn_t configuration,
      //DEPREC 				    const Transform3f& parentPosition,
      //DEPREC 				    Transform3f& position) const = 0;
      //DEPREC /// Compute position of this joint and all its descendents.
      //DEPREC void recursiveComputePosition (ConfigurationIn_t configuration,
      //DEPREC                const Transform3f& parentPosition) const;
      //DEPREC /// Compute jacobian matrix of joint and all its descendents.
      //DEPREC void computeJacobian ();

      /// Get neutral configuration of joint
//NOTYET      vector_t neutralConfiguration () const;

      ///\}
      // -----------------------------------------------------------------------
      /// \name Size and rank
      ///\}

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
      // DEPREC JointPtr_t parentJoint () const

      //DEPREC /// Add child joint
      //DEPREC /// \param joint child joint added to this one,
      //DEPREC /// \param computePositionInParent whether to compute position of the
      //DEPREC ///        child joint in this one's frame.
      //DEPREC ///
      //DEPREC /// \note When building a kinematic chain, we usually build the
      //DEPREC /// joint in its initial position and compute the (constant)
      //DEPREC /// position of the joint in its parent when adding the joint in
      //DEPREC /// the kinematic chain. When copying a kinematic chain, we copy the
      //DEPREC /// position of the joint in its parent frame and therefore we do
      //DEPREC /// not update it when adding the joint in the kinematic chain.
      //DEPREC void addChildJoint (JointPtr_t joint,
      //DEPREC			  bool computePositionInParent = true);

      /// Number of child joints
      std::size_t numberChildJoints () const;

      /// Get child joint
      JointPtr_t childJoint (std::size_t rank) const;

      /// Get (constant) placement of joint in parent frame, i.e. model.jointPlacement[idx]
      const Transform3f& positionInParentFrame () const;

      //DEPREC /// Set position of joint in parent frame
      //DEPREC void positionInParentFrame (const Transform3f& p);
      ///\}
      // -----------------------------------------------------------------------
      /// \name Bounds
      /// \{

      //DEPREC /// Set whether given degree of freedom is bounded
      //DEPREC void isBounded (size_type rank, bool bounded);
      //DEPREC /// Get whether given degree of freedom is bounded
      //DEPREC bool isBounded (size_type rank) const;
      //DEPREC /// Get lower bound of given degree of freedom
      //DEPREC value_type lowerBound (size_type rank) const;
      //DEPREC /// Get upper bound of given degree of freedom
      //DEPREC value_type upperBound (size_type rank) const;
      //DEPREC /// Set lower bound of given degree of freedom
      //DEPREC void lowerBound (size_type rank, value_type lowerBound);
      //DEPREC /// Set upper bound of given degree of freedom
      //DEPREC void upperBound (size_type rank, value_type upperBound);

      /// Get upper bound on linear velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\mathbf {v}\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\mathbf{v} = J(\mathbf{q})*\mathbf{\dot{q}} \f$ is the linear velocity of the joint frame.
//NOTYET      value_type upperBoundLinearVelocity () const;

      /// Get upper bound on angular velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\omega\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\omega = J(\mathbf{q})*\mathbf{\dot{q}}\f$ is the angular velocity of the joint frame.
//NOTYET      value_type upperBoundAngularVelocity () const;

      /// Maximal distance of joint origin to parent origin
//NOTYET      const value_type& maximalDistanceToParent () const;
      /// Compute the maximal distance. \sa maximalDistanceToParent
//NOTYET      void computeMaximalDistanceToParent ();

      /// \}
      // -----------------------------------------------------------------------
      /// \name Jacobian
      /// \{

      /// Get const reference to Jacobian
      /// \param localFrame if true, compute the jacobian (6d) in the local frame, 
      /// whose linear part corresponds to the velocity of the center of the frame.
      /// If false, the jacobian is expressed in the global frame and its linear part
      /// corresponds to the value of the velocity vector field at the center of the world.
      const JointJacobian_t& jacobian (const bool localFrame=true) const;

      /// Get non const reference to Jacobian
      /// \param localFrame if true, compute the jacobian (6d) in the local frame, 
      /// whose linear part corresponds to the velocity of the center of the frame.
      /// If false, the jacobian is expressed in the global frame and its linear part
      /// corresponds to the value of the velocity vector field at the center of the world.
      JointJacobian_t& jacobian (const bool localFrame=true);

      /// \}
      // -----------------------------------------------------------------------

      //DEPREC /// Access to configuration space
      //DEPREC JointConfiguration* configuration () const {return configuration_;}
      //DEPREC /// Set robot owning the kinematic chain
      //DEPREC void robot (const DeviceWkPtr_t& device) {robot_ = device;}

      /// Access robot owning the object
      DeviceConstPtr_t robot () const { selfAssert();  return devicePtr;}
      /// Access robot owning the object
      DevicePtr_t robot () { selfAssert(); return devicePtr;}

      /// \name Body linked to the joint
      /// \{

      /// Get linked body
      BodyPtr_t linkedBody () const;

      //DEPREC /// Set linked body
      //DEPREC void setLinkedBody (const BodyPtr_t& body);

      /// \}

      //DEPREC /// \name Compatibility with urdf
      //DEPREC /// \{

      //DEPREC /// Get urdf link position in joint frame
      //DEPREC ///
      //DEPREC /// When parsing urdf pinocchios, joint frames are reoriented in order
      //DEPREC /// to rotate about their x-axis. For some applications, it is necessary
      //DEPREC /// to be able to recover the position of the urdf link attached to
      //DEPREC /// the joint.
      //DEPREC const Transform3f& linkInJointFrame () const;
      //DEPREC /// Set urdf link position in joint frame
      //DEPREC void linkInJointFrame (const Transform3f& transform);
      //DEPREC /// Get link name
      //DEPREC const std::string& linkName () const;
      //DEPREC /// Set link name
      //DEPREC void linkName (const std::string& linkName)

      //DEPREC /// \}

      /// Display joint
      virtual std::ostream& display (std::ostream& os) const;

      /// \name Pinocchio API
      /// \{

      const Index& index () const
      {
        return jointIndex;
      }

      /// \}

    protected:
      value_type maximalDistanceToParent_;
      vector_t neutralConfiguration_;
      DevicePtr_t devicePtr;
      mutable JointJacobian_t jacobian_;
      Index jointIndex;
      std::vector<Index> children;

      /// Store list of childrens.
      void setChildList();
      ModelPtr_t       model() ;      
      ModelConstPtr_t  model() const ;
      DataPtr_t        data()  ;      
      DataConstPtr_t   data()  const ;

      /// Assert that the members of the struct are valid (no null pointer, etc).
      void selfAssert() const;

      friend class Device;
    }; // class Joint

    inline std::ostream& operator<< (std::ostream& os, const Joint& joint) { return joint.display(os); }

    /** Fake std::vector<Joint>, used to comply with the actual structure of hpp::model.
     *
     * You can use it for the following loop:
     *       for (JointVector_t::const_iterator it = jv.begin (); 
     *               it != jv.end (); ++it) 
     *          cout << (*it)->name;
     */
    struct JointVector
      : public FakeContainer<JointPtr_t,JointConstPtr_t>
    {
      JointVector(DevicePtr_t device) : FakeContainer<JointPtr_t,JointConstPtr_t>(device) {}
      JointVector() {}
      virtual ~JointVector() {}

      virtual JointPtr_t at(const size_type i) ;
      virtual JointConstPtr_t at(const size_type i) const ;
      virtual size_type size() const ;
      virtual size_type iend() const ;

      void selfAssert(size_type i = 0) const;
    };

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_JOINT_HH
