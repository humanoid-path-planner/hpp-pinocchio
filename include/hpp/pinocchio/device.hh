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

#ifndef HPP_PINOCCHIO_DEVICE_HH
#define HPP_PINOCCHIO_DEVICE_HH

# include <iostream>
# include <vector>
# include <list>

# include <hpp/util/debug.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/config.hh>
//# include <hpp/pinocchio/distance-result.hh>
# include <hpp/pinocchio/collision-object.hh>
# include <hpp/pinocchio/extra-config-space.hh>
# include <hpp/pinocchio/joint.hh>
# include <hpp/pinocchio/device-object-vector.hh>
# include <pinocchio/multibody/model.hpp>

namespace hpp {
  namespace pinocchio {
    /// \brief Robot with geometric and dynamic pinocchio

    /// The creation of the device is done by Device::create(const
    /// std::string name).  This function returns a shared pointer
    /// to the newly created object.  \sa Smart pointers
    /// documentation:
    /// http://www.boost.org/libs/smart_ptr/smart_ptr.htm
    class HPP_PINOCCHIO_DLLAPI Device
    {
//NOTYET      friend class Body;
      friend class Joint;
    public:
      /// Flags to select computation
      /// To optimize computation time, computations performed by method
      /// computeForwardKinematics can be selected by calling method
      /// controlComputation.
      enum Computation_t {
	JOINT_POSITION = 0x0,
	JACOBIAN = 0x1,
	VELOCITY = 0x2,
	ACCELERATION = 0x4,
	COM = 0x8,
	ALL = 0Xffff
      };

      /// Collision pairs between bodies
      typedef std::pair <JointPtr_t, JointPtr_t> CollisionPair_t;
      typedef std::list <CollisionPair_t> CollisionPairs_t;

      // -----------------------------------------------------------------------
      /// \name Construction, copy and destruction
      /// \{
      virtual ~Device() {};

      /// \brief Clone as a CkwsDevice
      /// The pinocchio model is not copied (only copy the pointer).
      /// A new Pinocchio "data" is created.
      /// As the model is not copied, cloning is a non constant operation. \sa cloneConst
      DevicePtr_t clone() { return createCopy(weakPtr_.lock()); }
      /// \brief Clone as a CkwsDevice
      /// Both pinocchio objects model and data are copied.
      /// TODO: this method is not implemented yet (assert if called)
      DevicePtr_t cloneConst() const { return createCopyConst(weakPtr_.lock()); }
 
      /// Get name of device
      const std::string& name () const {return name_;}

      /// \brief Creation of a new device
      /// \return a shared pointer to the new device
      /// \param name Name of the device
      static DevicePtr_t create (const std::string & name);

      /// \brief Copy of a device
      /// \return A shared pointer to new device.
      /// \param device Device to be copied.
      /// The pinocchio model is not copied (only copy the pointer).
      /// A new Pinocchio "data" is created.
      static DevicePtr_t createCopy (const DevicePtr_t& device);
      static DevicePtr_t createCopyConst (const DeviceConstPtr_t& device);

      /// Set pinocchio model.
      void model( ModelPtr_t modelPtr ) { model_ = modelPtr; }
      /// Access to pinocchio model
      ModelConstPtr_t model() const { return model_; }
      /// Access to pinocchio model
      ModelPtr_t model() { return model_; }

      /// Set pinocchio geom.
      void geomModel( GeomModelPtr_t geomModelPtr ) { geomModel_ = geomModelPtr; }
      /// Access to pinocchio geomModel
      GeomModelConstPtr_t geomModel() const { return geomModel_; }
      /// Access to pinocchio geomModel
      GeomModelPtr_t geomModel() { return geomModel_; }

      /// Set Pinocchio data corresponding to model
      void data( DataPtr_t dataPtr ) { data_ = dataPtr; resizeState(); }
      /// Access to Pinocchio data/
      DataConstPtr_t data() const { return data_; }
      /// Access to Pinocchio data/
      DataPtr_t data() { return data_; }
      /// Create Pinocchio data from model.
      void createData();

      /// Set Pinocchio geomData corresponding to model
      void geomData( GeomDataPtr_t geomDataPtr ) { geomData_ = geomDataPtr; resizeState(); }
      /// Access to Pinocchio geomData/
      GeomDataConstPtr_t geomData() const { return geomData_; }
      /// Access to Pinocchio geomData/
      GeomDataPtr_t geomData() { return geomData_; }
      /// Create Pinocchio geomData from model.
      void createGeomData();

      /// \}
      // -----------------------------------------------------------------------
      /// \name Joints
      /// \{

      //DEPREC /// Set the root of the kinematic chain
      //DEPREC virtual void rootJoint (JointPtr_t joint);
      //DEPREC /// Set position of root joint in world frame
      //DEPREC void rootJointPosition (const Transform3f& position);

      /// Get root joint
      JointPtr_t rootJoint () const;

      //DEPREC /// Register joint in internal containers
      //DEPREC void registerJoint (const JointPtr_t& joint);
      /// Get vector of joints
      inline const JointVector& getJointVector () const { return jointVector_; }

      /// Get the joint at configuration rank r
      JointPtr_t getJointAtConfigRank (const size_type& r) const;

      /// Get the joint at velocity rank r
      JointPtr_t getJointAtVelocityRank (const size_type& r) const;

      /// Get joint by name
      /// \param name name of the joint.
      /// \throw runtime_error if device has no joint with this name
      JointPtr_t getJointByName (const std::string& name) const;

      /// Get joint by body name
      /// \throw runtime_error if device has no body with this name
      JointPtr_t getJointByBodyName (const std::string& name) const;

      /// Size of configuration vectors
      /// Sum of joint dimensions and of extra configuration space dimension
      size_type configSize () const;

      /// Size of velocity vectors
      /// Sum of joint number of degrees of freedom and of extra configuration
      /// space dimension
      size_type numberDof () const;

      /// \}
      // -----------------------------------------------------------------------
      /// \name Extra configuration space
      /// \{

      /// Get degrees of freedom to store internal values in configurations
      ///
      /// In some applications, it is useful to store extra variables with
      /// the configuration vector. For instance, when planning motions in
      /// state space using roadmap based methods, the velocity of the robot
      /// is stored in the nodes of the roadmap.
      ExtraConfigSpace& extraConfigSpace () { return extraConfigSpace_; }

      /// Get degrees of freedom to store internal values in configurations
      ///
      /// In some applications, it is useful to store extra variables with
      /// the configuration vector. For instance, when planning motions in
      /// state space using roadmap based methods, the velocity of the robot
      /// is stored in the nodes of the roadmap.
      const ExtraConfigSpace& extraConfigSpace () const { return extraConfigSpace_; }

      /// Set dimension of extra configuration space
      virtual void setDimensionExtraConfigSpace (const size_type& dimension)
      {
	extraConfigSpace_.setDimension (dimension);
	resizeState ();
      }

      /// \}
      // -----------------------------------------------------------------------
      /// \name Current state
      /// \{

      /// Get current configuration
      const Configuration_t& currentConfiguration () const
      {
	return currentConfiguration_;
      }
      /// Set current configuration
      /// \return True if the current configuration was modified and false if
      ///         the current configuration did not change.
      virtual bool currentConfiguration (ConfigurationIn_t configuration);

      /// Get the neutral configuration
      const Configuration_t & neutralConfiguration () const;

      /// Get current velocity
      const vector_t& currentVelocity () const
      {
	return currentVelocity_;
      }

      /// Set current velocity
      void currentVelocity (vectorIn_t velocity)
      {
	upToDate_ = false;
	currentVelocity_ = velocity;
      }

      /// Get current acceleration
      const vector_t& currentAcceleration () const
      {
	return currentAcceleration_;
      }

      /// Set current acceleration
      void currentAcceleration (vectorIn_t acceleration)
      {
	upToDate_ = false;
	currentAcceleration_ = acceleration;
      }
      /// \}
      // -----------------------------------------------------------------------
      /// \name Mass and center of mass
      /// \{

      /// Get mass of robot
      const value_type& mass () const;

      /// Get position of center of mass
      const vector3_t& positionCenterOfMass () const;

      /// Get Jacobian of center of mass with respect to configuration
      const ComJacobian_t& jacobianCenterOfMass () const;

      /// \}
      // -----------------------------------------------------------------------
      /// \name Grippers
      /// \{

      /// Add a gripper to the Device
      void addGripper (const GripperPtr_t& gripper)
      {
	grippers_.push_back (gripper);
      }

      /// Return list of grippers of the Device
      Grippers_t& grippers ()
      {
	return grippers_;
      }

      /// Return list of grippers of the Device
      const Grippers_t& grippers () const
      {
	return grippers_;
      }

      /// \}
      // -----------------------------------------------------------------------
      /// \name Collision and distance computation
      /// \{

      /// Get list of obstacles
      const ObjectVector_t& obstacles () const
      {
        return obstacles_;
      }

      /// Get list of obstacles
      /// \deprecated Use Device::obstacles() const
      const ObjectVector_t& obstacles (Request_t /*type*/) const HPP_PINOCCHIO_DEPRECATED
      {
        return obstacles();
      }

      //DEPREC /// Add collision pairs between objects attached to two joints
      //DEPREC ///
      //DEPREC /// \param joint1 first joint
      //DEPREC /// \param joint2 second joint
      //DEPREC /// \param type collision or distance.
      //DEPREC ///
      //DEPREC /// Define collision pair between each object of joint 1 body and
      //DEPREC /// each object of joint2 body.
      //DEPREC virtual void addCollisionPairs (const JointPtr_t& joint1,
      //DEPREC 				      const JointPtr_t& joint2,
      //DEPREC 				      Request_t type);

      //DEPREC /// Remove collision pairs between objects attached to two joints
      //DEPREC ///
      //DEPREC /// \param joint1 first joint
      //DEPREC /// \param joint2 second joint
      //DEPREC /// \param type collision or distance.
      //DEPREC ///
      //DEPREC /// remove collision between each object of joint 1 body and
      //DEPREC /// each object of joint2 body
      //DEPREC virtual void removeCollisionPairs(const JointPtr_t& joint1,
      //DEPREC                                   const JointPtr_t& joint2,
      //DEPREC 				        Request_t type);

      /// Get list of collision or distance pairs
      /// \param type collision or distance.
//NOTYET      const CollisionPairs_t& collisionPairs (Request_t type) const;

      /// Iterator over inner objects of the device
      /// \param type Collision or distance
      DeviceObjectVector& objectVector () {return objectVector_; }
      const DeviceObjectVector& objectVector () const { return objectVector_; }

      /// Test collision of current configuration
      /// \param stopAtFirstCollision act as named
      /// \warning Users should call computeForwardKinematics first.
      bool collisionTest (const bool stopAtFirstCollision=true);

      /// Compute distances between pairs of objects stored in bodies
      /// \warning Users should call computeForwardKinematics first.
      void computeDistances ();

      /// Get result of distance computations
      const DistanceResults_t& distanceResults () const;
      /// \}
      // -----------------------------------------------------------------------
      /// \name Forward kinematics
      /// \{

      /// Select computation
      /// Optimize computation time by selecting only necessary values in
      /// method computeForwardKinematics.
      void controlComputation (const Computation_t& flag)
      {
	computationFlag_ = flag;
	upToDate_ = false;
      }
      /// Get computation flag
      Computation_t computationFlag () const
      {
	return computationFlag_;
      }
      /// Compute forward kinematics
      virtual void computeForwardKinematics ();
      /// \}
      // -----------------------------------------------------------------------

      /// Print object in a stream
      virtual std::ostream& print (std::ostream& os) const;

    protected:
      /// \brief Constructor
      Device(const std::string& name);

      //DEPREC /// \brief Initialization.
      //DEPREC ///
      //DEPREC void init(const DeviceWkPtr_t& weakPtr);
      //DEPREC /// \brief Initialization of of a clone device.
      //DEPREC ///
      //DEPREC void initCopy(const DeviceWkPtr_t& weakPtr, const Device& model);

      //DEPREC /// Recompute the number of distance pairs and resize the vector of distance results.
      //DEPREC void updateDistances ();

    private:
      /// \brief Copy Constructor
      Device(const Device& device);

    private:
      //DEPREC void computeJointPositions ();
      //DEPREC void computeJointJacobians ();
      //DEPREC void computeMass ();
      //DEPREC void computePositionCenterOfMass ();
      //DEPREC void computeJacobianCenterOfMass ();

      /// Resize configuration when changing data or extra-config.
      void resizeState ();
      void resizeJacobians ();

    protected:
      // Pinocchio objects
      ModelPtr_t model_; 
      DataPtr_t data_;
      GeomModelPtr_t geomModel_;
      GeomDataPtr_t geomData_;

      std::string name_;
//DEPREC      DistanceResults_t distances_;
      //DEPREC JointByName_t jointByName_;
      JointVector jointVector_; // fake container with iterator mimicking hpp::model::JointVector_t
      //DEPREC JointVector_t jointByConfigRank_;
      //DEPREC JointVector_t jointByVelocityRank_;
      //DEPREC JointPtr_t rootJoint_;
      //DEPREC size_type numberDof_;
      //DEPREC size_type configSize_;
      Configuration_t currentConfiguration_;
      vector_t currentVelocity_;
      vector_t currentAcceleration_;
      //DEPREC vector3_t com_;
      //DEPREC ComJacobian_t jacobianCom_;
      //DEPREC value_type mass_;
      bool upToDate_;
      Computation_t computationFlag_;
//NOTYET      // Collision pairs between bodies
//NOTYET      CollisionPairs_t collisionPairs_;
//NOTYET      CollisionPairs_t distancePairs_;
      //DEPREC // Obstacles
      ObjectVector_t obstacles_;
      //DEPREC ObjectVector_t collisionObstacles_;
      //DEPREC ObjectVector_t distanceObstacles_;
      DeviceObjectVector objectVector_;
      // Grippers
      Grippers_t grippers_;
      // Extra configuration space
      ExtraConfigSpace extraConfigSpace_;
      DeviceWkPtr_t weakPtr_;

    }; // class Device

    std::ostream& operator<< (std::ostream& os, const hpp::pinocchio::Device& device);
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_DEVICE_HH
