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

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/frame.hh>
# include <hpp/pinocchio/config.hh>
# include <hpp/pinocchio/deprecated.hh>
# include <hpp/pinocchio/extra-config-space.hh>
# include <hpp/pinocchio/device-object-vector.hh>

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
      friend class Joint;
      friend class Frame;
    public:
      /// Flags to select computation
      /// To optimize computation time, computations performed by method
      /// computeForwardKinematics can be selected by calling method
      /// controlComputation.
      enum Computation_t {
	JOINT_POSITION = 0x1,
	JACOBIAN       = 0x2,
	VELOCITY       = 0x4,
	ACCELERATION   = 0x8,
	COM            = 0x10,
	ALL            = 0Xffff
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
      virtual DevicePtr_t clone() const { return createCopy(weakPtr_.lock()); }
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

      /// \}
      // -----------------------------------------------------------------------
      /// \name Access to pinocchio API
      /// \{

      /// Set pinocchio model.
      void model( ModelPtr_t modelPtr ) { model_ = modelPtr; }
      /// Access to pinocchio model
      ModelConstPtr_t   modelPtr() const { return model_; }
      /// Access to pinocchio model
      ModelPtr_t        modelPtr()       { return model_; }
      /// Access to pinocchio model
      const Model& model()    const { assert(model_); return *model_; }
      /// Access to pinocchio model
      Model&       model()          { assert(model_); return *model_; }

      /// Set pinocchio geom.
      void geomModel( GeomModelPtr_t geomModelPtr ) { geomModel_ = geomModelPtr; }
      /// Access to pinocchio geomModel
      GeomModelConstPtr_t        geomModelPtr() const { return geomModel_; }
      /// Access to pinocchio geomModel
      GeomModelPtr_t             geomModelPtr() { return geomModel_; }
      /// Access to pinocchio geomModel
      const GeomModel & geomModel() const { assert(geomModel_); return *geomModel_; }
      /// Access to pinocchio geomModel
      GeomModel &       geomModel() { assert(geomModel_); return *geomModel_; }

      /// Set Pinocchio data corresponding to model
      void data( DataPtr_t dataPtr ) { data_ = dataPtr; resizeState(); }
      /// Access to Pinocchio data/
      DataConstPtr_t    dataPtr() const { return data_; }
      /// Access to Pinocchio data/
      DataPtr_t         dataPtr() { return data_; }
      /// Access to Pinocchio data/
      const Data & data() const { assert(data_); return *data_; }
      /// Access to Pinocchio data/
      Data &       data() { assert(data_); return *data_; }
      /// Create Pinocchio data from model.
      void createData();

      /// Set Pinocchio geomData corresponding to model
      void geomData( GeomDataPtr_t geomDataPtr ) { geomData_ = geomDataPtr; resizeState(); }
      /// Access to Pinocchio geomData/
      GeomDataConstPtr_t       geomDataPtr() const { return geomData_; }
      /// Access to Pinocchio geomData/
      GeomDataPtr_t            geomDataPtr()       { return geomData_; }
      /// Access to Pinocchio geomData/
      const GeomData& geomData() const    { assert(geomData_); return *geomData_; }
      /// Access to Pinocchio geomData/
      GeomData&       geomData()          { assert(geomData_); return *geomData_; }
      /// Create Pinocchio geomData from model.
      void createGeomData();

      /// \}
      // -----------------------------------------------------------------------
      /// \name Joints
      /// \{

      /// Get root joint
      JointPtr_t rootJoint () const;

      /// Get root frame
      Frame rootFrame () const;

      /// Get vector of joints
      inline const JointVector& getJointVector () const HPP_PINOCCHIO_DEPRECATED { return jointVector_; }
      inline JointVector& getJointVector () HPP_PINOCCHIO_DEPRECATED { return jointVector_; }

      /// Get number of joints
      size_type nbJoints () const;

      /// Access i-th joint
      JointPtr_t jointAt (const size_type& i) const;

      /// Get the joint at configuration rank r
      /// \return  joint j such that j->rankInConfiguration () <=
      ///          r < j->rankInConfiguration () + j->configSize ()
      JointPtr_t getJointAtConfigRank (const size_type& r) const;

      /// Get the joint at velocity rank r
      /// \return  joint j such that j->rankInVelocity () <=
      ///          r < j->rankInVelocity () + j->numberDof ()
      JointPtr_t getJointAtVelocityRank (const size_type& r) const;

      /// Get joint by name
      /// \param name name of the joint.
      /// \throw runtime_error if device has no joint with this name
      JointPtr_t getJointByName (const std::string& name) const;

      /// Get joint by body name
      /// \throw runtime_error if device has no body with this name
      JointPtr_t getJointByBodyName (const std::string& name) const;

      /// Get frame by name
      /// \param name name of the frame.
      /// \throw runtime_error if device has no frame with this name
      Frame getFrameByName (const std::string& name) const;

      /// Size of configuration vectors
      /// Sum of joint dimensions and of extra configuration space dimension
      size_type configSize () const;

      /// Size of velocity vectors
      /// Sum of joint number of degrees of freedom and of extra configuration
      /// space dimension
      size_type numberDof () const;

      /// Returns a LiegroupSpace representing the configuration space.
      const LiegroupSpacePtr_t& configSpace () const { return configSpace_; }

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
      Configuration_t neutralConfiguration () const;

      /// Get current velocity
      const vector_t& currentVelocity () const
      {
	return currentVelocity_;
      }

      /// Set current velocity
      void currentVelocity (vectorIn_t velocity)
      {
        invalidate();
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
        invalidate();
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

      /// Get the static obstacles
      /// It corresponds to the geometries attached to the *universe* in
      /// pinocchio.
      BodyPtr_t obstacles () const;

      /// Vector of inner objects of the device
      /// \deprecated Use Device::nbObjects and Device::objectAt
      DeviceObjectVector& objectVector () HPP_PINOCCHIO_DEPRECATED {return objectVector_; }
      const DeviceObjectVector& objectVector () const HPP_PINOCCHIO_DEPRECATED { return objectVector_; }

      /// Number of objects
      size_type nbObjects () const;

      /// Access the i-th object.
      /// \sa Device::nbObjects() const
      CollisionObjectPtr_t objectAt (const size_type& i) const;

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
        invalidate();
      }
      /// Get computation flag
      Computation_t computationFlag () const
      {
	return computationFlag_;
      }
      /// Compute forward kinematics
      void computeForwardKinematics ();
      /// Compute frame forward kinematics
      void computeFramesForwardKinematics ();
      /// Update the geometry placement to the currentConfiguration
      void updateGeometryPlacements ();
      /// \}
      // -----------------------------------------------------------------------

      /// Print object in a stream
      virtual std::ostream& print (std::ostream& os) const;

      /// Compute an aligned bounding around the robot.
      /// The bounding box is computed as follows:
      /// - for each direct children of the universe joint:
      ///   - compute a bounding box of its center (using the bounds in translation)
      ///   - compute the maximal distance between the its center and
      ///     each bodies attach its subtree.
      ///   - sum the two.
      /// - sum all the BB obtained.
      fcl::AABB computeAABB() const;

    protected:
      /// \brief Constructor
      Device(const std::string& name);

      /// \brief Initialization.
      ///
      void init(const DeviceWkPtr_t& weakPtr);
      /// \brief Initialization of of a clone device.
      ///
      void initCopy(const DeviceWkPtr_t& weakPtr, const Device& other);

      /// \brief Copy Constructor
      Device(const Device& device);

    private:

      /// Resize configuration when changing data or extra-config.
      void resizeState ();

    protected:
      // Pinocchio objects
      ModelPtr_t model_; 
      DataPtr_t data_;
      GeomModelPtr_t geomModel_;
      GeomDataPtr_t geomData_;

      inline void invalidate () { upToDate_ = false; frameUpToDate_ = false; geomUpToDate_ = false; }

      std::string name_;
      JointVector jointVector_; // fake container with iterator mimicking hpp::model::JointVector_t
      Configuration_t currentConfiguration_;
      vector_t currentVelocity_;
      vector_t currentAcceleration_;
      bool upToDate_, frameUpToDate_, geomUpToDate_;
      Computation_t computationFlag_;
      // Obstacles
      DeviceObjectVector objectVector_;
      // Grippers
      Grippers_t grippers_;
      LiegroupSpacePtr_t configSpace_;
      // Extra configuration space
      ExtraConfigSpace extraConfigSpace_;
      DeviceWkPtr_t weakPtr_;

      /// Temporary variable to avoid dynamic allocation
      mutable Configuration_t robotConf_;
    }; // class Device

    inline std::ostream& operator<< (std::ostream& os, const hpp::pinocchio::Device& device)
    { return device.print(os); }

  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_DEVICE_HH
