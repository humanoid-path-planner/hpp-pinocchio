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

#ifndef HPP_PINOCCHIO_DEVICE_SYNC_HH
#define HPP_PINOCCHIO_DEVICE_SYNC_HH

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/device-data.hh>

namespace hpp {
  namespace pinocchio {
    /// Abstract class representing a Device.
    class HPP_PINOCCHIO_DLLAPI AbstractDevice
    {
      public:
        /// \name Access to pinocchio API
        /// \{

        /// Access to pinocchio model
        ModelConstPtr_t   modelPtr() const { return model_; }
        /// Access to pinocchio model
        ModelPtr_t        modelPtr()       { return model_; }
        /// Access to pinocchio model
        const Model& model()    const { assert(model_); return *model_; }
        /// Access to pinocchio model
        Model&       model()          { assert(model_); return *model_; }

        /// Access to pinocchio geomModel
        GeomModelConstPtr_t        geomModelPtr() const { return geomModel_; }
        /// Access to pinocchio geomModel
        GeomModelPtr_t             geomModelPtr() { return geomModel_; }
        /// Access to pinocchio geomModel
        const GeomModel & geomModel() const { assert(geomModel_); return *geomModel_; }
        /// Access to pinocchio geomModel
        GeomModel &       geomModel() { assert(geomModel_); return *geomModel_; }

        /// Access to Pinocchio data/
        DataConstPtr_t    dataPtr() const { return d().data_; }
        /// Access to Pinocchio data/
        DataPtr_t         dataPtr() { return d().data_; }
        /// Access to Pinocchio data/
        const Data & data() const { assert(d().data_); return *d().data_; }
        /// Access to Pinocchio data/
        Data &       data() { assert(d().data_); return *d().data_; }

        /// Access to Pinocchio geomData/
        GeomDataConstPtr_t       geomDataPtr() const { return d().geomData_; }
        /// Access to Pinocchio geomData/
        GeomDataPtr_t            geomDataPtr()       { return d().geomData_; }
        /// Access to Pinocchio geomData/
        const GeomData& geomData() const    { assert(d().geomData_); return *d().geomData_; }
        /// Access to Pinocchio geomData/
        GeomData&       geomData()          { assert(d().geomData_); return *d().geomData_; }

        /// \}
        // -----------------------------------------------------------------------
        /// \name Current state
        /// \{

        /// Get current configuration
        const Configuration_t& currentConfiguration () const { return d().currentConfiguration_; }
        /// Set current configuration
        /// \return True if the current configuration was modified and false if
        ///         the current configuration did not change.
        virtual bool currentConfiguration (ConfigurationIn_t configuration);

        /// Get current velocity
        const vector_t& currentVelocity () const { return d().currentVelocity_; }

        /// Set current velocity
        bool currentVelocity (vectorIn_t velocity);

        /// Get current acceleration
        const vector_t& currentAcceleration () const { return d().currentAcceleration_; }

        /// Set current acceleration
        bool currentAcceleration (vectorIn_t acceleration);
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
        /// \name Forward kinematics
        /// \{

        /// Select computation
        /// Optimize computation time by selecting only necessary values in
        /// method computeForwardKinematics.
        virtual void controlComputation (const Computation_t& flag);
        /// Get computation flag
        Computation_t computationFlag () const { return d().computationFlag_; }
        /// Compute forward kinematics
        void computeForwardKinematics () { d().computeForwardKinematics(modelPtr()); }
        /// Compute frame forward kinematics
        /// \note call AbstractDevice::computeForwardKinematics.
        void computeFramesForwardKinematics () { d().computeFramesForwardKinematics(modelPtr()); }
        /// Update the geometry placement to the currentConfiguration
        void updateGeometryPlacements () { d().updateGeometryPlacements(modelPtr(), geomModelPtr()); }
        /// \}
        // -----------------------------------------------------------------------
      protected:
        AbstractDevice ();
        AbstractDevice (const ModelPtr_t& m, const GeomModelPtr_t& gm);

        virtual DeviceData      & d ()       = 0;
        virtual DeviceData const& d () const = 0;

        // Pinocchio objects
        ModelPtr_t model_;
        GeomModelPtr_t geomModel_;
    }; // class AbstractDevice

    /// A thread-safe access to a Device.
    ///
    /// To ensure thread safety, one can do
    /// \code{cpp}
    /// // In some place of the code:
    /// DevicePtr_t device = ...;
    /// device->numberDeviceData (4);
    ///
    /// // Acquires a lock on the device.
    /// DeviceSync deviceSync (device);
    /// deviceSync.currentConfiguration(q);
    /// deviceSync.computeForwardKinematics();
    ///
    /// JointPtr_t joint = ...;
    /// joint->currentTransformation (deviceSync.d());
    ///
    /// CollisionObjectPtr_t body = ...;
    /// body->fcl          (deviceSync.d());
    /// body->getTransform (deviceSync.d());
    ///
    /// // The lock is release when deviceSync is destroyed.
    /// \endcode
    class HPP_PINOCCHIO_DLLAPI DeviceSync : public AbstractDevice
    {
      public:
        /// Constructor
        /// \param lock whether to acquire the lock.
        DeviceSync (const DevicePtr_t& d, bool lock = true);

        /// Destructor.
        /// The lock is released if necessary.
        virtual ~DeviceSync ();

        /// Accessor to the locked DeviceData.
        /// \note this asserts that this isLocked()
        DeviceData      & d ()       { assert (isLocked()); return *d_; }
        /// Const accessor to the locked DeviceData.
        /// \note this asserts that this isLocked()
        DeviceData const& d () const { assert (isLocked()); return *d_; }

        /// Lock the current DeviceData
        void lock ();
        /// Check if the current DeviceData is locked
        bool isLocked () const { return d_ != NULL; }
        /// Unlock the current DeviceData
        void unlock ();

      private:
        DevicePtr_t device_;
        DeviceData* d_;
    }; // class DeviceSync
  } // namespace pinocchio
} // namespace hpp

#endif // HPP_PINOCCHIO_DEVICE_SYNC_HH
