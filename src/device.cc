#include <Eigen/Core>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/pinocchio/distance-result.hh>
#include <hpp/pinocchio/extra-config-space.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/device.hh>


namespace hpp {
  namespace pinocchio {

    Device::
    Device(const std::string& name)
      : model_()
      , data_ ()
      , name_ (name)
      , weakPtr_()
    {}

    // static method
    DevicePtr_t Device::
    create (const std::string & name)
    {
      DevicePtr_t res = DevicePtr_t(new Device(name)); // init shared ptr
      res->weakPtr_ = res;
      return res;
    }
    
    // static method
    DevicePtr_t Device::
    createCopy (const DevicePtr_t& device)
    {
      DevicePtr_t res = Device::create(device->name()); // init shared ptr
      res->model(device->model());  // Copy pointer to pinocchio model
      res->createData();    // Create a new data, dont copy the pointer.
      return res;
    }

    // static method
    DevicePtr_t Device::
    createCopyConst (const DeviceConstPtr_t& device)
    {
      DevicePtr_t res = Device::create(device->name()); // init shared ptr
      /* The copy of Pinocchio::Model is not implemented yet. */
      /* We need this feature to finish the implementation of this method. */
      assert( false && "TODO: createCopyConst is not implemented yet." );
      return res;
    }

    void Device::
    createData()
    {
      data_ = DataPtr_t( new se3::Data(*model_) );
    }
    
    JointPtr_t Device::
    rootJoint () const
    {
      return JointPtr_t( new Joint(weakPtr_,0) );
    }
    /*
    JointPtr_t Device::
    getJointAtConfigRank (const size_type& r) const
    {
      
    }

    JointPtr_t Device::
    getJointAtVelocityRank (const size_type& r) const
    {}

    JointPtr_t Device::
    getJointByName (const std::string& name) const
    {}

    JointPtr_t Device::
    getJointByBodyName (const std::string& name) const
    {}

    size_type Device::
    configSize () const
    {}

    size_type Device::
    numberDof () const
    */

    /*
      ExtraConfigSpace& Device::
    extraConfigSpace () {
	return extraConfigSpace_;
      }







      const ExtraConfigSpace& Device::
    extraConfigSpace () const {
	return extraConfigSpace_;
      }


      virtual void Device::
    setDimensionExtraConfigSpace (const size_type& dimension)
      {
	extraConfigSpace_.setDimension (dimension);
	resizeState (0x0);
      }







      const Configuration_t& Device::
    currentConfiguration () const
      {
	return currentConfiguration_;
      }



      virtual bool Device::
    currentConfiguration (ConfigurationIn_t configuration)
      {
	if (configuration != currentConfiguration_) {
	  upToDate_ = false;
	  currentConfiguration_ = configuration;
          return true;
	}
        return false;
      }

      Configuration_t Device::
    neutralConfiguration () const;


      const vector_t& Device::
    currentVelocity () const
      {
	return currentVelocity_;
      }


      void Device::
    currentVelocity (vectorIn_t velocity)
      {
	upToDate_ = false;
	currentVelocity_ = velocity;
      }


      const vector_t& currentAcceleration () const
      {
	return currentAcceleration_;
      }


      void currentAcceleration (vectorIn_t acceleration)
      {
	upToDate_ = false;
	currentAcceleration_ = acceleration;
      }






      const value_type& Device::
    mass () const;


      const vector3_t& Device::
    positionCenterOfMass () const;


      const ComJacobian_t& Device::
    jacobianCenterOfMass () const;

















































































      void Device::
    controlComputation (const Computation_t& flag)
      {
	computationFlag_ = flag;
	upToDate_ = false;
      }

      Computation_t Device::
    computationFlag () const
      {
	return computationFlag_;
      }

      virtual void Device::
    computeForwardKinematics ();




      virtual std::ostream& Device::
    print (std::ostream& os) const;






      void Device::
    init(const DeviceWkPtr_t& weakPtr);




      void Device::
    initCopy(const DeviceWkPtr_t& weakPtr, const Device& model);


      void Device::
    updateDistances ();
    */

  } // namespace pinocchio
} // namespace hpp
