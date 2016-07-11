//
// Copyright (c) 2016 CNRS
// Author: NMansard
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

#define BOOST_TEST_MODULE tdevice

#include <boost/test/unit_test.hpp>

#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <pinocchio/multibody/parser/urdf.hpp>



//      virtual ~Device() ;
//NOTCHECKED      DevicePtr_t clone()
//NOTCHECKED      DevicePtr_t cloneConst() const
//NOTCHECKED      const std::string& name () const
//      static DevicePtr_t create (const std::string & name);
//NOTCHECKED      static DevicePtr_t createCopy (const DevicePtr_t& device);
//NOTCHECKED      static DevicePtr_t createCopyConst (const DeviceConstPtr_t& device);
//      void model( ModelPtr_t modelPtr )
//      ModelConstPtr_t model() const
//      ModelPtr_t model()
//      void data( DataPtr_t dataPtr )
//      DataConstPtr_t data() const
//      DataPtr_t data()
//      void createData();
//      const JointVector_t& getJointVector () const;
//      JointPtr_t rootJoint () const;
//      JointPtr_t getJointAtConfigRank (const size_type& r) const;
//      JointPtr_t getJointAtVelocityRank (const size_type& r) const;
//      JointPtr_t getJointByName (const std::string& name) const;
//NOTCHECKED      JointPtr_t getJointByBodyName (const std::string& name) const;
//      size_type configSize () const;
//      size_type numberDof () const;
//NOTCHECKED      ExtraConfigSpace& extraConfigSpace ()
//NOTCHECKED      const ExtraConfigSpace& extraConfigSpace () const
//NOTCHECKED      virtual void setDimensionExtraConfigSpace (const size_type& dimension)
//      const Configuration_t& currentConfiguration () const
//      virtual bool currentConfiguration (ConfigurationIn_t configuration);
//NOTCHECKED      Configuration_t neutralConfiguration () const;
//      const vector_t& currentVelocity () const
//      void currentVelocity (vectorIn_t velocity)
//      const vector_t& currentAcceleration () const
//      void currentAcceleration (vectorIn_t acceleration)
//      const value_type& mass () const;
//      vector3_t positionCenterOfMass () const;
//      const ComJacobian_t& jacobianCenterOfMass () const;
//NOTCHECKED      void addGripper (const GripperPtr_t& gripper)
//NOTCHECKED      Grippers_t& grippers ()
//NOTCHECKED      const Grippers_t& grippers () const
//NOTCHECKED      const ObjectVector_t& obstacles (Request_t type) const;
//NOTCHECKED      virtual void addCollisionPairs ;
//NOTCHECKED      virtual void removeCollisionPairs;
//NOTCHECKED      const CollisionPairs_t& collisionPairs (Request_t type) const;
//NOTCHECKED      ObjectIterator objectIterator (Request_t type);
//NOTCHECKED      bool collisionTest () const;
//NOTCHECKED      void computeDistances ();
//NOTCHECKED      const DistanceResults_t& distanceResults () const ;
//NOTCHECKED      void controlComputation (const Computation_t& flag)
//NOTCHECKED      Computation_t computationFlag () const
//NOTCHECKED      virtual void computeForwardKinematics ();
//NOTCHECKED      virtual std::ostream& print (std::ostream& os) const;
// protected:
//      Device(const std::string& name);
//NOTCHECKED      void updateDistances ();
//NOTCHECKED      Device(const Device& device);
//      void resizeState ();
//NOTCHECKED      void resizeJacobians ();
// protected:
//NOTCHECKED      ModelPtr_t model_;
//NOTCHECKED      DataPtr_t data_;
//NOTCHECKED      std::string name_;
//NOTCHECKED      DistanceResults_t distances_;
//NOTCHECKED      Configuration_t currentConfiguration_;
//NOTCHECKED      vector_t currentVelocity_;
//NOTCHECKED      vector_t currentAcceleration_;
//NOTCHECKED      bool upToDate_;
//NOTCHECKED      Computation_t computationFlag_;
//NOTCHECKED      CollisionPairs_t collisionPairs_;
//NOTCHECKED      CollisionPairs_t distancePairs_;
//NOTCHECKED      Grippers_t grippers_;
//NOTCHECKED      ExtraConfigSpace extraConfigSpace_;
//NOTCHECKED      DeviceWkPtr_t weakPtr_;






/* -------------------------------------------------------------------------- */
/* --- CONVERTIONS ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

struct m2p
{
  static hpp::pinocchio::Configuration_t
  q (const hpp::model::Configuration_t & qin )
  {
    hpp::pinocchio::Configuration_t qout = qin;
    double q3 = qin[3];
    qout.segment<3>(3) = qin.segment<3>(4);
    qout[6] = q3;
    return qout;
  }

  /* Convert velocities in configuration space. 
   * Use it with:
   *    vq_pinocchio = Xq(oRb)*vq_model
   *    Jq_model = Jq_pinocchio*Xq(oRb)
   */
  struct Xq
  {
    Eigen::MatrixXd oRb; // Orientation of the base in the world frame.
    Xq(const Eigen::MatrixXd & R) : oRb(R) {}

    // vq in hpp::model is expressed in world frame. Convert it to base frame.
    Eigen::VectorXd operator* (const Eigen::VectorXd & vq)
    {
      Eigen::VectorXd vout = vq;
      vout.head<3>() = oRb.transpose()*vq.head<3>();
      vout.segment<3>(3) = oRb.transpose()*vq.segment<3>(3);
      return vout;
    }

    friend Eigen::MatrixXd operator* (const Eigen::MatrixXd & J, const Xq & x)
    {
      Eigen::MatrixXd Jout = J;
      Jout.leftCols<3>() = J.leftCols<3>()*x.oRb.transpose();
      Jout.middleCols<3>(3) = J.middleCols<3>(3)*x.oRb.transpose();
      return Jout;
    }
  };

  static hpp::pinocchio::Transform3f
  SE3( const hpp::model::Transform3f & Mm )
  {
    return se3::SE3(Mm.getRotation(),Mm.getTranslation());
  }
}; // struct m2p

struct p2m
{
  static hpp::model::Configuration_t
  q (const hpp::pinocchio::Configuration_t & qin )
  {
    hpp::model::Configuration_t qout = qin;
    double q6 = qin[6];
    qout.segment<3>(4) = qin.segment<3>(3);
    qout[3] = q6;
    return qout;
  }

  /* Convert velocities in configuration space. 
   * Use it with:
   *    vq_model = Xq(oRb)*vq_pinocchio
   *    Jq_pinocchio = Jq_model*Xq(oRb)
   */
  struct Xq
  {
    Eigen::MatrixXd oRb; // Orientation of the base in the world frame.
    Xq(const Eigen::MatrixXd & R) : oRb(R) {}

    // vq in hpp::pinocchio is expressed in base frame. Convert it to world frame.
    Eigen::VectorXd operator* (const Eigen::VectorXd & vq)
    {
      Eigen::VectorXd vout = vq;
      vout.head<3>() = oRb*vq.head<3>();
      vout.segment<3>(3) = oRb*vq.segment<3>(3);
      return vout;
    }

    friend Eigen::MatrixXd operator* (const Eigen::MatrixXd & J, const Xq & x)
    {
      Eigen::MatrixXd Jout = J;
      Jout.leftCols<3>() = J.leftCols<3>()*x.oRb;
      Jout.middleCols<3>(3) = J.middleCols<3>(3)*x.oRb;
      return Jout;
    }

  };

  static hpp::model::Transform3f 
  SE3( const hpp::pinocchio::Transform3f & Mp )
  {
    return hpp::model::Transform3f(Mp.rotation(),Mp.translation());
  }

}; // struct p2m

template<typename D>
bool isPermutation(const Eigen::MatrixBase<D> & R)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);

  for( int row=0;row<3;row++ )
    for( int col=0;col<3;col++ )
      if ( (std::abs(R(row,col))>1e-6) && (std::abs(std::abs(R(row,col))-1)>1e-6) )
        return false;
  return true;
}

/* Frames in hpp::model are "permuted" (i.e. combination of PI/2 Cartesian rotation)
 * so check here that the two placements fits, under this permutation. 
 */
bool isApproxPermutation( hpp::model::Transform3f Mm,
                          hpp::pinocchio::Transform3f Mp )
{
  return  Mm.getTranslation().isApprox( Mp.translation() )
    &&  isPermutation(Eigen::Matrix3d(Mp.rotation().transpose()*Mm.getRotation())) ; 
}


BOOST_AUTO_TEST_CASE(convert)
{
  Eigen::VectorXd q = Eigen::VectorXd::Random(20);
  BOOST_CHECK( q.isApprox(m2p::q(p2m::q(q))) );

  Eigen::VectorXd v = Eigen::VectorXd::Random(20);
  Eigen::MatrixXd J = Eigen::MatrixXd::Random(3,20); 
  Eigen::MatrixXd R(3,3);
  R << 0,0,1 ,1,0,0, 0,1,0;
  
  BOOST_CHECK( v.isApprox(p2m::Xq(R)*(m2p::Xq(R)*v)) );
  BOOST_CHECK( J.isApprox((J*p2m::Xq(R))*m2p::Xq(R)) );
  BOOST_CHECK( ((J*m2p::Xq(R))*(p2m::Xq(R)*v)).isApprox(J*v) );

  se3::SE3 Mp = se3::SE3::Random();
  BOOST_CHECK( Mp.isApprox( m2p::SE3(p2m::SE3(Mp)) ) );
}


/* -------------------------------------------------------------------------- */
/* --- URDF WRAP ------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */
namespace hpp {
  namespace model {
    DevicePtr_t
    robotFromUrdf( const std::string filename, const std::string rootType="freeflyer" )
    {
      // Create the robot object.
      DevicePtr_t robot  = hpp::model::Device::create(filename);
      // Build robot model from URDF.
      urdf::Parser urdfParser (rootType, robot);
      urdfParser.parse (std::string("file://")+filename);
      hppDout (notice, "Finished parsing URDF file.");
      return robot;
    }
  } // namespace hpp
} // namespace model

/* Default path of the urdf file describing the robot to parse. */
static const std::string urdfDefaultFilename =
  "/home/nmansard/src/pinocchio/hpp/hpp-model-urdf/romeo.urdf";

/* Build a hpp::model::Device from urdf path. */  
hpp::model::DevicePtr_t hppModel( const std::string urdfFilename = urdfDefaultFilename )
{ return hpp::model::robotFromUrdf(urdfFilename); }

/* Build a hpp::pinocchio::Device from urdf path. */
hpp::pinocchio::DevicePtr_t hppPinocchio( const std::string urdfFilename = urdfDefaultFilename )
{
  hpp::pinocchio::DevicePtr_t pinocchio = hpp::pinocchio::Device::create(urdfFilename);
  hpp::pinocchio::ModelPtr_t model( new se3::Model() );
  *model = se3::urdf::buildModel(urdfFilename,se3::JointModelFreeFlyer());
  pinocchio->model(model);
  pinocchio->createData();
  return pinocchio;
}

/* -------------------------------------------------------------------------- */
/* --- UNIT TESTS ----------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

static bool verbose = false;

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (create)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  std::ofstream log ("./tdevice.log");
  log << *(model.get ()) << std::endl;
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (easyGetter)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  /* --- Check NQ NV */
  if(verbose) 
    {
      std::cout << "nq = " << model->configSize() << " vs " << pinocchio->configSize() << std::endl;
      std::cout << "nq = " << model->numberDof()  << " vs " << pinocchio->numberDof()  << std::endl;
    }

  BOOST_CHECK ( model->configSize()==pinocchio->configSize() );
  BOOST_CHECK ( model->numberDof() ==pinocchio->numberDof() );

  /* --- Check configuration */
  {
    BOOST_CHECK ( model    ->currentConfiguration().size()==model    ->configSize() );
    BOOST_CHECK ( pinocchio->currentConfiguration().size()==pinocchio->configSize() );
    Eigen::VectorXd q = Eigen::VectorXd::Random( model->configSize() );
    const Eigen::VectorXd qcopy = q;
    model->currentConfiguration(q);
    BOOST_CHECK( model->currentConfiguration() == qcopy );
    q = qcopy;
    pinocchio->currentConfiguration(q);
    BOOST_CHECK( pinocchio->currentConfiguration() == qcopy );
  }

  /* --- Check vel acc */
  {
    BOOST_CHECK ( model    ->currentVelocity().size()==model    ->numberDof() );
    BOOST_CHECK ( pinocchio->currentVelocity().size()==pinocchio->numberDof() );
    BOOST_CHECK ( model    ->currentAcceleration().size()==model    ->numberDof() );
    BOOST_CHECK ( pinocchio->currentAcceleration().size()==pinocchio->numberDof() );

    Eigen::VectorXd q = Eigen::VectorXd::Random( model->numberDof() );
    const Eigen::VectorXd qcopy = q;
    model->currentVelocity(q);
    BOOST_CHECK( model->currentVelocity() == qcopy );
    q = qcopy;
    pinocchio->currentVelocity(q);
    BOOST_CHECK( pinocchio->currentVelocity() == qcopy );
    q = qcopy;
     model->currentAcceleration(q);
    BOOST_CHECK( model->currentAcceleration() == qcopy );
    q = qcopy;
    pinocchio->currentAcceleration(q);
    BOOST_CHECK( pinocchio->currentAcceleration() == qcopy );
  }
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (compute)
{
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  Eigen::VectorXd q = Eigen::VectorXd::Random( model->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  model->currentConfiguration(q);
  model->controlComputation (hpp::model::Device::ALL);
  model->computeForwardKinematics();

  pinocchio->currentConfiguration(m2p::q(q));
  pinocchio->controlComputation (hpp::pinocchio::Device::ALL);
  pinocchio->computeForwardKinematics();
  
  if( verbose )
    {
      std::cout << "com_model     = " << model    ->positionCenterOfMass() << std::endl;
      std::cout << "com_pinocchio = " << pinocchio->positionCenterOfMass() << std::endl;

      std::cout << model    ->jacobianCenterOfMass().leftCols<9>() << std::endl;
      std::cout << pinocchio->jacobianCenterOfMass().leftCols<9>() << std::endl;
    }

  BOOST_CHECK( model->mass() == pinocchio->mass() );

  BOOST_CHECK( model->positionCenterOfMass().isApprox
               (pinocchio->positionCenterOfMass()) );

  Eigen::MatrixXd Jmodel     = model    ->jacobianCenterOfMass();
  Eigen::MatrixXd Jpinocchio = pinocchio->jacobianCenterOfMass();
  Eigen::MatrixXd oRb = Jpinocchio.leftCols<3>();
  
  BOOST_CHECK( (Jmodel    *p2m::Xq(oRb)).isApprox(Jpinocchio) );
  BOOST_CHECK( (Jpinocchio*m2p::Xq(oRb)).isApprox(Jmodel) );
}


BOOST_AUTO_TEST_CASE(jointAccess)
{
  static bool verbose = true;
  hpp::model::DevicePtr_t model = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  BOOST_CHECK (pinocchio->rootJoint()->rankInConfiguration() == 0);

  if(verbose)
    std::cout << pinocchio->getJointAtConfigRank(0)->name() << " -- "
              <<  pinocchio->rootJoint()->name() <<std::endl;
  BOOST_CHECK (pinocchio->getJointAtConfigRank(0)->name() == pinocchio->rootJoint()->name());

  BOOST_CHECK (pinocchio->getJointAtVelocityRank(0)->name() == pinocchio->rootJoint()->name());

  for (int i=1;i<pinocchio->model()->njoint;++i)
    {
      BOOST_CHECK( pinocchio->getJointAtConfigRank(pinocchio->model()->joints[i].idx_q())->name()
                   == pinocchio->model()->names[i] );
      BOOST_CHECK( pinocchio->getJointAtVelocityRank(pinocchio->model()->joints[i].idx_v())->name()
                   == pinocchio->model()->names[i] );
    }
  
  hpp::model::JointPtr_t jm = model->getJointVector()[5];
  BOOST_CHECK( pinocchio->getJointByName (jm->name())->name() == jm->name() );

  // Weak test: the first joint is free flyer, with different names in pinocchio and model
  // The second joint of model is an anchor, so not added inside Pinocchio
  // So we check pinocchio[1] (hip) against model[3] (hip as well).
  if(verbose)
    std::cout << pinocchio->getJointVector()[1]->name() << " -- " 
              << model->getJointVector()[3]->name() << std::endl;
  BOOST_CHECK( pinocchio->getJointVector()[1]->name() == 
               model->getJointVector()[3]->name() );
}
/* -------------------------------------------------------------------------- */

//      Joint (DeviceWkPtr_t device, Index indexInJointList );
//      ~Joint() {}
//      const std::string& name() const;
//      const Transform3f& currentTransformation () const;
//NOTCHECKED      vector_t neutralConfiguration () const;
//      size_type numberDof () const;
//      size_type configSize () const;
//      size_type rankInConfiguration () const;
//      size_type rankInVelocity () const;
//      std::size_t numberChildJoints () const;
//      JointPtr_t childJoint (std::size_t rank) const;
//      const Transform3f& positionInParentFrame () const;
//NOTCHECKED      value_type upperBoundLinearVelocity () const;
//NOTCHECKED      value_type upperBoundAngularVelocity () const;
//NOTCHECKED      const value_type& maximalDistanceToParent () const;
//NOTCHECKED      void computeMaximalDistanceToParent ();
//NOTCHECKED      const JointJacobian_t& jacobian () const;
//NOTCHECKED      JointJacobian_t& jacobian ();
//      DeviceConstPtr_t robot () const { assert(robot_.lock());  return robot_.lock ();}
//      DevicePtr_t robot () { assert(robot_.lock()); return robot_.lock ();}
//NOTCHECKED      BodyPtr_t linkedBody () const;
//NOTCHECKED      virtual std::ostream& display (std::ostream& os) const;
//    protected:
//NOTCHECKED      value_type maximalDistanceToParent_;
//NOTCHECKED      vector_t neutralConfiguration_;
//NOTCHECKED      DeviceWkPtr_t robot_;
//NOTCHECKED      JointJacobian_t jacobian_;
//NOTCHECKED      Index id;
//NOTCHECKED      std::vector<Index> children;
//NOTCHECKED      void setChildList();
//NOTCHECKED      ModelPtr_t       model() ;      
//NOTCHECKED      ModelConstPtr_t  model() const ;
//NOTCHECKED      DataPtr_t        data()  ;      
//NOTCHECKED      DataConstPtr_t   data()  const ;

bool hasChild (hpp::pinocchio::JointPtr_t jp,std::string name)
{
  //std::cout << "father name " << jp->name() << " nc=" << jp->numberChildJoints() << std::endl;
  for( unsigned int c=0;c<jp->numberChildJoints();++c )
    {
      //std::cout <<"child name " << jp->childJoint(c)->name() << std::endl;
      if( jp->childJoint(c)->name() == name ) return true;
    }
  return false;
}

BOOST_AUTO_TEST_CASE (joint)
{
  hpp::model::DevicePtr_t     model     = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  Eigen::VectorXd q = Eigen::VectorXd::Random( model->configSize() );
  q[3] += 1.0;
  q.segment<4>(3) /= q.segment<4>(3).norm();

  model->currentConfiguration(q);
  model->controlComputation (hpp::model::Device::ALL);
  model->computeForwardKinematics();

  pinocchio->currentConfiguration(m2p::q(q));
  pinocchio->controlComputation (hpp::pinocchio::Device::ALL);
  pinocchio->computeForwardKinematics();  

  if( verbose )
    {
      for( int i=1;i<8;++i)
        {
          std::cout << " *************************************************** " << std::endl;
          std::cout << "model = " << model->getJointVector()[i+1]->name() << std::endl;
          std::cout << "pino = " << pinocchio->model()->names[i] << std::endl;
          std::cout << "type = " << pinocchio->model()->joints[i].shortname() << std::endl;
          std::cout << typeid(*model->getJointVector()[i+1]).name() << std::endl;
          
          std::cout << "5M6n = " << model->getJointVector()[i+1]->positionInParentFrame() << std::endl;
          std::cout << "5M6n = " << pinocchio->model()->jointPlacements[i] << std::endl;
          
          std::cout << "oM6n = " << model->getJointVector()[i+1]->currentTransformation() << std::endl;
          std::cout << "oM6n = " << pinocchio->data()->oMi[i] << std::endl;
        }
    }

  pinocchio->model()->names[1] = "waist";

  for( unsigned int i=2;i<model->getJointVector().size();++i)
    {
      hpp::model::JointPtr_t jm = model->getJointVector()[i];
      if(verbose)std::cout << " ***************** " << jm->name()<< std::endl;
      if(dynamic_cast<hpp::model::JointAnchor *> (jm)) // is an anchor
        continue;

      hpp::pinocchio::JointPtr_t jp;
      try
        { jp = pinocchio->getJointByName(jm->name()); }
      catch (...)
        { BOOST_CHECK( false && "Joint name does not exist" ); }
      
      BOOST_CHECK( jp->name()                == jm->name() );
      BOOST_CHECK( jp->numberDof()           == jm->numberDof() );
      BOOST_CHECK( jp->configSize()          == jm->configSize() );
      BOOST_CHECK( jp->rankInConfiguration() == jm->rankInConfiguration() );
      BOOST_CHECK( jp->rankInVelocity()      == jm->rankInVelocity() );
      BOOST_CHECK( isApproxPermutation(jm->currentTransformation(),
                                       jp->currentTransformation()) );
      
      // Possibly some children are anchor, and then removed from Pinocchio
      BOOST_CHECK( jp->numberChildJoints()   <= jm->numberChildJoints() );
      for( unsigned int c=0; c<jm->numberChildJoints(); ++c )
        {
          hpp::model::JointPtr_t cm = jm->childJoint(c);
          if(dynamic_cast<hpp::model::JointAnchor *> (cm)) // is not an anchor
            continue;
          BOOST_CHECK ( hasChild(jp,cm->name()) );
        }
      
      BOOST_CHECK( jp->robot() == pinocchio );
      BOOST_CHECK( jm->robot() == model );
      
      //std::cout << *jm << std::endl;
      //std::cout << *jp << std::endl;
    }

  /* Checking positionInParentFrame is difficult because of the modification of revolute
   * joints in hpp::model (all oriented along X). Then only check the two first ones, where
   * we can confidently assume that they are properly aligned. */
  BOOST_CHECK( isApproxPermutation( model    ->getJointByName("waist")->positionInParentFrame(),
                                    pinocchio->getJointByName("waist")->positionInParentFrame() ) );
  BOOST_CHECK( isApproxPermutation( model    ->getJointByName("waist")
                                    ->childJoint(0)->positionInParentFrame(),
                                    pinocchio->getJointByName("waist")
                                    ->childJoint(0)->positionInParentFrame() ) );

  
}

/* -------------------------------------------------------------------------- */
BOOST_AUTO_TEST_CASE (jointVector)
{
  hpp::model::DevicePtr_t     model     = hppModel();
  hpp::pinocchio::DevicePtr_t pinocchio = hppPinocchio();

  hpp::pinocchio::JointVector jv(pinocchio);
  {
    hpp::pinocchio::JointVector::iterator it = jv.begin();
    BOOST_CHECK( (*it)->name() == pinocchio->rootJoint()->name() );
  }

  /* Check regular iterator, skipping root joint (names are different). */
  int check = 1;
  for (hpp::pinocchio::JointVector::iterator it = ++jv.begin (); it != jv.end (); ++it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[++check];
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
      // std::cout << "\"" << jm->name()
      //           << "\"->\"" << (*it)->name () << "\"" << std::endl;
    }

  /* Check reverse iterator. */
  check = model->getJointVector().size()-1;
  for (hpp::pinocchio::JointVector::iterator it = jv.rbegin (); it != ++jv.rend (); --it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[check]; --check;
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
      //std::cout << "\"" << jm->name()
      //<< "\"->\"" << (*it)->name () << "\"" << std::endl;
    }

  /* Check constant iterator. */
  const hpp::pinocchio::JointVector jvconst(pinocchio);
  check = 1;
  for (hpp::pinocchio::JointVector::const_iterator it = ++jvconst.begin (); it != jv.end (); ++it) 
    {
      hpp::model::JointPtr_t jm;
      do
        {
          jm = model->getJointVector()[++check];
        } while(dynamic_cast<hpp::model::JointAnchor *> (jm)); // is not an anchor

      BOOST_CHECK (jm->name() == (*it)->name());
    }

}
