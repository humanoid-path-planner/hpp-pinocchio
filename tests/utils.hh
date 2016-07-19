#include <pinocchio/parsers/urdf.hpp>

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
       ROMEO_MODEL_DIR "/romeo_description/urdf/romeo_small.urdf";

/* Build a hpp::model::Device from urdf path. */  
hpp::model::DevicePtr_t hppModel( const std::string urdfFilename = urdfDefaultFilename )
{ return hpp::model::robotFromUrdf(urdfFilename); }

/* Build a hpp::pinocchio::Device from urdf path. */
hpp::pinocchio::DevicePtr_t hppPinocchio( bool withGeoms = false,
                                          const std::string urdfFilename = urdfDefaultFilename)
{
  hpp::pinocchio::DevicePtr_t pinocchio = hpp::pinocchio::Device::create(urdfFilename);
  hpp::pinocchio::ModelPtr_t model( new se3::Model() );
  *model = se3::urdf::buildModel(urdfFilename,se3::JointModelFreeFlyer());
  pinocchio->model(model);
  pinocchio->createData();

#ifdef __se3_geom_hpp__
  if( withGeoms )
    {
      std::vector<std::string> baseDirs; baseDirs.push_back(ROMEO_MODEL_DIR);
      hpp::pinocchio::GeomModelPtr_t geom( new se3::GeometryModel() );
      se3::GeometryModel & geomRef = *geom;
      geomRef = se3::urdf::buildGeom(*pinocchio->model(),pinocchio->name(),baseDirs,se3::COLLISION);

      pinocchio->geomModel(geom);
      pinocchio->createGeomData();
    }
#endif // __se3_geom_hpp__

  return pinocchio;
}


