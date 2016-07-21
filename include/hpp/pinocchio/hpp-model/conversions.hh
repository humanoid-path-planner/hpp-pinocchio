//
// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
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
    Xq(const se3::SE3 & M)        : oRb(M.rotation()) {}

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

  /* Convert velocities in cartesian space. 
   * Use it with:
   *    v_pinocchio = X(oRe)*v_model
   *    J_pinocchio = X(oRe)*J_model
   */
  struct X
  {
    Eigen::MatrixXd oRe; // Orientation of the operational frame in the world frame.
    X(const Eigen::MatrixXd & R) : oRe(R) {}
    X(const se3::SE3 & M)        : oRe(M.rotation()) {}

    // v6 in hpp::pinocchio is expressed in local frame (at the center of the
    // frame). Convert it to world frame.
    template<typename D>
    Eigen::Matrix<double,D::RowsAtCompileTime,D::ColsAtCompileTime>
    operator* (const Eigen::MatrixBase<D> & M6)
    {
      assert(M6.rows()==6);
      Eigen::Matrix<double,D::RowsAtCompileTime,D::ColsAtCompileTime> Mout = M6;
      Mout.template topRows   <3>() = oRe.transpose()*M6.template topRows   <3>();
      Mout.template bottomRows<3>() = oRe.transpose()*M6.template bottomRows<3>();
      return Mout;
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
    Xq(const se3::SE3 & M)        : oRb(M.rotation()) {}

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

  /* Convert velocities in cartesian space. 
   * Use it with:
   *    v_model = X(oRe)*v_pinocchio
   *    J_model = X(oRe)*J_pinocchio
   */
  struct X
  {
    Eigen::MatrixXd oRe; // Orientation of the operational frame in the world frame.
    X(const Eigen::MatrixXd & R) : oRe(R) {}
    X(const se3::SE3 & M)        : oRe(M.rotation()) {}

    // v6 in hpp::pinocchio is expressed in local frame (at the center of the
    // frame). Convert it to world frame.
    template<typename D>
    Eigen::Matrix<double,D::RowsAtCompileTime,D::ColsAtCompileTime>
    operator* (const Eigen::MatrixBase<D> & M6)
    {
      assert(M6.rows()==6);
      Eigen::Matrix<double,D::RowsAtCompileTime,D::ColsAtCompileTime> Mout = M6;
      Mout.template topRows   <3>() = oRe*M6.template topRows   <3>();
      Mout.template bottomRows<3>() = oRe*M6.template bottomRows<3>();
      return Mout;
    }
  };

  static hpp::model::Transform3f 
  SE3( const hpp::pinocchio::Transform3f & Mp )
  {
    return hpp::model::Transform3f(Mp.rotation(),Mp.translation());
  }

}; // struct p2m
