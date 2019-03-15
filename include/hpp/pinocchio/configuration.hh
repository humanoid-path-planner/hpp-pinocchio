//
// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
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

#ifndef HPP_PINOCCHIO_CONFIGURATION_HH
# define HPP_PINOCCHIO_CONFIGURATION_HH

# include <iomanip>
# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace pinocchio {
    /// Saturate joint parameter value so that they are not out of bounds.
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param[in,out] configuration initial and result configurations
    /// \retval configuration reached after saturation.
    void saturate (const DevicePtr_t& robot,
                   ConfigurationOut_t configuration);

    /// Saturate joint parameter value so that they are not out of bounds.
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param[in,out] configuration initial and result configurations
    /// \retval saturation an array of boolean saying who saturated (size robot.numberDof()).
    bool saturate (const DevicePtr_t& robot,
                   ConfigurationOut_t configuration,
                   ArrayXb& saturation);

    /// Integrate a constant velocity during unit time.
    ///
    /// \tparam saturateConfig when true, calls saturate at the end
    /// \param robot robot that describes the kinematic chain
    /// \param configuration initial and result configurations
    /// \param velocity velocity vector
    /// \retval result configuration reached after integration.
    /// Velocity is dispatched to each joint that integrates according to its
    /// Lie group structure, i.e.
    /// \li \f$q_i += v_i\f$ for translation joint and bounded rotation joints,
    /// \li \f$q_i += v_i \mbox{ modulo } 2\pi\f$ for unbounded rotation joints,
    /// \li constant rotation velocity for SO(3) joints.
    ///
    /// \note bounded degrees of freedom are saturated if the result of the
    ///       above operation is beyond a bound.
    template<bool saturateConfig, typename LieGroup>
    void integrate (const DevicePtr_t& robot,
                    ConfigurationIn_t configuration,
                    vectorIn_t velocity, ConfigurationOut_t result);

    /// Same as integrate<true, DefaultLieGroupMap>
    void integrate (const DevicePtr_t& robot,
                    ConfigurationIn_t configuration,
                    vectorIn_t velocity, ConfigurationOut_t result);

    /// Interpolate between two configurations of the robot
    /// \param robot robot that describes the kinematic chain
    /// \param q0, q1 two configurations to interpolate
    /// \param u in [0,1] position along the interpolation: q0 for u=0,
    /// q1 for u=1
    /// \retval result interpolated configuration
    template <typename LieGroup>
    void interpolate  (const DevicePtr_t& robot,
                       ConfigurationIn_t q0,
                       ConfigurationIn_t q1,
                       const value_type& u,
                       ConfigurationOut_t result);

    void interpolate (const DevicePtr_t& robot,
                      ConfigurationIn_t q0,
                      ConfigurationIn_t q1,
                      const value_type& u,
                      ConfigurationOut_t result);

    /// Difference between two configurations as a vector
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param q1 first configuration,
    /// \param q2 second configuration,
    /// \retval result difference as a vector \f$\textbf{v}\f$ such that
    /// q1 is the result of method integrate from q2 with vector
    /// \f$\textbf{v}\f$
    /// \note If the configuration space is a vector space, this is
    /// \f$\textbf{v} = q_1 - q_2\f$
    template <typename LieGroup>
    void difference (const DevicePtr_t& robot, ConfigurationIn_t q1,
                     ConfigurationIn_t q2, vectorOut_t result);

    void difference (const DevicePtr_t& robot, ConfigurationIn_t q1,
                     ConfigurationIn_t q2, vectorOut_t result);

    /// Test that two configurations are close
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param q1 first configuration,
    /// \param q2 second configuration,
    /// \param eps numerical threshold
    /// \return true if the configurations are closer than the numerical
    /// threshold
    bool isApprox (const DevicePtr_t& robot, ConfigurationIn_t q1,
                   ConfigurationIn_t q2, value_type eps);

    /// Distance between two configuration.
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param q1 first configuration,
    /// \param q2 second configuration,
    value_type distance (const DevicePtr_t& robot, ConfigurationIn_t q1,
                         ConfigurationIn_t q2);

    /// Normalize configuration
    ///
    /// Configuration space is a represented by a sub-manifold of a vector
    /// space. Normalization consists in projecting a vector on this
    /// sub-manifold. It mostly consists in normalizing quaternions for 
    /// SO3 joints and 2D-vectors for unbounded rotations.
    void normalize (const DevicePtr_t& robot, Configuration_t& q);

    /// For backward compatibility.
    /// See normalize normalize (const DevicePtr_t&, Configuration_t&)
    inline void normalize (const DevicePtr_t& robot, ConfigurationOut_t q)
    {
      Configuration_t qq = q;
      normalize(robot, qq);
      q = qq;
    }

    /// Check if a configuration is normalized
    ///
    /// It consists in checking that norm of quaternions and complex is one.
    bool isNormalized (const DevicePtr_t& robot, ConfigurationIn_t q,
                       const value_type& eps);

    /// Write configuration in a string
    inline std::string displayConfig (ConfigurationIn_t q, int precision = 20)
    {
      std::ostringstream oss; oss << "(" <<  std::setprecision (precision);
      for (size_type i=0; i < q.size (); ++i) {
	oss << q [i] << ",";
      }
      oss << ")";
      return oss.str ();
    }

    /// Write a SE3 taking into account the indentation
    std::ostream& display (std::ostream& os, const SE3& m);
  } // namespace pinocchio
} // namespace hpp
#endif // HPP_PINOCCHIO_CONFIGURATION_HH
