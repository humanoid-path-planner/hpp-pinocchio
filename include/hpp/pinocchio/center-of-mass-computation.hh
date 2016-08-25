// Copyright (c) 2016, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-pinocchio.
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
// hpp-pinocchio. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH
# define HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH

# include <list>

# include <pinocchio/multibody/model.hpp> // se3::Data

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace pinocchio {
    class CenterOfMassComputation
    {
      public:
        typedef std::vector <JointIndex> JointRootIndexes_t;
        /// \cond
        // This fixes an alignment issue of se3::Data::hg
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /// \endcond

      public:

        static CenterOfMassComputationPtr_t create (const DevicePtr_t& device);

        void add (const JointPtr_t& rootOfSubtree);

        void compute (const Device::Computation_t& flag
            = Device::ALL);

        const vector3_t&     com         () const { return data.com [0]; }
        const value_type&    mass        () const { return data.mass[0]; }
        const ComJacobian_t& jacobian    () const { return data.Jcom   ; }
        const JointRootIndexes_t & roots () const { return roots_; }
        void computeMass ()  HPP_PINOCCHIO_DEPRECATED {}

        ~CenterOfMassComputation ();

      protected:
        CenterOfMassComputation (const DevicePtr_t& device);

      private:
        DevicePtr_t robot_;
        // Root of the subtrees
        JointRootIndexes_t roots_;
        // Specific pinocchio Data to store the computation results
        Data data;

        // value_type mass_;
        // vector3_t com_;
        // ComJacobian_t jacobianCom_;

    }; // class CenterOfMassComputation
  }  // namespace pinocchio
}  // namespace hpp
#endif // HPP_PINOCCHIO_CENTER_OF_MASS_COMPUTATION_HH
