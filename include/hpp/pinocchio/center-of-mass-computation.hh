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

# include <pinocchio/multibody/data.hpp> // se3::Data

# include <hpp/pinocchio/fwd.hh>
# include <hpp/pinocchio/device.hh>

namespace hpp {
  namespace pinocchio {
    /// Computation of the center of mass of a subtree of a kinematic tree
    ///
    /// To use this class, create an instance using
    /// CenterOfMassComputation::create method and call method
    /// CenterOfMassComputation::add with parameter the root joint
    /// of the subtree.
    ///
    /// In most cases, the root joint of the subtree is the root joint of
    /// the robot (hpp::pinocchio::Device::rootJoint ()), but in a manipulation
    /// context, the kinematic tree contains several robots and objects.
    /// This class enables users to compute the center of mass of only one
    /// robot or object.
    class CenterOfMassComputation
    {
      public:
        typedef std::vector <JointIndex> JointRootIndexes_t;
        /// \cond
        // This fixes an alignment issue of se3::Data::hg
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /// \endcond

      public:
        /// Create instance and return shared pointer.
        ///
        /// Do not forget to call method add to specify the root joint of
        /// relevant kinematic tree.
        static CenterOfMassComputationPtr_t create (const DevicePtr_t& device);

        /// Add a subtree to the computation of the center of mass.
        ///
        /// When several subtrees are provided, method \c compute computes the
        /// center of mass of the union of the subtrees.
        void add (const JointPtr_t& rootOfSubtree);

        /// Compute the center of mass and Jacobian of the sub-trees.
        void compute (const Device::Computation_t& flag
            = Device::ALL);

        /// Get center of mass of the subtree.
        const vector3_t&     com         () const { return data.com [0]; }
        /// Get mass of the sub-tree.
        const value_type&    mass        () const { return data.mass[0]; }
        /// Get Jacobian of center of mass of the sub-tree.
        const ComJacobian_t& jacobian    () const { return data.Jcom   ; }
        /// Get const reference to the vector of sub-tree roots.
        const JointRootIndexes_t & roots () const { return roots_; }

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
