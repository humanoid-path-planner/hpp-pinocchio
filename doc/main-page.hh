/** \mainpage
    \section hpp_pinocchio_intro Introduction
   
    This package implements kinematic chains for motion and manipulation
    planning in the Humanoid Path Planner platform.
   
    Kinematic and dynamic computations are performed by package pinocchio,
    while collision avoidance in performed by package hpp-fcl.
   
    \defgroup liegroup Lie group
   
    Configurations of robots, as well as some values like the position of a
    robot joint with respect to a fixed frame belong to differential manifolds
    that have the structure of Lie groups. These manifolds are the
    cartesian products of the following elementary Lie groups
    \li \f$\mathbf{R}^n\f$,
    \li \f$SO(2)\f$ the group of rotations in the plane represented by a vector
        of dimension 2 \f$(\cos \theta, \sin \theta)\f$ for a rotation of angle
        \f$\theta\f$,
    \li \f$SO(3)\f$the group of rotations in the 3d space represented by a
        quaternion \f$Xi + Yj + Zk + W\f$ (ie a vector of dimension 4),
    \li \f$SE(2)\f$ the group of rigid-body motions in the plane, represented
        by a vector of dimention 4 \f$(x,y,\cos\theta,\sin\theta)\f$,
    \li \f$SE(3)\f$, the group of rigid-body motion in 3d space, represented by
        a vector of dimension 7 \f$(x,y,z,X,Y,Z,W)\f$.
*/
