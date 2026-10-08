# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [9.0.2] - 2026-07-24



## [9.0.0] - 2026-07-06



## [7.0.0] - 2026-03-06



## [6.1.0] - 2025-10-23



## [6.0.0] - 2024-12-07

Changes in v6.0.0
- hpp-fcl dependency has been replaced by coal
- updates for coal v3
- allow use of hpp-util v6


## [5.2.0] - 2024-10-09

Changes in v5.2.0:
- nix: move package to nixpkgs
- ci: use https
- setup mergify


## [5.1.0] - 2024-07-02

Changes in v5.1.0:
- Update to pinocchio v3
- CMake: enable compatibility with pinocchio 2
- Fix deprecated declaration
- Nix: initial support


## [5.0.0] - 2024-03-31

Changes in v5.0.0
- :warning: BREAKING :warning: Remove computation flag and pass it to Device::computeForwardKinematics
- deprecate ConfigurationPtr_t
- remove deprecated RnxSOnLieGroupMap
- update packaging
- update tooling


## [4.15.1] - 2023-01-20



## [4.14.0] - 2022-11-02



## [4.13.0] - 2022-05-31



## [4.12.0] - 2021-10-06

Changes in v4.12.0:
- add required consts for C++17
- [doc] Make LiegroupSpace::dIntegrate_[dq|dv] clearer.

## [4.11.0] - 2021-05-04

Changes in v4.11.0:
- Frame computes child list only when required
- Serialization
- Switch to std shared_ptr
- Add argument to URDF loading functions.


## [4.10.2] - 2020-11-10

Changes in v4.10.2:
- fix header guard for C++11

## [4.10.1] - 2020-09-24

Changes since v4.9.0:
* Enable users to create LiegroupSpace::R1 with rotation template argument.
* Use CMake to handle dependencies (cmake submodule).
* Add CollisionObject::geometry and prepare deprecation of
  GeometryModel.colisionObjects.
* Add function replaceGeometryByConvexHull.
* Add package.xml
* Add equality operator in class Joint.
* Add serialization functions.

## [4.10.0] - 2020-08-17



## [4.9.1] - 2020-05-14

Fix documentation generation.

## [4.9.0] - 2020-04-29

Changes in v4.9.0:
- Remove dependency to PINOCCHIO_URDF_SHARED_PTR
- [test] Use example-robot-data instead of romeo_description
- Add LiegroupSpace::interpolate + misc.
- CMake Exports

## [4.8.0] - 2019-11-28

Changes since v4.7.0:
- update to changes in pinocchio
- Store URDF mimic joints in Device.
- Add thread safe API to CenterOfMassComputation.
- Bug fixes
- update Cmake

## [4.7.0] - 2019-10-07

Changes since v4.6.1:
- updates for pinocchio v2.1.5 & v2.1.6
- update doc
- fix warnings


## [4.5.0] - 2019-04-24

Changes since v4.4.0:
- fix compilation warnings
- s/BOOST_MESSAGE/BOOST_TEST_MESSAGE
- Make PINOCCHIO_WITH_HPP_FCL mandatory
- add optionnal dependencies to romeo & baxter description
- fix unit test
- update CI


## [4.4.0] - 2019-03-19

Changes since v4.3.0:
- Update minimum versions of Pinocchio and Eigen.
- Update to pinocchio v2 & hpp-fcl v1
- loadRobotModel now parse the neutralConfiguration field
- [URDF] Use cache mechanism when loading meshes.
- update minimal Eigen version


## [4.3.0] - 2019-01-31

- correctly copy deviceData in device copy constructor
- Enable multithreading.


## [4.2.0] - 2018-10-11

Changes since v4.1:
* Update to API changes in pinocchio: se3::LieGroupTpl -> se3::LieGroupMap.
* Rename hpp::pinocchio::LieGroupTpl -> hpp::pinocchio::RnxSOnLieGroupMap.
* Device::getFrameByName accepts all frame types.
* Add optional doc generation.
* Replace Jintegrate by dIntegrate_dq and dIntegrate_dv in LiegroupSpace.
* Set Device::clone() method as const.

## [4.1.1] - 2018-06-11

This is mostly a maintenance release.
**hpp-pinocchio** is now fully compatible with OS X systems.
The packaging have been simplified to comply with **robotpkg** policy.

## [4.0] - 2018-03-23

Initial Release

[Unreleased]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v9.0.2...HEAD
[9.0.2]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v9.0.0...v9.0.2
[9.0.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v7.0.0...v9.0.0
[7.0.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v6.1.0...v7.0.0
[6.1.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v6.0.0...v6.1.0
[6.0.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v5.2.0...v6.0.0
[5.2.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v5.1.0...v5.2.0
[5.1.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v5.0.0...v5.1.0
[5.0.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.15.1...v5.0.0
[4.15.1]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.14.0...v4.15.1
[4.14.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.13.0...v4.14.0
[4.13.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.12.0...v4.13.0
[4.12.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.11.0...v4.12.0
[4.11.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.10.2...v4.11.0
[4.10.2]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.10.1...v4.10.2
[4.10.1]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.10.0...v4.10.1
[4.10.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.9.1...v4.10.0
[4.9.1]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.9.0...v4.9.1
[4.9.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.8.0...v4.9.0
[4.8.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.7.0...v4.8.0
[4.7.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.5.0...v4.7.0
[4.5.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.4.0...v4.5.0
[4.4.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.3.0...v4.4.0
[4.3.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.2.0...v4.3.0
[4.2.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.1.1...v4.2.0
[4.1.1]: https://github.com/humanoid-path-planner/hpp-pinocchio/compare/v4.0...v4.1.1
[4.0]: https://github.com/humanoid-path-planner/hpp-pinocchio/releases/tag/v4.0
