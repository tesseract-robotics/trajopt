^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vhacd
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------
* Fixes some includes (required for newer GCC on Ubuntu 22.04)
* Contributors: Roelof Oomen

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------
* Rename trajopt_utils to trajopt_common
* Contributors: Levi Armstrong

0.5.2 (2023-06-06)
------------------

0.5.1 (2023-04-11)
------------------

0.5.0 (2023-04-09)
------------------

0.4.2 (2023-03-15)
------------------

0.4.1 (2023-03-14)
------------------

0.4.0 (2023-03-03)
------------------

0.3.1 (2022-10-23)
------------------
* CPack (`#290 <https://github.com/tesseract-robotics/trajopt/issues/290>`_)
* Contributors: Michael Ripperger

0.3.0 (2022-07-01)
------------------

0.2.5 (2022-04-24)
------------------

0.2.4 (2022-04-19)
------------------

0.2.3 (2022-03-24)
------------------

0.2.2 (2022-01-19)
------------------

0.2.1 (2021-12-16)
------------------

0.2.0 (2021-12-04)
------------------

0.1.1 (2021-11-29)
------------------

0.1.0 (2021-11-02)
------------------
* Add CMake Format Support
* Use Boost and Eigen targets
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add original author to vhacd package
* fix unit test due to removal of start_fixed
* Add windows support stage 1
* Fix warnings and update to use tesseract Manipulator Manager
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Address remaining warnings
* Make clang-tidy only run if ENABLE_CLANG_TIDY or ENABLE_TESTS is enabled
* Update based on Clang-Tidy
* Update based on Clang-Tidy and Clazy
* Disable AVX Instructions to Fix Eigen Alignment Issues
* Clange format version 8
* Fix vhacd cmake file and dependencies
* Add VHACD compile option to use C++11
* Fix kinetic c++11 cmake flag
* Add cmake support for xenial builds
* Make vhacd a pure cmake package
* Update VHACD to latest version fix folder structure
* Fix formatting using clang
* Add additional compiler warning options
* Merge pull request `#12 <https://github.com/tesseract-robotics/trajopt/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* clang format code
* Merge pull request `#9 <https://github.com/tesseract-robotics/trajopt/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Add vhacd to trajopt_ext
* Contributors: Armstrong, Levi H, Levi, Levi Armstrong, Levi-Armstrong, Matthew Powelson
