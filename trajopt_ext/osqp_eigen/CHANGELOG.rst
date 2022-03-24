^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osqp_eigen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Update osqp_eigen to use upstream and v0.6.3
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Set the Eigen version for Xenial builds
* trajopt_ifopt/trajopt_sqp: Changes after review
  This includes cleaning up the OSQPEigenSolver interface and a lot of style changes.
* trajopt_ifopt: Misc cleanup for pull request
* Refactor trajopt_sqp
  Major changes:
  *  Added callbacks
  *  Added slack variables
  *  Split optimization into SQP solver, QP Problem, and QP Solver
* Add OSQP_Eigen to Trajopt_Ext
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson
