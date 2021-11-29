^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osqp
^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2021-11-29)
------------------

0.1.0 (2021-11-02)
------------------
* Add CMake Format Support
* Upgrade osqp to version 0.6.2
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* trajopt_ifopt/trajopt_sqp: Changes after review
  This includes cleaning up the OSQPEigenSolver interface and a lot of style changes.
* Add trajopt_sco depend on osqp
* Add OSQP to trajopt_ext
  Now OSQP will be downloaded at compile time if it is not found
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson
