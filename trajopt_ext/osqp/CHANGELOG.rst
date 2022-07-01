^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osqp
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.5 (2022-04-24)
------------------
* Patch OSQP build to fix target output name collision (`#287 <https://github.com/tesseract-robotics/trajopt/issues/287>`_)
* Contributors: John Wason

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
* Upgrade osqp to version 0.6.2
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* trajopt_ifopt/trajopt_sqp: Changes after review
  This includes cleaning up the OSQPEigenSolver interface and a lot of style changes.
* Add trajopt_sco depend on osqp
* Add OSQP to trajopt_ext
  Now OSQP will be downloaded at compile time if it is not found
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson
