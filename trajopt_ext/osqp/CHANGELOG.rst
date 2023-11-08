^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osqp
^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------

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
* Update windows CI to leverage colcon
* Contributors: Levi Armstrong, Michael Ripperger

0.3.0 (2022-07-01)
------------------

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
