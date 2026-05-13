^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osqp_eigen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.34.4 (2026-02-18)
-------------------
* Change name of OsqpEigen Ext package, as it collided with OsqpEigen proper
* Add nodes variable set (`#438 <https://github.com/tesseract-robotics/trajopt/issues/438>`_)
* Contributors: Levi Armstrong, Roelof Oomen

0.33.0 (2025-10-28)
-------------------
* Update to osqp_eigen 0.11.0
* Contributors: Levi Armstrong

0.32.0 (2025-09-10)
-------------------
* Update OSQP dependency to v1.0.0 and OSQPEigen to v0.10.3 (`#474 <https://github.com/tesseract-robotics/trajopt/issues/474>`_)
* Fix disable external cmake variable names
* Add ability to disable packages in external build via cmake
* Contributors: Levi Armstrong, Roelof Oomen

0.31.0 (2025-07-06)
-------------------

0.30.0 (2025-04-23)
-------------------

0.29.1 (2025-03-20)
-------------------

0.29.0 (2025-03-20)
-------------------

0.28.3 (2025-01-22)
-------------------

0.28.2 (2025-01-22)
-------------------

0.28.1 (2025-01-18)
-------------------

0.28.0 (2025-01-16)
-------------------
* Update cmake format
* Contributors: Levi Armstrong

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------

0.25.1 (2024-09-29)
-------------------

0.25.0 (2024-09-28)
-------------------

0.24.0 (2024-08-14)
-------------------

0.23.2 (2024-07-31)
-------------------

0.23.1 (2024-07-24)
-------------------

0.23.0 (2024-07-24)
-------------------

0.22.0 (2024-06-02)
-------------------

0.7.2 (2023-11-27)
------------------

0.7.1 (2023-11-22)
------------------

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
