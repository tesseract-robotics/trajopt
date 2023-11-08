^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_sqp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------
* Move TrajOptIfopt collision gradient types and utils to trajopt_common package
* - Fix termination condition for the trust region loop to match trajopt_sco and the original paper.
  - Add initial_merit_error_coeff to match trajopt_sco.
* Disable OSQP_COMPARE_DEBUG_MODE
* Remove osqp headers from osqp_eigen_solver (`#344 <https://github.com/tesseract-robotics/trajopt/issues/344>`_)
* Fix osqp eigen solver hessian and linear constraint matrix
* Fix ifopt continuous collision evaluator to distinguish CONTINUOUS and LVS_CONTINUOUS (`#342 <https://github.com/tesseract-robotics/trajopt/issues/342>`_)
* Fix TrustRegionSQPSolver solve method to correctly set status (`#341 <https://github.com/tesseract-robotics/trajopt/issues/341>`_)
* Contributors: Levi Armstrong, Roelof Oomen

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------
* Move shared data to trajopt_common
* Rename trajopt_utils to trajopt_common
* Enable qpOASES on Windows and fix Windows build (`#327 <https://github.com/tesseract-robotics/trajopt/issues/327>`_)
* Contributors: John Wason, Levi Armstrong

0.5.2 (2023-06-06)
------------------
* Change error into warning for "Approximate merit function got worse [...]"
* Contributors: Roelof Oomen

0.5.1 (2023-04-11)
------------------

0.5.0 (2023-04-09)
------------------
* Improve trajopt ifopt collision evaluators (`#308 <https://github.com/tesseract-robotics/trajopt/issues/308>`_)
  * Updated trajopt ifopt collision evaluators to create an equation for each shape pair
  * Add fixed_sparsity param to collision constraint classes
  * Fix clang-tidy errors
* Update to support new contact results class
* Contributors: Levi Armstrong

0.4.2 (2023-03-15)
------------------

0.4.1 (2023-03-14)
------------------
* Fix places where reserve should be used to reduce number of memory allocations
* Contributors: Levi Armstrong

0.4.0 (2023-03-03)
------------------
* Fix unit tests and add solve benchmarks
* catkin dependency for ROS1 only
* Contributors: Levi Armstrong, Roelof Oomen

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
* Update resource locator for tests
* Contributors: Levi Armstrong

0.2.3 (2022-03-24)
------------------
* Expose convex solver settings and set ospq adaptive_rho to default value (`#285 <https://github.com/tesseract-robotics/trajopt/issues/285>`_)
  * Expose convex solver settings and set ospq adaptive_rho to default value
  * Fix windows CI build
  * Fix unit tests
  Co-authored-by: Tyler Marr <tylermarr17@gmail.com>
* Contributors: Levi Armstrong

0.2.2 (2022-01-19)
------------------

0.2.1 (2021-12-16)
------------------

0.2.0 (2021-12-04)
------------------
* Add ContactManagerConfig inside CollisionCheckConfig (`#280 <https://github.com/tesseract-robotics/trajopt/issues/280>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix clang-tidy errors
* Fix bug in verifySQPSolverConvergence and adjustPenalty
* Remove unused header
* Contributors: Levi Armstrong, Matthew Powelson

0.1.1 (2021-11-29)
------------------
* Add coeffs to Vel, Accel, and Jerk Ifopt constraint
* Contributors: Levi Armstrong

0.1.0 (2021-11-02)
------------------
* Add JointAccellConstraint and JointJerkConstraint (`#275 <https://github.com/tesseract-robotics/trajopt/issues/275>`_)
* Add CMake Format Support
* Update cartesian pose constraints to support source and target frames
* Update to leverage Tesseract JointGroup and KinematicGroup
* Remove trajopt_ifopt dependency on trajopt
* Add clang-tidy to missing targets and add missing link target
* Update trajopt ifopt collision constraints to handle fixed states
* Fix bugs in trajopt_ifopt and fix unit tests
* Add continuous and discrete collision numerical constraints used for debug
* Fix clang tidy errors and update to leverage .clang-tidy file
* set super debug to false
* Simplify code down to a single method of merging collision data
* Restructure trajopt_ifopt include and src into subdirectories
* Fix trajopt_qp_problem evaluateConvexCosts
* Add absolute cost support to trajopt_sqp trajopt_qp_problem
* Add hinge cost support to trajopt_sqp trajopt_qp_problem
* The objective function hessian needs to be multiplied by 2 for OSQP because it multiplies by 0.5
* Add unit tests for expressions and fix createQuadExprs
* Simplify trajopt_sqp units leveraging new QPProblem Interface
* Add trajopt problem unit test for the planning unit test
* Clean up squared cost and create AffExprs and QuadExprs for trajopt_sqp
* Fix squared cost calculation gradient and hessian calculation using old trajopt exprSquare
* Add TrajOptQPProblem unit tests
* Update trust_region_sqp_solver to leverage qp_problem interface
* Change trajopt_ifopt namespace to prevent conflicts, update cart pos constraint, sqp solver with common interface
* Share collision cache between evaluators for trajopt ifopt
* Pass TrajoptCollisionCheckConfig as ConstPtr to evaluators
* Add dof to GradientResultsSet structure
* Add DiscreteCombineCollisionData structure
* Add ContinuousCombineCollisionData structure
* Add absolute cost along with unit tests for squared and absolute costs
* Add utility functions calcBoundsErrors and calcBoundsViolations with unit tests
* Add documentation related to slack variables
* Add missing licenses to files
* Rename getWeightedAvgGradient to getWeightedScaledAvgGradient and normalize error weight based on max error
* Add setBoxSize to TrustRegionSQPSolver for online planning
* Break up functions further
* Split TrustRegionSQPSolver Solve function into multiple functions
* Cleanup Trust Region printStepInfo
* Add weighted average gradient to LVSCollisionConstraint
* Fix how the Trust Region Results are calculated
* Initial support for LVS collision constraints
* Use Boost and Eigen targets
* Update to new forward and inverse kinematics interface
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update related to changes in visualization interface
* Add exec depend on catkin and buildtool depend on cmake per REP 136
* fix unit test due to removal of start_fixed
* Clean up QPSolverStatus in trajopt_sqp
* Clean up SQPStatus in trajopt_sqp
* Update due to tesseract package being removed
* Fix to handle console_bridge target renaming in noetic
* Add public compiler option -mno-avx
* Add windows support stage 1
* Fix warnings and update to use tesseract Manipulator Manager
* Improve const-correctness of reference passing.
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Add init method to trust region sqp solver
  Need some way of initializing when not using the Solve method.
* Fix trajopt_sqp cart_position_optimization_unit test
* trajopt_ifopt/trajopt_sqp: Changes after review
  This includes cleaning up the OSQPEigenSolver interface and a lot of style changes.
* trajopt_ifopt: Misc cleanup for pull request
* trajopt_ifopt/trajopt_sqp: Add Apache 2 license notices
* trajopt_sqp: Add clear plotter and wait for input callbacks
  These are necessary since the callbacks are divided up now and not associated with the cost terms themselves. To replicate trajopt_sco behavior add a clear plotter callback, then the cost term callbacks, and finally the wait for input.
* trajopt_sqp: Convert examples into unit tests
* Improve trajopt_sqp debug printouts
* Refactor trajopt_sqp
  Major changes:
  *  Added callbacks
  *  Added slack variables
  *  Split optimization into SQP solver, QP Problem, and QP Solver
* Trajopt_ifopt: Minor Enhancements
* trajopt_ifopt bug fixes
* Add SQP solver based on IFOPT
* Contributors: Andrew Price, Levi Armstrong, Levi-Armstrong, Matthew Powelson
