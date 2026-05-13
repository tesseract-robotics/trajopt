^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_sqp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.34.4 (2026-02-18)
-------------------
* Add warm start functionality to QPSolver
* Fix bugs and performance issues
* Update ifopt trust region solver to attempt inplace update of qp solver
* Avoid prune in ifopt code because it changes sparsity
* Disable ifopt warmstart
* Reduce memory allocations with sqp affine and quad expressions
* Update trajopt_ifopt cartesian pose constraint to align with legacy version
* Contributors: Levi Armstrong

0.34.2 (2026-02-03)
-------------------
* Remove CompositeVariables to simplify trajopt_ifopt codebase
* Add back collision data caching to trajopt ifopt
* Reduce memory allocation in trajopt_ifopt and trajopt_sqp
* Update Problem and QP Problem interface
* Verify bounds are correct based on penalty type in debug
* Update enum in trajopt_ifopt and trajopt_sqp to follow google style guide
* Add range bound support to cartesian position constraint
* Fix size bug in ifopt qp problem
* Add back cvp.gradient.setZero()
* Consolidate TrajOptQPProblem convexify
* Simplify TrajOptQPProblem convexify
* Add ComponentInfoType
* Consolidate slack variables selection into confexify method
* Add getNonZeros to trajopt_ifopt Differentiable interface
* Rename trajopt qp problem member variables
* Optimize trajopt ifopt qp problem (`#511 <https://github.com/tesseract-robotics/trajopt/issues/511>`_)
* Cleanup variable names in trajopt qp problem
* Consolidate cost and constraint info containers
* Consolidate everything into the convexify function
* Simplification by consolidation of similar components
* Simplify trajopt_ifopt by removing use of CompositeDifferentiable
* Remove unused PCL dependencies
* Add dynamic constraint support
* Remove dependency on Ifopt repository
* Separate static from dynamic variables in qp problem
* Update so there is only a single merit coeff per constraint set like the original trajopt
* Optimize expressions
* Minor update to trust region sqp solver
* Minor update to osqp eigen solver
* Cache bounds during setup
* Optimize trajopt qp problem class
* Add nodes variable set (`#438 <https://github.com/tesseract-robotics/trajopt/issues/438>`_)
* Cleanup doxygen file headers
* Remove old console bridge cmake target logic
* Switch to using Cereal for serialization
* Contributors: Levi Armstrong, Roelof Oomen

0.33.0 (2025-10-28)
-------------------

0.32.0 (2025-09-10)
-------------------
* Update OSQP dependency to v1.0.0 and OSQPEigen to v0.10.3 (`#474 <https://github.com/tesseract-robotics/trajopt/issues/474>`_)
* Fix bounds problem (`#472 <https://github.com/tesseract-robotics/trajopt/issues/472>`_)
  * Also ensure the trust box does not shrink when a variable is close to a bound
* Fix benchmarks (`#471 <https://github.com/tesseract-robotics/trajopt/issues/471>`_)
* Contributors: Roelof Oomen

0.31.0 (2025-07-06)
-------------------

0.30.0 (2025-04-23)
-------------------
* Fix issues with how Collision Check Config was being used
* Requested changes and additional bug fixes
* Fix bug in trajopt and trajopt_ifopt collision evaluators
* Contributors: Levi Armstrong

0.29.1 (2025-03-20)
-------------------

0.29.0 (2025-03-20)
-------------------
* Update to leverage std::filesystem
* Contributors: Levi Armstrong

0.28.3 (2025-01-22)
-------------------

0.28.2 (2025-01-22)
-------------------

0.28.1 (2025-01-18)
-------------------

0.28.0 (2025-01-16)
-------------------
* Fix trust region params boost serialization
* Update cmake format
* Fix cpack typo to use trajopt_common over trajopt_utils (`#436 <https://github.com/tesseract-robotics/trajopt/issues/436>`_)
* Add missing forward decl
* Add boost serialization support to config objects
* Contributors: Levi Armstrong, Max DeSantis

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------

0.25.1 (2024-09-29)
-------------------

0.25.0 (2024-09-28)
-------------------
* Add missing package libraries cmake variable
* Rename Timer to Stopwatch
* Contributors: Levi Armstrong, Roelof Oomen

0.24.0 (2024-08-14)
-------------------

0.23.2 (2024-07-31)
-------------------

0.23.1 (2024-07-24)
-------------------

0.23.0 (2024-07-24)
-------------------
* Trajopt clang-tidy clean-up v3 (`#414 <https://github.com/tesseract-robotics/trajopt/issues/414>`_)
* Trajopt clang-tidy clean-up v2
* Fixes for building on Ubuntu Noble
* Contributors: Roelof Oomen

0.22.0 (2024-06-02)
-------------------
* Update to leverage forward declarations
* Update trajopt ifopt planning unit tests to match trajopt version
* Update to use transform error diff function for numerical jacobian (`#386 <https://github.com/tesseract-robotics/trajopt/issues/386>`_)
* Fixes two forgotten csc_spfree calls
* Handle solver failures like TrajOpt does (`#369 <https://github.com/tesseract-robotics/trajopt/issues/369>`_)
* Fix trajopt ifopt collision with fixed states (`#372 <https://github.com/tesseract-robotics/trajopt/issues/372>`_)
* Contributors: Levi Armstrong, Roelof, Roelof Oomen

0.7.2 (2023-11-27)
------------------
* Fix TrajOpt Ifopt handling of constraint merit coefficient (`#366 <https://github.com/tesseract-robotics/trajopt/issues/366>`_)
* Contributors: Levi Armstrong

0.7.1 (2023-11-22)
------------------
* Fix box size setting when adjusting penalties
* Remove use of Industrial CI (`#359 <https://github.com/tesseract-robotics/trajopt/issues/359>`_)
* - Use member initializers instead of constructor. Allows for showing defaults in doc tooltips or when going to definition. Matches trajopt_sqp.
  - Clean up comments
* Contributors: Levi Armstrong, Roelof Oomen

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
