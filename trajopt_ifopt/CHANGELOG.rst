^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_ifopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------
* Move TrajOptIfopt collision gradient types and utils to trajopt_common package
* Fix ifopt continuous collision evaluator to distinguish CONTINUOUS and LVS_CONTINUOUS (`#342 <https://github.com/tesseract-robotics/trajopt/issues/342>`_)
* Contributors: Levi Armstrong

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------
* Move shared data to trajopt_common
* Rename trajopt_utils to trajopt_common
* Fix unit tests
* Filter contact pairs with zero coeffs
* Contributors: Levi Armstrong

0.5.2 (2023-06-06)
------------------
* Removed "Only Discrete Evaluator is implemented" messages
* Rename setPairCollisionMarginData to setPairCollisionCoeff
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
* Fix undefined behavior bug in processInterpolatedCollisionResults
* Contributors: Levi Armstrong

0.4.1 (2023-03-14)
------------------

0.4.0 (2023-03-03)
------------------
* catkin dependency for ROS1 only
* Contributors: Roelof Oomen

0.3.1 (2022-10-23)
------------------
* Fixes `#289 <https://github.com/tesseract-robotics/trajopt/issues/289>`_
* CPack (`#290 <https://github.com/tesseract-robotics/trajopt/issues/290>`_)
* Contributors: Michael Ripperger, Roelof Oomen

0.3.0 (2022-07-01)
------------------

0.2.5 (2022-04-24)
------------------

0.2.4 (2022-04-19)
------------------
* Update resource locator for tests
* Update tesseract joint trajectory
* Contributors: Levi Armstrong

0.2.3 (2022-03-24)
------------------

0.2.2 (2022-01-19)
------------------
* Fix processInterpolatedCollisionResults cc_type for discrete continuous
* Contributors: Levi Armstrong

0.2.1 (2021-12-16)
------------------

0.2.0 (2021-12-04)
------------------
* Add ContactManagerConfig inside CollisionCheckConfig (`#280 <https://github.com/tesseract-robotics/trajopt/issues/280>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix clang-tidy errors
* Update trajopt ifopt constraints to leverage setFromTriplets
* Remove unused header
* Contributors: Levi Armstrong, Matthew Powelson

0.1.1 (2021-11-29)
------------------
* Add coeffs to Vel, Accel, and Jerk Ifopt constraint
* Contributors: Levi Armstrong

0.1.0 (2021-11-02)
------------------
* Feature add line constraint (`#246 <https://github.com/tesseract-robotics/trajopt/issues/246>`_)
  Co-authored-by: ctlewis <colin.lewis@swri.org>
  Co-authored-by: Levi-Armstrong <levi.armstrong@gmail.com>
* Add JointAccellConstraint and JointJerkConstraint (`#275 <https://github.com/tesseract-robotics/trajopt/issues/275>`_)
* Add CMake Format Support
* Fix trajopt_ifopt inverse kinematic constraint
* Fix bug in trajopt collision term for discrete continuous
* Update cartesian pose constraints to support source and target frames
* Update to leverage Tesseract JointGroup and KinematicGroup
* Remove trajopt_ifopt dependency on trajopt
* Add clang-tidy to missing targets and add missing link target
* Fix handling of fixed states when calculating error and jacobian
* Update trajopt ifopt collision constraints to handle fixed states
* Fix bugs in trajopt_ifopt and fix unit tests
* Add continuous and discrete collision numerical constraints used for debug
* Fix unit tests
* Initialize collision jacobian to all zeros because sparsity cannot change for snopt
* Fix clang tidy errors and update to leverage .clang-tidy file
* Simplify code down to a single method of merging collision data
* Add different variants for continuous collision constraint
* Add two different variants for discrete collision constraint
* Restructure trajopt_ifopt include and src into subdirectories
* Add hinge cost support to trajopt_sqp trajopt_qp_problem
* Add unit tests for expressions and fix createQuadExprs
* Change trajopt_ifopt namespace to prevent conflicts, update cart pos constraint, sqp solver with common interface
* Exclude results at time0 and time1 when calculating values and gradients
* Share collision cache between evaluators for trajopt ifopt
* Add collision config data pointer to the collision data hash
* Fix collision constraints so GetValues also returns negative values
* Pass TrajoptCollisionCheckConfig as ConstPtr to evaluators
* Add dof to GradientResultsSet structure
* Add DiscreteCombineCollisionData structure
* Add ContinuousCombineCollisionData structure
* Add absolute cost along with unit tests for squared and absolute costs
* Add utility functions calcBoundsErrors and calcBoundsViolations with unit tests
* Add bounds check on initial values provided to JointPositionVariable
* Add unit tests for calcRotationalError and calcRotationalError2
* Add missing licenses to files
* Rename getWeightedAvgGradient to getWeightedScaledAvgGradient and normalize error weight based on max error
* Add GradientResultsSet structure
* Add weighted average gradient to LVSCollisionConstraint
* Fix how the Trust Region Results are calculated
* Initial support for LVS collision constraints
* Fix trajopt_ifopt inverse kinematics constraint unit test
* Use Boost and Eigen targets
* Update for changes with CollisionMarginData
* Update to new forward and inverse kinematics interface
* Update to latest tesseract_environment changes
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update related to changes in visualization interface
* Add exec depend on catkin and buildtool depend on cmake per REP 136
* Port LVSContinuousCollisionEvaluator to trajopt_ifopt
* Port LVSDiscreteCollisionEvaluator to trajopt_ifopt
* Switch collision constraint to use getCollisionCached
* Trajopt_ifopt: Add Collision Evaluators
* Update due to tesseract package being removed
* Fix unit test calling checkTrajectory
* Add toBounds utility that takes VectorXd
* Add trajopt_ifopt.h
* Fix to handle console_bridge target renaming in noetic
* Add public compiler option -mno-avx
* Add windows support stage 1
* Fix warnings and update to use tesseract Manipulator Manager
* Clang formatting
* Improve const-correctness of reference passing.
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Set the Eigen version for Xenial builds
* trajopt_ifopt/trajopt_sqp: Changes after review
  This includes cleaning up the OSQPEigenSolver interface and a lot of style changes.
* trajopt_ifopt/trajopt_sqp: Add Apache 2 license notices
* trajopt_sqp: Convert examples into unit tests
* Improve trajopt_sqp debug printouts
* Refactor trajopt_sqp
  Major changes:
  *  Added callbacks
  *  Added slack variables
  *  Split optimization into SQP solver, QP Problem, and QP Solver
* Trajopt_ifopt: Simple Readme
* Add numeric jacobian  calculation to cartesian cost unit tests
* Trajopt_ifopt: Add CalcValue and CalcJacobian methods to constraints
  I added these methods to make them easier to call without an IFOPT problem. The joint terms didn't seem to make a lot of sense because they operate on multiple ifopt variables.
* Trajopt_ifopt: Add utility to get closest valid point within bounds
* Trajopt_ifopt: Add InverseKinematicsConstraint and CartPos Unit Test
* Trajopt_ifopt: Minor Enhancements
* Add utilities and convenience functions for setting JointPosition bounds
* trajopt_ifopt bug fixes
* Add SQP solver based on IFOPT
* Cleanup based on review comments
* Add IFOPT Collision Constraint
* Clang Tidy Cleanup
* Add Cartesian Position Constraint
* TrajOpt IFOPT: Joint Level Costs/Constraints
  Adds the trajopt IFOPT package. Includes joint position and velocity constraints as well as the squared error cost. Includes 2 small examples of usage with IPOPT
* Contributors: Andrew Price, Colin Lewis, Levi Armstrong, Levi-Armstrong, Matthew Powelson, Michael Ripperger
