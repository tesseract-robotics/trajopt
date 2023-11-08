^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------
* Move shared data to trajopt_common
* Rename trajopt_utils to trajopt_common
* Add multi-threaded support to TrajOpt (`#333 <https://github.com/tesseract-robotics/trajopt/issues/333>`_)
* Enable qpOASES on Windows and fix Windows build (`#327 <https://github.com/tesseract-robotics/trajopt/issues/327>`_)
* Filter contact pairs with zero coeffs
* Contributors: John Wason, Levi Armstrong

0.5.2 (2023-06-06)
------------------
* MSVC 2022 build fixes (`#323 <https://github.com/tesseract-robotics/trajopt/issues/323>`_)
* Fix CollisionsToDistanceExpressionsW versions
* Contributors: John Wason, Levi Armstrong

0.5.1 (2023-04-11)
------------------
* Fix debug build
* Contributors: Levi Armstrong

0.5.0 (2023-04-09)
------------------
* Update to support new contact results class
* Contributors: Levi Armstrong

0.4.2 (2023-03-15)
------------------
* Fix undefined behavior bug in processInterpolatedCollisionResults
* Contributors: Levi Armstrong

0.4.1 (2023-03-14)
------------------
* Update CI builds
* Fix places where reserve should be used to reduce number of memory allocations
* Update collision cache to use vector of std::reference_wrapper to memory usage
* Reduce heap allocations in the collision results cache
* Contributors: Levi Armstrong

0.4.0 (2023-03-03)
------------------
* Fix unit tests and add solve benchmarks
* Move json include to cpp files
* Contributors: Levi Armstrong

0.3.1 (2022-10-23)
------------------
* Fix jsoncpp link targets and sparseview issue (`#299 <https://github.com/tesseract-robotics/trajopt/issues/299>`_)
* CPack (`#290 <https://github.com/tesseract-robotics/trajopt/issues/290>`_)
* Contributors: Michael Ripperger, Roelof

0.3.0 (2022-07-01)
------------------
* Simplify PlotCallback
* Add the ability to store sco::Optimizer::Callbacks in ProblemConstructionInfo class
* Contributors: Levi Armstrong

0.2.5 (2022-04-24)
------------------
* Protect jsoncpp against multiple find_package() calls (`#288 <https://github.com/tesseract-robotics/trajopt/issues/288>`_)
* Contributors: John Wason

0.2.4 (2022-04-19)
------------------
* Update resource locator for tests
* Update tesseract joint trajectory
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
* Fix processInterpolatedCollisionResults cc_type for discrete continuous
* Contributors: Levi Armstrong

0.2.1 (2021-12-16)
------------------

0.2.0 (2021-12-04)
------------------
* Add ContactManagerConfig inside CollisionCheckConfig (`#280 <https://github.com/tesseract-robotics/trajopt/issues/280>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Remove unused header
* Contributors: Matthew Powelson

0.1.1 (2021-11-29)
------------------

0.1.0 (2021-11-02)
------------------
* Feature add line constraint (`#246 <https://github.com/tesseract-robotics/trajopt/issues/246>`_)
  Co-authored-by: ctlewis <colin.lewis@swri.org>
  Co-authored-by: Levi-Armstrong <levi.armstrong@gmail.com>
* Update to support new kinematic plugin format and contact managers
* Add CMake Format Support
* Fix trajopt_ifopt inverse kinematic constraint
* Fix bug in trajopt collision term for discrete continuous
* Update cartesian pose constraints to support source and target frames
* Update to leverage Tesseract JointGroup and KinematicGroup
* Remove trajopt_ifopt dependency on trajopt
* Fix bugs in trajopt_ifopt and fix unit tests
* Fix clang tidy errors and update to leverage .clang-tidy file
* Add absolute cost support to trajopt_sqp trajopt_qp_problem
* Add hinge cost support to trajopt_sqp trajopt_qp_problem
* Share collision cache between evaluators for trajopt ifopt
* Add absolute cost along with unit tests for squared and absolute costs
* Add unit tests for calcRotationalError and calcRotationalError2
* Initial support for LVS collision constraints
* Fix JointPosIneqConstraint
* Use Boost and Eigen targets
* Remove checkJoints check no longer needed
* Update to new forward and inverse kinematics interface
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Correctly use lower_tol in JointPosIneqCost AffExpr
* Fix misnaming of constraints
* Update related to changes in visualization interface
* Update unit tests
* Remove use of new operator
* Add exec depend on catkin and buildtool depend on cmake per REP 136
* fix unit test due to removal of start_fixed
* Improve fixed timesteps and dofs
* Update due to tesseract package being removed
* Fix unit test calling checkTrajectory
* Clean up contact manager warnings
* Fix to handle console_bridge target renaming in noetic
* Add public compiler option -mno-avx
* Add windows support stage 1
* Expose tesseract object in problem description
* Fix warnings and update to use tesseract Manipulator Manager
* Update do to changes in tesseract limits
* Clang formatting
* Updated avoid singularity cost name
* Changed dofs_fixed name to fixed_timesteps
* Improve const-correctness of reference passing.
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Remove Boost Python dependency in trajopt
* Disable test
* Add Flag to collision evaluator for dynamic environments
  If set, the state is pulled from the environment rather than from the frozen state solver
  fix clang
* Install trajopt test data for use in other packages
* Add Ptr and ConstPtr to collision evaluator implementations
  Otherwise it calls the base class which can lead to perplexing errors when using methods not in the base class.
* Add macro to run benchmarks if -DTRAJOPT_ENABLE_RUN_BENCHMARKING=ON
* Add Joint Term Benchmarks
* Update to use renamed EnvState member link_transforms
* Add CalcCollisions that takes only joint values instead of Vars
* Fix bug in collision getGradient
* Add GetGradient function to CollisionEvaluator
* Updated trajopt planning unit test to use OSQP
* Updated planning unit test solver to BPMPD
* Fixed bug in collision interpolation step
* Check init_info.data.size() when using JOINT_INTERPOLATED
  Allow either 1 x DOF or DOF x 1.
* Use std::move when calling addLink
  The unit tests were broken in a recent Tesseract PR.
* Add eigen to package.xml
  and alphabetize the entries.
* Clang formatting
* Add ability to use weighted sum jac calculation for contact link pairs
* Add a safety margin buffer to collision evaluators (`#160 <https://github.com/tesseract-robotics/trajopt/issues/160>`_)
  * add missing osqp dependency to trajopt_sco
  * Fix bug where optimization returned SCO iteration limit even if problem converged successfully
  * Add safety_margin_buffer to evaluate close contacts that are out of collision
  Co-authored by: Levi Armstrong <levi.armstrong@gmail.com>
  Co-authored by: Joe Schornak <joe.schornak@gmail.com>
  * Clang format
  * Remove duplicate osqp depend.
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Available solvers priority is set by the ModelType Value enum so make BPMPD last due to license
* Update trajopt unit tests to use fixed_steps and longest_valid_segment_length in collision term
* Change Eigen arguments that are passed by value to reference
* Add discrete continuous collision checking
* Update casted collision evaluator to handle fixed start and end states
* Remove the additional 0.04 added to contact distance threshold
* Change CastCollisionEvaluator::CalcCollisions to include all contacts for sub trajectories
* Add documentation to CastCollisionEvaluator::CalcCollisions code changes
* Fix spelling
* Add longest valid segment length to Continuous collision
* Enable continuous collision checking for moving to moving objects
* Pull request review changes
* Clang format
* Added singularity avoidance cost for subset of optimization problem variable state
* Added avoid singularity cost
* Rebase Fixes
* Inflate only constraints that fail
  This changes the default behavior of the SQP optimizer to only inflate the merit coefficients associated with constraints that are not satisfied. This should make it less important that the constraints have been manually balanced.
* Update test due to changes in tesseract checkTrajectory
* Add processing of header files to clang-tidy
* Change how unit test are ran
* Set trajopt log level to Error to limit CI error log to long
* Update due to changes in checkTrajectory function
* Address remaining warnings
* Add ability to add user defined trajopt constraint type and coeff
* Update based on Clang-Tidy
* Update based on Clang-Tidy and Clazy
* Use ResourceLocator instead of ResourceLocaterFn in tesseract unit tests
* Add user defined term info
* Disable jacobian calculator for cart pose and dynamic cart pose
* Fixed incorrect orientation error calculation
* Fix CartPoseTermInfo empty target
* Replaced exception handling with a throw instead of print
* Address issues per PR review
* Updated test .json file
* Added exception handling for transform lookup
* Changed Cartesian pose term info to accept poses defined relative to a specified frame
* Disable AVX Instructions to Fix Eigen Alignment Issues
* Add OptStatus to TrajOptResult
* Clang format
* Add DynamicCartPoseJacCalculator
* Fix CartPoseJac
* Explicit removal of functions if coeff is zero in CartPose
* Fix cart pose jacobian calculation and unit test
* Switch to using angle axis for rotational error
* Add jacobian to CartPoseTermInfo
* Added target TCP transform to dynamic cartesian pose error calculator instead of using default identity transform
* Add ability to log iteration results to files
* Improvements to Json parameters
* Update DynamicCartPose to allow target link tcp
* Add Plot Callback that doesn't require the problem
  This is important for Tesseract Planners
* Add assert in kinematic_terms for null kinematic link
* Cherry pick bmagyar@094c49398c919958617aba2a8afeb87731099e7e
* Add documentation to SafetyMarginData class and rename SetSafetyMarginData to setSafetyMarginData
* Fix collision term info CalcCollision
* Add dependencies for tests on package libraries
* Fix clang warnings
* Clange format version 8
* Unify shared pointer definition and switch typedef to using
* Fix find_dependency for components in kinetic again
* Update unit tests
* Namepsace targets and update to use tesseract namespace targets
* Fix kinetic c++11 cmake flag
* Add cmake support for xenial builds
* Update to use tesseract kinematics factory
* Change problem description constructor to take tesseract object
* Update to use tesseract class
* Update test
* Fix issue with jacobian calculation collision terms
* Add update to allowed collision matrix in cast_cost_attached_unit
* Update for tesseract_environment changing getState to getCurrentState
* Add console_bridge and remove rosconsole and fix tests
* Clean up config.cmake and update due to changes in tesseract
* Update to work with new version of tesseract
* fixup
* Update to account for changes in tesseract_collision
* Clang Format
  Hopefully will pass Travis now.
* Fix Total Time jacobian - and misc other small bug/doc fixes
* Add cblock to BasicArray
  Used to clean up some dirty code in problem_description. This commit also includes some minor changes that got lost in the rebase somehow.
* Add jointVel with time unit test
  Also fixes some bugs that it exposed
* Convert to using (1/dt) and added total time cost
* Add time param joint cost/cnt
* Add term_type switch for time parameterization
* Add unit tests to initial trajectory via json and other fixes
* Add term_type switch for time parameterization
* Replace GetJointVarRow with GetVarRow
* Add unit tests to initial trajectory via json and other fixes
* Add term_type switch for time parameterization
* Fix clang formatting
* Fix test warnings
* Add target specific compiler flags
* Add flag to allow Collision constraints
  This is just a bug fix. The functionality was already there. The flag was just not set.
* Fix Joint Term Default Values
  Time interval defaults to the whole problem. Updated the docs to state that coeffs has a default value, but targets is required. Also updated the examples to add the time steps to them.
* Bug fixes for examples
* Fix formatting using clang
* Replace GetJointVarRow with GetVarRow
* Add check that costs/cnts are pushed to correct term info
* Add unit tests to initial trajectory via json and other fixes
* Add initial trajectory unit tests
* Add term_type switch for time parameterization
* Add additional compiler warning options
* Change coeffs default to 1 and remove default target
* Update loops to be refs
  Replace  for (sco::AffExpr expr : expr_vec\_) with  for (sco::AffExpr& expr : expr_vec\_)
* Update jointPos term
* Merge pull request `#40 <https://github.com/tesseract-robotics/trajopt/issues/40>`_ from arocchi/add_free_solvers_upstream
  Adds osqp and qpOASES solver interfaces
* Renamed ConvexSolver into ModelType
* Merge remote-tracking branch 'rosind/kinetic-devel' into add_free_solvers_upstream
* Refactors and Doc updates
  Addresses comments from review. Renamed targs to targets, eliminated unneeded for loops, added some documentation, and removed errant TODOs.
* Update JointAcc and JointJerk costs/constraints
* Fixes for rebase removing using namespace
* Joint Trajectory costs fixes
  Store each expression seperately to avoid them cancelling out, and realized that ExprMult does not multiply in place.
* Inequality Terms fixed
* Add Unit Test
* Added time step limits
* Bug Fixes
* Add joint velocity constraint with tolerance
* Add joint velocity cost tolerance
* Update Docs
* Eigen alignment fixes
* Remove the use of 'using namespace'
* ProblemConstructionInfo now contains info on which convex solver to use
* Merge remote-tracking branch 'levi/kinetic-devel' into add_free_solvers_upstream
* Merge remote-tracking branch 'rosind/kinetic-devel' into add_free_solvers_upstream
* Remove the use of 'using namespace'
* Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW to struct/classes that have fixed size eigen member variables
* Fix Unit Tests
  Also changes position constraint from a limit to an equality (This is what the test needed). This is probably a more common use case than the limit anyway. Regardless, this will be resolved in the next PR overhauling the joint cost/constraints.
* Remove currently unused parameters
* Change back to CartPose from CartPos
* Fixes and more changes to increase uniformity in naming
  Renamed ConstraintFromFunc to ConstraintFromErrFunc to match cost version.
  Dropped the "static" from StaticCartPosErrCalculator and added dynamic to the dynamic one.
  Fixed some Doxygen comments
* Add constraints to joint terms
* Update examples and minor fixes
* Add Cost/Constraint Switch to CartVelTermInfo
  Also ran Clang format which changed a few things
* Add Doxygen comments to the term infos and error calculators
* Rename costs/constraints to "Terms" with switches
  Also renamed error calculators to match the terms that they are used to create. The goal is to make the whole system less confusing.
* Add pr2_description test depend
* File Write Calback: Change to const
* File Write Callback: Update License and minor fixes
* File Write Callback: Add License Info
* Refactor file write callback
* Clean up file write callback
  Made proposed changes and fixed one small bug in the plot script
* Add script to plot costs vs iteration
  Also renamed scripts to avoid confusion
* Add writing costs/constraints to file_write_callback
* Change callbacks from taking only the x matrix to the whole results obj
* File write callback - Change affine3d to isometry3d
* Clang format file writing callback
* Removed pose inverses/errors and changed file name arg to ofstream object in file writing utility
* Added file_write_callback.cpp to its CMakesList
* Added file writing and graphing utilities as a way to compare produced trajectories
* Add pcl_conversions depends
* Fix pcl depends
* Add test depends to trajopt pacakge (`#30 <https://github.com/tesseract-robotics/trajopt/issues/30>`_)
  * Add libpcl-dev test depends to trajopt pacakge
  * Add trajopt_test_support test depends to trajopt pacakge
  * Add octomap_ros test depends to trajopt pacakge
* Add cmake install command
* Fixed copy-paste error in JointJerkCost::value
* Refractor out tesseract ContactRequest type
* Add plotting of collision jacobian vector
* Add ability to plot for costs from error functions and fix axis plotter
* Jacobian should be a 6 x N matrix, not a N x 6; was trigger faults or asserts (`#14 <https://github.com/tesseract-robotics/trajopt/issues/14>`_)
* Use isometry (`#11 <https://github.com/tesseract-robotics/trajopt/issues/11>`_)
  * Update to use new tesseract contact managers
  * switch from using affine3d to isometry3d
* Update to use new tesseract contact managers (`#10 <https://github.com/tesseract-robotics/trajopt/issues/10>`_)
* Merge pull request `#1 <https://github.com/tesseract-robotics/trajopt/issues/1>`_ from Levi-Armstrong/fixSubmodule
  Fix submodule and trajopt_sco unit tests
* Remove submodule for bullet3
* Merge pull request `#12 <https://github.com/tesseract-robotics/trajopt/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* Add kdl_joint_kin to handle auxillary axes
* Fix kdl_chain_kin to handle links not in chain
* Make use of Eigen::Ref
* clang format code
* Merge pull request `#11 <https://github.com/tesseract-robotics/trajopt/issues/11>`_ from larmstrong/unusedParamWarn
  Fix remaining warning
* Uncomment unused names in headers
* Fix planning_unit.cpp test
* Fix remaining warning
* Merge pull request `#10 <https://github.com/tesseract-robotics/trajopt/issues/10>`_ from larmstrong/mergeJMeyer
  Merge jmeyer pull requests
* Merge pull request `#9 <https://github.com/tesseract-robotics/trajopt/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Removed warnings again. Just too many in included libraries to deal with.
* Gobs more small fixups. I don't believe I changed anything that would affect actual logic.
* Removed use of deprecated JSON_CPP function calls
* Cleaning up warnings
* Fix contact monitoring
* Create custom rviz environment plugin
* Add Car Seat Example
* Add ability to define collision object type
* Refractor collision checking into its own package
* Switch boost::function to std::function
* Switch boost::shared_ptr to std::shared_ptr
* Add missing license information
* Rename DistanceRequest DistanceResults to ContactRequest ContactResults
* Separate Plotting from environment and fix object color typedef
* Add tesseract packages
* replace std::map with std::unordered_map
* Make AllowedCollisionMatrix a class
* replace trajopt_scene with tesseract package
* Add ability to set safety margin for link pairs
* Move data directory content to trajopt_test_support/config directory
* Remove const from std::map key
* Add ability to visualize trajopt_scene using robot state
* Move moveit items to its own package and create trajopt_scene package
* Remove moveit depend from ros_kin_chain
* Add system depend to CMakeLists.txt
* Fix bug in collision_common.h
* Add ability to get global minimum for pair instead of just all
* Move the plotWaitForInput to the plot callback function
* Rename ROSKin to ROSKinChain and add JointAccCost JointJerkCost
* Rename getManipulatorKin to getManipulator
* Add alternative continuousCollisionCheckTrajectory function
* Integrate changes to moveit collision
* Add tcp capability to kinematics_terms
* Update the iiwa dae to be shadeless
* Fix commented out plotting calls
* Add ability to publish axes
* Remove additional refferences to openrave
* Make distance and collision calls const and fix ROS_INFO warnings
* Add glass up right example
* Expose optimization parameters to user via cpp and json
* Remove the use of global ProblemConstructionInfo variable when parsing json data
* Add trajopt_examples package with one cartesian example
* Remove old json unit tests
* Remove old test collision-checker-unit
* Remove local version of jsoncpp
* Remove pr2 moveit_cofig package
* Add octomap unit test and fix convert bullet convertBulletCollisions
* Add test for objects attached to links without geometry
* Fix bullet collision to handle attached object connected to links without geometry
* Fix use of attached collision objects and add a unit test for it
* Make use of BULLET_DEFAULT_CONTACT_DISTANCE
* Implement remaining collision_robot bullet methods
* Add attached object functionality
* Add collision world test and make use of xacros
* Integrate collision world
* Update isCollision allowed to handle Attached objects
* Change link2cow typedef
* Remove temp file
* Add/Update cast cost unit test
* Remove osgviewer package
* Switch planning unit test to use ROS_DEBUG
* Fix continuous collision checking and add original cast method
* Add Continuous Collision Checking and Filter Masking
* Add plotting parameter to trajopt_planning_unit
* MoveIt Bullet Collision Checker (Single State)
* Second pass at planning-unit test
* First pass at planning-unit test
* Working numerical ik test
* Fixup
* Add test support package and moveit config package
* Divide package into multiple packages
* Contributors: Alessio Rocchi, Andrew Price, Armstrong, Levi H, Colin Lewis, Hervé Audren, John Wason, Jonathan Meyer, Joseph Schornak, Levi, Levi Armstrong, Levi-Armstrong, Matthew Powelson, Michael Ripperger, Reid Christopher, mpowelson, mripperger
