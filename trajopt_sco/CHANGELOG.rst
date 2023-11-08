^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_sco
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-11-07)
------------------
* Fixes some includes (required for newer GCC on Ubuntu 22.04)
* Disable OSQP_COMPARE_DEBUG_MODE
* Fix osqp eigen solver hessian and linear constraint matrix
* Contributors: Levi Armstrong, Roelof Oomen

0.6.1 (2023-07-10)
------------------
* Add qpOASES cmake option to trajopt_sco
* Contributors: Levi Armstrong

0.6.0 (2023-06-30)
------------------
* Rename trajopt_utils to trajopt_common
* Fix clang-tidy errors
* Add multi-threaded support to TrajOpt (`#333 <https://github.com/tesseract-robotics/trajopt/issues/333>`_)
* Fix enable and clang-tidy errors (`#332 <https://github.com/tesseract-robotics/trajopt/issues/332>`_)
* Enable qpOASES on Windows and fix Windows build (`#327 <https://github.com/tesseract-robotics/trajopt/issues/327>`_)
* Filter contact pairs with zero coeffs
* Contributors: John Wason, Levi Armstrong

0.5.2 (2023-06-06)
------------------

0.5.1 (2023-04-11)
------------------

0.5.0 (2023-04-09)
------------------
* Update to support new contact results class
* updated optimizer error reporting. Downgraded two ERRORs to WARNs
* Contributors: Levi Armstrong, mike-o

0.4.2 (2023-03-15)
------------------

0.4.1 (2023-03-14)
------------------
* Fix places where reserve should be used to reduce number of memory allocations
* Fix trajopt_sco exprToEigen larg heap allocations
* Contributors: Levi Armstrong

0.4.0 (2023-03-03)
------------------
* Move json include to cpp files
* catkin dependency for ROS1 only
* Contributors: Levi Armstrong, Roelof Oomen

0.3.1 (2022-10-23)
------------------
* Fix jsoncpp link targets and sparseview issue (`#299 <https://github.com/tesseract-robotics/trajopt/issues/299>`_)
* Build Failure with GUROBI support (`#296 <https://github.com/tesseract-robotics/trajopt/issues/296>`_)
  * trajopt_sco: include recent gurobi version
  * trajopt_sco: fix gurobi solver interface
* CPack (`#290 <https://github.com/tesseract-robotics/trajopt/issues/290>`_)
* Contributors: Michael Ripperger, Roelof, cpetersmeier

0.3.0 (2022-07-01)
------------------

0.2.5 (2022-04-24)
------------------
* Protect jsoncpp against multiple find_package() calls (`#288 <https://github.com/tesseract-robotics/trajopt/issues/288>`_)
* Contributors: John Wason

0.2.4 (2022-04-19)
------------------

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
* On final convex solver failure attempt set trust region to minimum (`#281 <https://github.com/tesseract-robotics/trajopt/issues/281>`_)
* Contributors: Levi Armstrong

0.2.0 (2021-12-04)
------------------
* Update trajopt ifopt constraints to leverage setFromTriplets
* Update legacy trajopt print table
* Contributors: Levi Armstrong

0.1.1 (2021-11-29)
------------------
* TrajOpt timeout check constraint convergence and if satisfied report converged
* Contributors: Levi Armstrong

0.1.0 (2021-11-02)
------------------
* Add CMake Format Support
* Fix clang tidy errors and update to leverage .clang-tidy file
* set super debug to false
* Add hinge cost support to trajopt_sqp trajopt_qp_problem
* Change trajopt_ifopt namespace to prevent conflicts, update cart pos constraint, sqp solver with common interface
* Test fix for bpmpd_caller copy issue during CI
* Add setBoxSize to TrustRegionSQPSolver for online planning
* Initial support for LVS collision constraints
* Shrink trust region if convex solver fails
* Use Boost and Eigen targets
* Implement max_time
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update related to changes in visualization interface
* Update unit tests
* Switch VarRep variable in Var from raw pointer to shared pointer
* Remove use of new operator
* Fix memory leak in osqp_interface
* Add exec depend on catkin and buildtool depend on cmake per REP 136
* Fix test dependencies for Windows builds
* Catch exception when OSQP setup fails
* Add public compiler option -mno-avx
* Add windows support stage 1
* Fix warnings and update to use tesseract Manipulator Manager
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Add super debug mode to trajopt_sco
  Adds a flag to the SQP optimizer and the OSQP interface. If both are true, then it prints all SQP Results, writes the model to a file in a human readable format, and prints the optimizer matrices to the terminal.
* Add getClosestFeasiblePoint that doesn't use a QP
  The old method solved a QP to get a feasible point, but that could sometimes change the starting point from the one passed in.
* Fix different parameters error from clang-tidy
* Clang format
* explicitly cast type conversions to satisfy warnings
* Update FindGUROBI.cmake to match newer example
* handle both signed and unsigned index vector types
* Updated OSQP default argument to not use an adaptive rho
* Add eigen to package.xml
  and alphabetize the entries.
* Add a safety margin buffer to collision evaluators (`#160 <https://github.com/tesseract-robotics/trajopt/issues/160>`_)
  * add missing osqp dependency to trajopt_sco
  * Fix bug where optimization returned SCO iteration limit even if problem converged successfully
  * Add safety_margin_buffer to evaluate close contacts that are out of collision
  Co-authored by: Levi Armstrong <levi.armstrong@gmail.com>
  Co-authored by: Joe Schornak <joe.schornak@gmail.com>
  * Clang format
  * Remove duplicate osqp depend.
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Compose triplet vector directly
* initialize sparse matrix using vectors of triples instead of coeffRef
* Available solvers priority is set by the ModelType Value enum so make BPMPD last due to license
* Add compiler defines to the target instead of cpp file
* Revert use of unique_ptr for osqpworkspace
* Fix issue in osqp interface to update osqp_data object with new A and P
* Add LP format printing to OSQP solver (`#161 <https://github.com/tesseract-robotics/trajopt/issues/161>`_)
  * Add LP format printing to OSQP solver
  Create a few helpers in osqp_printing and use those when solving fails.
  * Remove OSQP printing
  * Mark writeToFile const and implement it for OSQP
  * Revert change in iostream include
  * Fix redundant string initialization
* Replace OSQPWorkspace* with unique_ptr
  This is meant to handle the memory management of the OSQPWorkspace in a single location.
* Add A and P as unique ptrs to OSQP interface
* Change Eigen arguments that are passed by value to reference
* Add NOLINT to freeing c members
* Fix OSQP Interface for Clang-tidy
  This includes changing the index stored in VarRep to a std::size_t from an int.
* Add trajopt_sco depend on osqp
* Change OSQP to the default solver after Gurobi
  This is due to licensing restrictions. Now the default solver will have a permissive license. BPMPD is still included and can be enabled by setting convex_solver to BPMPD.
* Just some formatting
* Use triangular matrices and throw if setup fails
* Update interface to OSQP 0.6.0
* Add merit coeff to print out and fix error in cntsToCosts function
* Fix clang-tidy errors
* Rebase Fixes
* Inflate only constraints that fail
  This changes the default behavior of the SQP optimizer to only inflate the merit coefficients associated with constraints that are not satisfied. This should make it less important that the constraints have been manually balanced.
* Add new_exact to Trajopt debug output
  While you could get this information by comparing across iterations, I find it convenient to have both side by side.
* Remove missed line when replacing for loop in optimizers.cpp
* Change how unit test are ran
* Add missing cmake install for bpmpd_caller
* Address remaining warnings
* Update based on Clang-Tidy
* Update based on Clang-Tidy and Clazy
* ScalarOfVector, VectorOfVector, MatrixOfVector function changed to take by reference in std::function
* Add missing implementation of MatrixOfVector::construct
* Disable AVX Instructions to Fix Eigen Alignment Issues
* Add ability to log iteration results to files
* Fix return type in bpmpd_io.hpp
* Make option libraries private when linking
* Add dependencies for tests on package libraries
* Fix clang warnings
* Clange format version 8
* Unify shared pointer definition and switch typedef to using
* Update unit tests
* Namepsace targets and update to use tesseract namespace targets
* Fix kinetic c++11 cmake flag
* Add cmake support for xenial builds
* Add console_bridge and remove rosconsole and fix tests
* Clean up config.cmake and update due to changes in tesseract
* Fixes in gurobi interface
  change string to std::string and some minor clang fixes. There are still more clang warnings that need to be addressed.
* Set OSQP verbosity to false
  This keeps it from spamming the terminal when running TrajOpt many times in a row.
* Fix test warnings
* Add target specific compiler flags
* Fix formatting using clang
* Add additional compiler warning options
* Merge pull request `#40 <https://github.com/tesseract-robotics/trajopt/issues/40>`_ from arocchi/add_free_solvers_upstream
  Adds osqp and qpOASES solver interfaces
* Renamed ConvexSolver into ModelType
* Addressed most comments in first round of review
* Merge remote-tracking branch 'rosind/kinetic-devel' into add_free_solvers_upstream
* Cleanup ConvexSolver to string and back
* Fixes for rebase removing using namespace
* Added AffExprToString
* Add constructors to derived classes and rearranged for readability
* Added missing JSONCPP from trajopt_sco/CMakeLists.txt
* Remove the use of 'using namespace'
* ProblemConstructionInfo now contains info on which convex solver to use
* clang-format
* Refactored qpOASES, osqp, solver_utils
* solver_utils tests are passing
* Added #pragma once for all solvers interfaces
* Added solver_utils
* Removed evil cleanupQuad from trajopt_sco/expr_ops.*
* Using typedefs instead of std::vector for common types: osqp_interface, qpoases_interface
* Merge remote-tracking branch 'levi/kinetic-devel' into add_free_solvers_upstream
* Remove the use of 'using namespace'
* small refactor towards clang / roscpp guidelines
* Fixed memory leaks in osqp solver
* Made qpOASES solver more robust.
  Notice this should be reviewed after bpmpd is removed and all memory
  alignment problems are resolved. In fact, right now the solver is
  occasionally instantiated twice in each solve cycle: this makes
  test pass.
* Fixed availableSolvers()
* Changed order of preference for solvers: Gurobi > bpmpd > osqp > qpOASES
  Notice that while this change seems trivial, it actually causes tests to pass.
  Since bpmpd interface is quite brittle, it was the case that using it as a third option
  in tests caused some of them to fail. This means the order of execution of tests
  has an influence on the solver, which is a bad sign.
* Tests that use optimize() now run for all available solvers
* Tests that use optimize() will now run for all solvers automatically
* Disabled test with negative matrix, tuned osqp to work with nilpotent matrix
* Added qpOASES interface and tests
* Added osqp solver and changed default logging level.
* Add unit test
* Add exprMult(AffExpr, AffExpr)
* Fixes and more changes to increase uniformity in naming
  Renamed ConstraintFromFunc to ConstraintFromErrFunc to match cost version.
  Dropped the "static" from StaticCartPosErrCalculator and added dynamic to the dynamic one.
  Fixed some Doxygen comments
* Make Gurobi not required
* cleanup of GUROBI_LIBRARIES
* Fixed Gurobi
* File Write Calback: Change to const
* Clean up file write callback
  Made proposed changes and fixed one small bug in the plot script
* Change callbacks from taking only the x matrix to the whole results obj
* Add cmake install command
* Specified that the bpmpd caller should be explicitly statically linked (`#19 <https://github.com/tesseract-robotics/trajopt/issues/19>`_)
* Changed scaling from coefficients in CostFromErrFunc to be linear for all penalty types. (`#5 <https://github.com/tesseract-robotics/trajopt/issues/5>`_)
  * Changed scaling from coefficients in CostFromErrFunc to be linear for all penalty types. It was previously quadratic for the SQUARED penalty type.
  * Refactored the scaling fix to use expression operations
* Merge pull request `#3 <https://github.com/tesseract-robotics/trajopt/issues/3>`_ from johnwason/kinetic-devel
  Use CMAKE_CURRENT_SOURCE_DIR instead of CMAKE_SOURCE_DIR for catkin
* Use CMAKE_CURRENT_SOURCE_DIR instead of CMAKE_SOURCE_DIR for catkin compatibility.
* Merge pull request `#1 <https://github.com/tesseract-robotics/trajopt/issues/1>`_ from Levi-Armstrong/fixSubmodule
  Fix submodule and trajopt_sco unit tests
* Fix trajopt_sco unit test
* Merge pull request `#12 <https://github.com/tesseract-robotics/trajopt/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* clang format code
* Merge pull request `#11 <https://github.com/tesseract-robotics/trajopt/issues/11>`_ from larmstrong/unusedParamWarn
  Fix remaining warning
* Uncomment unused names in headers
* Fix remaining warning
* Merge pull request `#10 <https://github.com/tesseract-robotics/trajopt/issues/10>`_ from larmstrong/mergeJMeyer
  Merge jmeyer pull requests
* Merge pull request `#9 <https://github.com/tesseract-robotics/trajopt/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Gobs more small fixups. I don't believe I changed anything that would affect actual logic.
* Switch boost::function to std::function
* Switch boost::shared_ptr to std::shared_ptr
* Add missing license information
* Expose optimization parameters to user via cpp and json
* Divide package into multiple packages
* Contributors: Alessio Rocchi, Armstrong, Levi H, Hervé Audren, Joe Schornak, John Wason, Jonathan Meyer, Joseph Schornak, Levi, Levi Armstrong, Levi-Armstrong, Matthew Powelson, Michael Ripperger, mpowelson, reidchristopher
