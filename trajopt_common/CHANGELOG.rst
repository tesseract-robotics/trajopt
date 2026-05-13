^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.34.4 (2026-02-18)
-------------------
* Do not add compiler option -mno-avx for arm or if processor is unknown
* Update cache to also be a pool of object to retrieve unused objects (`#530 <https://github.com/tesseract-robotics/trajopt/issues/530>`_)
* Reduce memory allocation in trajopt_ifopt and trajopt_sqp
* Reduce allocations
* Minor optimizations in the weighted average methods
* Cleanup doxygen file headers
* Remove old console bridge cmake target logic
* Switch to using Cereal for serialization
* Contributors: Levi Armstrong, Roelof Oomen

0.33.0 (2025-10-28)
-------------------

0.32.0 (2025-09-10)
-------------------
* Fix benchmarks (`#471 <https://github.com/tesseract-robotics/trajopt/issues/471>`_)
* Add yaml conversions and related tests (`#469 <https://github.com/tesseract-robotics/trajopt/issues/469>`_)
* Remove PairHash (replaced by implicit std::hash in tesseract_common)
* Contributors: Roelof Oomen, Tyler Marr

0.31.0 (2025-07-06)
-------------------
* Make sure serialized objects have friend struct tesseract_common::Serialization
* Remove SafetyMarginData from fwd.h
  This struct was removed in a previous commit, but accidentally left in the forward declarations.
* Contributors: Levi Armstrong, Roelof Oomen

0.30.0 (2025-04-23)
-------------------
* Removed weighted sum methods from legacy Trajopt (`#457 <https://github.com/tesseract-robotics/trajopt/issues/457>`_)
* Minor updates (`#456 <https://github.com/tesseract-robotics/trajopt/issues/456>`_)
  * Simplify logic for filtering collisions
  * Removed pairing of margin and coefficient into vector
* Fix issues with how Collision Check Config was being used
* Requested changes and additional bug fixes
* Fix bug in trajopt and trajopt_ifopt collision evaluators
* Store single TrajOptCollisionConfig instead of vector
* Update default collision margin buffer to 0.01
* Update legacy trajopt to leverage trajopt_common::TrajOptCollisionConfig
* Contributors: Levi Armstrong, Michael Ripperger

0.29.1 (2025-03-20)
-------------------
* Add tesseract:make_convex attribute to urdf files
* Contributors: Levi Armstrong

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
* Add boost serialization support to config objects
* Contributors: Levi Armstrong

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------

0.25.1 (2024-09-29)
-------------------

0.25.0 (2024-09-28)
-------------------
* Add missing package libraries cmake variable
* Contributors: Levi Armstrong

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
* Trajopt clang-tidy clean-up (`#411 <https://github.com/tesseract-robotics/trajopt/issues/411>`_)
* Various include cleanups and clang-tidy fixes, and an error message fix.
* Contributors: Roelof Oomen

0.22.0 (2024-06-02)
-------------------
* Update to leverage forward declarations
* Fix trajopt ifopt collision with fixed states (`#372 <https://github.com/tesseract-robotics/trajopt/issues/372>`_)
* Contributors: Levi Armstrong

0.7.2 (2023-11-27)
------------------
* Fix TrajOpt Ifopt handling of constraint merit coefficient (`#366 <https://github.com/tesseract-robotics/trajopt/issues/366>`_)
* Removed gcc-specific options from clang config (see https://github.com/tesseract-robotics/tesseract/commit/43d08870034e85a3f335d37ded00df282e5ec46e)
* Contributors: Levi Armstrong, Roelof Oomen

0.7.1 (2023-11-22)
------------------
* Remove use of Industrial CI (`#359 <https://github.com/tesseract-robotics/trajopt/issues/359>`_)
* Contributors: Levi Armstrong

0.7.0 (2023-11-07)
------------------
* Move TrajOptIfopt collision gradient types and utils to trajopt_common package
* Contributors: Levi Armstrong

0.6.1 (2023-07-10)
------------------

0.6.0 (2023-06-30)
------------------
* Move shared data to trajopt_common
* Rename trajopt_utils to trajopt_common
* Contributors: Levi Armstrong

0.5.2 (2023-06-06)
------------------

0.5.1 (2023-04-11)
------------------

0.5.0 (2023-04-09)
------------------
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
* catkin dependency for ROS1 only
* Contributors: Roelof Oomen

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

0.2.3 (2022-03-24)
------------------

0.2.2 (2022-01-19)
------------------

0.2.1 (2021-12-16)
------------------

0.2.0 (2021-12-04)
------------------
* Add ContactManagerConfig inside CollisionCheckConfig (`#280 <https://github.com/tesseract-robotics/trajopt/issues/280>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: Matthew Powelson

0.1.1 (2021-11-29)
------------------

0.1.0 (2021-11-02)
------------------
* Add CMake Format Support
* Update to leverage Tesseract JointGroup and KinematicGroup
* Remove trajopt_ifopt dependency on trajopt
* Fix clang tidy errors and update to leverage .clang-tidy file
* Add absolute cost along with unit tests for squared and absolute costs
* Use Boost and Eigen targets
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add exec depend on catkin and buildtool depend on cmake per REP 136
* Add missing include gaurds in trajopt_utils
* Add public compiler option -mno-avx
* Add windows support stage 1
* Manually enable clang-tidy build
* Fix clang-tidy errors in Focal build
* Disable clang tidy check misc-non-private-member-variables-in-classes
* Disable clang tidy check modernize-use-trailing-return-type
* Add Colcon environment hooks
  Fixes rosdep issues when building trajopt in an extended workspace.
* Update CMake to work better with clang
* Remove Boost Python dependency
* Add macro to run benchmarks if -DTRAJOPT_ENABLE_RUN_BENCHMARKING=ON
* Add Clang-tidy flags to the readme
* Add eigen to package.xml
  and alphabetize the entries.
* Add processing of header files to clang-tidy
* Change how unit test are ran
* Make warnings errors when ENABLE_TESTS is enabled
* Address remaining warnings
* Make clang-tidy only run if ENABLE_CLANG_TIDY or ENABLE_TESTS is enabled
* Update based on Clang-Tidy
* Update based on Clang-Tidy and Clazy
* Disable AVX Instructions to Fix Eigen Alignment Issues
* Fix clang warnings
* Add flags to ignore formating the macros.h file
* Fix macro in trajop_utils macros.h
* Clange format version 8
* Unify shared pointer definition and switch typedef to using
* Namepsace targets and update to use tesseract namespace targets
* Fix find_dependency for components in kinetic
* Fix kinetic c++11 cmake flag
* Add cmake support for xenial builds
* Clean up config.cmake and update due to changes in tesseract
* Clang Format
  Hopefully will pass Travis now.
* Add cblock to BasicArray
  Used to clean up some dirty code in problem_description. This commit also includes some minor changes that got lost in the rebase somehow.
* Fix test warnings
* Add target specific compiler flags
* Fix formatting using clang
* Add additional compiler warning options
* Merge pull request `#40 <https://github.com/tesseract-robotics/trajopt/issues/40>`_ from arocchi/add_free_solvers_upstream
  Adds osqp and qpOASES solver interfaces
* Addressed most comments in first round of review
* Merge remote-tracking branch 'rosind/kinetic-devel' into add_free_solvers_upstream
* Inequality Terms fixed
* Bug Fixes
* Remove the use of 'using namespace'
* Merge remote-tracking branch 'levi/kinetic-devel' into add_free_solvers_upstream
* Remove the use of 'using namespace'
* Added osqp solver and changed default logging level.
* Fix trajopt_utils install pattern
* Add cmake install command
* Merge pull request `#12 <https://github.com/tesseract-robotics/trajopt/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* clang format code
* Merge pull request `#11 <https://github.com/tesseract-robotics/trajopt/issues/11>`_ from larmstrong/unusedParamWarn
  Fix remaining warning
* Fix remaining warning
* Merge pull request `#10 <https://github.com/tesseract-robotics/trajopt/issues/10>`_ from larmstrong/mergeJMeyer
  Merge jmeyer pull requests
* Merge pull request `#9 <https://github.com/tesseract-robotics/trajopt/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Removed warnings again. Just too many in included libraries to deal with.
* Gobs more small fixups. I don't believe I changed anything that would affect actual logic.
* Switch boost::shared_ptr to std::shared_ptr
* Add missing license information
* Remove openrave utils
* Divide package into multiple packages
* Contributors: Alessio Rocchi, Armstrong, Levi H, Jonathan Meyer, Levi, Levi Armstrong, Levi-Armstrong, Matthew Powelson, Patrick Beeson, mpowelson
