^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajopt_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
