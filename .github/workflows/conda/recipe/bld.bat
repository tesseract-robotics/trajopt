set CXXFLAGS=%CXXFLAGS% -DEIGEN_DONT_ALIGN=1 -DEIGEN_DONT_VECTORIZE=1
set CXXFLAGS=%CXXFLAGS% /std:c++17

colcon build --merge-install --install-base="%PREFIX%\opt\tesseract_robotics" ^
   --event-handlers console_cohesion+ desktop_notification- status- terminal_title- ^
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp ^
   --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release ^
   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING="/MD /O2 /Ob0 /Zi /DNDEBUG" ^
   -DCMAKE_RELWITHDEBINFO_POSTFIX="" ^
   -DBUILD_SHARED_LIBS=ON ^
   -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON ^
   -DBUILD_IPOPT=OFF ^
   -DBUILD_SNOPT=OFF ^
   -DCMAKE_PREFIX_PATH:PATH="%LIBRARY_PREFIX%" ^
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF ^
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF ^
   -DPYTHON_EXECUTABLE="%PREFIX%\python.exe" ^
   -DTESSERACT_ENABLE_EXAMPLES=OFF ^
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF ^
   -DTESSERACT_ENABLE_TESTING=OFF ^
   -DTRAJOPT_ENABLE_TESTING=ON ^
   -DTRAJOPT_ENABLE_BENCHMARKING=ON ^
   -DTRAJOPT_ENABLE_RUN_BENCHMARKING=OFF

if %errorlevel% neq 0 exit /b %errorlevel%

set TESSERACT_RESOURCE_PATH=%PREFIX%\opt\tesseract_robotics\share

colcon test --event-handlers console_direct+ desktop_notification- status- terminal_title- ^
   --return-code-on-test-failure ^
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp tesseract_common ^
   tesseract_collision tesseract_environment tesseract_geometry tesseract_kinematics tesseract_scene_graph ^
   tesseract_srdf tesseract_state_solver tesseract_support tesseract_urdf tesseract_visualization ^
   --merge-install --install-base="%PREFIX%\opt\tesseract_robotics"

if %errorlevel% neq 0 exit /b %errorlevel%

setlocal EnableDelayedExpansion

:: Copy the [de]activate scripts to %PREFIX%\etc\conda\[de]activate.d.
:: This will allow them to be run on environment activation.
for %%F in (activate deactivate) DO (
    if not exist %PREFIX%\etc\conda\%%F.d mkdir %PREFIX%\etc\conda\%%F.d
    copy %RECIPE_DIR%\%%F.bat %PREFIX%\etc\conda\%%F.d\%PKG_NAME%_%%F.bat
)

if %errorlevel% neq 0 exit /b %errorlevel%