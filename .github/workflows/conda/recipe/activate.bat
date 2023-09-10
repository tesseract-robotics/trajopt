@if not defined CONDA_PREFIX goto:eof

@call "%CONDA_PREFIX%\opt\tesseract_robotics\setup.bat"

@set TESSERACT_PYTHON_DLL_PATH=%CONDA_PREFIX%\opt\tesseract_robotics\bin