set -e

ln -s $BUILD_PREFIX/bin/x86_64-conda-linux-gnu-gcc $BUILD_PREFIX/bin/gcc

colcon build --merge-install --install-base="$PREFIX/opt/tesseract_robotics" \
   --event-handlers console_cohesion+ \
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp \
   --cmake-args -DCMAKE_BUILD_TYPE=Release \
   -DBUILD_SHARED_LIBS=ON \
   -DBUILD_IPOPT=OFF \
   -DBUILD_SNOPT=OFF \
   -DCMAKE_PREFIX_PATH:PATH="$PREFIX" \
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF \
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF \
   -DTESSERACT_ENABLE_EXAMPLES=OFF \
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF \
   -DSETUPTOOLS_DEB_LAYOUT=OFF \
   -DTESSERACT_ENABLE_TESTING=ON

colcon test --event-handlers console_direct+ --return-code-on-test-failure \
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp tesseract_common \
   --merge-install --install-base="$PREFIX/opt/tesseract_robotics" 


for CHANGE in "activate" "deactivate"
do
    mkdir -p "${PREFIX}/etc/conda/${CHANGE}.d"
    cp "${RECIPE_DIR}/${CHANGE}.sh" "${PREFIX}/etc/conda/${CHANGE}.d/${PKG_NAME}_${CHANGE}.sh"
done