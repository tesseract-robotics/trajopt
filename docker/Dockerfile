ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/tesseract_planning
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/trajopt \
  cd ${WORKSPACE_DIR} \
  && vcs import src < src/trajopt/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/trajopt \
  source /opt/tesseract/install/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log
