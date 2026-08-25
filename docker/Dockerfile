ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Use a tmpfs mount for the workspace so as not to unnecessarily copy files into the final image
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/trajopt
ARG INSTALL_DIR=/opt/trajopt
RUN mkdir -p ${INSTALL_DIR}

RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/trajopt \
  cd ${WORKSPACE_DIR} \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
  && source /opt/tesseract/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}
