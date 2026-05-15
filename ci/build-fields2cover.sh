#!/bin/bash

# Fields2Cover v2 doesn't have a prebuilt binary, so we'll build our own.

set -euo pipefail

F2C_REF=v2.0.0
CLONE_DIR=/tmp/Fields2Cover

sudo apt-get install -y \
  libgdal-dev \
  libtbb-dev \
  swig

git clone --depth 1 --branch ${F2C_REF} \
  https://github.com/Fields2Cover/Fields2Cover.git \
  ${CLONE_DIR}

cmake -S ${CLONE_DIR} \
  -B ${CLONE_DIR}/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON=ON \
  -DBUILD_TUTORIALS=OFF \
  -DBUILD_TESTS=OFF
cmake --build ${CLONE_DIR}/build --parallel "$(nproc)"
cmake --install ${CLONE_DIR}/build
ldconfig

mkdir -p ${CLONE_DIR}/dist
find ${CLONE_DIR}/build -name "_fields2cover*.so" -exec cp {} ${CLONE_DIR}/dist/ \;
find ${CLONE_DIR}/build -name "fields2cover.py"  -exec cp {} ${CLONE_DIR}/dist/ \;

cd ${CLONE_DIR}/dist
tar -czvf fields2cover.tar.gz *.{so,py}