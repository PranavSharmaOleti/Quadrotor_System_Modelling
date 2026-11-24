#!/usr/bin/env bash
# PX4 SITL + Gazebo in Docker, headless, software rendering.
# Run from ~/Projects/Quadrotor-CrossSim
set -euo pipefail

cd "$(dirname "$0")"

IMAGE=px4io/px4-sitl-gazebo:v1.18.0-beta1-660-g61907962c9
mkdir -p px4-data

# Try ogre (1.x) render engine + software GL to avoid GPU/driver stalls in the container.
sudo docker run --rm -it \
  --privileged \
  --network host \
  -e HEADLESS=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  -e GZ_VERBOSE=4 \
  -e PX4_SIM_MODEL=gz_x500 \
  -e XDG_DATA_HOME=/px4-data \
  -v "$PWD/px4-data:/px4-data" \
  "$IMAGE"
