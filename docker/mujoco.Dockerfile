# viser web client builder: the vendored source under mujoco/viser_client
# (upstream v1.1.1 with the keyboard camera bindings removed at source
# level -- see that README) compiled by viser's own vite pipeline. Kept as
# a separate stage so the runtime image needs no node; the npmmirror
# registry default matches the apt/pypi mirror choices below.
FROM node:22-slim AS viser-client
ARG NPM_REGISTRY=https://registry.npmmirror.com
COPY mujoco/viser_client /viser_client
WORKDIR /viser_client
RUN npm ci --registry="$NPM_REGISTRY" --no-audit --no-fund && npm run build

# Overall-sim container: MuJoCo dog sim + wgpu splat + rclpy contract face.
#
# One image for the whole tinynav simulation side (hil.py publishes the
# looper contract straight into the shared host-network DDS domain via
# rclpy). Deliberately NOT from
# the uniflexai/tinynav base: this stack needs python>=3.11 for wgpu 0.32
# (humble = py3.10), and it needs none of torch/TensorRT/gz.
#
# Build (context = repo root; BuildKit required; the model/ asset bundle
# must be present — unzip model.zip at the repo root first: it carries the
# splat scene + go2 meshes, none of which are in git):
#   docker build -f docker/mujoco.Dockerfile -t mjsim:jazzy .
#
# Run: bash docker/run-mujoco-sim.sh [hil|sim|mj]   (wraps docker run with
# GPU / X11 / /dev/input / host-net flags)
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      libegl1 libgles2 libvulkan1 \
      libx11-6 libxext6 libxcursor1 libxrandr2 libxinerama1 libxi6 libgl1 \
      ros-jazzy-rmw-cyclonedds-cpp \
      python3-pip python3-venv tini \
    && rm -rf /var/lib/apt/lists/*
# ros images ship no pip and ubuntu 24.04 marks the interpreter
# externally-managed (PEP 668) -- a container image is exactly the place
# where overriding that is the right call
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# python stack in a dedicated venv: the debian python carries rclpy (made
# visible via --system-site-packages) but its apt numpy blocks pip upgrades;
# the venv shadows it cleanly with the same numpy version pinned.
# uv for the heavy install: pip serially backtracks through large opencv
# wheels at mirror speed (~15 min here); uv resolves once and downloads in
# parallel. uv and the deps both come from the same aliyun index (PyPI
# mirror; ghcr.io is flaky on this network). gausscam is fetched from its
# exact official wheel URL instead of the index: aliyun lags on fresh
# releases and 0.1.5 is not synced yet. Swap this back to `gausscam==0.1.5`
# once the mirror carries it (the installed artifact is 0.1.5,
# sha256 240f88a20cac49e0a406f482f0aaf3ef39acce07881e0e48d7b6e36cfdb09efd).
# The pin matters: hil exercises the whole gausscam API at boot, so a bad
# version fails the image loudly rather than at run time.
RUN python3 -m pip install --no-cache-dir \
      -i https://mirrors.aliyun.com/pypi/simple/ uv \
    && uv venv --system-site-packages -p /usr/bin/python3 /opt/mjsim \
    && uv pip install --python /opt/mjsim/bin/python \
      --index-url https://mirrors.aliyun.com/pypi/simple/ \
      numpy==1.26.4 \
      mujoco==3.14.0 opencv-python-headless wgpu==0.32.0 \
      viser==1.1.1 trimesh plyfile \
      onnxruntime-gpu \
      nvidia-cublas nvidia-cudnn-cu13 nvidia-cuda-runtime nvidia-cuda-nvrtc \
      https://files.pythonhosted.org/packages/76/95/b87b576f638b58d1a0104c27d7526cddf4f5fb0ec4a18743ea6f2f16c87c/gausscam-0.1.5-py3-none-any.whl \
    && uv pip install --python /opt/mjsim/bin/python --no-deps \
      --index-url https://mirrors.aliyun.com/pypi/simple/ \
      nvidia-curand-cu12 \
    && rm -rf /root/.cache/uv

# onnxruntime-gpu's CUDA EP (retinify stereo depth, sim/stereo.py) dlopens its
# CUDA libs from these pip wheel dirs; ORT 1.31 is a CUDA 13 build but links
# curand.so.10, hence the lone cu12 curand above. Registered with ldconfig so
# no process needs an LD_LIBRARY_PATH export.
RUN for d in /opt/mjsim/lib/python3.12/site-packages/nvidia/*/lib; do \
      echo "$d" >> /etc/ld.so.conf.d/zz-ort-nvidia.conf; done \
    && ldconfig

# robot_emu DDS deps (docs/multi-robot-sim-testbench.md §6, mujoco/README
# "dog" row): the emu wears the real robot's unitree-sdk2py face. TWO PINS
# ARE LOAD-BEARING:
# - the cyclonedds python binding must stay in the 0.10.x family. An 11.x
#   wheel publishes type objects that the PILOT side's 0.10.2 binding
#   segfaults on during discovery (same-
#   container 11<->11 is fine, so it surfaces only at the pilot boundary).
#   py3.12 has no prebuilt 0.10.x wheel, so build the binding from sdist
#   against the jazzy libddsc C core (0.10.5, same family as the pilot's).
# - unitree_sdk2py is NOT on PyPI: the exact pinned source (upstream
#   unitree_sdk2_python @814556d, the same commit tinynav/pyproject.toml
#   pins) is checked out at mujoco/robot_emu/vendor/unitree_sdk2py and
#   pip-installed below. It stands in for a submodule (no-commit
#   discipline keeps it out of .gitmodules; convert with
#   `git submodule add` at commit time). Vendoring is for the exact-pin
#   + pilot-parity guarantee, NOT the build network: github.com was
#   verified reachable from this host (the earlier "build network may
#   not reach GitHub" note is retired). Refresh: git -C that dir fetch origin && git checkout <sha>.
#   NOTE: keep the empty unitree_sdk2py/as2/sport/__init__.py added in
#   the vendored tree -- upstream 814556d ships as2/sport WITHOUT one,
#   and setup.py find_packages() then silently omits the subpackage
#   (import unitree_sdk2py.as2.sport.* fails in the image).
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake g++ python3-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /opt/cyclone-home/lib /opt/cyclone-home/bin \
    && ln -s /opt/ros/jazzy/include/CycloneDDS /opt/cyclone-home/include \
    && ln -sf /opt/ros/jazzy/lib/x86_64-linux-gnu/libddsc.so* \
         /opt/cyclone-home/lib/ \
    && for f in /opt/ros/jazzy/lib/x86_64-linux-gnu/libiceoryx*.so*; do \
         ln -sf "$f" /opt/cyclone-home/lib/; done \
    && test -e /opt/cyclone-home/lib/libddsc.so \
    && test -e /opt/cyclone-home/include \
    && CYCLONEDDS_HOME=/opt/cyclone-home \
       python3 -m pip install --no-cache-dir --no-build-isolation \
         -i https://mirrors.aliyun.com/pypi/simple/ \
         --no-binary cyclonedds "cyclonedds==0.10.2" \
    && echo /opt/cyclone-home/lib > /etc/ld.so.conf.d/cyclone-home.conf \
    && ldconfig \
    && rm -rf /root/.cache/pip
# cmake/g++/python3-dev are deliberately NOT purged after the sdist
# build. An earlier revision of this layer ended with
# `apt-get purge -y cmake g++ python3-dev && apt-get autoremove -y`,
# which cascade-removed the entire ros-jazzy tree (purge also removes
# installed packages that depend on the purged ones -- build-essential,
# python3-colcon-* -- and autoremove then swept the orphaned ros
# packages): /opt/ros vanished, the farm symlinks above dangled, and
# every unitree_sdk2py import failed on the missing libddsc
# incident). The two test -e guards above turn a repeat
# into a loud build failure instead of a silently dangling farm.
ENV CYCLONEDDS_HOME=/opt/cyclone-home

# viser's web client hardcodes keyboard camera controls (arrows rotate,
# KeyE elevates) with no upstream toggle (viser-project/viser#259 open) --
# arrows collide with the global /dev/input teleop, which fires regardless
# of browser focus. We ship the vendored mujoco/viser_client (upstream
# v1.1.1, bindings removed at source level in CameraControls.tsx) built by
# the node stage above. The sanity layer decodes the packed payload and
# refuses to build an image whose client still binds camera keys.
# WASD/KeyQ stay for camera work.
COPY --from=viser-client /viser_client/build/index.html \
     /opt/mjsim/lib/python3.12/site-packages/viser/client/build/index.html
RUN apt-get update -qq \
    && apt-get install -y --no-install-recommends zstd \
    && rm -rf /var/lib/apt/lists/* \
    && python3 - /opt/mjsim/lib/python3.12/site-packages/viser/client/build/index.html <<'PYEOF'
import re, subprocess, sys
raw = open(sys.argv[1], encoding="utf-8").read()
m = re.search(r'data-p="([^"]*)"', raw, re.S)
assert m, "viser client is not in the expected packed layout"
ALPHA = "!#$%()*+,-./0123456789:;=?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_abcdefghijklmnopqrstuvwxyz{|}~"
L = [0]*127
for i in range(88):
    L[ord(ALPHA[i])] = i
def de(s):
    n = len(s)//5; r = len(s) % 5
    out = bytearray(n*4 + (r-1 if r else 0)); o = 0
    for i in range(n):
        v = 0
        for c in s[i*5:(i+1)*5]:
            v = v*88 + L[ord(c)]
        out[o:o+4] = bytes((v >> sh & 255 for sh in (24, 16, 8, 0))); o += 4
    if r:
        v = 0
        for c in s[n*5:]:
            v = v*88 + L[ord(c)]
        out[o:] = bytes((v >> 8*j) & 255 for j in range(r-2, -1, -1))
    return bytes(out)
blob = subprocess.run(["zstd", "-d", "-c", "-"], input=de(m.group(1)),
                      capture_output=True, check=True).stdout
bound = (b"held.has(`Arrow" in blob or b"held.has(`KeyE`)" in blob
         or b"elevate(i,!1),r.has(`KeyE`)" in blob)
assert not bound, "viser client still binds camera keys -- fix mujoco/viser_client/src/CameraControls.tsx and rebuild the image"
print("[viser] client payload verified: camera keybindings absent")
PYEOF
ENV PATH=/opt/mjsim/bin:$PATH

COPY mujoco /workspace/tinynav-sim/mujoco
# unitree_sdk2py: pip-install the pinned source checkout (deps already
# baked: cyclonedds 0.10.2 above; --no-deps/--no-build-isolation keep pip
# off the network entirely).
RUN python3 -m pip install --no-cache-dir --no-deps --no-build-isolation \
      /workspace/tinynav-sim/mujoco/robot_emu/vendor/unitree_sdk2py
# DDS: bake the single-host loopback-unicast Cyclone config (same file as
# sim.Dockerfile; proven against VPN-TUN route hijacking -- without it
# large image samples degrade to ~4 Hz for peers on the host network).
COPY tools/probes/cyclone_localhost_unicast.xml /opt/dds/cyclone_localhost_unicast.xml
# model/ asset bundle (model.zip, not in git): splat scene + go2 meshes,
# baked AT THE REPO ROOT -- the same layout a host checkout has after
# unzipping model.zip, so the repo-root model/ lookups in plant/hil/view
# work unchanged in-container.
COPY model /workspace/tinynav-sim/model
COPY docker/mjsim-entrypoint.sh /usr/local/bin/mjsim
RUN chmod +x /usr/local/bin/mjsim

WORKDIR /workspace/tinynav-sim
# vsync kill: with __GL_SYNC_TO_VBLANK=0 the viewer's swap never blocks on
# an XWayland frame callback. NO GLVND/GLX vendor pins here: the viewer
# window renders via Mesa/iris on the Intel GPU (the X screen's GPU) and
# needs /dev/dri passed through at `docker run` (run-mujoco-sim.sh does);
# the GPU-heavy paths (wgpu splat, EGL depth) pick NVIDIA on their own.
# (Historical note: the "container viewer freezes after one frame" case was
# root-caused to hil.py never calling viewer.sync() -- NOT to
# GLX/NVIDIA. The no-pins guidance above still holds.)
ENV MUJOCO_GL=egl \
      __GL_SYNC_TO_VBLANK=0 \
      RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      CYCLONEDDS_URI=file:///opt/dds/cyclone_localhost_unicast.xml \
      OPENBLAS_NUM_THREADS=1 \
      OMP_NUM_THREADS=1 \
      PYTHONPATH=/workspace/tinynav-sim/mujoco
# RMW pinned: jazzy defaults to Fast DDS whose discovery/logging threads
# burn extra cores beside the tinynav stack's CycloneDDS.
# CYCLONEDDS_URI pins DDS to loopback unicast (same file the sim.Dockerfile
# bakes): with defaults, large best-effort samples (the ~348 KB stereo
# frames) fragment across every interface including any VPN TUN and die
# mid-path -- subscribers saw 3.9 Hz of a clean 10 Hz while small topics
# stayed perfect. See tools/probes/cyclone_localhost_unicast.xml and
# docs/plan-sim-image-split.md.
# OPENBLAS/OMP pinned: numpy's bundled OpenBLAS spins its whole pool on
# every per-tick BLAS call (the PIE tick does tiny ones at 200 Hz) —
# unpinned the idle-ish sim burns 5+ cores on pool spin-wait
# (10.5ms-cpu/tick -> 0.9ms pinned).
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/mjsim"]
CMD ["hil"]
