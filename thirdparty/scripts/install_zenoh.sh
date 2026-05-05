#!/usr/bin/env bash
set -e

# Version for both zenoh-c and zenoh-cpp
ver_tag="1.9.0"

# Root directory of the script
root_dir="$(pwd)"

# Install rust toolchain if cargo is missing.
ensure_cargo() {
  if command -v cargo >/dev/null 2>&1; then
    echo "Found cargo: $(command -v cargo)"
    return
  fi

  echo "cargo is not found, installing rustup and cargo"

  if ! command -v curl >/dev/null 2>&1; then
    echo "Error: curl is required to install cargo via rustup"
    exit 1
  fi

  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

  if [ -f "$HOME/.cargo/env" ]; then
    source "$HOME/.cargo/env"
  fi

  if ! command -v cargo >/dev/null 2>&1; then
    echo "Error: cargo installation failed"
    exit 1
  fi

  echo "Installed cargo: $(command -v cargo)"
}

# Install zenoh-c backend
install_zenoh_c() {
  local repo="zenoh-c"
  echo "Installing $repo ($ver_tag)"
  cd "$root_dir"

  if [ ! -d "$repo" ]; then
    git clone https://github.com/eclipse-zenoh/$repo.git --branch $ver_tag
    cd "$repo"
  else
    cd "$repo"
    git fetch --all
    git checkout $ver_tag
  fi

  mkdir -p build && cd build
  cmake .. $SHARED_CMAKE_ARGS \
           -DBUILD_SHARED_LIBS=ON \
           -DZENOHC_BUILD_WITH_UNSTABLE_API=ON

  cmake --build . --target install --config Release -j $NUM_JOBS

  echo "Installed $repo"
}

# Install zenoh-cpp aligned to the same version as zenoh-c
install_zenoh_cpp() {
  local repo="zenoh-cpp"
  echo "Installing $repo ($ver_tag)"
  cd "$root_dir"

  if [ ! -d "$repo" ]; then
    git clone https://github.com/eclipse-zenoh/$repo.git --branch $ver_tag
    cd "$repo"
  else
    cd "$repo"
    git fetch --all
    git checkout $ver_tag
  fi

  mkdir -p build && cd build
  cmake .. $SHARED_CMAKE_ARGS \
           -DBUILD_SHARED_LIBS=ON \
           -DZENOHCXX_ENABLE_TESTS=OFF \
           -DZENOHCXX_ENABLE_EXAMPLES=OFF \
           -DZENOHCXX_EXAMPLES_PROTOBUF=OFF

  cmake --build . --target install --config Release -j $NUM_JOBS

  echo "Installed $repo"
}

ensure_cargo
install_zenoh_c
install_zenoh_cpp
