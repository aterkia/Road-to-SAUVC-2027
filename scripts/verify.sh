#!/usr/bin/env bash
set -euo pipefail
# Source the chosen ROS underlay before invoking this script.
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$workspace"
source scripts/ros_env.sh

# Container validation can keep its path-dependent outputs outside the shared checkout.
output_root="${ATERKIA_BUILD_ROOT:-$workspace}"
mkdir -p -- "$output_root"
output_root="$(cd -- "$output_root" && pwd -P)"
build_base="$output_root/build"
install_base="$output_root/install"

for cache in "$build_base"/*/CMakeCache.txt; do
  [[ -f "$cache" ]] || continue
  cached_build="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "$cache")"
  cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")"
  if [[ -n "$cached_build" && "$cached_build" != "$(dirname -- "$cache")" ]] ||
     [[ -n "$cached_source" && "$cached_source" != "$workspace"/* ]]; then
    printf 'Build cache belongs to another workspace path: %s\n' "$cache" >&2
    printf 'Cached build: %s\nCached source: %s\n' "$cached_build" "$cached_source" >&2
    printf 'Move build/, install/, and log/ out of %s, or set ATERKIA_BUILD_ROOT to a new directory.\n' "$output_root" >&2
    printf 'Retry from a clean terminal with only the ROS underlay sourced.\n' >&2
    exit 2
  fi
done

# Keep the test stack separate from the operator's ordinary ROS domain.
export ROS_DOMAIN_ID="${ATERKIA_TEST_DOMAIN_ID:-77}"
colcon --log-base "$output_root/log" build --build-base "$build_base" \
  --install-base "$install_base" --symlink-install --parallel-workers 2
set +u
source "$install_base/setup.bash"
set -u
colcon --log-base "$output_root/log" test --build-base "$build_base" \
  --install-base "$install_base" --event-handlers console_direct+
colcon test-result --test-result-base "$build_base" --verbose
