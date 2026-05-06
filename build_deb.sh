#!/bin/bash
# Build roboto-motors Debian package
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PACKAGE="roboto-motors"
VERSION="2.1.0"
ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/roboparty"
DEB_DIR="${PACKAGE}_${VERSION}_${ARCH}"
WITH_ROS="${WITH_ROS:-0}"

# Optionally source ROS 2 for ament-based builds.
# Default is deployment mode (WITH_ROS=0), which does not source ROS.
if [ "$WITH_ROS" = "1" ]; then
    ROS_SOURCED=0
    for distro in jazzy iron humble rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            echo ">>> WITH_ROS=1, sourcing ROS 2 ${distro}"
            source "/opt/ros/${distro}/setup.bash"
            ROS_SOURCED=1
            break
        fi
    done

    if [ "$ROS_SOURCED" = "0" ]; then
        echo "Error: WITH_ROS=1 but no ROS 2 setup.bash found under /opt/ros/."
        exit 1
    fi
else
    echo ">>> WITH_ROS=0, skip sourcing ROS environment"
fi

echo ">>> Starting compilation..."
rm -rf build && mkdir -p build
pushd build > /dev/null
cmake .. \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_PREFIX_PATH="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
DESTDIR="${SCRIPT_DIR}/build/destdir" cmake --install .
popd > /dev/null

echo ">>> Preparing Debian package structure..."
rm -rf "${DEB_DIR}" "${DEB_DIR}.deb"
mkdir -p "${DEB_DIR}/DEBIAN"

# Copy cmake installed files
if [ -d "build/destdir${PREFIX}" ]; then
    mkdir -p "${DEB_DIR}${PREFIX}"
    cp -a "build/destdir${PREFIX}/." "${DEB_DIR}${PREFIX}/"
fi

# Copy DEBIAN maintainer scripts
cp debian/postinst "${DEB_DIR}/DEBIAN/"
cp debian/postrm   "${DEB_DIR}/DEBIAN/"
[ -f debian/conffiles ] && cp debian/conffiles "${DEB_DIR}/DEBIAN/"
chmod 755 "${DEB_DIR}/DEBIAN/postinst" "${DEB_DIR}/DEBIAN/postrm"

# Generate Control file (Replace placeholders)
sed -e "s/ARCH_PLACEHOLDER/${ARCH}/g" \
    -e "s/VERSION_PLACEHOLDER/${VERSION}/g" \
    debian/control > "${DEB_DIR}/DEBIAN/control"

echo ">>> Executing dpkg-deb build..."
dpkg-deb --root-owner-group --build "${DEB_DIR}"

echo ">>> Success! Generated ${DEB_DIR}.deb"
