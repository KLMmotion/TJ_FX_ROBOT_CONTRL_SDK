#!/bin/bash
# make_deb.sh — Build libMarvinSDK.so + Python extension, then package as .deb
#
# Usage:
#   cd contrlSDK/
#   bash scripts/make_deb.sh [VERSION]
#
# Output (in contrlSDK/):
#   marvin-sdk_<VERSION>_amd64.deb         libMarvinSDK.so + headers
#   python3-marvin-sdk_<VERSION>_amd64.deb Python extension (standalone)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION="${1:-1.0.0}"
ARCH="$(dpkg --print-architecture)"
MULTIARCH="$(gcc -dumpmachine)"   # e.g. x86_64-linux-gnu

PY_VER="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
PY_EXT="$(python3-config --extension-suffix)"
PY_SITE="/usr/lib/python3/dist-packages"
LIB_DIR="/usr/lib/${MULTIARCH}"
INC_DIR="/usr/include/marvin"

BUILD_DIR="${SDK_DIR}/build"

echo "=== Marvin SDK deb builder v${VERSION} ==="
echo "    arch:    ${ARCH}"
echo "    python:  ${PY_VER}"
echo "    lib dir: ${LIB_DIR}"
echo ""

# ─── Step 1: Build C SDK ──────────────────────────────────────────────────────
echo "[1/4] Building libMarvinSDK.so ..."
cd "${SDK_DIR}"
make -j"$(nproc)"

# ─── Step 2: Build Python extension ──────────────────────────────────────────
echo "[2/4] Building Python extension ..."
cmake -S "${SDK_DIR}" -B "${BUILD_DIR}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DPYTHON_SITE_PACKAGES="${PY_SITE}" \
      -Wno-dev
cmake --build "${BUILD_DIR}" -j"$(nproc)"

PY_SO="$(ls "${BUILD_DIR}"/marvin_sdk*.so)"

# ─── Step 3: Package marvin-sdk (C++ runtime + headers) ──────────────────────
echo "[3/4] Packaging marvin-sdk ..."

C_PKG="${SDK_DIR}/marvin-sdk_${VERSION}_${ARCH}"
rm -rf "${C_PKG}"
install -d "${C_PKG}/DEBIAN"
install -d "${C_PKG}${LIB_DIR}"
install -d "${C_PKG}${INC_DIR}"

# Shared library (with versioned soname)
cp "${SDK_DIR}/libMarvinSDK.so" "${C_PKG}${LIB_DIR}/libMarvinSDK.so.${VERSION}"
ln -sf "libMarvinSDK.so.${VERSION}" "${C_PKG}${LIB_DIR}/libMarvinSDK.so.1"
ln -sf "libMarvinSDK.so.1"          "${C_PKG}${LIB_DIR}/libMarvinSDK.so"

# Public headers
for h in src/MarvinSDK.h src/FxRtCSDef.h src/FxType.h src/PointSet.h src/Robot.h; do
    cp "${SDK_DIR}/${h}" "${C_PKG}${INC_DIR}/"
done

cat > "${C_PKG}/DEBIAN/control" <<EOF
Package: marvin-sdk
Version: ${VERSION}
Architecture: ${ARCH}
Maintainer: TJ FX Robot <support@tjfxrobot.com>
Section: libs
Priority: optional
Depends: libc6, libstdc++6
Description: Marvin Robot Control SDK - C++ runtime
 libMarvinSDK.so for controlling Marvin series robotic arms via UDP at 1 KHz.
 Supports position, PVT, impedance and force-control modes.
 .
 Also includes C++ development headers in /usr/include/marvin/.
EOF

cat > "${C_PKG}/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
ldconfig
EOF

cat > "${C_PKG}/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
ldconfig
EOF

chmod 0755 "${C_PKG}/DEBIAN/postinst" "${C_PKG}/DEBIAN/postrm"

dpkg-deb --build --root-owner-group "${C_PKG}" \
    "${SDK_DIR}/marvin-sdk_${VERSION}_${ARCH}.deb"
rm -rf "${C_PKG}"
echo "    → marvin-sdk_${VERSION}_${ARCH}.deb"

# ─── Step 4: Package python3-marvin-sdk (standalone Python extension) ─────────
echo "[4/4] Packaging python3-marvin-sdk ..."

PY_PKG="${SDK_DIR}/python3-marvin-sdk_${VERSION}_${ARCH}"
rm -rf "${PY_PKG}"
install -d "${PY_PKG}/DEBIAN"
install -d "${PY_PKG}${PY_SITE}"

cp "${PY_SO}" "${PY_PKG}${PY_SITE}/"

cat > "${PY_PKG}/DEBIAN/control" <<EOF
Package: python3-marvin-sdk
Version: ${VERSION}
Architecture: ${ARCH}
Maintainer: TJ FX Robot <support@tjfxrobot.com>
Section: python
Priority: optional
Depends: python3 (>= 3.8)
Description: Marvin Robot Control SDK - Python 3 bindings
 pybind11 extension module for controlling Marvin series robotic arms.
 Self-contained: all SDK code is compiled in, no dependency on marvin-sdk.
 .
 Usage: import marvin_sdk as sdk
EOF

dpkg-deb --build --root-owner-group "${PY_PKG}" \
    "${SDK_DIR}/python3-marvin-sdk_${VERSION}_${ARCH}.deb"
rm -rf "${PY_PKG}"
echo "    → python3-marvin-sdk_${VERSION}_${ARCH}.deb"

echo ""
echo "=== Done ==="
ls -lh "${SDK_DIR}"/*.deb
