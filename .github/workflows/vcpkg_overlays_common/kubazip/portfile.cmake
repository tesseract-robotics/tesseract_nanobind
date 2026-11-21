# vcpkg 2026.10.16's kubazip port, fetched by commit instead of by tag. kuba--/zip rewrote its
# history on 2026-11-20 and re-created v0.3.15 on the rewritten commit: the tree is unchanged
# (17d3c64a), but GitHub's v0.3.15 archive changed bytes and no longer matches the port's SHA512
# (microsoft/vcpkg#55200). 5f680055 is v0.3.15 after the rewrite. Drop this overlay once the
# declared vcpkg release carries a fixed kubazip port.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO kuba--/zip
    REF 5f68005531b1505129baa27de2dfbf26c0d4172a
    SHA512 290418f0b1d77144b6d6ac96d66181a114975933df4514cf8ad4f7de99a91c2b273f0761b97dc047f4ce8116f15842ac9e3606b804fd46adf70b4cbad626ebb0
    HEAD_REF master
    PATCHES
        fix-name-conflict.diff
        disable-werror.patch
        fix-project-version.patch # https://github.com/kuba--/zip/commit/dd80aeab4293d1e11e31af20fabfd538d88625ef
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCMAKE_DISABLE_TESTING=ON
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/zip" PACKAGE_NAME "zip-kuba--")

if(VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/kubazip/zip/zip.h" "#ifndef ZIP_SHARED" "#if 0")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

# legacy polyfill
file(INSTALL "${CURRENT_PORT_DIR}/kubazipConfig.cmake" "${CURRENT_PORT_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")

vcpkg_install_copyright(
    FILE_LIST
        "${SOURCE_PATH}/LICENSE.txt"
        "${SOURCE_PATH}/src/miniz.h"
)
