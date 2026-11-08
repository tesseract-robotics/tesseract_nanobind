# Overlay of vcpkg's octomap port (2026.10.16), shared on Linux and macOS. Upstream forces a
# static library everywhere, so every tesseract library that links octomap carries its own
# copy; on Linux ELF interposition binds all copies' globals to one object and each copy's
# destructor frees it, a double free at exit. Windows keeps the static build (no DLL exports).
if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO OctoMap/octomap
    REF "v${VERSION}"
    SHA512 9d204ae1a8f7e01176677503c2b7996e862b993f622a50fcefb99a24dd322027b506490061110198f06fb48337e39056b9eb92efcdc8da8ecef7544842f016ae
    HEAD_REF devel
    PATCHES
      001-fix-exported-targets.patch
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    DISABLE_PARALLEL_CONFIGURE
    OPTIONS
        -DBUILD_TESTING=OFF
        -DBUILD_OCTOVIS_SUBPROJECT=OFF
        -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF
)

vcpkg_cmake_install()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_copy_tools(
    TOOL_NAMES binvox2bt bt2vrml compare_octrees convert_octree edit_octree eval_octree_accuracy graph2tree log2graph
    AUTO_CLEAN)

vcpkg_cmake_config_fixup()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")

if(VCPKG_LIBRARY_LINKAGE STREQUAL static)
    file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/bin" "${CURRENT_PACKAGES_DIR}/debug/bin")
endif()

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/vcpkg-cmake-wrapper.cmake" DESTINATION "${CURRENT_PACKAGES_DIR}/share/octomap")

# Handle copyright
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/octomap/LICENSE.txt")

vcpkg_copy_pdbs()

vcpkg_fixup_pkgconfig()
