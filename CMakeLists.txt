cmake_minimum_required(VERSION 3.16)

# Explicit version, not git describe.
#
# Left to itself IDF stamps the build with something like
# "v0.2.0-2-g3eeaf4a-dirty". That is fine in a boot log and wrong in a settings
# page: the version is shown to a user deciding whether to install an update,
# and it has to read like a version rather than like a fault.
file(STRINGS "VERSION" PROJECT_VER)

# ...and re-run CMake when that file changes.
#
# Without this the version is read once at configure time and then reused: an
# edited VERSION file produces a build stamped with the OLD version, so a
# release is named one thing and reports another. That is not a cosmetic bug --
# it made a working firmware update look like a failure, because the only
# evidence a user has that an update landed is the version changing.
set_property(DIRECTORY APPEND PROPERTY
             CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/VERSION")

# The development tree, when it is present.
#
# dev/ holds the bench console and the desk tooling. It is gitignored, so a
# public checkout simply does not have it -- and this stays quiet rather than
# failing, which is what lets the same CMakeLists serve both. The console is
# additionally behind CONFIG_CAMLINK_BENCH, off by default, so having the
# directory is not enough to build it in by accident.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/dev/components")
    set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/dev/components")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Trim the build to main + its dependencies, as the upstream DJI demo does.
idf_build_set_property(MINIMAL_BUILD ON)
project(slate)
