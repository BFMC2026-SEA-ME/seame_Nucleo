# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-src")
  file(MAKE_DIRECTORY "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-src")
endif()
file(MAKE_DIRECTORY
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-build"
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix"
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp"
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src"
  "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/choeminhyeog/Desktop/seame/bfmc2026/workspace/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
