# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/viktor/esp/esp-idf/components/bootloader/subproject"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix/tmp"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix/src"
  "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/viktor/esp-homekit-sdk/examples/garagedoor/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
