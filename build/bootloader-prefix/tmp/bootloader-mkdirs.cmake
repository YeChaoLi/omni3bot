# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/chao/esp/master/esp-idf/components/bootloader/subproject"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/tmp"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/src/bootloader-stamp"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/src"
  "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/chao/Workspace/Hackathon/omni3bot/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
