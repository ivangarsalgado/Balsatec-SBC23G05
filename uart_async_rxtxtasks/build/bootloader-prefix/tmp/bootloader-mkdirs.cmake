# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v4.4.5/components/bootloader/subproject"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix/tmp"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix/src"
  "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/ivang/Desktop/UART3/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
