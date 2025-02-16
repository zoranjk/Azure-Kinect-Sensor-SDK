# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(LMZA_LIBRARIES lzma)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall -Wextra -pthread")

FIND_PACKAGE( OpenMP )
      if(OPENMP_FOUND)
         message("OPENMP FOUND")
         set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
         set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
      endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH}
  ${CMAKE_SOURCE_DIR}/extern/rtaudio/include
  ${CMAKE_SOURCE_DIR}/extern/rtaudio
  ${CMAKE_SOURCE_DIR}/extern/libmp3lame-CMAKE
  ${CMAKE_SOURCE_DIR}/extern/libmp3lame-CMAKE/include)

# find and include opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# include lzma
include_directories(/usr/include/lzma)

# include libusb
include_directories(/usr/include/libusb-1.0)

add_executable(minverva main.cpp keyboard.cpp keyboard.hpp)
# target_link_libraries(minverva PRIVATE k4a::k4a ${OpenCV_LIBS} ${LMZA_LIBRARIES} rtaudio mp3lame)
target_link_libraries(minverva PRIVATE k4a::k4a ${OpenCV_LIBS} ${LMZA_LIBRARIES})
# add_dependencies(minverva liblzma rtaudio mp3lame)

