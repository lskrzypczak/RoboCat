cmake_minimum_required (VERSION 3.12.4)

include(env.cmake)

project(RoboCat)

# The version number.
set (ROBOCAT_VERSION_MAJOR 1)
set (ROBOCAT_VERSION_MINOR 0)

include(cppcheck.cmake)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file(
  ${CMAKE_SOURCE_DIR}/include/version.h.in
  ${CMAKE_BINARY_DIR}/generated/version.h
)

add_executable(${PROJECT_NAME} "")

include_directories(${CMAKE_BINARY_DIR}/generated)

# If build type not passed assume it is Debug
if(CMAKE_BUILD_TYPE STREQUAL  "")
   
  message(STATUS "CMAKE_BUILD_TYPE not set, assuming HOST build...") 
endif()

if(CMAKE_BUILD_TYPE STREQUAL  "Debug")
  set(PROJECT_TARGET Device)
  message(STATUS "Build type: Debug") 
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DDEBUG)
endif()

if(CMAKE_BUILD_TYPE STREQUAL  "Release")
  set(PROJECT_TARGET Device)
  message(STATUS "Build type: Release") 
  target_compile_options(${PROJECT_NAME} PUBLIC -O0)
endif()

# User defines

# Platform specific compilation options

target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)
if (PROJECT_TARGET STREQUAL "Device")
  target_compile_options(${PROJECT_NAME} PUBLIC -mfpu=vfp)
  target_compile_options(${PROJECT_NAME} PUBLIC -marm)
  target_compile_options(${PROJECT_NAME} PUBLIC -mfloat-abi=hard)
  target_compile_options(${PROJECT_NAME} PUBLIC -march=armv6zk)
  target_compile_options(${PROJECT_NAME} PUBLIC -mtune=arm1176jzf-s)
endif()
target_compile_options(${PROJECT_NAME} PUBLIC -fsingle-precision-constant)
target_compile_options(${PROJECT_NAME} PUBLIC -ffunction-sections)
target_compile_options(${PROJECT_NAME} PUBLIC -fdata-sections)
target_compile_options(${PROJECT_NAME} PUBLIC -MMD )
target_compile_options(${PROJECT_NAME} PUBLIC -MP)
target_compile_options(${PROJECT_NAME} PUBLIC -fpermissive)

# Warnings related flags
target_compile_options(${PROJECT_NAME} PUBLIC -Wall)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:C>:-Wno-discarded-qualifiers>)

# Flags used only for .cpp files
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_noexcept)
#target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-non-call-exceptions>)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-literal-suffix>)

# "source" and "include" folders
target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE    
    "${CMAKE_CURRENT_LIST_DIR}/src/RoboCat.cpp"  
    "${CMAKE_CURRENT_LIST_DIR}/src/servo.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/servoController.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/Leg.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/Simulation.cpp"
	PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include/servo.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/servoController.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/Leg.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/Simulation.h"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build) 

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")

# Include CMakeLists.txt from each directory
include_directories(${CMAKE_BINARY_DIR}/ik/include/public)
include_directories(/opt/local/include/gtk-2.0)
include_directories(/opt/local/include/glib-2.0)
include_directories(/opt/local/lib/glib-2.0/include)
include_directories(/opt/local/include/cairo)
include_directories(/opt/local/include/pango-1.0)
include_directories(/opt/local/lib/gtk-2.0/include)
include_directories(/opt/local/include/gdk-pixbuf-2.0)
include_directories(/opt/local/include/atk-1.0)
include_directories(/usr/local/include)
include_directories(/usr/local/include/eigen3)

find_library(GLIB2_LIB glib-2.0 PATHS /opt/local/lib)
message(STATUS "GLIB2_LIB: " ${GLIB2_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${GLIB2_LIB})

find_library(GTK2_LIB gtk-x11-2.0 PATHS /opt/local/lib)
message(STATUS "GTK2_LIB: " ${GTK2_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${GTK2_LIB})

find_library(GOBJECT2_LIB gobject-2.0 PATHS /opt/local/lib)
message(STATUS "GOBJECT2_LIB: " ${GOBJECT2_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${GOBJECT2_LIB})

find_library(GDK2_LIB gdk-x11-2.0 PATHS /opt/local/lib)
message(STATUS "GDK2_LIB: " ${GDK2_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${GDK2_LIB})

find_library(KDL_LIB orocos-kdl PATHS /opt/local/lib)
message(STATUS "KDL_LIB: " ${KDL_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${KDL_LIB})

find_library(CAIRO_LIB cairo PATHS /opt/local/lib)
message(STATUS "CAIRO_LIB: " ${CAIRO_LIB})
target_link_libraries(${CMAKE_PROJECT_NAME} ${CAIRO_LIB})

#set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINK_FLAGS " -Xlinker -Map=${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s")
if (PROJECT_TARGET STREQUAL "Device")
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINK_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s")
endif()
