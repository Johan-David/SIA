project(sia_td8)

cmake_minimum_required(VERSION 3.2.0)

add_subdirectory(ext/glfw)
add_subdirectory(ext/glbinding)
add_subdirectory(ext/SOIL)

include_directories(ext/glfw/include)
include_directories(ext/glbinding/include)
include_directories(ext/eigen3)
include_directories(ext/SOIL/src)
include_directories(ext)
include_directories(include)

if (APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  CHECK_CXX_COMPILER_FLAG("-std=c++14" HAS_CPP14_FLAG)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_CPP11_FLAG)

  if (HAS_CPP14_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  elseif (HAS_CPP11_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else()
    message(FATAL_ERROR "Unsupported compiler -- Mitsuba requires C++11 support!")
  endif()
  # Enable link time optimization and set the default symbol
  # visibility to hidden (very important to obtain small binaries)
  if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    # Default symbol visibility
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
  endif()
endif()

set(SRC_FILES 
    include/viewer.h
    include/shader.h
    include/opengl.h
    include/shape.h
    include/cube.h
    include/sphere.h
    include/quad.h
    include/camera.h
    include/mesh.h
    include/trackball.h
    include/fbo.h
    include/bvh.h
    include/ray.h
    include/integration.h
    include/particles.h
    include/collider.h

    src/main.cpp
    src/viewer.cpp
    src/shader.cpp
    src/cube.cpp
    src/sphere.cpp
    src/quad.cpp
    src/camera.cpp
    src/mesh.cpp
    src/trackball.cpp
    src/fbo.cpp
    src/bvh.cpp
    src/integration.cpp
    src/particles.cpp
    src/collider.cpp
)

add_definitions(-DDATA_DIR="${PROJECT_SOURCE_DIR}/data")

add_executable(sia_td8 ${SRC_FILES})

target_link_libraries(sia_td8 glfw ${GLFW_LIBRARIES} glbinding SOIL)
