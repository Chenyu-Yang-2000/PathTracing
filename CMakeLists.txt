CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PT CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O2 -g")

IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(PT_SOURCES
        src/image.cpp
        src/main.cpp
        src/mesh.cpp
        src/scene_parser.cpp
        src/texture.cpp
        src/raytracing.cpp
        src/pathtracing.cpp
        src/kdtree.cpp)

SET(PT_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        include/curve.hpp
        include/revsurface.hpp
        include/bound.hpp
        include/texture.hpp
        include/render.hpp
        include/raytracing.hpp
        include/kdtree.hpp
        include/pathtracing.hpp
        include/stb_image.h)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

ADD_EXECUTABLE(${PROJECT_NAME} ${PT_SOURCES} ${PT_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
