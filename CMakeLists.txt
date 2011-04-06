PROJECT(dense_stereo)
cmake_minimum_required(VERSION 2.6)

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(dense_stereoBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

