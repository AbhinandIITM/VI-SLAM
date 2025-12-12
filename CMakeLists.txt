cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
set(Pangolin_DIR /home/abhinand/Bionex-lab/Torso-Stabilization/Dev/Pangolin/build)
find_package(Pangolin REQUIRED)
find_package(realsense2)

# Find Boost for WebSocket++
find_package(Boost REQUIRED COMPONENTS system thread)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
endif()

# Find WebSocket++
find_package(websocketpp REQUIRED)
if(WEBSOCKETPP_FOUND)
    message(STATUS "WebSocket++ found: ${WEBSOCKETPP_INCLUDE_DIRS}")
endif()

# Find nlohmann/json (try multiple methods)
# Method 1: Using find_package (preferred for modern installations)
find_package(nlohmann_json 3.2.0 QUIET)
if(nlohmann_json_FOUND)
    message(STATUS "nlohmann/json found via find_package")
else()
    # Method 2: Check common system include paths
    find_path(NLOHMANN_JSON_INCLUDE_DIR
        NAMES nlohmann/json.hpp
        PATHS 
            /usr/include
            /usr/local/include
            /opt/homebrew/include  # For macOS Homebrew
    )
    if(NLOHMANN_JSON_INCLUDE_DIR)
        message(STATUS "nlohmann/json found at: ${NLOHMANN_JSON_INCLUDE_DIR}")
    else()
        message(WARNING "nlohmann/json NOT found. Please install it:")
        message("  Ubuntu/Debian: sudo apt-get install nlohmann-json3-dev")
        message("  Or download single header from: https://github.com/nlohmann/json/releases")
        message("  Place json.hpp in: ${PROJECT_SOURCE_DIR}/Thirdparty/nlohmann/")
    endif()
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/Thirdparty  # For nlohmann/json if using local copy
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${WEBSOCKETPP_INCLUDE_DIRS}
${NLOHMANN_JSON_INCLUDE_DIR}  # Add nlohmann/json include path
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
DBoW2
g2o
-lboost_serialization
-lcrypto
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()

# Build examples

#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

# mono_inertial_live with WebSocket++ and nlohmann/json
add_executable(mono_inertial_live
        Examples/Monocular-Inertial/mono_inertial_live.cc)
target_link_libraries(mono_inertial_live 
    ${PROJECT_NAME} 
    ${Boost_LIBRARIES}
    pthread
)

# If nlohmann_json was found via find_package, link it
if(nlohmann_json_FOUND)
    target_link_libraries(mono_inertial_live nlohmann_json::nlohmann_json)
endif()
