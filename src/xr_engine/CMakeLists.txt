# Copyright (c) 2017 The Khronos Group Inc.
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author:
#

file(GLOB LOCAL_HEADERS "*.h")
file(GLOB LOCAL_SOURCE "*.cpp")
file(GLOB VULKAN_SHADERS "vulkan_shaders/*.glsl")

# For including compiled shaders
include_directories(${CMAKE_CURRENT_BINARY_DIR})

if(ANDROID)
    add_library(xr_engine MODULE
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS})
        #$<TARGET_OBJECTS:android_native_app_glue>)
    target_link_libraries(xr_engine ${ANDROID_LIBRARY} ${ANDROID_LOG_LIBRARY})
else()
    add_library(xr_engine SHARED
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS})
endif()
set_target_properties(xr_engine PROPERTIES FOLDER ${SAMPLES_FOLDER})

source_group("Headers" FILES ${LOCAL_HEADERS})
source_group("Shaders" FILES ${VULKAN_SHADERS})

compile_glsl(run_xr_engine_glsl_compiles ${VULKAN_SHADERS})

add_dependencies(xr_engine
    generate_openxr_header
    run_xr_engine_glsl_compiles
)

set(ALVR_ROOT_DIR ${PROJECT_SOURCE_DIR}/../../../.. CACHE PATH "Root directory of ALVR source")
set(ALVR_COMMON_DIR ${ALVR_ROOT_DIR}/alvr/client/android/ALVR-common)
file(GLOB_RECURSE ALVR_COMMON_HEADERS ${ALVR_COMMON_DIR}/*.h)
file(GLOB_RECURSE ALVR_COMMON_SOURCE ${ALVR_COMMON_DIR}/*.c ${ALVR_COMMON_DIR}/*.cpp)
add_library(alvr_common ${ALVR_COMMON_SOURCE} ${ALVR_COMMON_SOURCE})

target_include_directories(xr_engine
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common

    # for OpenXR headers
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include

    # for helper headers
    ${PROJECT_SOURCE_DIR}/external/include

    ${ALVR_COMMON_DIR}/../
)

if(GLSLANG_VALIDATOR AND NOT GLSLC_COMMAND)
    target_compile_definitions(xr_engine PRIVATE USE_GLSLANGVALIDATOR)
endif()

if(Vulkan_FOUND)
    target_include_directories(xr_engine
        PRIVATE
        ${Vulkan_INCLUDE_DIRS}
    )
endif()

target_link_libraries(xr_engine openxr_loader alvr_common)
if(TARGET openxr-gfxwrapper)
    target_link_libraries(xr_engine openxr-gfxwrapper)
endif()
if(WIN32)
    if(MSVC)
        target_compile_definitions(xr_engine PRIVATE _CRT_SECURE_NO_WARNINGS)
        target_compile_options(xr_engine PRIVATE /Zc:wchar_t /Zc:forScope /W4 /WX /Zi)
        target_compile_options(xr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/Zi>")
        target_link_options(xr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/DEBUG>")
        target_link_options(xr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/OPT:REF>")
        target_link_options(xr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/OPT:ICF>")
    endif()
    target_link_libraries(xr_engine ole32)
    if(MSVC)
        # Right now can't build this on MinGW because of directxcolors, etc.
        target_link_libraries(xr_engine d3d11 d3d12 d3dcompiler dxgi)
    else()
        target_compile_definitions(xr_engine PRIVATE MISSING_DIRECTX_COLORS)
    endif()
endif()

if(Vulkan_LIBRARY)
    target_link_libraries(xr_engine ${Vulkan_LIBRARY})
endif()

if(NOT ANDROID)
    #set(ALVR_BIN_DIR ${ALVR_ROOT_DIR}/target/debug/examples)
    install(TARGETS xr_engine
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR})
    if(WIN32)
        install(FILES $<TARGET_PDB_FILE:xr_engine> DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
    #    install(TARGETS xr_engine
    #    RUNTIME DESTINATION ${ALVR_BIN_DIR}
    #    ARCHIVE DESTINATION ${ALVR_BIN_DIR})
    #install(FILES $<TARGET_PDB_FILE:xr_engine> DESTINATION ${ALVR_BIN_DIR})
    #if(NOT WIN32)
    #    install(FILES xr_engine.1 DESTINATION ${CMAKE_INSTALL_MANDIR}/man1/ COMPONENT ManPages)
    #endif()
endif()

