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
    add_library(xr_app MODULE
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS}
        $<TARGET_OBJECTS:android_native_app_glue>)
    target_link_libraries(xr_app ${ANDROID_LIBRARY} ${ANDROID_LOG_LIBRARY})
else()
    add_executable(xr_app
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS})
endif()
set_target_properties(xr_app PROPERTIES FOLDER ${SAMPLES_FOLDER})

source_group("Headers" FILES ${LOCAL_HEADERS})
source_group("Shaders" FILES ${VULKAN_SHADERS})

compile_glsl(run_xr_app_glsl_compiles ${VULKAN_SHADERS})

add_dependencies(xr_app
    generate_openxr_header
    run_xr_app_glsl_compiles
)

target_include_directories(xr_app
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common

    # for OpenXR headers
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include

    # for helper headers
    ${PROJECT_SOURCE_DIR}/external/include
)

if(GLSLANG_VALIDATOR AND NOT GLSLC_COMMAND)
    target_compile_definitions(xr_app PRIVATE USE_GLSLANGVALIDATOR)
endif()

if(Vulkan_FOUND)
    target_include_directories(xr_app
        PRIVATE
        ${Vulkan_INCLUDE_DIRS}
    )
endif()

target_link_libraries(xr_app openxr_loader)
if(TARGET openxr-gfxwrapper)
    target_link_libraries(xr_app openxr-gfxwrapper)
endif()
if(WIN32)
    if(MSVC)
        target_compile_definitions(xr_app PRIVATE _CRT_SECURE_NO_WARNINGS)
        target_compile_options(xr_app PRIVATE /Zc:wchar_t /Zc:forScope /W4 /WX)
    endif()
    target_link_libraries(xr_app ole32)
    if(MSVC)
        # Right now can't build this on MinGW because of directxcolors, etc.
        target_link_libraries(xr_app d3d11 d3d12 d3dcompiler dxgi)
    else()
        target_compile_definitions(xr_app PRIVATE MISSING_DIRECTX_COLORS)
    endif()
endif()

if(Vulkan_LIBRARY)
    target_link_libraries(xr_app ${Vulkan_LIBRARY})
endif()

if(NOT ANDROID)
    install(TARGETS xr_app
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    if(NOT WIN32)
        install(FILES xr_app.1 DESTINATION ${CMAKE_INSTALL_MANDIR}/man1/ COMPONENT ManPages)
    endif()
endif()

