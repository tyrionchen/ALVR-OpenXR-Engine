// Copyright (c) 2017-2021, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#define ENGINE_DLL_EXPORTS

#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformdata.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include <common/xr_linear.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <ctime>
#include <tuple>
#include <numeric>
#include <span>
#include <unordered_map>
#include <map>
#include <string_view>
#include <string>
#include <ratio>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <shared_mutex>

#include "concurrent_queue.h"
#include "alxr_engine.h"
#include "ALVR-common/packet_types.h"
#include "timing.h"
#include "latency_manager.h"

#ifdef XR_USE_PLATFORM_ANDROID
#ifndef ALXR_ENGINE_DISABLE_QUIT_ACTION
#define ALXR_ENGINE_DISABLE_QUIT_ACTION
#endif
#endif

namespace {

#if !defined(XR_USE_PLATFORM_WIN32)
#define strcpy_s(dest, source) strncpy((dest), (source), sizeof(dest))
#endif

namespace Side {
const int LEFT = 0;
const int RIGHT = 1;
const int COUNT = 2;
}  // namespace Side

inline std::string GetXrVersionString(XrVersion ver) {
    return Fmt("%d.%d.%d", XR_VERSION_MAJOR(ver), XR_VERSION_MINOR(ver), XR_VERSION_PATCH(ver));
}

inline XrFormFactor GetXrFormFactor(const std::string& formFactorStr) {
    if (EqualsIgnoreCase(formFactorStr, "Hmd")) {
        return XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    }
    if (EqualsIgnoreCase(formFactorStr, "Handheld")) {
        return XR_FORM_FACTOR_HANDHELD_DISPLAY;
    }
    throw std::invalid_argument(Fmt("Unknown form factor '%s'", formFactorStr.c_str()));
}

inline XrViewConfigurationType GetXrViewConfigurationType(const std::string& viewConfigurationStr) {
    if (EqualsIgnoreCase(viewConfigurationStr, "Mono")) {
        return XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO;
    }
    if (EqualsIgnoreCase(viewConfigurationStr, "Stereo")) {
        return XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    }
    throw std::invalid_argument(Fmt("Unknown view configuration '%s'", viewConfigurationStr.c_str()));
}

inline XrEnvironmentBlendMode GetXrEnvironmentBlendMode(const std::string& environmentBlendModeStr) {
    if (EqualsIgnoreCase(environmentBlendModeStr, "Opaque")) {
        return XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    }
    if (EqualsIgnoreCase(environmentBlendModeStr, "Additive")) {
        return XR_ENVIRONMENT_BLEND_MODE_ADDITIVE;
    }
    if (EqualsIgnoreCase(environmentBlendModeStr, "AlphaBlend")) {
        return XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
    }
    throw std::invalid_argument(Fmt("Unknown environment blend mode '%s'", environmentBlendModeStr.c_str()));
}

namespace Math {
template < typename RealT >
constexpr inline RealT ToDegrees(const RealT radians)
{
    return static_cast<RealT>(radians * (180.0 / 3.14159265358979323846));
}

inline void XrMatrix4x4f_CreateFromPose(XrMatrix4x4f& m, const XrPosef& pose)
{
    constexpr const XrVector3f scale{ 1.0f,1.0f,1.0f };
    XrMatrix4x4f_CreateTranslationRotationScale(&m, &pose.position, &pose.orientation, &scale);
}

inline XrMatrix4x4f XrMatrix4x4f_CreateFromPose(const XrPosef& pose)
{
    XrMatrix4x4f ret;
    XrMatrix4x4f_CreateFromPose(ret, pose);
    return ret;
}

namespace Pose {
XrPosef Identity() {
    XrPosef t{};
    t.orientation.w = 1;
    return t;
}

XrPosef Translation(const XrVector3f& translation) {
    XrPosef t = Identity();
    t.position = translation;
    return t;
}

XrPosef RotateCCWAboutYAxis(float radians, XrVector3f translation) {
    XrPosef t = Identity();
    t.orientation.x = 0.f;
    t.orientation.y = std::sin(radians * 0.5f);
    t.orientation.z = 0.f;
    t.orientation.w = std::cos(radians * 0.5f);
    t.position = translation;
    return t;
}

constexpr bool IsPoseValid(XrSpaceLocationFlags locationFlags) {
    constexpr XrSpaceLocationFlags PoseValidFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    return (locationFlags & PoseValidFlags) == PoseValidFlags;
}

constexpr bool IsPoseTracked(XrSpaceLocationFlags locationFlags) {
    constexpr XrSpaceLocationFlags PoseTrackedFlags =
        XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    return (locationFlags & PoseTrackedFlags) == PoseTrackedFlags;
}

constexpr bool IsPoseValid(const XrSpaceLocation& spaceLocation) {
    return IsPoseValid(spaceLocation.locationFlags);
}

constexpr bool IsPoseTracked(const XrSpaceLocation& spaceLocation) {
    return IsPoseTracked(spaceLocation.locationFlags);
}

constexpr bool IsPoseValid(const XrHandJointLocationEXT& jointLocation) {
    return IsPoseValid(jointLocation.locationFlags);
}

constexpr bool IsPoseTracked(const XrHandJointLocationEXT& jointLocation) {
    return IsPoseTracked(jointLocation.locationFlags);
}
}  // namespace Pose
}  // namespace Math

constexpr inline auto ToTrackingSpaceName(const ALXRTrackingSpace ts)
{
    switch (ts)
    {
    case ALXRTrackingSpace::LocalRefSpace: return "Local";
    case ALXRTrackingSpace::ViewRefSpace: return "View";
    }
    return "Stage";
}

/*constexpr*/ inline ALXRTrackingSpace ToTrackingSpace(const std::string_view& tsname)
{
    if (EqualsIgnoreCase(tsname, "Local"))
        return ALXRTrackingSpace::LocalRefSpace;
    if (EqualsIgnoreCase(tsname, "View"))
        return ALXRTrackingSpace::ViewRefSpace;
    return ALXRTrackingSpace::StageRefSpace;
}

constexpr inline ALXRTrackingSpace ToTrackingSpace(const XrReferenceSpaceType xrreftype)
{
    switch (xrreftype) {
    case XR_REFERENCE_SPACE_TYPE_VIEW: return ALXRTrackingSpace::ViewRefSpace;
    case XR_REFERENCE_SPACE_TYPE_LOCAL: return ALXRTrackingSpace::LocalRefSpace;
    }
    return ALXRTrackingSpace::StageRefSpace;
}

constexpr inline XrReferenceSpaceType ToXrReferenceSpaceType(const ALXRTrackingSpace xrreftype)
{
    switch (xrreftype) {
    case ALXRTrackingSpace::ViewRefSpace: return XR_REFERENCE_SPACE_TYPE_VIEW;
    case ALXRTrackingSpace::LocalRefSpace: return XR_REFERENCE_SPACE_TYPE_LOCAL;
    }
    return XR_REFERENCE_SPACE_TYPE_STAGE;
}

inline XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const std::string_view& referenceSpaceTypeStr) {
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::Identity();
    if (EqualsIgnoreCase(referenceSpaceTypeStr, "View")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "ViewFront")) {
        // Render head-locked 2m in front of device.
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::Translation({0.f, 0.f, -2.f}),
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Local")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Stage")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeft")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, {-2.f, 0.f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRight")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, {2.f, 0.f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeftRotated")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(3.14f / 3.f, {-2.f, 0.5f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRightRotated")) {
        referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(-3.14f / 3.f, {2.f, 0.5f, -2.f});
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "UboundedMSFT")) {
        referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
    } else {
        throw std::invalid_argument(Fmt("Unknown reference space type '%s'", referenceSpaceTypeStr.data()));
    }
    return referenceSpaceCreateInfo;
}

inline XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const ALXRTrackingSpace ts) {
    return GetXrReferenceSpaceCreateInfo(ToTrackingSpaceName(ts));
}

constexpr inline TrackingVector3 ToTrackingVector3(const XrVector3f& v)
{
    return { v.x, v.y, v.z };
}

constexpr inline TrackingQuat ToTrackingQuat(const XrQuaternionf& v)
{
    return { v.x, v.y, v.z, v.w };
}

constexpr inline XrPosef IdentityPose = { {0,0,0,1},{0,0,0} };
constexpr inline XrPosef ZeroPose = { {0,0,0,0},{0,0,0} };
constexpr inline const XrView IdentityView {
    .type = XR_TYPE_VIEW,
    .next = nullptr,
    .pose = IdentityPose,
    .fov = { 0,0,0,0 }
};

constexpr inline XrHandJointEXT GetJointParent(const XrHandJointEXT h)
{
    switch (h)
    {
    case XR_HAND_JOINT_PALM_EXT: return XR_HAND_JOINT_PALM_EXT;
    case XR_HAND_JOINT_WRIST_EXT: return XR_HAND_JOINT_PALM_EXT;
    case XR_HAND_JOINT_THUMB_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_THUMB_PROXIMAL_EXT: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
    case XR_HAND_JOINT_THUMB_DISTAL_EXT: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
    case XR_HAND_JOINT_THUMB_TIP_EXT: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
    case XR_HAND_JOINT_INDEX_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_INDEX_PROXIMAL_EXT: return XR_HAND_JOINT_INDEX_METACARPAL_EXT;
    case XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
    case XR_HAND_JOINT_INDEX_DISTAL_EXT: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_INDEX_TIP_EXT: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
    case XR_HAND_JOINT_MIDDLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT: return XR_HAND_JOINT_MIDDLE_METACARPAL_EXT;
    case XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
    case XR_HAND_JOINT_MIDDLE_DISTAL_EXT: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_MIDDLE_TIP_EXT: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
    case XR_HAND_JOINT_RING_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_RING_PROXIMAL_EXT: return XR_HAND_JOINT_RING_METACARPAL_EXT;
    case XR_HAND_JOINT_RING_INTERMEDIATE_EXT: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
    case XR_HAND_JOINT_RING_DISTAL_EXT: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_RING_TIP_EXT: return XR_HAND_JOINT_RING_DISTAL_EXT;
    case XR_HAND_JOINT_LITTLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
    case XR_HAND_JOINT_LITTLE_PROXIMAL_EXT: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
    case XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
    case XR_HAND_JOINT_LITTLE_DISTAL_EXT: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
    case XR_HAND_JOINT_LITTLE_TIP_EXT: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;
    }
    return h;
}

constexpr inline XrHandJointEXT ToXRHandJointType(const ALVR_HAND h)
{
    switch (h)
    {
    case ALVR_HAND::alvrHandBone_WristRoot: return XR_HAND_JOINT_WRIST_EXT;
    case ALVR_HAND::alvrHandBone_Thumb0: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb1: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb2: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Thumb3: return XR_HAND_JOINT_THUMB_TIP_EXT;
    case ALVR_HAND::alvrHandBone_Index1: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Index2: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Index3: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Middle1: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Middle2: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Middle3: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Ring1: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Ring2: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Ring3: return XR_HAND_JOINT_RING_DISTAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky0: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky1: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
    case ALVR_HAND::alvrHandBone_Pinky2: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
    case ALVR_HAND::alvrHandBone_Pinky3: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;
    default: return XR_HAND_JOINT_MAX_ENUM_EXT;
    }
}

struct OpenXrProgram final : IOpenXrProgram {
    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin,
        const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin)
        : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin(graphicsPlugin)
    {
        LogLayersAndExtensions();
    }

    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin)
        : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin{ nullptr }
    {
        LogLayersAndExtensions();
        auto& graphicsApi = options->GraphicsPlugin;
        if (graphicsApi.empty() || graphicsApi == "auto")
        {
            Log::Write(Log::Level::Info, "Running auto graphics api selection.");
            constexpr const auto to_graphics_api_str = [](const ALXRGraphicsApi gapi) -> std::tuple<std::string_view, std::string_view>
            {
                using namespace std::string_view_literals;
                switch (gapi)
                {
                case ALXRGraphicsApi::Vulkan2: return std::make_tuple("XR_KHR_vulkan_enable2"sv, "Vulkan2"sv);
                case ALXRGraphicsApi::Vulkan: return std::make_tuple("XR_KHR_vulkan_enable"sv, "Vulkan"sv);
                case ALXRGraphicsApi::D3D12: return std::make_tuple("XR_KHR_D3D12_enable"sv, "D3D12"sv);
                case ALXRGraphicsApi::D3D11: return std::make_tuple("XR_KHR_D3D11_enable"sv, "D3D11"sv);
                case ALXRGraphicsApi::OpenGLES: return std::make_tuple("XR_KHR_opengl_es_enable"sv, "OpenGLES"sv);
                default: return std::make_tuple("XR_KHR_opengl_enable"sv, "OpenGL"sv);
                }
            };
            for (size_t apiIndex = ALXRGraphicsApi::Vulkan2; apiIndex < size_t(ALXRGraphicsApi::ApiCount); ++apiIndex) {
                const auto& [ext_name, gapi] = to_graphics_api_str(static_cast<ALXRGraphicsApi>(apiIndex));
                auto itr = m_supportedGraphicsContexts.find(ext_name);
                if (itr != m_supportedGraphicsContexts.end() && itr->second) {
                    graphicsApi = gapi;
                    break;
                }
            }
        }
        m_graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);
        Log::Write(Log::Level::Info, Fmt("Selected Graphics API: %s", graphicsApi.c_str()));
    }

    virtual ~OpenXrProgram() override {

        if (m_pfnDestroyHandTrackerEXT != nullptr)
        {
            assert(m_pfnCreateHandTrackerEXT != nullptr);
            for (auto& handTracker : m_input.handerTrackers) {
                if (handTracker.tracker != XR_NULL_HANDLE) {
                    m_pfnDestroyHandTrackerEXT(handTracker.tracker);
                }
            }
        }

        if (m_input.actionSet != XR_NULL_HANDLE) {
            for (auto hand : { Side::LEFT, Side::RIGHT }) {
                xrDestroySpace(m_input.handSpace[hand]);
            }
            xrDestroyActionSet(m_input.actionSet);
        }

        for (Swapchain swapchain : m_swapchains) {
            xrDestroySwapchain(swapchain.handle);
        }

        for (XrSpace visualizedSpace : m_visualizedSpaces) {
            xrDestroySpace(visualizedSpace);
        }

        if (m_viewSpace != XR_NULL_HANDLE) {
            xrDestroySpace(m_viewSpace);
        }

        if (m_boundingStageSpace != XR_NULL_HANDLE) {
            xrDestroySpace(m_boundingStageSpace);
        }

        if (m_appSpace != XR_NULL_HANDLE) {
            xrDestroySpace(m_appSpace);
        }

        if (m_session != XR_NULL_HANDLE) {
            xrDestroySession(m_session);
        }

        if (m_instance != XR_NULL_HANDLE) {
            xrDestroyInstance(m_instance);
        }

        m_graphicsPlugin.reset();
        m_platformPlugin.reset();
    }

    using ExtensionMap = std::unordered_map<std::string_view, bool>;
    ExtensionMap m_availableSupportedExtMap = {
#ifdef XR_USE_PLATFORM_UWP
#pragma message ("UWP Extensions Enabled.")
        // Require XR_EXT_win32_appcontainer_compatible extension when building in UWP context.
        { XR_EXT_WIN32_APPCONTAINER_COMPATIBLE_EXTENSION_NAME, false },
#endif
        { XR_MSFT_UNBOUNDED_REFERENCE_SPACE_EXTENSION_NAME, false },
        { XR_MSFT_HAND_INTERACTION_EXTENSION_NAME, false },
        { "XR_KHR_convert_timespec_time", false },
        { "XR_KHR_win32_convert_performance_counter_time", false },
        { "XR_EXT_hand_tracking", false },
        { "XR_FB_display_refresh_rate", false },
        { "XR_FB_color_space", false },
        //{ XR_FB_PASSTHROUGH_EXTENSION_NAME, false },
#ifdef XR_USE_OXR_PICO
#pragma message ("Pico Neo 3 OXR Extensions Enabled.")
        { XR_PICO_VIEW_STATE_EXT_ENABLE_EXTENSION_NAME, false },
        { XR_PICO_FRAME_END_INFO_EXT_EXTENSION_NAME, false },
        { XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME, false },
        { XR_PICO_CONFIGS_EXT_EXTENSION_NAME, false },
        { XR_PICO_RESET_SENSOR_EXTENSION_NAME, false }
#endif
    };
    ExtensionMap m_supportedGraphicsContexts = {
        { "XR_KHR_vulkan_enable2",   false },
        { "XR_KHR_vulkan_enable",    false },
        { "XR_KHR_D3D12_enable",     false },
        { "XR_KHR_D3D11_enable",     false },
        { "XR_KHR_opengl_enable",    false },
        { "XR_KHR_opengl_es_enable", false }
    };

    void LogLayersAndExtensions() {
        // Write out extension properties for a given layer.
        const auto logExtensions = [this](const char* layerName, int indent = 0) {
            uint32_t instanceExtensionCount;
            CHECK_XRCMD(xrEnumerateInstanceExtensionProperties(layerName, 0, &instanceExtensionCount, nullptr));

            std::vector<XrExtensionProperties> extensions(instanceExtensionCount, {
                .type = XR_TYPE_EXTENSION_PROPERTIES,
                .next = nullptr
            });

            CHECK_XRCMD(xrEnumerateInstanceExtensionProperties(layerName, (uint32_t)extensions.size(), &instanceExtensionCount,
                extensions.data()));

            constexpr const auto SetExtensionMap = [](auto& extMap, const std::string_view extName)
            {
                const auto itr = extMap.find(extName);
                if (itr == extMap.end())
                    return;
                itr->second = true;
            };
            const std::string indentStr(indent, ' ');
            Log::Write(Log::Level::Verbose, Fmt("%sAvailable Extensions: (%d)", indentStr.c_str(), instanceExtensionCount));
            for (const XrExtensionProperties& extension : extensions) {

                SetExtensionMap(m_availableSupportedExtMap, extension.extensionName);
                SetExtensionMap(m_supportedGraphicsContexts, extension.extensionName);
                Log::Write(Log::Level::Verbose, Fmt("%s  Name=%s SpecVersion=%d", indentStr.c_str(), extension.extensionName,
                    extension.extensionVersion));
            }
        };

        // Log non-layer extensions (layerName==nullptr).
        logExtensions(nullptr);

        // Log layers and any of their extensions.
        {
            uint32_t layerCount;
            CHECK_XRCMD(xrEnumerateApiLayerProperties(0, &layerCount, nullptr));

            std::vector<XrApiLayerProperties> layers(layerCount, {
                .type = XR_TYPE_API_LAYER_PROPERTIES,
                .next = nullptr
            });
            CHECK_XRCMD(xrEnumerateApiLayerProperties((uint32_t)layers.size(), &layerCount, layers.data()));

            Log::Write(Log::Level::Info, Fmt("Available Layers: (%d)", layerCount));
            for (const XrApiLayerProperties& layer : layers) {
                Log::Write(Log::Level::Verbose,
                    Fmt("  Name=%s SpecVersion=%s LayerVersion=%d Description=%s", layer.layerName,
                        GetXrVersionString(layer.specVersion).c_str(), layer.layerVersion, layer.description));
                logExtensions(layer.layerName, 4);
            }
        }
    }

    void LogInstanceInfo() {
        CHECK(m_instance != XR_NULL_HANDLE && m_graphicsPlugin != nullptr);

        XrInstanceProperties instanceProperties{
            .type = XR_TYPE_INSTANCE_PROPERTIES,
            .next = nullptr
        };
        CHECK_XRCMD(xrGetInstanceProperties(m_instance, &instanceProperties));

        Log::Write(Log::Level::Info, Fmt("Instance RuntimeName=%s RuntimeVersion=%s", instanceProperties.runtimeName,
            GetXrVersionString(instanceProperties.runtimeVersion).c_str()));

        m_runtimeType = FromString(instanceProperties.runtimeName);
#ifdef XR_USE_OXR_PICO
        m_graphicsPlugin->SetEnableLinearizeRGB(false);
#else
        m_graphicsPlugin->SetEnableLinearizeRGB(!m_options->DisableLinearizeSrgb);
#endif
    }

    void CreateInstanceInternal() {
        CHECK(m_instance == XR_NULL_HANDLE);

        // Create union of extensions required by platform and graphics plugins.
        std::vector<const char*> extensions;

        // Transform platform and graphics extension std::strings to C strings.
        const std::vector<std::string> platformExtensions = m_platformPlugin->GetInstanceExtensions();
        std::transform(platformExtensions.begin(), platformExtensions.end(), std::back_inserter(extensions),
            [](const std::string& ext) { return ext.c_str(); });
        const std::vector<std::string> graphicsExtensions = m_graphicsPlugin->GetInstanceExtensions();
        std::transform(graphicsExtensions.begin(), graphicsExtensions.end(), std::back_inserter(extensions),
            [](const std::string& ext) { return ext.c_str(); });

        for (const auto& [extName, extAvaileble] : m_availableSupportedExtMap) {
            if (extAvaileble) {
                extensions.push_back(extName.data());
            }
        }

        Log::Write(Log::Level::Info, "Selected extensions to enable:");
        for (const auto& extName : extensions) {
            Log::Write(Log::Level::Info, Fmt("\t%s", extName));
        }

        XrInstanceCreateInfo createInfo {
            .type = XR_TYPE_INSTANCE_CREATE_INFO,
            .next = m_platformPlugin->GetInstanceCreateExtension(),
            .applicationInfo {
                .applicationVersion = 1,
                .engineVersion = 1,
                .apiVersion = XR_CURRENT_API_VERSION
            },
            .enabledExtensionCount = (uint32_t)extensions.size(),
            .enabledExtensionNames = extensions.data()
        };
        strcpy(createInfo.applicationInfo.applicationName, "alxr-client");
        strcpy(createInfo.applicationInfo.engineName, "alxr-engine");
        CHECK_XRCMD(xrCreateInstance(&createInfo, &m_instance));
    }

    void CreateInstance() override {
        //LogLayersAndExtensions();

        CreateInstanceInternal();

        LogInstanceInfo();
    }

    void LogViewConfigurations() {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);

        uint32_t viewConfigTypeCount;
        CHECK_XRCMD(xrEnumerateViewConfigurations(m_instance, m_systemId, 0, &viewConfigTypeCount, nullptr));
        std::vector<XrViewConfigurationType> viewConfigTypes(viewConfigTypeCount);
        CHECK_XRCMD(xrEnumerateViewConfigurations(m_instance, m_systemId, viewConfigTypeCount, &viewConfigTypeCount,
            viewConfigTypes.data()));
        CHECK((uint32_t)viewConfigTypes.size() == viewConfigTypeCount);

        Log::Write(Log::Level::Info, Fmt("Available View Configuration Types: (%d)", viewConfigTypeCount));
        for (XrViewConfigurationType viewConfigType : viewConfigTypes) {
            Log::Write(Log::Level::Verbose, Fmt("  View Configuration Type: %s %s", to_string(viewConfigType),
                viewConfigType == m_viewConfigType ? "(Selected)" : ""));

            XrViewConfigurationProperties viewConfigProperties{ .type=XR_TYPE_VIEW_CONFIGURATION_PROPERTIES, .next=nullptr };
            CHECK_XRCMD(xrGetViewConfigurationProperties(m_instance, m_systemId, viewConfigType, &viewConfigProperties));

            Log::Write(Log::Level::Verbose,
                Fmt("  View configuration FovMutable=%s", viewConfigProperties.fovMutable == XR_TRUE ? "True" : "False"));

            uint32_t viewCount;
            CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, 0, &viewCount, nullptr));
            if (viewCount > 0) {
                std::vector<XrViewConfigurationView> views(viewCount, { .type=XR_TYPE_VIEW_CONFIGURATION_VIEW, .next=nullptr });
                CHECK_XRCMD(
                    xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, viewCount, &viewCount, views.data()));

                for (uint32_t i = 0; i < views.size(); ++i) {
                    const XrViewConfigurationView& view = views[i];

                    Log::Write(Log::Level::Verbose, Fmt("    View [%d]: Recommended Width=%d Height=%d SampleCount=%d", i,
                        view.recommendedImageRectWidth, view.recommendedImageRectHeight,
                        view.recommendedSwapchainSampleCount));
                    Log::Write(Log::Level::Verbose,
                        Fmt("    View [%d]:     Maximum Width=%d Height=%d SampleCount=%d", i, view.maxImageRectWidth,
                            view.maxImageRectHeight, view.maxSwapchainSampleCount));
                }
            }
            else {
                Log::Write(Log::Level::Error, Fmt("Empty view configuration type"));
            }
            LogEnvironmentBlendMode(viewConfigType);
        }
    }

    using XrEnvironmentBlendModeList = std::vector<XrEnvironmentBlendMode>;
    XrEnvironmentBlendModeList GetEnvironmentBlendModes(const XrViewConfigurationType type) const
    {
        uint32_t count;
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, 0, &count, nullptr));
        if (count == 0)
            return {};
        std::vector<XrEnvironmentBlendMode> blendModes(count);
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, count, &count, blendModes.data()));
        return blendModes;
    }

    void LogEnvironmentBlendMode(XrViewConfigurationType type) {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != 0);

        const auto blendModes = GetEnvironmentBlendModes(type);
        Log::Write(Log::Level::Info, Fmt("Available Environment Blend Mode count : (%zu)", blendModes.size()));
        
        for (XrEnvironmentBlendMode mode : blendModes) {
            const bool blendModeMatch = (mode == m_environmentBlendMode);
            Log::Write(Log::Level::Info,
                Fmt("Environment Blend Mode (%s) : %s", to_string(mode), blendModeMatch ? "(Selected)" : ""));
        }
    }

    void InitializeSystem(const ALXRPaths& alxrPaths) override {

        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId == XR_NULL_SYSTEM_ID);

        m_alxrPaths = alxrPaths;

        m_formFactor = GetXrFormFactor(m_options->FormFactor);
        m_viewConfigType = GetXrViewConfigurationType(m_options->ViewConfiguration);
        m_environmentBlendMode = GetXrEnvironmentBlendMode(m_options->EnvironmentBlendMode);

        const XrSystemGetInfo systemInfo{
            .type = XR_TYPE_SYSTEM_GET_INFO,
            .formFactor = m_formFactor
        };        
        CHECK_XRCMD(xrGetSystem(m_instance, &systemInfo, &m_systemId));

        Log::Write(Log::Level::Verbose, Fmt("Using system %d for form factor %s", m_systemId, to_string(m_formFactor)));
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);

        LogViewConfigurations();

        const auto blendModes = GetEnvironmentBlendModes(m_viewConfigType);
        if (std::find(blendModes.begin(), blendModes.end(), m_environmentBlendMode) == blendModes.end() && !blendModes.empty()) {
            Log::Write(Log::Level::Info, Fmt
            (
                "Requested environment blend mode (%s) is not available, using first available mode (%s)",
                to_string(m_environmentBlendMode),
                to_string(blendModes[0])
            ));
            m_environmentBlendMode = blendModes[0];
        }

        // The graphics API can initialize the graphics device now that the systemId and instance
        // handle are available.
        m_graphicsPlugin->InitializeDevice(m_instance, m_systemId);
    }

    inline std::vector<XrReferenceSpaceType> GetAvailableReferenceSpaces() const
    {
        CHECK(m_session != XR_NULL_HANDLE);
        uint32_t spaceCount;
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, 0, &spaceCount, nullptr));
        assert(spaceCount > 0);
        std::vector<XrReferenceSpaceType> spaces(spaceCount);
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, spaceCount, &spaceCount, spaces.data()));
        return spaces;
    }

    inline XrReferenceSpaceCreateInfo GetAppReferenceSpaceCreateInfo() const {

        const auto appReferenceSpaceType = [this]() -> std::string_view
        {
            constexpr const auto refSpaceName = [](const XrReferenceSpaceType refType) {
                switch (refType) {
                case XR_REFERENCE_SPACE_TYPE_VIEW: return "View";
                case XR_REFERENCE_SPACE_TYPE_LOCAL: return "Local";
                case XR_REFERENCE_SPACE_TYPE_STAGE: return "Stage";
                case XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT: return "UboundedMSFT";
                //case XR_REFERENCE_SPACE_TYPE_COMBINED_EYE_VARJO:
                };
                assert(false); // "Uknown HMD reference space type"
                return "Stage";
            };
            const auto availSpaces = GetAvailableReferenceSpaces();
            assert(availSpaces.size() > 0);
            // iterate through order of preference/priority, STAGE is the most preferred if available.
            for (const auto spaceType : {   XR_REFERENCE_SPACE_TYPE_STAGE,
                                            XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT,
                                            XR_REFERENCE_SPACE_TYPE_LOCAL,
                                            XR_REFERENCE_SPACE_TYPE_VIEW })
            {
                if (std::find(availSpaces.begin(), availSpaces.end(), spaceType) != availSpaces.end())
                    return refSpaceName(spaceType);
            }
            // should never reach this point.
            return refSpaceName(availSpaces[0]);
        }();
        return GetXrReferenceSpaceCreateInfo(appReferenceSpaceType);
    }

#ifdef XR_USE_PLATFORM_WIN32
    static inline std::uint64_t ToTimeUs(const LARGE_INTEGER& ctr)
    {
        const std::int64_t freq = _Query_perf_frequency(); // doesn't change after system boot
        const std::int64_t whole = (ctr.QuadPart / freq) * std::micro::den;
        const std::int64_t part = (ctr.QuadPart % freq) * std::micro::den / freq;
        return static_cast<std::uint64_t>(whole + part);
    }
#else
    static inline std::uint64_t ToTimeUs(const struct timespec& ts)
    {
        return (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
    }
#endif

    inline std::uint64_t FromXrTimeUs(const XrTime xrt, const std::uint64_t defaultVal = std::uint64_t(-1)) const
    {
#ifdef XR_USE_PLATFORM_WIN32
        if (m_pfnConvertTimeToWin32PerformanceCounterKHR == nullptr)
            return defaultVal;
        LARGE_INTEGER ctr;
        if (m_pfnConvertTimeToWin32PerformanceCounterKHR(m_instance, xrt, &ctr) == XR_ERROR_TIME_INVALID)
            return defaultVal;
        return ToTimeUs(ctr);
#else
        if (m_pfnConvertTimeToTimespecTimeKHR == nullptr)
            return defaultVal;
        struct timespec ts;
        if (m_pfnConvertTimeToTimespecTimeKHR(m_instance, xrt, &ts) == XR_ERROR_TIME_INVALID)
            return defaultVal;
        return ToTimeUs(ts);
#endif
    }

    virtual inline std::tuple<XrTime, std::uint64_t> XrTimeNow() const override
    {
#ifdef XR_USE_PLATFORM_WIN32
        if (m_pfnConvertWin32PerformanceCounterToTimeKHR == nullptr)
            return { -1, std::uint64_t(-1) };
        LARGE_INTEGER ctr;
        QueryPerformanceCounter(&ctr);
        XrTime xrTimeNow;
        if (m_pfnConvertWin32PerformanceCounterToTimeKHR(m_instance, &ctr, &xrTimeNow) == XR_ERROR_TIME_INVALID)
            return { -1, std::uint64_t(-1) };
        return { xrTimeNow, ToTimeUs(ctr) };
#else
        if (m_pfnConvertTimespecTimeToTimeKHR == nullptr)
            return { -1, std::uint64_t(-1) };
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
            return { -1, std::uint64_t(-1) };
        XrTime xrTimeNow;
        if (m_pfnConvertTimespecTimeToTimeKHR(m_instance, &ts, &xrTimeNow) == XR_ERROR_TIME_INVALID)
            return { -1, std::uint64_t(-1) };
        return { xrTimeNow, ToTimeUs(ts) };
#endif
    }

    void LogReferenceSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);

        const auto spaces = GetAvailableReferenceSpaces();
        Log::Write(Log::Level::Info, Fmt("Available reference spaces: %d", spaces.size()));
        for (const XrReferenceSpaceType space : spaces) {
            Log::Write(Log::Level::Verbose, Fmt("  Name: %s", to_string(space)));
        }
    }

    struct InputState {
        XrActionSet actionSet{XR_NULL_HANDLE};
        XrAction grabAction{XR_NULL_HANDLE};
        XrAction poseAction{XR_NULL_HANDLE};
        XrAction vibrateAction{XR_NULL_HANDLE};
        XrAction quitAction{XR_NULL_HANDLE};

        std::array<XrPath, Side::COUNT> handSubactionPath;
        std::array<XrSpace, Side::COUNT> handSpace;
        std::array<float, Side::COUNT> handScale = {{1.0f, 1.0f}};
        std::array<XrBool32, Side::COUNT> handActive;
        std::array<TrackingInfo::Controller, Side::COUNT> controllerInfo{};

        using ClockType = XrSteadyClock;
        static_assert(ClockType::is_steady);
        using time_point = ClockType::time_point;
        time_point quitStartTime{};

        struct HandTrackerData
        {
            std::array<XrHandJointLocationEXT, XR_HAND_JOINT_COUNT_EXT> jointLocations;
            //std::array<XrHandJointVelocityEXT, XR_HAND_JOINT_COUNT_EXT> jointVelocities;
            XrMatrix4x4f baseOrientation;
            XrHandTrackerEXT tracker{ XR_NULL_HANDLE };
        };
        std::array<HandTrackerData, Side::COUNT> handerTrackers;

        struct ALVRAction
        {
            std::string_view name;
            std::string_view localizedName;
            XrAction xrAction{ XR_NULL_HANDLE };
        };
        using ALVRActionMap = std::unordered_map<ALVR_INPUT, ALVRAction>;
        struct ALVRScalarToBoolAction : ALVRAction
        {
            std::array<float,2> lastValues {0,0};
        };
        using ALVRScalarToBoolActionMap = std::unordered_map<ALVR_INPUT, ALVRScalarToBoolAction>;

        ALVRActionMap boolActionMap =
        {
            { ALVR_INPUT_SYSTEM_CLICK, { "system_click", "System Click" }},
            { ALVR_INPUT_APPLICATION_MENU_CLICK, { "appliction_click", "Appliction Click" }},
            { ALVR_INPUT_GRIP_CLICK, { "grip_click", "Grip Click" }},
            { ALVR_INPUT_GRIP_TOUCH, { "grip_touch", "Grip Touch" }},
            //{ ALVR_INPUT_DPAD_LEFT_CLICK, { "dpad_left_click", "Dpad Left Click" }},
            //{ ALVR_INPUT_DPAD_UP_CLICK, { "dpad_up_click", "Dpad Up Click" }},
            //{ ALVR_INPUT_DPAD_RIGHT_CLICK, { "dpad_right_click", "Dpad Right Click" }},
            //{ ALVR_INPUT_DPAD_DOWN_CLICK, { "dpad_down_click", "Dpad Down Click" }},
            { ALVR_INPUT_A_CLICK, { "a_click", "A Click" }},
            { ALVR_INPUT_A_TOUCH, { "a_touch", "A Touch" }},
            { ALVR_INPUT_B_CLICK, { "b_click", "B Click" }},
            { ALVR_INPUT_B_TOUCH, { "b_touch", "B Touch" }},
            { ALVR_INPUT_X_CLICK, { "x_click", "X Click" }},
            { ALVR_INPUT_X_TOUCH, { "x_touch", "X Touch" }},
            { ALVR_INPUT_Y_CLICK, { "y_click", "Y Click" }},
            { ALVR_INPUT_Y_TOUCH, { "y_touch", "Y Touch" }},
            { ALVR_INPUT_JOYSTICK_CLICK, { "joystick_click", "Joystick Click" }},
            { ALVR_INPUT_JOYSTICK_TOUCH, { "joystick_touch", "Joystick Touch" }},
            
            { ALVR_INPUT_BACK_CLICK, { "back_click", "Back Click" }},
            // { ALVR_INPUT_GUIDE_CLICK, { "guide_click", "Guide Click" }},
            // { ALVR_INPUT_START_CLICK, { "start_click", "Start Click" }},

            { ALVR_INPUT_TRIGGER_CLICK, { "trigger_click", "Trigger Click" }},
            { ALVR_INPUT_TRIGGER_TOUCH, { "trigger_touch", "Trigger Touch" }},
            { ALVR_INPUT_TRACKPAD_CLICK, { "trackpad_click", "Trackpad Click" }},
            { ALVR_INPUT_TRACKPAD_TOUCH, { "trackpad_touch", "Trackpad Touch" }},

            { ALVR_INPUT_THUMB_REST_TOUCH, { "thumbrest_touch", "Thumbrest Touch" }},
        };
        ALVRActionMap scalarActionMap =
        {
            { ALVR_INPUT_GRIP_VALUE,    { "grip_value", "Grip Value" }},
            { ALVR_INPUT_JOYSTICK_X,    { "joystick_x", "Joystick X" }},
            { ALVR_INPUT_JOYSTICK_Y,    { "joystick_y", "Joystick Y" }},
            { ALVR_INPUT_TRIGGER_VALUE, { "trigger_value", "Trigger Value" }},
            { ALVR_INPUT_TRACKPAD_X,    { "trackpad_x", "Trackpad X" }},
            { ALVR_INPUT_TRACKPAD_Y,    { "trackpad_y", "Trackpad Y" }},
        };
        ALVRActionMap vector2fActionMap =
        {
            { ALVR_INPUT_JOYSTICK_X, { "joystick_pos", "Joystick Pos" }},
        };
        ALVRScalarToBoolActionMap scalarToBoolActionMap =
        {
            { ALVR_INPUT_GRIP_CLICK,    { "grip_value_to_click", "Grip Value To Click" }, },
            { ALVR_INPUT_TRIGGER_CLICK, { "trigger_value_to_click", "Trigger Value To Click" } }
        };
        ALVRActionMap boolToScalarActionMap =
        {
            { ALVR_INPUT_GRIP_VALUE, { "grip_click_to_value", "Grip Click To Value" }}
        };
    };

    void InitializeActions() {
        // Create an action set.
        {
            XrActionSetCreateInfo actionSetInfo{
                .type= XR_TYPE_ACTION_SET_CREATE_INFO,
                .next= nullptr,
                .priority = 0
            };
            strcpy_s(actionSetInfo.actionSetName, "alxr");
            strcpy_s(actionSetInfo.localizedActionSetName, "ALXR");
            CHECK_XRCMD(xrCreateActionSet(m_instance, &actionSetInfo, &m_input.actionSet));
        }

        // Get the XrPath for the left and right hands - we will use them as subaction paths.
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left", &m_input.handSubactionPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right", &m_input.handSubactionPath[Side::RIGHT]));

        // Create actions.
        {
            //// Create an input action for grabbing objects with the left and right hands.
            XrActionCreateInfo actionInfo {
                .type = XR_TYPE_ACTION_CREATE_INFO,
                .next = nullptr
            };
            // Create an input action getting the left and right hand poses.
            actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
            strcpy_s(actionInfo.actionName, "hand_pose");
            strcpy_s(actionInfo.localizedActionName, "Hand Pose");
            actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
            actionInfo.subactionPaths = m_input.handSubactionPath.data();
            CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.poseAction));

            // Create output actions for vibrating the left and right controller.
            actionInfo.actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT;
            strcpy_s(actionInfo.actionName, "vibrate_hand");
            strcpy_s(actionInfo.localizedActionName, "Vibrate Hand");
            actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
            actionInfo.subactionPaths = m_input.handSubactionPath.data();
            CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.vibrateAction));

            //// Create input actions for quitting the session using the left and right controller.
            //// Since it doesn't matter which hand did this, we do not specify subaction paths for it.
            //// We will just suggest bindings for both hands, where possible.
            actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
            strcpy_s(actionInfo.actionName, "quit_session");
            strcpy_s(actionInfo.localizedActionName, "Quit Session");
            actionInfo.countSubactionPaths = 0;
            actionInfo.subactionPaths = nullptr;
            CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.quitAction));

            const auto CreateActions = [&](const XrActionType actType, auto& actionMap)
            {
                //XrActionCreateInfo actionInfo{ XR_TYPE_ACTION_CREATE_INFO };
                actionInfo.actionType = actType;
                for (auto& [k, alvrAction] : actionMap)
                {
                    strcpy_s(actionInfo.actionName, alvrAction.name.data());
                    strcpy_s(actionInfo.localizedActionName, alvrAction.localizedName.data());
                    actionInfo.countSubactionPaths = std::uint32_t(m_input.handSubactionPath.size());
                    actionInfo.subactionPaths = m_input.handSubactionPath.data();
                    CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &alvrAction.xrAction));
                }
            };
            CreateActions(XR_ACTION_TYPE_BOOLEAN_INPUT, m_input.boolActionMap);
            CreateActions(XR_ACTION_TYPE_FLOAT_INPUT, m_input.scalarActionMap);
            CreateActions(XR_ACTION_TYPE_VECTOR2F_INPUT, m_input.vector2fActionMap);
            CreateActions(XR_ACTION_TYPE_BOOLEAN_INPUT, m_input.boolToScalarActionMap);
            CreateActions(XR_ACTION_TYPE_FLOAT_INPUT, m_input.scalarToBoolActionMap);
        }

        std::array<XrPath, Side::COUNT> selectClickPath, selectValuePath;
        std::array<XrPath, Side::COUNT> squeezeClickPath, squeezeValuePath, squeezeForcePath;
        std::array<XrPath, Side::COUNT> gripPosePath, aimPosePath;
        std::array<XrPath, Side::COUNT> hapticPath;
        std::array<XrPath, Side::COUNT> menuClickPath, systemClickPath, backClickPath;
        std::array<XrPath, Side::COUNT> triggerClickPath, triggerTouchPath, triggerValuePath;
        std::array<XrPath, Side::COUNT> thumbstickXPath, thumbstickYPath, thumbstickPosPath,
                                        thumbstickClickPath, thumbstickTouchPath,
                                        thumbrestTouchPath;
        std::array<XrPath, Side::COUNT> trackpadXPath, trackpadYPath, trackpadForcePath,
                                        trackpadClickPath, trackpadTouchPath;

        std::array<XrPath, Side::COUNT> aClickPath, aTouchPath, bClickPath, bTouchPath,
                                        xClickPath, xTouchPath, yClickPath, yTouchPath;
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/select/click", &selectClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/select/click", &selectClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/select/value", &selectValuePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/select/value", &selectValuePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/force", &squeezeForcePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/force", &squeezeForcePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/click", &squeezeClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/click", &squeezeClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/grip/pose", &gripPosePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/grip/pose", &gripPosePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/aim/pose", &aimPosePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/aim/pose", &aimPosePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/output/haptic", &hapticPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/output/haptic", &hapticPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/system/click", &systemClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/system/click", &systemClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/menu/click", &menuClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/menu/click", &menuClickPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/back/click", &backClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/back/click", &backClickPath[Side::RIGHT]));
        
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/a/click", &aClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/a/click", &aClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/a/touch", &aTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/a/touch", &aTouchPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/b/click", &bClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/b/click", &bClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/b/touch", &bTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/b/touch", &bTouchPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/x/click", &xClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/x/click", &xClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/x/touch", &xTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/x/touch", &xTouchPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/y/click", &yClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/y/click", &yClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/y/touch", &yTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/y/touch", &yTouchPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trigger/click", &triggerClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trigger/click", &triggerClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trigger/touch", &triggerTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trigger/touch", &triggerTouchPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trigger/value", &triggerValuePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trigger/value", &triggerValuePath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbstick", &thumbstickPosPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbstick", &thumbstickPosPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbstick/x", &thumbstickXPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbstick/x", &thumbstickXPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbstick/y", &thumbstickYPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbstick/y", &thumbstickYPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbstick/click", &thumbstickClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbstick/click", &thumbstickClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbstick/touch", &thumbstickTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbstick/touch", &thumbstickTouchPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/thumbrest/touch", &thumbrestTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/thumbrest/touch", &thumbrestTouchPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trackpad/x", &trackpadXPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trackpad/x", &trackpadXPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trackpad/y", &trackpadYPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trackpad/y", &trackpadYPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trackpad/click", &trackpadClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trackpad/click", &trackpadClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trackpad/touch", &trackpadTouchPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trackpad/touch", &trackpadTouchPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trackpad/force", &trackpadForcePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trackpad/force", &trackpadForcePath[Side::RIGHT]));

//#define XR_DISABLE_SUGGESTED_BINDINGS
#ifndef XR_DISABLE_SUGGESTED_BINDINGS
#ifndef XR_USE_OXR_PICO
        // Suggest bindings for KHR Simple.
        {
            XrPath khrSimpleInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/khr/simple_controller", &khrSimpleInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {// Fall back to a click input for the grab action.
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, selectClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, selectClickPath[Side::RIGHT]},
                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},
                //ALVR servers currently does not use APP_MENU_CLICK event.
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::RIGHT]},

                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]},
                {m_input.quitAction, menuClickPath[Side::LEFT]},
                {m_input.quitAction, menuClickPath[Side::RIGHT]}} };
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = khrSimpleInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }
        // Suggest bindings for the Oculus Touch.
        {
            XrPath oculusTouchInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/oculus/touch_controller", &oculusTouchInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                // oculus left controller
                {m_input.boolActionMap[ALVR_INPUT_X_CLICK].xrAction, xClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_X_TOUCH].xrAction, xTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_Y_CLICK].xrAction, yClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_Y_TOUCH].xrAction, yTouchPath[Side::LEFT]},
                //ALVR servers currently does not use APP_MENU_CLICK event.
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::LEFT]},
                // oculus right controller
                {m_input.boolActionMap[ALVR_INPUT_A_CLICK].xrAction, aClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_A_TOUCH].xrAction, aTouchPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_B_CLICK].xrAction, bClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_B_TOUCH].xrAction, bTouchPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::RIGHT]},
                // oculus both controllers
                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::RIGHT]},
                //// grip_value_to_click.
                
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::RIGHT]},
                
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::RIGHT]},

                {m_input.boolActionMap[ALVR_INPUT_THUMB_REST_TOUCH].xrAction, thumbrestTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_THUMB_REST_TOUCH].xrAction, thumbrestTouchPath[Side::RIGHT]},

                {m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeValuePath[Side::LEFT]},
                {m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeValuePath[Side::RIGHT]},
                {m_input.scalarToBoolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerValuePath[Side::LEFT]},
                {m_input.scalarToBoolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerValuePath[Side::RIGHT]},

                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},

                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]},
                {m_input.quitAction, menuClickPath[Side::LEFT]}} };
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = oculusTouchInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()                
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }
        // Suggest bindings for the Vive Controller.
        {
            XrPath viveControllerInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/htc/vive_controller", &viveControllerInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::RIGHT]},
                //ALVR servers currently does not use APP_MENU_CLICK event.
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::RIGHT]},

                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::RIGHT]},

                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::RIGHT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::RIGHT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, trackpadClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, trackpadClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, trackpadTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, trackpadTouchPath[Side::RIGHT]},

                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},

                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]},
                
                {m_input.quitAction, menuClickPath[Side::LEFT]},
                {m_input.quitAction, menuClickPath[Side::RIGHT]}} };
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = viveControllerInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()                
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }

        // Suggest bindings for the Valve Index Controller.
        {
            XrPath indexControllerInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/valve/index_controller", &indexControllerInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::RIGHT]},
                
                {m_input.boolActionMap[ALVR_INPUT_A_CLICK].xrAction, aClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_A_CLICK].xrAction, aClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_A_TOUCH].xrAction, aTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_A_TOUCH].xrAction, aTouchPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_B_CLICK].xrAction, bClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_B_CLICK].xrAction, bClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_B_TOUCH].xrAction, bTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_B_TOUCH].xrAction, bTouchPath[Side::RIGHT]},

                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::RIGHT]},
                //// grip_value_to_click
                
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::RIGHT]},
                
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::RIGHT]},

                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::RIGHT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::RIGHT]},

                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},
                {m_input.quitAction, thumbstickClickPath[Side::LEFT]},
                {m_input.quitAction, thumbstickClickPath[Side::RIGHT]},
                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = indexControllerInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()                
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }

        // Suggest bindings for Hololens/WMR hand interaction.
        if (IsExtEnabled(XR_MSFT_HAND_INTERACTION_EXTENSION_NAME))
        {
            XrPath microsoftHandInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/microsoft/hand_interaction", &microsoftHandInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{{
                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},
                // left hand
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, selectValuePath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, selectValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                // right hand
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, selectValuePath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, squeezeValuePath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, selectValuePath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, squeezeValuePath[Side::RIGHT]}
            }};
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = microsoftHandInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }

        // Suggest bindings for the Microsoft Mixed Reality Motion Controller.
        {
            XrPath microsoftMixedRealityInteractionProfilePath;
            CHECK_XRCMD(xrStringToPath(m_instance, "/interaction_profiles/microsoft/motion_controller",
                                       &microsoftMixedRealityInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, systemClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},

                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeClickPath[Side::RIGHT]},
                {m_input.boolToScalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeClickPath[Side::LEFT]},
                {m_input.boolToScalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeClickPath[Side::RIGHT]},
                
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::RIGHT]},

                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_JOYSTICK_Y].xrAction, thumbstickYPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::RIGHT]},

                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_X].xrAction, trackpadXPath[Side::RIGHT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::LEFT]},
                {m_input.scalarActionMap[ALVR_INPUT_TRACKPAD_Y].xrAction, trackpadYPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::RIGHT]},

                {m_input.poseAction, aimPosePath[Side::LEFT]},
                {m_input.poseAction, aimPosePath[Side::RIGHT]},
                
                {m_input.quitAction, menuClickPath[Side::LEFT]},
                {m_input.quitAction, menuClickPath[Side::RIGHT]},

                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = microsoftMixedRealityInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()                
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }
#else
        // Suggest bindings for the Pico Neo 3 Controller.
        {
            XrPath picoMixedRealityInteractionProfilePath;
            CHECK_XRCMD(xrStringToPath(m_instance, "/interaction_profiles/pico/neo3_controller",
                &picoMixedRealityInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                    {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_CLICK].xrAction, thumbstickClickPath[Side::RIGHT]},
                    {m_input.vector2fActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickPosPath[Side::LEFT]},
                    {m_input.vector2fActionMap[ALVR_INPUT_JOYSTICK_X].xrAction, thumbstickPosPath[Side::RIGHT]},
                    {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_JOYSTICK_TOUCH].xrAction, thumbstickTouchPath[Side::RIGHT]},

                    {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::LEFT]},
                    {m_input.scalarActionMap[ALVR_INPUT_TRIGGER_VALUE].xrAction, triggerValuePath[Side::RIGHT]},
                    {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_TRIGGER_TOUCH].xrAction, triggerTouchPath[Side::RIGHT]},
                    {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_TRIGGER_CLICK].xrAction, triggerClickPath[Side::RIGHT]},

                    {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeClickPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, squeezeClickPath[Side::RIGHT]},
                    {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                    {m_input.scalarActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::RIGHT]},

                    {m_input.poseAction, aimPosePath[Side::LEFT]},
                    {m_input.poseAction, aimPosePath[Side::RIGHT]},

                    {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, backClickPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, backClickPath[Side::RIGHT]},

                    // TODO: Find out and imp batteryPath/Action.
                    //{m_input.batteryAction, batteryPath[Side::LEFT]},
                    //{m_input.batteryAction, batteryPath[Side::RIGHT]},
                    
                    {m_input.boolActionMap[ALVR_INPUT_THUMB_REST_TOUCH].xrAction, thumbrestTouchPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_THUMB_REST_TOUCH].xrAction, thumbrestTouchPath[Side::RIGHT]},

                    {m_input.boolActionMap[ALVR_INPUT_X_TOUCH].xrAction, xTouchPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_Y_TOUCH].xrAction, yTouchPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_A_TOUCH].xrAction, aTouchPath[Side::RIGHT]},
                    {m_input.boolActionMap[ALVR_INPUT_B_TOUCH].xrAction, bTouchPath[Side::RIGHT]},

                    {m_input.boolActionMap[ALVR_INPUT_X_CLICK].xrAction, xClickPath[Side::LEFT]},                    
                    {m_input.boolActionMap[ALVR_INPUT_Y_CLICK].xrAction, yClickPath[Side::LEFT]},
                    {m_input.boolActionMap[ALVR_INPUT_A_CLICK].xrAction, aClickPath[Side::RIGHT]},                    
                    {m_input.boolActionMap[ALVR_INPUT_B_CLICK].xrAction, bClickPath[Side::RIGHT]} }};
            const XrInteractionProfileSuggestedBinding suggestedBindings{
                .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
                .next = nullptr,
                .interactionProfile = picoMixedRealityInteractionProfilePath,
                .countSuggestedBindings = (uint32_t)bindings.size(),
                .suggestedBindings = bindings.data()
            };
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }
#endif
#endif

        XrActionSpaceCreateInfo actionSpaceInfo {
            .type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
            .next = nullptr,
            .action = m_input.poseAction,
            .subactionPath = m_input.handSubactionPath[Side::LEFT],
            .poseInActionSpace{.orientation{.w = 1.f}},            
        };
        CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::LEFT]));
        actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::RIGHT];
        CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::RIGHT]));

        const XrSessionActionSetsAttachInfo attachInfo {
            .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
            .next = nullptr,
            .countActionSets = 1,
            .actionSets = &m_input.actionSet,
        };
        CHECK_XRCMD(xrAttachSessionActionSets(m_session, &attachInfo));
    }

    inline bool IsExtEnabled(const std::string_view& extName) const
    {
        auto ext_itr = m_availableSupportedExtMap.find(extName);
        return ext_itr != m_availableSupportedExtMap.end() && ext_itr->second;
    }

    bool InitializeExtensions()
    {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_session != XR_NULL_HANDLE);

#ifdef XR_USE_PLATFORM_WIN32
        if (IsExtEnabled("XR_KHR_win32_convert_performance_counter_time"))
        {
            Log::Write(Log::Level::Info, "XR_KHR_win32_convert_performance_counter_time enabled.");
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimeToWin32PerformanceCounterKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimeToWin32PerformanceCounterKHR)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertWin32PerformanceCounterToTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertWin32PerformanceCounterToTimeKHR)));
        }
#endif
        if (IsExtEnabled("XR_KHR_convert_timespec_time"))
        {
            Log::Write(Log::Level::Info, "XR_KHR_convert_timespec_time enabled.");
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimespecTimeToTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimespecTimeToTimeKHR)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrConvertTimeToTimespecTimeKHR",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnConvertTimeToTimespecTimeKHR)));
        }

        if (IsExtEnabled("XR_FB_color_space"))
        {
            Log::Write(Log::Level::Info, "XR_FB_color_space enabled.");
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrEnumerateColorSpacesFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnEnumerateColorSpacesFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrSetColorSpaceFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnSetColorSpaceFB)));
        }

        if (IsExtEnabled("XR_FB_display_refresh_rate"))
        {
            Log::Write(Log::Level::Info, "XR_FB_display_refresh_rate enabled.");
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrEnumerateDisplayRefreshRatesFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnEnumerateDisplayRefreshRatesFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrGetDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnGetDisplayRefreshRateFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrRequestDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnRequestDisplayRefreshRateFB)));
        }

#if 0
#define CAT(x,y) x ## y
#define INIT_PFN(ExtName)\
    CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xr"#ExtName, reinterpret_cast<PFN_xrVoidFunction*>(&CAT(m_pfn,ExtName))));

        if (IsExtEnabled(XR_FB_PASSTHROUGH_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_FB_PASSTHROUGH_EXTENSION_NAME));
            INIT_PFN(CreatePassthroughFB);
            INIT_PFN(DestroyPassthroughFB);
            INIT_PFN(PassthroughStartFB);
            INIT_PFN(PassthroughPauseFB);
            INIT_PFN(CreatePassthroughLayerFB);
            INIT_PFN(DestroyPassthroughLayerFB);
            INIT_PFN(PassthroughLayerSetStyleFB);
            INIT_PFN(PassthroughLayerPauseFB);
            INIT_PFN(PassthroughLayerResumeFB);
        }
#undef INIT_PFN
#undef CAT
#endif

#ifdef XR_USE_OXR_PICO
        const auto GetPicoInstanceProcAddr = [this](const char* const name, auto& fn)
        {
            const XrResult result = xrGetInstanceProcAddr(m_instance, name, reinterpret_cast<PFN_xrVoidFunction*>(&fn));
            if (result != XR_SUCCESS) {
                Log::Write(Log::Level::Warning, Fmt("Unable to load xr-extension function: %s, error-code: %d", name, result));
            }
        };

        if (IsExtEnabled(XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_ANDROID_CONTROLLER_FUNCTION_EXT_ENABLE_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrGetControllerConnectionStatePico", m_pfnGetControllerConnectionStatePico);
            GetPicoInstanceProcAddr("xrSetEngineVersionPico", m_pfnSetEngineVersionPico);
            GetPicoInstanceProcAddr("xrStartCVControllerThreadPico", m_pfnStartCVControllerThreadPico);
            GetPicoInstanceProcAddr("xrStopCVControllerThreadPico", m_pfnStopCVControllerThreadPico);
            GetPicoInstanceProcAddr("xrVibrateControllerPico", m_pfnXrVibrateControllerPico);
        }

        if (IsExtEnabled(XR_PICO_CONFIGS_EXT_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_CONFIGS_EXT_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrGetConfigPICO", m_pfnGetConfigPICO);
            GetPicoInstanceProcAddr("xrSetConfigPICO", m_pfnSetConfigPICO);
        }

        if (IsExtEnabled(XR_PICO_RESET_SENSOR_EXTENSION_NAME))
        {
            Log::Write(Log::Level::Info, Fmt("%s enabled.", XR_PICO_RESET_SENSOR_EXTENSION_NAME));
            GetPicoInstanceProcAddr("xrResetSensorPICO", m_pfnResetSensorPICO);
        }
        
        if (m_pfnSetConfigPICO) {
            // const auto picoPlatformStr = std::to_string(Platform::NATIVE);
            // m_pfnSetConfigPICO(m_session, ConfigsSetEXT::PLATFORM, const_cast<char*>(picoPlatformStr.c_str()));
            // m_pfnSetConfigPICO(m_session, ConfigsSetEXT::ENABLE_SIX_DOF, "1");

            const auto trackingOriginStr = std::to_string(TrackingOrigin::STAGELEVEL);
            Log::Write(Log::Level::Info, Fmt("Setting Pico Tracking Origin: %s", trackingOriginStr.c_str()));
            m_pfnSetConfigPICO(m_session, ConfigsSetEXT::TRACKING_ORIGIN, const_cast<char*>(trackingOriginStr.c_str()));
        }
#endif

        SetDeviceColorSpace();
        UpdateSupportedDisplayRefreshRates();
        //InitializePassthroughAPI();
        return InitializeHandTrackers();
    }

    bool SetDeviceColorSpace()
    {
        if (m_pfnSetColorSpaceFB == nullptr)
            return false;

        //constexpr const auto to_string = [](const XrColorSpaceFB csType)
        //{
        //    switch (csType)
        //    {
        //    case XR_COLOR_SPACE_UNMANAGED_FB: return "UNMANAGED";
        //    case XR_COLOR_SPACE_REC2020_FB: return "REC2020";
        //    case XR_COLOR_SPACE_REC709_FB: return "REC709";
        //    case XR_COLOR_SPACE_RIFT_CV1_FB: return "RIFT_CV1";
        //    case XR_COLOR_SPACE_RIFT_S_FB: return "RIFT_S";
        //    case XR_COLOR_SPACE_QUEST_FB: return "QUEST";
        //    case XR_COLOR_SPACE_P3_FB: return "P3";
        //    case XR_COLOR_SPACE_ADOBE_RGB_FB: return "ADOBE_RGB";
        //    }
        //    return "unknown-color-space-type";
        //};

        //uint32_t colorSpaceCount = 0;
        //CHECK_XRCMD(m_pfnEnumerateColorSpacesFB(m_session, 0, &colorSpaceCount, nullptr));

        //std::vector<XrColorSpaceFB> colorSpaceTypes{ colorSpaceCount, XR_COLOR_SPACE_UNMANAGED_FB };
        //CHECK_XRCMD(m_pfnEnumerateColorSpacesFB(m_session, colorSpaceCount, &colorSpaceCount, colorSpaceTypes.data()));

        CHECK_XRCMD(m_pfnSetColorSpaceFB(m_session, XR_COLOR_SPACE_REC2020_FB));
        Log::Write(Log::Level::Info, "Color space set.");
        return true;
    }

    bool InitializeHandTrackers()
    {
        //if (m_instance != XR_NULL_HANDLE && m_systemId != XR_NULL_SYSTEM_ID)
        //    return false;

        // Inspect hand tracking system properties
        XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties{ .type=XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT, .next=nullptr };
        XrSystemProperties systemProperties{ .type=XR_TYPE_SYSTEM_PROPERTIES, .next = &handTrackingSystemProperties };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));
        if (!handTrackingSystemProperties.supportsHandTracking) {
            Log::Write(Log::Level::Info, "XR_EXT_hand_tracking is not supported.");
            // The system does not support hand tracking
            return false;
        }

        // Get function pointer for xrCreateHandTrackerEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrCreateHandTrackerEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnCreateHandTrackerEXT)));

        // Get function pointer for xrLocateHandJointsEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrLocateHandJointsEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnLocateHandJointsEXT)));

        // Get function pointer for xrLocateHandJointsEXT
        CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrDestroyHandTrackerEXT",
            reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnDestroyHandTrackerEXT)));

        if (m_pfnCreateHandTrackerEXT == nullptr ||
            m_pfnLocateHandJointsEXT == nullptr  ||
            m_pfnDestroyHandTrackerEXT == nullptr)
            return false;

        // Create a hand tracker for left hand that tracks default set of hand joints.
        const auto createHandTracker = [&](auto& handerTracker, const XrHandEXT hand)
        {
            const XrHandTrackerCreateInfoEXT createInfo{
                .type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
                .hand = hand,
                .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT
            };
            CHECK_XRCMD(m_pfnCreateHandTrackerEXT(m_session, &createInfo, &handerTracker.tracker));
        };
        createHandTracker(m_input.handerTrackers[0], XR_HAND_LEFT_EXT);
        createHandTracker(m_input.handerTrackers[1], XR_HAND_RIGHT_EXT);

        auto& leftHandBaseOrientation = m_input.handerTrackers[0].baseOrientation;
        auto& rightHandBaseOrientation = m_input.handerTrackers[1].baseOrientation;   
        XrMatrix4x4f zRot;
        XrMatrix4x4f& yRot = rightHandBaseOrientation;
        XrMatrix4x4f_CreateRotation(&yRot, 0.0, -90.0f, 0.0f);
        XrMatrix4x4f_CreateRotation(&zRot, 0.0, 0.0f, 180.0f);
        XrMatrix4x4f_Multiply(&leftHandBaseOrientation, &yRot, &zRot);
        return true;
    }

    void InitializePassthroughAPI()
    {
        if (m_session == XR_NULL_HANDLE ||
            !IsExtEnabled(XR_FB_PASSTHROUGH_EXTENSION_NAME) ||
            m_pfnCreatePassthroughFB == nullptr)
            return;

        constexpr const XrPassthroughCreateInfoFB ptci {
            .type = XR_TYPE_PASSTHROUGH_CREATE_INFO_FB,
            .next = nullptr            
        };
        if (XR_FAILED(m_pfnCreatePassthroughFB(m_session, &ptci, &m_ptLayerData.passthrough))) {
            Log::Write(Log::Level::Error, "Failed to create passthrough object!");
            m_ptLayerData = {};
            return;
        }

        const XrPassthroughLayerCreateInfoFB plci {
            .type = XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB,
            .next = nullptr,
            .passthrough = m_ptLayerData.passthrough,
            .purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB
        };
        if (XR_FAILED(m_pfnCreatePassthroughLayerFB(m_session, &plci, &m_ptLayerData.reconPassthroughLayer))) {
            Log::Write(Log::Level::Error, "Failed to create passthrough layer!");
            m_ptLayerData = {};
            return;
        }

        Log::Write(Log::Level::Info, "Passthrough API is initialized.");
    }

    void SetMaskedPassthrough() {
        if (&m_ptLayerData.reconPassthroughLayer == XR_NULL_HANDLE)
            return;

        static std::once_flag once{};
        std::call_once(once, [&]() {
            CHECK_XRCMD(m_pfnPassthroughStartFB(m_ptLayerData.passthrough));
            CHECK_XRCMD(m_pfnPassthroughLayerResumeFB(m_ptLayerData.reconPassthroughLayer));
            Log::Write(Log::Level::Info, "Passthrough Layer is resumed.");
        });
        constexpr const XrPassthroughStyleFB style {
            .type = XR_TYPE_PASSTHROUGH_STYLE_FB,
            .next = nullptr,
            .textureOpacityFactor = 0.5f,
            .edgeColor = { 0.0f, 0.0f, 0.0f, 0.0f },
        };
        CHECK_XRCMD(m_pfnPassthroughLayerSetStyleFB(m_ptLayerData.reconPassthroughLayer, &style));
    }

    void CreateVisualizedSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);

#ifdef ALXR_ENGINE_ENABLE_VIZ_SPACES
        constexpr const std::string_view visualizedSpaces[] = { "ViewFront",        "Local", "Stage", "StageLeft", "StageRight", "StageLeftRotated",
                                          "StageRightRotated" };

        for (const auto& visualizedSpace : visualizedSpaces) {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(visualizedSpace);
            XrSpace space;
            XrResult res = xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &space);
            if (XR_SUCCEEDED(res)) {
                m_visualizedSpaces.push_back(space);
                Log::Write(Log::Level::Info, Fmt("visualized-space %s added", visualizedSpace.data()));
            } else {
                Log::Write(Log::Level::Warning,
                           Fmt("Failed to create reference space %s with error %d", visualizedSpace.data(), res));
            }
        }
#endif
    }

    void InitializeSession() override {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_session == XR_NULL_HANDLE);
        {
            Log::Write(Log::Level::Verbose, Fmt("Creating session..."));

            const XrSessionCreateInfo createInfo{
                .type = XR_TYPE_SESSION_CREATE_INFO,
                .next = m_graphicsPlugin->GetGraphicsBinding(),
                .createFlags = 0,
                .systemId = m_systemId
            };
            CHECK_XRCMD(xrCreateSession(m_instance, &createInfo, &m_session));
            CHECK(m_session != XR_NULL_HANDLE);
        }
        
        InitializeExtensions();
        LogReferenceSpaces();
        InitializeActions();
        CreateVisualizedSpaces();

        {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetAppReferenceSpaceCreateInfo();
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));
            Log::Write(Log::Level::Verbose, Fmt("Selected app reference space: %s", to_string(referenceSpaceCreateInfo.referenceSpaceType)));
            m_streamConfig.trackingSpaceType = ToTrackingSpace(referenceSpaceCreateInfo.referenceSpaceType);

            referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo("Stage");
            if (XR_FAILED(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_boundingStageSpace)))
                m_boundingStageSpace = XR_NULL_HANDLE;

            referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo("View");
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_viewSpace));
        }
    }

    void ClearSwapchains()
    {
        m_swapchainImages.clear();
        m_graphicsPlugin->ClearSwapchainImageStructs();
        for (const auto& swapchain : m_swapchains)
            xrDestroySwapchain(swapchain.handle);
        m_swapchains.clear();
        m_configViews.clear();
    }

    void CreateSwapchains(const std::uint32_t eyeWidth /*= 0*/, const std::uint32_t eyeHeight /*= 0*/) override {
        CHECK(m_session != XR_NULL_HANDLE);

        if (m_swapchains.size() > 0)
        {
            CHECK(m_configViews.size() > 0 && m_swapchainImages.size() > 0);
            if (eyeWidth == 0 || eyeHeight == 0)
                return;
            const bool isSameSize = std::all_of(m_configViews.begin(), m_configViews.end(), [&](const auto& vp)
            {
                const auto eW = std::min(eyeWidth,  vp.maxImageRectWidth);
                const auto eH = std::min(eyeHeight, vp.maxImageRectHeight);
                return eW == vp.recommendedImageRectWidth && eH == vp.recommendedImageRectHeight;
            });
            if (isSameSize)
                return;
            Log::Write(Log::Level::Info, "Clearing current swapchains...");
            ClearSwapchains();
            Log::Write(Log::Level::Info, "Creating new swapchains...");
        }
        CHECK(m_swapchainImages.empty());
        CHECK(m_swapchains.empty());
        CHECK(m_configViews.empty());

        // Read graphics properties for preferred swapchain length and logging.
        XrSystemProperties systemProperties{
            .type = XR_TYPE_SYSTEM_PROPERTIES,
            .next = nullptr
        };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));

        // Log system properties.
        Log::Write(Log::Level::Info,
                   Fmt("System Properties: Name=%s VendorId=%d", systemProperties.systemName, systemProperties.vendorId));
        Log::Write(Log::Level::Info, Fmt("System Graphics Properties: MaxWidth=%d MaxHeight=%d MaxLayers=%d",
                                         systemProperties.graphicsProperties.maxSwapchainImageWidth,
                                         systemProperties.graphicsProperties.maxSwapchainImageHeight,
                                         systemProperties.graphicsProperties.maxLayerCount));
        Log::Write(Log::Level::Info, Fmt("System Tracking Properties: OrientationTracking=%s PositionTracking=%s",
                                         systemProperties.trackingProperties.orientationTracking == XR_TRUE ? "True" : "False",
                                         systemProperties.trackingProperties.positionTracking == XR_TRUE ? "True" : "False"));

        // Note: No other view configurations exist at the time this code was written. If this
        // condition is not met, the project will need to be audited to see how support should be
        // added.
        CHECK_MSG(m_viewConfigType == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, "Unsupported view configuration type");

        // Query and cache view configuration views.
        uint32_t viewCount = 0;
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, 0, &viewCount, nullptr));
        m_configViews.resize(viewCount, {
            .type = XR_TYPE_VIEW_CONFIGURATION_VIEW,
            .next = nullptr
        });
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, viewCount, &viewCount,
                                                      m_configViews.data()));
        // override recommended eye resolution
        if (eyeWidth != 0 && eyeHeight != 0) {
            for (auto& configView : m_configViews) {
                configView.recommendedImageRectWidth  = std::min(eyeWidth, configView.maxImageRectWidth);
                configView.recommendedImageRectHeight = std::min(eyeHeight, configView.maxImageRectHeight);
            }
        }

        // Create and cache view buffer for xrLocateViews later.
        m_views.resize(viewCount, IdentityView);

        // Create the swapchain and get the images.
        if (viewCount > 0) {
            // Select a swapchain format.
            uint32_t swapchainFormatCount;
            CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, 0, &swapchainFormatCount, nullptr));
            std::vector<int64_t> swapchainFormats(swapchainFormatCount);
            CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, (uint32_t)swapchainFormats.size(), &swapchainFormatCount,
                                                    swapchainFormats.data()));
            CHECK(swapchainFormatCount == swapchainFormats.size());
            m_colorSwapchainFormat = m_graphicsPlugin->SelectColorSwapchainFormat(swapchainFormats);

            // Print swapchain formats and the selected one.
            {
                std::string swapchainFormatsString;
                for (int64_t format : swapchainFormats) {
                    const bool selected = format == m_colorSwapchainFormat;
                    swapchainFormatsString += " ";
                    if (selected) {
                        swapchainFormatsString += "[";
                    }
                    swapchainFormatsString += std::to_string(format);
                    if (selected) {
                        swapchainFormatsString += "]";
                    }
                }
                Log::Write(Log::Level::Verbose, Fmt("Swapchain Formats: %s", swapchainFormatsString.c_str()));
            }

            // Create a swapchain for each view.
            for (uint32_t i = 0; i < viewCount; i++) {
                const XrViewConfigurationView& vp = m_configViews[i];
                Log::Write(Log::Level::Info,
                           Fmt("Creating swapchain for view %d with dimensions Width=%d Height=%d SampleCount=%d", i,
                               vp.recommendedImageRectWidth, vp.recommendedImageRectHeight, vp.recommendedSwapchainSampleCount));

                // Create the swapchain.
                const XrSwapchainCreateInfo swapchainCreateInfo{
                    .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
                    .next = nullptr,
                    .usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
                    .format = m_colorSwapchainFormat,
                    .sampleCount = m_graphicsPlugin->GetSupportedSwapchainSampleCount(vp),
                    .width = vp.recommendedImageRectWidth,
                    .height = vp.recommendedImageRectHeight,
                    .faceCount = 1,
                    .arraySize = 1,
                    .mipCount = 1,
                };
                Swapchain swapchain{
                    .width  = static_cast<std::int32_t>(swapchainCreateInfo.width),
                    .height = static_cast<std::int32_t>(swapchainCreateInfo.height)
                };
                CHECK_XRCMD(xrCreateSwapchain(m_session, &swapchainCreateInfo, &swapchain.handle));
                CHECK(swapchain.handle != 0);

                m_swapchains.push_back(swapchain);

                uint32_t imageCount;
                CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, 0, &imageCount, nullptr));
                // XXX This should really just return XrSwapchainImageBaseHeader*
                std::vector<XrSwapchainImageBaseHeader*> swapchainImages =
                    m_graphicsPlugin->AllocateSwapchainImageStructs(imageCount, swapchainCreateInfo);
                CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, imageCount, &imageCount, swapchainImages[0]));

                m_swapchainImages.insert(std::make_pair(swapchain.handle, std::move(swapchainImages)));
            }
        }
    }

    // Return event if one is available, otherwise return null.
    const XrEventDataBaseHeader* TryReadNextEvent() {
        // It is sufficient to clear the just the XrEventDataBuffer header to
        // XR_TYPE_EVENT_DATA_BUFFER
        XrEventDataBaseHeader* baseHeader = reinterpret_cast<XrEventDataBaseHeader*>(&m_eventDataBuffer);
        *baseHeader = {.type=XR_TYPE_EVENT_DATA_BUFFER, .next=nullptr};
        const XrResult xr = xrPollEvent(m_instance, &m_eventDataBuffer);
        if (xr == XR_SUCCESS) {
            if (baseHeader->type == XR_TYPE_EVENT_DATA_EVENTS_LOST) {
                const XrEventDataEventsLost* const eventsLost = reinterpret_cast<const XrEventDataEventsLost*>(baseHeader);
                Log::Write(Log::Level::Warning, Fmt("%d events lost", eventsLost));
            }

            return baseHeader;
        }
        if (xr == XR_EVENT_UNAVAILABLE) {
            return nullptr;
        }
        THROW_XR(xr, "xrPollEvent");
    }

    void PollEvents(bool* exitRenderLoop, bool* requestRestart) override {
        *exitRenderLoop = *requestRestart = false;

        PollStreamConfigEvents();

        // Process all pending messages.
        while (const XrEventDataBaseHeader* event = TryReadNextEvent()) {
            switch (event->type) {
                case XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB: {
                    const auto& refreshRateChangedEvent = *reinterpret_cast<const XrEventDataDisplayRefreshRateChangedFB*>(event);
                    Log::Write(Log::Level::Info, Fmt("display refresh rate has changed from %f Hz to %f Hz", refreshRateChangedEvent.fromDisplayRefreshRate, refreshRateChangedEvent.toDisplayRefreshRate));
                    m_streamConfig.renderConfig.refreshRate = refreshRateChangedEvent.toDisplayRefreshRate;
                    break;
                }
                case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                    const auto& instanceLossPending = *reinterpret_cast<const XrEventDataInstanceLossPending*>(event);
                    Log::Write(Log::Level::Warning, Fmt("XrEventDataInstanceLossPending by %lld", instanceLossPending.lossTime));
                    *exitRenderLoop = true;
                    *requestRestart = true;
                    return;
                }
                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                    const auto& sessionStateChangedEvent = *reinterpret_cast<const XrEventDataSessionStateChanged*>(event);
                    HandleSessionStateChangedEvent(sessionStateChangedEvent, exitRenderLoop, requestRestart);
                    break;
                }
                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                    LogActionSourceName(m_input.quitAction, "Quit");
                    LogActionSourceName(m_input.poseAction, "Pose");
                    LogActionSourceName(m_input.vibrateAction, "Vibrate");
                    for (const auto& [k,v] : m_input.boolActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    for (const auto& [k, v] : m_input.boolToScalarActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    for (const auto& [k, v] : m_input.scalarActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    for (const auto& [k, v] : m_input.vector2fActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    break;
                case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
                    const auto& spaceChangedEvent = *reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(event);
                    Log::Write(Log::Level::Verbose, Fmt("reference space: %d changing", spaceChangedEvent.referenceSpaceType));
                    const auto appRefSpace = ToXrReferenceSpaceType(m_streamConfig.trackingSpaceType);
                    if (spaceChangedEvent.referenceSpaceType == appRefSpace)
                        enqueueGuardianChanged(spaceChangedEvent.changeTime);
                }  break;
                default: {
                    Log::Write(Log::Level::Verbose, Fmt("Ignoring event type %d", event->type));
                    break;
                }
            }
        }
    }

    void HandleSessionStateChangedEvent(const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop,
                                        bool* requestRestart) {
        const XrSessionState oldState = m_sessionState;
        m_sessionState = stateChangedEvent.state;

        Log::Write(Log::Level::Info, Fmt("XrEventDataSessionStateChanged: state %s->%s session=%lld time=%lld", to_string(oldState),
                                         to_string(m_sessionState), stateChangedEvent.session, stateChangedEvent.time));

        if ((stateChangedEvent.session != XR_NULL_HANDLE) && (stateChangedEvent.session != m_session)) {
            Log::Write(Log::Level::Error, "XrEventDataSessionStateChanged for unknown session");
            return;
        }

        switch (m_sessionState) {
            case XR_SESSION_STATE_SYNCHRONIZED: {
                m_delayOnGuardianChanged = true;
                break;
            }
            case XR_SESSION_STATE_READY: {
                CHECK(m_session != XR_NULL_HANDLE);
                const XrSessionBeginInfo sessionBeginInfo{
                    .type = XR_TYPE_SESSION_BEGIN_INFO,
                    .next = nullptr,
                    .primaryViewConfigurationType = m_viewConfigType
                };
                XrResult result;
                CHECK_XRCMD(result = xrBeginSession(m_session, &sessionBeginInfo));
                m_sessionRunning = (result == XR_SUCCESS);
                break;
            }
            case XR_SESSION_STATE_STOPPING: {
                CHECK(m_session != XR_NULL_HANDLE);
                CHECK_XRCMD(xrEndSession(m_session))
                m_sessionRunning = false;
                break;
            }
            case XR_SESSION_STATE_EXITING: {
                *exitRenderLoop = true;
                // Do not attempt to restart because user closed this session.
                *requestRestart = false;
                break;
            }
            case XR_SESSION_STATE_LOSS_PENDING: {
                *exitRenderLoop = true;
                // Poll for a new instance.
                *requestRestart = true;
                break;
            }
            default:
                break;
        }
    }

    void LogActionSourceName(XrAction action, const std::string& actionName) const {
        XrBoundSourcesForActionEnumerateInfo getInfo = {XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO};
        getInfo.action = action;
        uint32_t pathCount = 0;
        CHECK_XRCMD(xrEnumerateBoundSourcesForAction(m_session, &getInfo, 0, &pathCount, nullptr));
        std::vector<XrPath> paths(pathCount);
        CHECK_XRCMD(xrEnumerateBoundSourcesForAction(m_session, &getInfo, uint32_t(paths.size()), &pathCount, paths.data()));

        std::string sourceName;
        for (uint32_t i = 0; i < pathCount; ++i) {
            constexpr XrInputSourceLocalizedNameFlags all = XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT |
                                                            XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT |
                                                            XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT;

            const XrInputSourceLocalizedNameGetInfo nameInfo {
                .type = XR_TYPE_INPUT_SOURCE_LOCALIZED_NAME_GET_INFO,
                .next = nullptr,
                .sourcePath = paths[i],
                .whichComponents = all
            };
            uint32_t size = 0;
            CHECK_XRCMD(xrGetInputSourceLocalizedName(m_session, &nameInfo, 0, &size, nullptr));
            if (size < 1) {
                continue;
            }
            std::vector<char> grabSource(size);
            CHECK_XRCMD(xrGetInputSourceLocalizedName(m_session, &nameInfo, uint32_t(grabSource.size()), &size, grabSource.data()));
            if (!sourceName.empty()) {
                sourceName += " and ";
            }
            sourceName += "'";
            sourceName += std::string(grabSource.data(), size - 1);
            sourceName += "'";
        }

        Log::Write(Log::Level::Info,
                   Fmt("%s action is bound to %s", actionName.c_str(), ((!sourceName.empty()) ? sourceName.c_str() : "nothing")));
    }

    bool IsSessionRunning() const override { return m_sessionRunning; }

    bool IsSessionFocused() const override { return m_sessionState == XR_SESSION_STATE_FOCUSED; }

    template < typename ControllerInfoArray >
    void PollHandTrackers(const XrTime time, ControllerInfoArray& controllerInfo) //std::array<TrackingInfo::Controller, Side::COUNT>& controllerInfo)
    {
        if (m_pfnLocateHandJointsEXT == nullptr || time == 0)
            return;

        std::array<XrMatrix4x4f, XR_HAND_JOINT_COUNT_EXT> oculusOrientedJointPoses;
        for (const auto hand : { Side::LEFT,Side::RIGHT })
        {
            auto& controller = controllerInfo[hand];//m_input.controllerInfo[hand];
            // TODO: v17/18 server does not allow for both controller & hand tracking data, this needs changing,
            //       we don't want to override a controller device pose with potentially an emulated pose for
            //       runtimes such as WMR & SteamVR.
            if (controller.enabled)
                continue;

            auto& handerTracker = m_input.handerTrackers[hand];
            //XrHandJointVelocitiesEXT velocities {
            //    .type = XR_TYPE_HAND_JOINT_VELOCITIES_EXT,
            //    .next = nullptr,
            //    .jointCount = XR_HAND_JOINT_COUNT_EXT,
            //    .jointVelocities = handerTracker.jointVelocities.data(),
            //};
            XrHandJointLocationsEXT locations {
                .type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT,
                .isActive = XR_FALSE,
                //.next = &velocities,
                .jointCount = XR_HAND_JOINT_COUNT_EXT,
                .jointLocations = handerTracker.jointLocations.data(),
            };
            const XrHandJointsLocateInfoEXT locateInfo{
                .type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT,
                .next = nullptr,
                .baseSpace = m_appSpace,
                .time = time
            };
            CHECK_XRCMD(m_pfnLocateHandJointsEXT(handerTracker.tracker, &locateInfo, &locations));
            if (locations.isActive == XR_FALSE)
                continue;

            const auto& jointLocations = handerTracker.jointLocations;
            const auto& handBaseOrientation = handerTracker.baseOrientation;
            for (size_t jointIdx = 0; jointIdx < XR_HAND_JOINT_COUNT_EXT; ++jointIdx)
            {
                const auto& jointLoc = jointLocations[jointIdx];
                XrMatrix4x4f& jointMatFixed = oculusOrientedJointPoses[jointIdx];
                if (!Math::Pose::IsPoseValid(jointLoc))
                {
                    XrMatrix4x4f_CreateIdentity(&jointMatFixed);
                    continue;
                }
                const auto jointMat = Math::XrMatrix4x4f_CreateFromPose(jointLoc.pose);
                XrMatrix4x4f_CreateIdentity(&jointMatFixed);
                XrMatrix4x4f_Multiply(&jointMatFixed, &jointMat, &handBaseOrientation);
            }
            
            for (size_t boneIndex = 0; boneIndex < ALVR_HAND::alvrHandBone_MaxSkinnable; ++boneIndex)
            {
                auto& boneRot = controller.boneRotations[boneIndex];
                auto& bonePos = controller.bonePositionsBase[boneIndex];
                boneRot = { 0,0,0,1 };
                bonePos = { 0,0,0 };
                
                const auto xrJoint = ToXRHandJointType(static_cast<ALVR_HAND>(boneIndex));
                if (xrJoint == XR_HAND_JOINT_MAX_ENUM_EXT)
                    continue;

                const auto xrJointParent = GetJointParent(xrJoint);
                const XrMatrix4x4f& jointParentWorld = oculusOrientedJointPoses[xrJointParent];
                const XrMatrix4x4f& JointWorld       = oculusOrientedJointPoses[xrJoint];

                XrMatrix4x4f jointLocal, jointParentInv;
                XrMatrix4x4f_InvertRigidBody(&jointParentInv, &jointParentWorld);
                XrMatrix4x4f_Multiply(&jointLocal, &jointParentInv, &JointWorld);

                XrQuaternionf localizedRot;
                XrMatrix4x4f_GetRotation(&localizedRot, &jointLocal);
                XrVector3f localizedPos;
                XrMatrix4x4f_GetTranslation(&localizedPos, &jointLocal);

                boneRot = ToTrackingQuat(localizedRot);
                bonePos = ToTrackingVector3(localizedPos);
            }

            controller.enabled = true;
            controller.isHand = true;

            const XrMatrix4x4f& palmMatP = oculusOrientedJointPoses[XR_HAND_JOINT_PALM_EXT];
            XrQuaternionf palmRot;
            XrVector3f palmPos;
            XrMatrix4x4f_GetTranslation(&palmPos, &palmMatP);
            XrMatrix4x4f_GetRotation(&palmRot, &palmMatP);
            controller.boneRootPosition = ToTrackingVector3(palmPos);
            controller.boneRootOrientation = ToTrackingQuat(palmRot);
        }
    }

    void PollActions() override {
        constexpr static const TrackingInfo::Controller ControllerIdentity {
            .enabled = false,
            .isHand = false,
            .buttons = 0,
            .trackpadPosition = { 0,0 },
            .triggerValue = 0.0f,
            .gripValue = 0.0f,
            .orientation = { 0,0,0,1 },
            .position = { 0,0,0 },
            .angularVelocity = { 0,0,0 },
            .linearVelocity = { 0,0,0 },
            .boneRotations = {},
            .bonePositionsBase = {},
            .boneRootOrientation = {0,0,0,1},
            .boneRootPosition = {0,0,0},
            .handFingerConfidences = 0
        };
        m_input.handActive = { XR_FALSE, XR_FALSE };
        m_input.controllerInfo = { ControllerIdentity, ControllerIdentity };

        // Sync actions
        const XrActiveActionSet activeActionSet {
            .actionSet = m_input.actionSet,
            .subactionPath = XR_NULL_PATH
        };
        const XrActionsSyncInfo syncInfo {
            .type = XR_TYPE_ACTIONS_SYNC_INFO,
            .next = nullptr,
            .countActiveActionSets = 1,
            .activeActionSets = &activeActionSet
        };
        CHECK_XRCMD(xrSyncActions(m_session, &syncInfo));

        // Get pose and grab action state and start haptic vibrate when hand is 90% squeezed.
        for (auto hand : {Side::LEFT, Side::RIGHT})
        {
            XrActionStateGetInfo getInfo {
                .type = XR_TYPE_ACTION_STATE_GET_INFO,
                .next = nullptr,
                .action = m_input.poseAction,
                .subactionPath = m_input.handSubactionPath[hand]
            };
            XrActionStatePose poseState{.type=XR_TYPE_ACTION_STATE_POSE, .next=nullptr};
            CHECK_XRCMD(xrGetActionStatePose(m_session, &getInfo, &poseState));
            m_input.handActive[hand] = poseState.isActive;

            auto& controllerInfo = m_input.controllerInfo[hand];
            if (poseState.isActive == XR_TRUE)
                controllerInfo.enabled = true;
            
            for (const auto& [buttonType, v] : m_input.boolActionMap)
            {
                if (v.xrAction == XR_NULL_HANDLE)
                    continue;
                getInfo.action = v.xrAction;
                XrActionStateBoolean boolValue{.type=XR_TYPE_ACTION_STATE_BOOLEAN, .next=nullptr};
                CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &boolValue));
                if ((boolValue.isActive == XR_TRUE) /*&& (boolValue.changedSinceLastSync == XR_TRUE)*/ && (boolValue.currentState == XR_TRUE)) {
                    controllerInfo.buttons |= ALVR_BUTTON_FLAG(buttonType);
                }
            }

            const auto GetFloatValue = [&](const InputState::ALVRAction& v, float& val)
            {
                if (v.xrAction == XR_NULL_HANDLE)
                    return;
                getInfo.action = v.xrAction;
                XrActionStateFloat floatValue{.type=XR_TYPE_ACTION_STATE_FLOAT, .next=nullptr};
                CHECK_XRCMD(xrGetActionStateFloat(m_session, &getInfo, &floatValue));
                if (floatValue.isActive == XR_FALSE)
                    return;
                val = floatValue.currentState;
                controllerInfo.enabled = true;
            };
            /*const*/ auto& scalarActionMap = m_input.scalarActionMap;
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRACKPAD_X], controllerInfo.trackpadPosition.x);
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRACKPAD_Y], controllerInfo.trackpadPosition.y);
            GetFloatValue(scalarActionMap[ALVR_INPUT_JOYSTICK_X], controllerInfo.trackpadPosition.x);
            GetFloatValue(scalarActionMap[ALVR_INPUT_JOYSTICK_Y], controllerInfo.trackpadPosition.y);
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRIGGER_VALUE], controllerInfo.triggerValue);
            GetFloatValue(scalarActionMap[ALVR_INPUT_GRIP_VALUE], controllerInfo.gripValue);

            const auto GetVector2fValue = [&](const InputState::ALVRAction& v, auto& val)
            {
                if (v.xrAction == XR_NULL_HANDLE)
                    return;
                getInfo.action = v.xrAction;
                XrActionStateVector2f vec2Value{.type=XR_TYPE_ACTION_STATE_VECTOR2F, .next=nullptr};
                CHECK_XRCMD(xrGetActionStateVector2f(m_session, &getInfo, &vec2Value));
                if (vec2Value.isActive == XR_FALSE)
                    return;
                val.x = vec2Value.currentState.x;
                val.y = vec2Value.currentState.y;
                controllerInfo.enabled = true;
            };
            /*const*/ auto& vec2fActionMap = m_input.vector2fActionMap;
            GetVector2fValue(vec2fActionMap[ALVR_INPUT_JOYSTICK_X], controllerInfo.trackpadPosition);

            for (auto& [buttonType, v] : m_input.scalarToBoolActionMap)
            {
                if (v.xrAction == XR_NULL_HANDLE)
                    continue;
                getInfo.action = v.xrAction;
                XrActionStateFloat floatValue{.type = XR_TYPE_ACTION_STATE_FLOAT, .next = nullptr};
                CHECK_XRCMD(xrGetActionStateFloat(m_session, &getInfo, &floatValue));
                if (!floatValue.isActive || !floatValue.changedSinceLastSync)
                    continue;
                if (floatValue.currentState < v.lastValues[hand]) {
                    controllerInfo.buttons |= ALVR_BUTTON_FLAG(buttonType);
                }
                v.lastValues[hand] = floatValue.currentState;
            }
            
            const auto GetFloatFromBool = [&](const InputState::ALVRAction& v, float& val)
            {
                if (v.xrAction == XR_NULL_HANDLE)
                    return;
                getInfo.action = v.xrAction;
                XrActionStateBoolean boolValue{.type = XR_TYPE_ACTION_STATE_BOOLEAN, .next = nullptr};
                CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &boolValue));
                if ((boolValue.isActive == XR_TRUE) /*&& (boolValue.changedSinceLastSync == XR_TRUE)*/ && (boolValue.currentState == XR_TRUE)) {
                    val = 1.0f;
                    controllerInfo.enabled = true;
                }
            };
            auto& boolToScalarActionMap = m_input.boolToScalarActionMap;
            GetFloatFromBool(boolToScalarActionMap[ALVR_INPUT_GRIP_VALUE], controllerInfo.gripValue);

            if (controllerInfo.buttons != 0)
                controllerInfo.enabled = true;
        }

        //PollHandTrackers();

        // haptic feedback
        constexpr static const size_t MaxPopPerFrame = 20;
        size_t popCount = 0;
        HapticsFeedback hapticFeedback;
        while (m_hapticsQueue.try_pop(hapticFeedback) && popCount < MaxPopPerFrame)
        {
            const size_t hand = hapticFeedback.alxrPath == m_alxrPaths.right_haptics ? 1 : 0;
            if (!m_input.controllerInfo[hand].isHand)
            {
                //Log::Write(Log::Level::Info, Fmt("Haptics: amp:%f duration:%f freq:%f", hapticFeedback.amplitude, hapticFeedback.duration, hapticFeedback.frequency));
                const XrHapticVibration vibration {
                    .type = XR_TYPE_HAPTIC_VIBRATION,
                    .next = nullptr,
                    .duration = static_cast<XrDuration>(static_cast<double>(hapticFeedback.duration) * 1e+9),
                    .frequency = hapticFeedback.frequency,
                    .amplitude = hapticFeedback.amplitude
                };
                const XrHapticActionInfo hapticActionInfo {
                    .type = XR_TYPE_HAPTIC_ACTION_INFO,
                    .next = nullptr,
                    .action = m_input.vibrateAction,
                    .subactionPath = m_input.handSubactionPath[hand]
                };
                /*CHECK_XRCMD*/(xrApplyHapticFeedback(m_session, &hapticActionInfo, reinterpret_cast<const XrHapticBaseHeader*>(&vibration)));
            }
            ++popCount;
        }

#ifndef ALXR_ENGINE_DISABLE_QUIT_ACTION
        // There were no subaction paths specified for the quit action, because we don't care which hand did it.
        const XrActionStateGetInfo getInfo {
            .type = XR_TYPE_ACTION_STATE_GET_INFO,
            .next = nullptr,
            .action = m_input.quitAction,
            .subactionPath = XR_NULL_PATH            
        };
        XrActionStateBoolean quitValue{.type=XR_TYPE_ACTION_STATE_BOOLEAN, .next=nullptr};
        CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &quitValue));
        if (quitValue.isActive == XR_TRUE && quitValue.currentState == XR_TRUE) {
            using namespace std::literals::chrono_literals;
            if (quitValue.changedSinceLastSync == XR_TRUE) {
                m_input.quitStartTime = InputState::ClockType::now();
            }
            else {
                constexpr const auto QuitHoldTime = 4s;
                const auto currTime = InputState::ClockType::now();
                const auto holdTime = std::chrono::duration_cast<std::chrono::seconds>(currTime - m_input.quitStartTime);
                if (holdTime >= QuitHoldTime)
                {
                    Log::Write(Log::Level::Info, "Exit session requested.");
                    m_input.quitStartTime = currTime;
                    RequestExitSession();
                }
            }
        }
#endif
    }

    inline bool UseNetworkPredicatedDisplayTime() const
    {
        return  m_runtimeType != OxrRuntimeType::SteamVR &&
                m_runtimeType != OxrRuntimeType::Monado;
    }

    void RenderFrame() override {
        CHECK(m_session != XR_NULL_HANDLE);
        const auto renderMode = m_renderMode.load();
        const bool isVideoStream = renderMode == RenderMode::VideoStream;
        std::uint64_t videoFrameDisplayTime = std::uint64_t(-1);
        if (isVideoStream) {
            m_graphicsPlugin->BeginVideoView();
            videoFrameDisplayTime = m_graphicsPlugin->GetVideoFrameIndex();
        }
        const bool timeRender = videoFrameDisplayTime != std::uint64_t(-1);
        // rendered1 appears to be unused.
        //if (timeRender)
        //    LatencyCollector::Instance().rendered1(videoFrameIndex);

        constexpr const XrFrameWaitInfo frameWaitInfo{
            .type = XR_TYPE_FRAME_WAIT_INFO,
            .next = nullptr
        };
        XrFrameState frameState{
            .type = XR_TYPE_FRAME_STATE,
            .next = nullptr
        };
        CHECK_XRCMD(xrWaitFrame(m_session, &frameWaitInfo, &frameState));
        m_PredicatedLatencyOffset.store(frameState.predictedDisplayPeriod);
        m_lastPredicatedDisplayTime.store(frameState.predictedDisplayTime);

        XrTime predictedDisplayTime;
        const auto predictedViews = GetPredicatedViews(frameState, renderMode, videoFrameDisplayTime, /*out*/ predictedDisplayTime);

        constexpr const XrFrameBeginInfo frameBeginInfo{
            .type = XR_TYPE_FRAME_BEGIN_INFO,
            .next = nullptr
        };
        CHECK_XRCMD(xrBeginFrame(m_session, &frameBeginInfo));

        //SetMaskedPassthrough();

        std::uint32_t layerCount = 0;
        std::array<const XrCompositionLayerBaseHeader*, 1> layers{};
        //XrCompositionLayerPassthroughFB passthroughLayer;
        //if (m_ptLayerData.reconPassthroughLayer != XR_NULL_HANDLE) {
        //    passthroughLayer = {
        //        .type = XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB,
        //        .next = nullptr,
        //        .flags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
        //        .space = XR_NULL_HANDLE,
        //        .layerHandle = m_ptLayerData.reconPassthroughLayer,
        //    };
        //    layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&passthroughLayer);
        //}

        XrCompositionLayerProjection layer {
            .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
            .next = nullptr
        };        
        std::array<XrCompositionLayerProjectionView,2> projectionLayerViews;
        if (frameState.shouldRender == XR_TRUE) {
            const std::span<const XrView> views { predictedViews.begin(), predictedViews.end() };
            if (RenderLayer(predictedDisplayTime, views, projectionLayerViews, layer)) {
                layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer);
            }
        }

        if (timeRender)
            LatencyCollector::Instance().rendered2(videoFrameDisplayTime);

#ifdef XR_USE_OXR_PICO
        const XrFrameEndInfoEXT xrFrameEndInfoEXT {
            .type = XR_TYPE_FRAME_END_INFO,
            .next = nullptr,
            .useHeadposeExt = 1,
            .gsIndex = m_gsIndex.load()
        };
#endif
        const XrFrameEndInfo frameEndInfo{
            .type = XR_TYPE_FRAME_END_INFO,
#ifdef XR_USE_OXR_PICO
            .next = &xrFrameEndInfoEXT,
#else
            .next = nullptr,
#endif
            // TODO: Figure out why steamvr doesn't like using custom predicated display times!!!
            .displayTime = UseNetworkPredicatedDisplayTime() ?
                predictedDisplayTime : frameState.predictedDisplayTime,
            //.displayTime = frameState.predictedDisplayTime,
            .environmentBlendMode = m_environmentBlendMode,
            .layerCount = layerCount,
            .layers = layers.data()
        };
        CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

        LatencyManager::Instance().SubmitAndSync(videoFrameDisplayTime);
        if (isVideoStream)
            m_graphicsPlugin->EndVideoView();
        
        if (m_delayOnGuardianChanged)
        {
            m_delayOnGuardianChanged = false;
            enqueueGuardianChanged();
        }
    }

    inline bool LocateViews(const XrTime predictedDisplayTime, const std::uint32_t viewCapacityInput, XrView* views) const
    {
#ifdef XR_USE_OXR_PICO
        XrViewStatePICOEXT xrViewStatePICOEXT {};
#endif
        const XrViewLocateInfo viewLocateInfo{
            .type = XR_TYPE_VIEW_LOCATE_INFO,
#ifdef XR_USE_OXR_PICO
            .next = &xrViewStatePICOEXT,
#else
            .next = nullptr,
#endif
            .viewConfigurationType = m_viewConfigType,
            .displayTime = predictedDisplayTime,
            .space = m_appSpace,
        };
        XrViewState viewState {
            .type = XR_TYPE_VIEW_STATE,
            .next = nullptr
        };
        uint32_t viewCountOutput = 0;
        const XrResult res = xrLocateViews(m_session, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, views);
#ifdef XR_USE_OXR_PICO
        m_gsIndex.store(xrViewStatePICOEXT.gsIndex);
#endif
        CHECK_XRRESULT(res, "LocateViews");
        if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
            (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
            return false;  // There is no valid tracking poses for the views.
        }
        CHECK(viewCountOutput == viewCapacityInput);
#if 0
        CHECK(viewCountOutput == m_configViews.size());
        CHECK(viewCountOutput == m_swapchains.size());
#endif
        return true;
    }

    bool RenderLayer(const XrTime predictedDisplayTime, std::array<XrCompositionLayerProjectionView,2>& projectionLayerViews,
        XrCompositionLayerProjection& layer) {

        const uint32_t viewCapacityInput = static_cast<std::uint32_t>(m_views.size());
        if (!LocateViews(predictedDisplayTime, viewCapacityInput, m_views.data()))
            return false;
        const std::span<const XrView> views { m_views.data(), m_views.size() };
        return RenderLayer
        (
            predictedDisplayTime, views,
            projectionLayerViews, layer
        );
    }

    bool RenderLayer
    (
        const XrTime predictedDisplayTime,
        const std::span<const XrView>& views,
        std::array<XrCompositionLayerProjectionView,2>& projectionLayerViews,
        XrCompositionLayerProjection& layer
    )
    {
        //projectionLayerViews.resize(views.size());
        assert(projectionLayerViews.size() == views.size());

        // For each locatable space that we want to visualize, render a 25cm cube.
        std::vector<Cube> cubes;
#ifdef ALXR_ENGINE_ENABLE_VIZ_SPACES
        for (XrSpace visualizedSpace : m_visualizedSpaces) {
            XrSpaceLocation spaceLocation{.type=XR_TYPE_SPACE_LOCATION, .next=nullptr};
            XrResult res = xrLocateSpace(visualizedSpace, m_appSpace, predictedDisplayTime, &spaceLocation);
            CHECK_XRRESULT(res, "xrLocateSpace");
            if (XR_UNQUALIFIED_SUCCESS(res)) {
                if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                    (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                    cubes.push_back(Cube{spaceLocation.pose, {0.25f, 0.25f, 0.25f}});
                }
            } else {
                Log::Write(Log::Level::Verbose, Fmt("Unable to locate a visualized reference space in app space: %d", res));
            }
        }
#endif
        const bool isVideoStream = m_renderMode == RenderMode::VideoStream;
        if (!isVideoStream) {
            // Render a 10cm cube scaled by grabAction for each hand. Note renderHand will only be
            // true when the application has focus.
            cubes.reserve(2);
            for (auto hand : { Side::LEFT, Side::RIGHT }) {
                XrSpaceLocation spaceLocation {
                    .type = XR_TYPE_SPACE_LOCATION,
                    .next = nullptr
                };
                XrResult res = xrLocateSpace(m_input.handSpace[hand], m_appSpace, predictedDisplayTime, &spaceLocation);
                CHECK_XRRESULT(res, "xrLocateSpace");
                if (XR_UNQUALIFIED_SUCCESS(res)) {
                    if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                        (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                        float scale = 0.1f * m_input.handScale[hand];
                        cubes.push_back(Cube{ spaceLocation.pose, {scale, scale, scale} });
                    }
                }
                else {
                    // Tracking loss is expected when the hand is not active so only log a message
                    // if the hand is active.
                    if (m_input.handActive[hand] == XR_TRUE) {
                        const char* handName[] = { "left", "right" };
                        Log::Write(Log::Level::Verbose,
                            Fmt("Unable to locate %s hand action space in app space: %d", handName[hand], res));
                    }
                }
            }
        }

        //if (isVideoStream)
        //    m_graphicsPlugin->BeginVideoView();

        // Render view to the appropriate part of the swapchain image.
        for (std::uint32_t i = 0; i < views.size(); ++i) {
            // Each view has a separate swapchain which is acquired, rendered to, and released.
            const Swapchain& viewSwapchain = m_swapchains[i];
            
            std::uint32_t swapchainImageIndex;
            constexpr const XrSwapchainImageAcquireInfo acquireInfo {
                .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
                .next = nullptr
            };
            CHECK_XRCMD(xrAcquireSwapchainImage(viewSwapchain.handle, &acquireInfo, &swapchainImageIndex));

            constexpr const XrSwapchainImageWaitInfo waitInfo {
                .type    = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
                .next    = nullptr,
                .timeout = XR_INFINITE_DURATION
            };
            CHECK_XRCMD(xrWaitSwapchainImage(viewSwapchain.handle, &waitInfo));

            const auto& view = views[i];
            projectionLayerViews[i] = {
                .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
                .next = nullptr,
                .pose = view.pose,
                .fov  = view.fov,
                .subImage = {
                    .swapchain = viewSwapchain.handle,
                    .imageRect = {
                        .offset = {0, 0},
                        .extent = {viewSwapchain.width, viewSwapchain.height}
                    },
                    .imageArrayIndex = 0
                }
            };
            const XrSwapchainImageBaseHeader* const swapchainImage = m_swapchainImages[viewSwapchain.handle][swapchainImageIndex];
            if (isVideoStream)
                m_graphicsPlugin->RenderVideoView(i, projectionLayerViews[i], swapchainImage, m_colorSwapchainFormat);//, cubes);
            else
                m_graphicsPlugin->RenderView(projectionLayerViews[i], swapchainImage, m_colorSwapchainFormat, cubes);
            
            constexpr const XrSwapchainImageReleaseInfo releaseInfo{
                .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                .next = nullptr
            };
            CHECK_XRCMD(xrReleaseSwapchainImage(viewSwapchain.handle, &releaseInfo));
        }

        //if (isVideoStream)
        //    m_graphicsPlugin->EndVideoView();

        layer.space = m_appSpace;
        layer.layerFlags = XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT | XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        // passthrough api flags:
        //layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
        //                   XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT |
        //                   XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
        layer.viewCount = (uint32_t)projectionLayerViews.size();
        layer.views = projectionLayerViews.data();
        return true;
    }

    virtual void SetRenderMode(const RenderMode newMode) override
    {
        m_renderMode = newMode;
    }

    virtual RenderMode GetRenderMode() const override {
        return m_renderMode;
    }

    float EstimateDisplayRefreshRate()
    {
#ifdef ALXR_ENABLE_ESTIMATE_DISPLAY_REFRESH_RATE
        if (m_session == XR_NULL_HANDLE)
            return 60.0f;

        using ClockType = XrSteadyClock;
        static_assert(ClockType::is_steady);
        using secondsf = std::chrono::duration<float, std::chrono::seconds::period>;

        using namespace std::literals::chrono_literals;
        constexpr const auto OneSecond = 1s;
        constexpr const size_t SamplesPerSec = 30;

        std::vector<size_t> frame_count_per_sec;
        frame_count_per_sec.reserve(SamplesPerSec);

        bool isStarted = false;
        size_t frameIdx = 0;
        auto last = XrSteadyClock::now();
        while (frame_count_per_sec.size() != SamplesPerSec) {
            bool exitRenderLoop = false, requestRestart = false;
            PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop)
                break;
            if (!IsSessionRunning())
                continue;
            if (!isStarted)
            {
                last = ClockType::now();
                isStarted = true;
            }
            XrFrameState frameState{ .type=XR_TYPE_FRAME_STATE, .next=nullptr };
            CHECK_XRCMD(xrWaitFrame(m_session, nullptr, &frameState));            
            CHECK_XRCMD(xrBeginFrame(m_session, nullptr));
            const XrFrameEndInfo frameEndInfo{
                .type = XR_TYPE_FRAME_END_INFO,
                .next = nullptr,
                .displayTime = frameState.predictedDisplayTime,
                .environmentBlendMode = m_environmentBlendMode,
                .layerCount = 0,
                .layers = nullptr
            };
            CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

            if (!frameState.shouldRender)
                continue;
            
            ++frameIdx;
            auto curr = ClockType::now();
            if ((curr - last) >= OneSecond)
            {
                Log::Write(Log::Level::Info, Fmt("Frame Count at %d = %d frames", frame_count_per_sec.size(), frameIdx));
                frame_count_per_sec.push_back(frameIdx);
                last = curr;
                frameIdx = 0;              
            }
        }

        const float dom = static_cast<float>(std::accumulate(frame_count_per_sec.begin(), frame_count_per_sec.end(), size_t(0)));
        const float result = dom == 0 ? 60.0f : (dom / static_cast<float>(SamplesPerSec));
        Log::Write(Log::Level::Info, Fmt("Estimated display refresh rate: %f Hz", result));
        return result;
#else
        return 90.0f;
#endif
    }

    void UpdateSupportedDisplayRefreshRates()
    {
        if (m_pfnGetDisplayRefreshRateFB) {
            CHECK_XRCMD(m_pfnGetDisplayRefreshRateFB(m_session, &m_streamConfig.renderConfig.refreshRate));
        }

        if (m_pfnEnumerateDisplayRefreshRatesFB) {
            std::uint32_t size = 0;
            CHECK_XRCMD(m_pfnEnumerateDisplayRefreshRatesFB(m_session, 0, &size, nullptr));
            m_displayRefreshRates.resize(size);
            CHECK_XRCMD(m_pfnEnumerateDisplayRefreshRatesFB(m_session, size, &size, m_displayRefreshRates.data()));
            return;
        }
        // If OpenXR runtime does not support XR_FB_display_refresh_rate extension
        // and currently core spec has no method of query the supported refresh rates
        // the only way to determine this is with a dumy loop
        m_displayRefreshRates = 
#ifdef ALXR_ENABLE_ESTIMATE_DISPLAY_REFRESH_RATE
        m_displayRefreshRates = { EstimateDisplayRefreshRate() };
#else
        m_displayRefreshRates = { 60.0f, 72.0f, 80.0f, 90.0f, 120.0f, 144.0f };
#endif
        assert(m_displayRefreshRates.size() > 0);
    }

    virtual bool GetSystemProperties(ALXRSystemProperties& systemProps) const override
    {
        if (m_instance == XR_NULL_HANDLE)
            return false;
        XrSystemProperties xrSystemProps = { .type=XR_TYPE_SYSTEM_PROPERTIES, .next=nullptr };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &xrSystemProps));
        std::strncpy(systemProps.systemName, xrSystemProps.systemName, sizeof(systemProps.systemName));
        if (m_configViews.size() > 0)
        {
            const auto& configView = m_configViews[0];
            systemProps.recommendedEyeWidth = configView.recommendedImageRectWidth;// / 2;
            systemProps.recommendedEyeHeight = configView.recommendedImageRectHeight;
        }
        assert(m_displayRefreshRates.size() > 0);
        systemProps.refreshRates = m_displayRefreshRates.data();
        systemProps.refreshRatesCount = static_cast<std::uint32_t>(m_displayRefreshRates.size());
        systemProps.currentRefreshRate = m_displayRefreshRates.back();
        if (m_pfnGetDisplayRefreshRateFB)
            CHECK_XRCMD(m_pfnGetDisplayRefreshRateFB(m_session, &systemProps.currentRefreshRate));
        return true;
    }

    struct SpaceLoc
    {
        XrPosef pose = IdentityPose;
        XrVector3f linearVelocity = { 0,0,0 };
        XrVector3f angularVelocity = { 0,0,0 };

        constexpr inline bool is_zero() const {
            return  pose.position.x == 0 &&
                    pose.position.y == 0 &&
                    pose.position.z == 0 &&
                    pose.orientation.x == 0 &&
                    pose.orientation.y == 0 &&
                    pose.orientation.z == 0 &&
                    pose.orientation.w == 0;
        }
    };
    constexpr /*inline*/ static const SpaceLoc IdentitySpaceLoc = { IdentityPose, {0,0,0}, {0,0,0} };
    constexpr /*inline*/ static const SpaceLoc ZeroSpaceLoc = { ZeroPose, {0,0,0}, {0,0,0} };

    inline SpaceLoc GetSpaceLocation
    (
        const XrSpace& targetSpace,
        const XrSpace& baseSpace,
        const XrTime& time,
        const SpaceLoc& initLoc = IdentitySpaceLoc
    ) const
    {
        XrSpaceVelocity velocity{ XR_TYPE_SPACE_VELOCITY, nullptr };
        XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION, &velocity };
        const auto res = xrLocateSpace(targetSpace, baseSpace, time, &spaceLocation);
        //CHECK_XRRESULT(res, "xrLocateSpace");

        SpaceLoc result = initLoc;
        if (!XR_UNQUALIFIED_SUCCESS(res))
            return result;

        const auto& pose = spaceLocation.pose;
        if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0)
            result.pose.position = pose.position;

        if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
            result.pose.orientation = pose.orientation;

        if ((velocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0)
            result.linearVelocity = velocity.linearVelocity;

        if ((velocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) != 0)
            result.angularVelocity = velocity.angularVelocity;

        return result;
    }

    inline SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const XrSpace& baseSpace, const SpaceLoc& initLoc = IdentitySpaceLoc) const
    {
        return GetSpaceLocation(targetSpace, baseSpace, m_lastPredicatedDisplayTime, initLoc);
    }

    inline SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const XrTime& time, const SpaceLoc& initLoc = IdentitySpaceLoc) const
    {
        return GetSpaceLocation(targetSpace, m_appSpace, time, initLoc);
    }

    inline SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const SpaceLoc& initLoc = IdentitySpaceLoc) const
    {
        return GetSpaceLocation(targetSpace, m_lastPredicatedDisplayTime, initLoc);
    }

    inline std::array<XrView,2> GetPredicatedViews
    (
        const XrFrameState& frameState, const RenderMode renderMode, const std::uint64_t videoTimeStampNs,
        XrTime& predicateDisplayTime
    )
    {
        assert(frameState.predictedDisplayPeriod >= 0);
        const auto GetDefaultViews = [&]()-> std::array<XrView, 2> {
            LocateViews(frameState.predictedDisplayTime, (uint32_t)m_views.size(), m_views.data());
            return { m_views[0], m_views[1] };
        };
        predicateDisplayTime = frameState.predictedDisplayTime;
        if (renderMode == RenderMode::Lobby)
            return GetDefaultViews();

        std::shared_lock<std::shared_mutex> l(m_trackingFrameMapMutex);
        if (videoTimeStampNs != std::uint64_t(-1))
        {
            const auto trackingFrameItr = m_trackingFrameMap.find(videoTimeStampNs);
            if (trackingFrameItr != m_trackingFrameMap.cend()) {
                predicateDisplayTime = trackingFrameItr->second.displayTime;
                return trackingFrameItr->second.views;
            }
        }
        const auto result = m_trackingFrameMap.rbegin();
        if (result == m_trackingFrameMap.rend())
            return GetDefaultViews();
        predicateDisplayTime = result->second.displayTime;
        return result->second.views;
    }

    static inline ALXREyeInfo GetEyeInfo(const XrView& left_view, const XrView& right_view)
    {
        XrVector3f v;
        XrVector3f_Sub(&v, &right_view.pose.position, &left_view.pose.position);
        float ipd = std::fabs(XrVector3f_Length(&v));
        if (ipd < 0.00001f)
            ipd = 0.063f;
        constexpr const auto ToEyeFov = [](const XrFovf& fov) -> EyeFov
        {
            return EyeFov{
                .left   = fov.angleLeft,
                .right  = fov.angleRight,
                .top    = fov.angleUp,
                .bottom = fov.angleDown
            };
        };
        return ALXREyeInfo{
            .eyeFov = {
                ToEyeFov(left_view.fov),
                ToEyeFov(right_view.fov)
            },
            .ipd = ipd
        };
    }

    static inline ALXREyeInfo GetEyeInfo(const std::array<XrView, 2>& views)
    {
        return GetEyeInfo(views[0], views[1]);
    }

    virtual inline bool GetEyeInfo(ALXREyeInfo& eyeInfo, const XrTime& time) const override
    {
        std::array<XrView, 2> newViews{ IdentityView, IdentityView };
        LocateViews(time, static_cast<const std::uint32_t>(newViews.size()), newViews.data());
        eyeInfo = GetEyeInfo(newViews[0], newViews[1]);
        return true;
    }

    virtual inline bool GetEyeInfo(ALXREyeInfo& eyeInfo) const override
    {
        return GetEyeInfo(eyeInfo, m_lastPredicatedDisplayTime);
    }

    virtual bool GetTrackingInfo(TrackingInfo& info) /*const*/ override
    {
        const XrDuration predicatedLatencyOffsetNs = m_PredicatedLatencyOffset.load();
        info = {
            .mounted = true,
            .controller = { m_input.controllerInfo[0], m_input.controllerInfo[1] }
        };
        assert(predicatedLatencyOffsetNs >= 0);
        
        const auto trackingPredictionLatencyUs = LatencyCollector::Instance().getTrackingPredictionLatency();
        const auto [xrTimeStamp, timeStampUs] = XrTimeNow();
        assert(timeStampUs != std::uint64_t(-1) && xrTimeStamp >= 0);

        const XrDuration totalLatencyOffsetNs = static_cast<XrDuration>(trackingPredictionLatencyUs * 1000) + predicatedLatencyOffsetNs;
        const auto predicatedDisplayTimeXR = xrTimeStamp + totalLatencyOffsetNs;
        const auto predicatedDisplayTimeNs = (timeStampUs * 1000) + static_cast<std::uint64_t>(totalLatencyOffsetNs);
        
        std::array<XrView, 2> newViews { IdentityView, IdentityView };
        LocateViews(predicatedDisplayTimeXR, (const std::uint32_t)newViews.size(), newViews.data());
         {
             std::unique_lock<std::shared_mutex> lock(m_trackingFrameMapMutex);
             m_trackingFrameMap[predicatedDisplayTimeNs] = {
                 .views       = newViews,
                 //.timestamp   = predicatedDisplayTimeNs,
                 .displayTime = predicatedDisplayTimeXR
             };
             if (m_trackingFrameMap.size() > MaxTrackingFrameCount)
                 m_trackingFrameMap.erase(m_trackingFrameMap.begin());
         }
        info.targetTimestampNs = predicatedDisplayTimeNs;
        
        const auto hmdSpaceLoc = GetSpaceLocation(m_viewSpace, predicatedDisplayTimeXR);
        info.HeadPose_Pose_Orientation  = ToTrackingQuat(hmdSpaceLoc.pose.orientation);
        info.HeadPose_Pose_Position     = ToTrackingVector3(hmdSpaceLoc.pose.position);
        // info.HeadPose_LinearVelocity    = ToTrackingVector3(hmdSpaceLoc.linearVelocity);
        // info.HeadPose_AngularVelocity   = ToTrackingVector3(hmdSpaceLoc.angularVelocity);

        for (const auto hand : { Side::LEFT, Side::RIGHT }) {
            auto& newContInfo = info.controller[hand];
#ifdef XR_USE_OXR_PICO
            // 
            // As of writing, there are bugs in Pico's OXR runtime with either/both:
            //      * xrLocateSpace for controller action spaces not working with any other times beyond XrFrameState::predicateDisplayTime (and zero, in a non-conforming way).
            //      * xrConvertTimeToTimespecTimeKHR appears to return values in microseconds instead of nanoseconds and values seem to be completely of from what
            //        XrFrameState::predicateDisplayTime values are.   
            //
            //  This workaround will induce some small amount of "lag" as the times don't account for network latency and the HMD poses being in future times.
            //
            const auto spaceLoc = GetSpaceLocation(m_input.handSpace[hand], m_lastPredicatedDisplayTime);
#else
            const auto spaceLoc = GetSpaceLocation(m_input.handSpace[hand], predicatedDisplayTimeXR);
#endif
            newContInfo.position        = ToTrackingVector3(spaceLoc.pose.position);
            newContInfo.orientation     = ToTrackingQuat(spaceLoc.pose.orientation);
            newContInfo.linearVelocity  = ToTrackingVector3(spaceLoc.linearVelocity);
            newContInfo.angularVelocity = ToTrackingVector3(spaceLoc.angularVelocity);
        }

        PollHandTrackers(predicatedDisplayTimeXR, info.controller);

        LatencyCollector::Instance().tracking(predicatedDisplayTimeNs);
        return true;
    }

    virtual inline void EnqueueHapticFeedback(const HapticsFeedback& hapticFeedback) override
    {
        m_hapticsQueue.push(hapticFeedback);
    }

    virtual inline void SetStreamConfig(const ALXRStreamConfig& config) override
    {
        m_streamConfigQueue.push(config);
    }

    virtual inline bool GetStreamConfig(ALXRStreamConfig& config) const override
    {
        // TODO: Check for thread sync!
        config = m_streamConfig;
        return true;
    }

    void PollStreamConfigEvents()
    {
        ALXRStreamConfig newConfig;
        if (!m_streamConfigQueue.try_pop(newConfig))
            return;

        if (newConfig.trackingSpaceType != m_streamConfig.trackingSpaceType) {
            const auto IsRefSpaceTypeSupported = [this](const ALXRTrackingSpace ts) {
                const auto xrSpaceRefType = ToXrReferenceSpaceType(ts);
                const auto availSpaces = GetAvailableReferenceSpaces();
                return std::find(availSpaces.begin(), availSpaces.end(), xrSpaceRefType) != availSpaces.end();
            };
            if (IsRefSpaceTypeSupported(newConfig.trackingSpaceType)) {
                if (m_appSpace != XR_NULL_HANDLE) {
                    xrDestroySpace(m_appSpace);
                    m_appSpace = XR_NULL_HANDLE;
                }
                const auto oldTrackingSpaceName = ToTrackingSpaceName(m_streamConfig.trackingSpaceType);
                const auto newTrackingSpaceName = ToTrackingSpaceName(newConfig.trackingSpaceType);
                Log::Write(Log::Level::Info, Fmt("Changing tracking space from %s to %s", oldTrackingSpaceName, newTrackingSpaceName));
                XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(newTrackingSpaceName);
                CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));

                m_streamConfig.trackingSpaceType = newConfig.trackingSpaceType;
            }
            else {
                Log::Write(Log::Level::Warning, Fmt("Tracking space %s is not supported, tracking space is not changed.", ToTrackingSpaceName(newConfig.trackingSpaceType)));
            }
        }

        auto& currRenderConfig = m_streamConfig.renderConfig;
        const auto& newRenderConfig = newConfig.renderConfig;        
        if (newRenderConfig.refreshRate != currRenderConfig.refreshRate) {
            [&]() {
                if (m_pfnRequestDisplayRefreshRateFB == nullptr) {
                    Log::Write(Log::Level::Warning, "This OpenXR runtime does not support setting the display refresh rate.");
                    return;
                }

                const auto itr = std::find(m_displayRefreshRates.begin(), m_displayRefreshRates.end(), newRenderConfig.refreshRate);
                if (itr == m_displayRefreshRates.end()) {
                    Log::Write(Log::Level::Warning, Fmt("Selected new refresh rate %f Hz is not supported, no change has been made.", newRenderConfig.refreshRate));
                    return;
                }

                Log::Write(Log::Level::Info, Fmt("Setting display refresh rate from %f Hz to %f Hz.", currRenderConfig.refreshRate, newRenderConfig.refreshRate));
                CHECK_XRCMD(m_pfnRequestDisplayRefreshRateFB(m_session, newRenderConfig.refreshRate));
                currRenderConfig.refreshRate = newRenderConfig.refreshRate;
            }();
        }

        //m_streamConfig = newConfig;
    }

    virtual inline void RequestExitSession() override
    {
        if (m_session == XR_NULL_HANDLE)
            return;
        CHECK_XRCMD(xrRequestExitSession(m_session));
    }

    virtual inline bool GetGuardianData(ALXRGuardianData& gd) /*const*/ override
    {
        gd.shouldSync = false;
        return m_guardianChangedQueue.try_pop(gd);
    }

    inline bool GetBoundingStageSpace(const XrTime& time, SpaceLoc& space, XrExtent2Df& boundingArea) const
    {
        if (m_session == XR_NULL_HANDLE ||
            m_boundingStageSpace == XR_NULL_HANDLE)
            return false;
        if (XR_FAILED(xrGetReferenceSpaceBoundsRect(m_session, XR_REFERENCE_SPACE_TYPE_STAGE, &boundingArea)))
        {
            Log::Write(Log::Level::Info, "xrGetReferenceSpaceBoundsRect FAILED.");
            return false;
        }
        space = GetSpaceLocation(m_boundingStageSpace, time, ZeroSpaceLoc);
        return !space.is_zero();
    }

    inline bool GetBoundingStageSpace(const XrTime& time, ALXRGuardianData& gd) const
    {
        SpaceLoc loc;
        XrExtent2Df boundingArea;
        if (!GetBoundingStageSpace(time, loc, boundingArea))
            return false;
        gd = {
            .shouldSync = true,
            .areaWidth = boundingArea.width,
            .areaHeight = boundingArea.height
        };
        return true;
    }

    inline bool enqueueGuardianChanged(const XrTime& time)
    {
        Log::Write(Log::Level::Verbose, "Enqueuing guardian changed");
        ALXRGuardianData gd {
            .shouldSync = false
        };
        if (!GetBoundingStageSpace(time, gd))
            return false;
        Log::Write(Log::Level::Verbose, "Guardian changed enqueud successfully.");
        m_guardianChangedQueue.push(gd);
        return true;
    }

    bool enqueueGuardianChanged() {
        return enqueueGuardianChanged(m_lastPredicatedDisplayTime);
    }

#ifdef XR_USE_OXR_PICO
    enum PxrHmdDof : int
    {
        PXR_HMD_3DOF = 0,
        PXR_HMD_6DOF
    };
    enum PxrControllerDof : int
    {
        PXR_CONTROLLER_3DOF = 0,
        PXR_CONTROLLER_6DOF
    };
#endif
    virtual inline void Resume() override
    {
#ifdef XR_USE_OXR_PICO
        if (m_instance == XR_NULL_HANDLE) {
            Log::Write(Log::Level::Warning, "OpenXrProgram::Resume invoked but an openxr instance not yet set.");
            return;
        }
        if (m_pfnSetEngineVersionPico) {
            Log::Write(Log::Level::Info, "Setting pico engine version to 2.8.0.1");
            m_pfnSetEngineVersionPico(m_instance, "2.8.0.1");
        }
        if (m_pfnStartCVControllerThreadPico) {
            Log::Write(Log::Level::Info, "Starting pico cv controller thread");
            m_pfnStartCVControllerThreadPico(m_instance, PXR_HMD_6DOF, PXR_CONTROLLER_6DOF);
        }
#endif
    }

    virtual inline void Pause() override
    {
#ifdef XR_USE_OXR_PICO
        if (m_instance == XR_NULL_HANDLE) {
            Log::Write(Log::Level::Warning, "OpenXrProgram::Paused invoked but an openxr instance not yet set.");
            return;
        }
        if (m_pfnSetEngineVersionPico) {
            Log::Write(Log::Level::Info, "Setting pico engine version to 2.7.0.0");
            m_pfnSetEngineVersionPico(m_instance, "2.7.0.0");
        }
        if (m_pfnStopCVControllerThreadPico) {
            Log::Write(Log::Level::Info, "Stopping pico cv controller thread");
            m_pfnStopCVControllerThreadPico(m_instance, PXR_HMD_6DOF, PXR_CONTROLLER_6DOF);
        }
#endif
    }

    virtual inline std::shared_ptr<const IGraphicsPlugin> GetGraphicsPlugin() const override {
        return m_graphicsPlugin;
    }
    virtual inline std::shared_ptr<IGraphicsPlugin> GetGraphicsPlugin() override {
        return m_graphicsPlugin;
    }

   private:
    const std::shared_ptr<Options> m_options;
    std::shared_ptr<IPlatformPlugin> m_platformPlugin;
    std::shared_ptr<IGraphicsPlugin> m_graphicsPlugin;
    XrInstance m_instance{XR_NULL_HANDLE};
    XrSession m_session{XR_NULL_HANDLE};
    XrSpace m_appSpace{XR_NULL_HANDLE};
    XrSpace m_boundingStageSpace{ XR_NULL_HANDLE };
    XrSpace m_viewSpace{ XR_NULL_HANDLE };
    XrFormFactor m_formFactor{XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    XrViewConfigurationType m_viewConfigType{XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
    XrEnvironmentBlendMode m_environmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
    XrSystemId m_systemId{XR_NULL_SYSTEM_ID};

    std::vector<XrViewConfigurationView> m_configViews;
    std::vector<Swapchain> m_swapchains;
    std::map<XrSwapchain, std::vector<XrSwapchainImageBaseHeader*>> m_swapchainImages;
    std::vector<XrView> m_views;
    int64_t m_colorSwapchainFormat{-1};
    std::atomic<RenderMode> m_renderMode{ RenderMode::Lobby };

    std::vector<XrSpace> m_visualizedSpaces;

    // Application's current lifecycle state according to the runtime
    XrSessionState m_sessionState{XR_SESSION_STATE_UNKNOWN};
    std::atomic<bool> m_sessionRunning{false};
    OxrRuntimeType m_runtimeType { OxrRuntimeType::Unknown };

    XrEventDataBuffer m_eventDataBuffer;
    ALXRPaths  m_alxrPaths;
    InputState m_input;

    struct PassthroughLayerData
    {
        XrPassthroughFB passthrough = XR_NULL_HANDLE;
        XrPassthroughLayerFB reconPassthroughLayer = XR_NULL_HANDLE;
    };
    PassthroughLayerData m_ptLayerData {};

#ifdef XR_USE_PLATFORM_WIN32
    // XR_KHR_win32_convert_performance_counter_time
    PFN_xrConvertTimeToWin32PerformanceCounterKHR m_pfnConvertTimeToWin32PerformanceCounterKHR = nullptr;
    PFN_xrConvertWin32PerformanceCounterToTimeKHR m_pfnConvertWin32PerformanceCounterToTimeKHR = nullptr;
#endif
    // XR_KHR_convert_timespec_time
    PFN_xrConvertTimespecTimeToTimeKHR  m_pfnConvertTimespecTimeToTimeKHR = nullptr;
    PFN_xrConvertTimeToTimespecTimeKHR  m_pfnConvertTimeToTimespecTimeKHR = nullptr;
    
    // XR_FB_color_space
    PFN_xrEnumerateColorSpacesFB m_pfnEnumerateColorSpacesFB = nullptr;
    PFN_xrSetColorSpaceFB        m_pfnSetColorSpaceFB = nullptr;

    // XR_EXT_hand_tracking fun pointers.
    PFN_xrCreateHandTrackerEXT  m_pfnCreateHandTrackerEXT = nullptr;
    PFN_xrLocateHandJointsEXT   m_pfnLocateHandJointsEXT = nullptr;
    PFN_xrDestroyHandTrackerEXT m_pfnDestroyHandTrackerEXT = nullptr;

    // XR_FB_display_refresh_rate fun pointers.
    PFN_xrEnumerateDisplayRefreshRatesFB m_pfnEnumerateDisplayRefreshRatesFB = nullptr;
    PFN_xrGetDisplayRefreshRateFB m_pfnGetDisplayRefreshRateFB = nullptr;
    PFN_xrRequestDisplayRefreshRateFB m_pfnRequestDisplayRefreshRateFB = nullptr;

    // XR_FB_PASSTHROUGH_EXTENSION_NAME fun pointers.
    PFN_xrCreatePassthroughFB m_pfnCreatePassthroughFB = nullptr;
    PFN_xrDestroyPassthroughFB m_pfnDestroyPassthroughFB = nullptr;
    PFN_xrPassthroughStartFB m_pfnPassthroughStartFB = nullptr;
    PFN_xrPassthroughPauseFB m_pfnPassthroughPauseFB = nullptr;
    PFN_xrCreatePassthroughLayerFB m_pfnCreatePassthroughLayerFB = nullptr;
    PFN_xrDestroyPassthroughLayerFB m_pfnDestroyPassthroughLayerFB = nullptr;
    PFN_xrPassthroughLayerSetStyleFB m_pfnPassthroughLayerSetStyleFB = nullptr;
    PFN_xrPassthroughLayerPauseFB m_pfnPassthroughLayerPauseFB = nullptr;
    PFN_xrPassthroughLayerResumeFB m_pfnPassthroughLayerResumeFB = nullptr;

#ifdef XR_USE_OXR_PICO
    mutable std::atomic<int>    m_gsIndex{ 0 };
    PFN_xrResetSensorPICO       m_pfnResetSensorPICO = nullptr;
    PFN_xrGetConfigPICO         m_pfnGetConfigPICO = nullptr;
    PFN_xrSetConfigPICO         m_pfnSetConfigPICO = nullptr;
    PFN_xrGetControllerConnectionStatePico m_pfnGetControllerConnectionStatePico = nullptr;
    PFN_xrSetEngineVersionPico  m_pfnSetEngineVersionPico = nullptr;
    PFN_xrStartCVControllerThreadPico m_pfnStartCVControllerThreadPico = nullptr;
    PFN_xrStopCVControllerThreadPico  m_pfnStopCVControllerThreadPico = nullptr;
    PFN_xrVibrateControllerPico m_pfnXrVibrateControllerPico = nullptr;
#endif

    std::atomic<XrTime>      m_lastPredicatedDisplayTime{ 0 };

/// Tracking Thread State ////////////////////////////////////////////////////////
    struct TrackingFrame {
        std::array<XrView, 2> views;
        XrTime                displayTime;
    };
    using TrackingFrameMap = std::map<std::uint64_t, TrackingFrame>;
    mutable std::shared_mutex m_trackingFrameMapMutex;        
    TrackingFrameMap          m_trackingFrameMap{};
    std::atomic<XrDuration>   m_PredicatedLatencyOffset{ 0 };
    static constexpr const std::size_t MaxTrackingFrameCount = 360 * 3;
/// End Tracking Thread State ////////////////////////////////////////////////////

    std::vector<float> m_displayRefreshRates;
    ALXRStreamConfig m_streamConfig {
        .trackingSpaceType = ALXRTrackingSpace::LocalRefSpace,
        .renderConfig {
            .refreshRate = 90.0f,
            .enableFoveation = false
        }
    };

    using HapticsFeedbackQueue  = xrconcurrency::concurrent_queue<HapticsFeedback>;
    using StreamConfigQueue     = xrconcurrency::concurrent_queue<ALXRStreamConfig>;
    using GuardianChangedQueue  = xrconcurrency::concurrent_queue<ALXRGuardianData>;
    HapticsFeedbackQueue m_hapticsQueue;
    StreamConfigQueue    m_streamConfigQueue;
    GuardianChangedQueue m_guardianChangedQueue;
    bool                 m_delayOnGuardianChanged = false;
};
}  // namespace

std::shared_ptr<IOpenXrProgram> CreateOpenXrProgram(const std::shared_ptr<Options>& options,
                                                    const std::shared_ptr<IPlatformPlugin>& platformPlugin,
                                                    const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    return std::make_shared<OpenXrProgram>(options, platformPlugin, graphicsPlugin);
}

std::shared_ptr<IOpenXrProgram> CreateOpenXrProgram(const std::shared_ptr<Options>& options,
                                                    const std::shared_ptr<IPlatformPlugin>& platformPlugin) {
    return std::make_shared<OpenXrProgram>(options, platformPlugin);
}
