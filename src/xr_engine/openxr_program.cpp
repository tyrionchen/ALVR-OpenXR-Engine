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
#include <tuple>
#include <numeric>
#include <unordered_map>
#include <string_view>
#include <string>
#include <chrono>

#include "concurrent_queue.h"
#include "rust_bindings.h"
#include "ALVR-common/packet_types.h"

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

constexpr inline auto ToTrackingSpaceName(const TrackingSpace ts)
{
    if (ts == TrackingSpace::LocalRefSpace)
        return "Local";
    return "Stage";
}

/*constexpr*/ inline TrackingSpace ToTrackingSpace(const std::string_view& tsname)
{
    if (EqualsIgnoreCase(tsname, "Local"))
        return TrackingSpace::LocalRefSpace;
    return TrackingSpace::StageRefSpace;
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
    } else {
        throw std::invalid_argument(Fmt("Unknown reference space type '%s'", referenceSpaceTypeStr.data()));
    }
    return referenceSpaceCreateInfo;
}

inline XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const TrackingSpace ts) {
    return GetXrReferenceSpaceCreateInfo(ToTrackingSpaceName(ts));
}

inline std::uint64_t GetTimestampUs()
{
    using namespace std::chrono;
    using PeriodType = high_resolution_clock::period;
    using DurationType = high_resolution_clock::duration;
    using microsecondsU64 = std::chrono::duration<std::uint64_t, std::chrono::microseconds::period>;
    return duration_cast<microsecondsU64>(high_resolution_clock::now().time_since_epoch()).count();
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
    }
    return XR_HAND_JOINT_MAX_ENUM_EXT;
}

struct OpenXrProgram final : IOpenXrProgram {
    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin,
                  const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin)
        : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin(graphicsPlugin)
    {
        LogLayersAndExtensions();
    }

    OpenXrProgram(const std::shared_ptr<Options>& options, const std::shared_ptr<IPlatformPlugin>& platformPlugin)
    : m_options(options), m_platformPlugin(platformPlugin), m_graphicsPlugin { nullptr }
    {
        LogLayersAndExtensions();
        auto& graphicsApi = options->GraphicsPlugin;
        if (graphicsApi.empty() || graphicsApi == "auto")
        {
            Log::Write(Log::Level::Info, "Running auto graphics api selection.");
            constexpr const auto to_graphics_api_str = [](const GraphicsCtxApi gapi) -> std::tuple<std::string_view, std::string_view> {
                using namespace std::string_view_literals;
                switch (gapi)
                {
                case GraphicsCtxApi::Vulkan2: return std::make_tuple("XR_KHR_vulkan_enable2"sv, "Vulkan2"sv);
                case GraphicsCtxApi::Vulkan: return std::make_tuple("XR_KHR_vulkan_enable"sv, "Vulkan"sv);
                case GraphicsCtxApi::D3D12: return std::make_tuple("XR_KHR_D3D12_enable"sv, "D3D12"sv);
                case GraphicsCtxApi::D3D11: return std::make_tuple("XR_KHR_D3D11_enable"sv, "D3D11"sv);
                case GraphicsCtxApi::OpenGLES: return std::make_tuple("XR_KHR_opengl_es_enable"sv, "OpenGLES"sv);
                default: return std::make_tuple("XR_KHR_opengl_enable"sv, "OpenGL"sv);
                }
            };
            for (size_t apiIndex = GraphicsCtxApi::Vulkan2; apiIndex < size_t(GraphicsCtxApi::ApiCount); ++apiIndex) {
                const auto& [ext_name, gapi] = to_graphics_api_str(static_cast<GraphicsCtxApi>(apiIndex));
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
        if (m_input.actionSet != XR_NULL_HANDLE) {
            for (auto hand : {Side::LEFT, Side::RIGHT}) {
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

        if (m_appSpace != XR_NULL_HANDLE) {
            xrDestroySpace(m_appSpace);
        }

        if (m_session != XR_NULL_HANDLE) {
            xrDestroySession(m_session);
        }

        if (m_instance != XR_NULL_HANDLE) {
            xrDestroyInstance(m_instance);
        }
    }

    using ExtensionMap = std::unordered_map<std::string_view, bool>;
    ExtensionMap m_availableSupportedExtMap = {
       { "XR_EXT_hand_tracking", false },
       { "XR_FB_display_refresh_rate", false }
    };
    ExtensionMap m_supportedGraphicsContexts = {
        { "XR_KHR_vulkan_enable2",   false },
        { "XR_KHR_vulkan_enable",    false },
        { "XR_KHR_D3D12_enable",     false },
        { "XR_KHR_D3D11_enable",     false },
        { "XR_KHR_opengl_enable",    false },
        { "XR_KHR_opengl_es_enable", false },
    };

    void LogLayersAndExtensions() {
        // Write out extension properties for a given layer.
        const auto logExtensions = [this](const char* layerName, int indent = 0) {
            uint32_t instanceExtensionCount;
            CHECK_XRCMD(xrEnumerateInstanceExtensionProperties(layerName, 0, &instanceExtensionCount, nullptr));

            std::vector<XrExtensionProperties> extensions(instanceExtensionCount);
            for (XrExtensionProperties& extension : extensions) {
                extension.type = XR_TYPE_EXTENSION_PROPERTIES;
            }

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

            std::vector<XrApiLayerProperties> layers(layerCount);
            for (XrApiLayerProperties& layer : layers) {
                layer.type = XR_TYPE_API_LAYER_PROPERTIES;
            }

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
        CHECK(m_instance != XR_NULL_HANDLE);

        XrInstanceProperties instanceProperties{XR_TYPE_INSTANCE_PROPERTIES};
        CHECK_XRCMD(xrGetInstanceProperties(m_instance, &instanceProperties));

        Log::Write(Log::Level::Info, Fmt("Instance RuntimeName=%s RuntimeVersion=%s", instanceProperties.runtimeName,
                                         GetXrVersionString(instanceProperties.runtimeVersion).c_str()));
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
                       
        for (const auto& [extName,extAvaileble] : m_availableSupportedExtMap) {
            if (extAvaileble) {
                extensions.push_back(extName.data());
            }
        }
        
        XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
        createInfo.next = m_platformPlugin->GetInstanceCreateExtension();
        createInfo.enabledExtensionCount = (uint32_t)extensions.size();
        createInfo.enabledExtensionNames = extensions.data();

        strcpy(createInfo.applicationInfo.applicationName, "openxr_client");
        createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

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

            XrViewConfigurationProperties viewConfigProperties{XR_TYPE_VIEW_CONFIGURATION_PROPERTIES};
            CHECK_XRCMD(xrGetViewConfigurationProperties(m_instance, m_systemId, viewConfigType, &viewConfigProperties));

            Log::Write(Log::Level::Verbose,
                       Fmt("  View configuration FovMutable=%s", viewConfigProperties.fovMutable == XR_TRUE ? "True" : "False"));

            uint32_t viewCount;
            CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, 0, &viewCount, nullptr));
            if (viewCount > 0) {
                std::vector<XrViewConfigurationView> views(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
                CHECK_XRCMD(
                    xrEnumerateViewConfigurationViews(m_instance, m_systemId, viewConfigType, viewCount, &viewCount, views.data()));

                for (uint32_t i = 0; i < views.size(); i++) {
                    const XrViewConfigurationView& view = views[i];

                    Log::Write(Log::Level::Verbose, Fmt("    View [%d]: Recommended Width=%d Height=%d SampleCount=%d", i,
                                                        view.recommendedImageRectWidth, view.recommendedImageRectHeight,
                                                        view.recommendedSwapchainSampleCount));
                    Log::Write(Log::Level::Verbose,
                               Fmt("    View [%d]:     Maximum Width=%d Height=%d SampleCount=%d", i, view.maxImageRectWidth,
                                   view.maxImageRectHeight, view.maxSwapchainSampleCount));
                }
            } else {
                Log::Write(Log::Level::Error, Fmt("Empty view configuration type"));
            }

            LogEnvironmentBlendMode(viewConfigType);
        }
    }

    void LogEnvironmentBlendMode(XrViewConfigurationType type) {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != 0);

        uint32_t count;
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, 0, &count, nullptr));
        CHECK(count > 0);

        Log::Write(Log::Level::Info, Fmt("Available Environment Blend Mode count : (%d)", count));

        std::vector<XrEnvironmentBlendMode> blendModes(count);
        CHECK_XRCMD(xrEnumerateEnvironmentBlendModes(m_instance, m_systemId, type, count, &count, blendModes.data()));

        bool blendModeFound = false;
        for (XrEnvironmentBlendMode mode : blendModes) {
            const bool blendModeMatch = (mode == m_environmentBlendMode);
            Log::Write(Log::Level::Info,
                       Fmt("Environment Blend Mode (%s) : %s", to_string(mode), blendModeMatch ? "(Selected)" : ""));
            blendModeFound |= blendModeMatch;
        }
        CHECK(blendModeFound);
    }

    void InitializeSystem() override {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId == XR_NULL_SYSTEM_ID);

        m_formFactor = GetXrFormFactor(m_options->FormFactor);
        m_viewConfigType = GetXrViewConfigurationType(m_options->ViewConfiguration);
        m_environmentBlendMode = GetXrEnvironmentBlendMode(m_options->EnvironmentBlendMode);

        XrSystemGetInfo systemInfo{XR_TYPE_SYSTEM_GET_INFO};
        systemInfo.formFactor = m_formFactor;
        CHECK_XRCMD(xrGetSystem(m_instance, &systemInfo, &m_systemId));

        Log::Write(Log::Level::Verbose, Fmt("Using system %d for form factor %s", m_systemId, to_string(m_formFactor)));
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_systemId != XR_NULL_SYSTEM_ID);

        LogViewConfigurations();

        // The graphics API can initialize the graphics device now that the systemId and instance
        // handle are available.
        m_graphicsPlugin->InitializeDevice(m_instance, m_systemId);
    }

    void LogReferenceSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);

        uint32_t spaceCount;
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, 0, &spaceCount, nullptr));
        std::vector<XrReferenceSpaceType> spaces(spaceCount);
        CHECK_XRCMD(xrEnumerateReferenceSpaces(m_session, spaceCount, &spaceCount, spaces.data()));

        Log::Write(Log::Level::Info, Fmt("Available reference spaces: %d", spaceCount));
        for (XrReferenceSpaceType space : spaces) {
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

        ALVRActionMap boolActionMap =
        {
            { ALVR_INPUT_SYSTEM_CLICK, { "system_click", "System Click" }},
            { ALVR_INPUT_APPLICATION_MENU_CLICK, { "appliction_click", "Appliction Click" }},
            { ALVR_INPUT_GRIP_CLICK, { "grip_click", "Grip Click" }},
            { ALVR_INPUT_GRIP_TOUCH, { "grip_touch", "Grip Touch" }},
            { ALVR_INPUT_GRIP_VALUE, { "grip_value_to_click", "Grip Value To Click" }},
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
            //{ ALVR_INPUT_BACK_CLICK, { "back_click", "Back Click" }},
            //{ ALVR_INPUT_GUIDE_CLICK, { "guide_click", "Guide Click" }},
            //{ ALVR_INPUT_START_CLICK, { "start_click", "Start Click" }},
            { ALVR_INPUT_TRIGGER_CLICK, { "trigger_click", "Trigger Click" }},
            { ALVR_INPUT_TRIGGER_TOUCH, { "trigger_touch", "Trigger Touch" }},
            { ALVR_INPUT_TRACKPAD_CLICK, { "trackpad_click", "Trackpad Click" }},
            { ALVR_INPUT_TRACKPAD_TOUCH, { "trackpad_touch", "Trackpad Touch" }},
        };
        ALVRActionMap scalarActionMap =
        {
            { ALVR_INPUT_GRIP_VALUE, { "grip_value", "Grip Value" }},
            { ALVR_INPUT_JOYSTICK_X, { "joystick_x", "Joystick X" }},
            { ALVR_INPUT_JOYSTICK_Y, { "joystick_y", "Joystick Y" }},
            { ALVR_INPUT_TRIGGER_VALUE, { "trigger_value", "Trigger Value" }},
            { ALVR_INPUT_TRACKPAD_X, { "trackpad_x", "Trackpad X" }},
            { ALVR_INPUT_TRACKPAD_Y, { "trackpad_y", "Trackpad Y" }},
        };
        //ALVRActionMap scalarToBoolActionMap =
        //{
        //    { ALVR_INPUT_GRIP_VALUE, { "grip_value_to_click", "Grip Value To Click" }}
        //};
        ALVRActionMap boolToScalarActionMap =
        {
            { ALVR_INPUT_GRIP_VALUE, { "grip_click_to_value", "Grip Click To Value" }}
        };
    };

    void InitializeActions() {
        // Create an action set.
        {
            XrActionSetCreateInfo actionSetInfo{XR_TYPE_ACTION_SET_CREATE_INFO};
            strcpy_s(actionSetInfo.actionSetName, "gameplay");
            strcpy_s(actionSetInfo.localizedActionSetName, "Gameplay");
            actionSetInfo.priority = 0;
            CHECK_XRCMD(xrCreateActionSet(m_instance, &actionSetInfo, &m_input.actionSet));
        }

        // Get the XrPath for the left and right hands - we will use them as subaction paths.
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left", &m_input.handSubactionPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right", &m_input.handSubactionPath[Side::RIGHT]));

        // Create actions.
        {
            //// Create an input action for grabbing objects with the left and right hands.
            XrActionCreateInfo actionInfo{XR_TYPE_ACTION_CREATE_INFO};
            //actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
            //strcpy_s(actionInfo.actionName, "grab_object");
            //strcpy_s(actionInfo.localizedActionName, "Grab Object");
            //actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
            //actionInfo.subactionPaths = m_input.handSubactionPath.data();
            //CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.grabAction));

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
            //actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
            //strcpy_s(actionInfo.actionName, "quit_session");
            //strcpy_s(actionInfo.localizedActionName, "Quit Session");
            //actionInfo.countSubactionPaths = 0;
            //actionInfo.subactionPaths = nullptr;
            //CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.quitAction));

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
            CreateActions(XR_ACTION_TYPE_BOOLEAN_INPUT, m_input.boolToScalarActionMap);
            CreateActions(XR_ACTION_TYPE_FLOAT_INPUT, m_input.scalarActionMap);
            //CreateActions(XR_ACTION_TYPE_FLOAT_INPUT, m_input.scalarToBoolActionMap);
        }

        std::array<XrPath, Side::COUNT> selectPath;
        std::array<XrPath, Side::COUNT> squeezeClickPath, squeezeValuePath, squeezeForcePath;
        std::array<XrPath, Side::COUNT> posePath;
        std::array<XrPath, Side::COUNT> hapticPath;
        std::array<XrPath, Side::COUNT> menuClickPath, systemClickPath;
        std::array<XrPath, Side::COUNT> triggerClickPath, triggerTouchPath, triggerValuePath;
        std::array<XrPath, Side::COUNT> thumbstickXPath, thumbstickYPath,
                                        thumbstickClickPath, thumbstickTouchPath,
                                        thumbrestTouchPath;
        std::array<XrPath, Side::COUNT> trackpadXPath, trackpadYPath, trackpadForcePath,
                                        trackpadClickPath, trackpadTouchPath;

        std::array<XrPath, Side::COUNT> aClickPath, aTouchPath, bClickPath, bTouchPath,
                                        xClickPath, xTouchPath, yClickPath, yTouchPath;
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/select/click", &selectPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/select/click", &selectPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/force", &squeezeForcePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/force", &squeezeForcePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/click", &squeezeClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/click", &squeezeClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/grip/pose", &posePath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/grip/pose", &posePath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/output/haptic", &hapticPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/output/haptic", &hapticPath[Side::RIGHT]));

        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/system/click", &systemClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/system/click", &systemClickPath[Side::RIGHT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/menu/click", &menuClickPath[Side::LEFT]));
        CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/menu/click", &menuClickPath[Side::RIGHT]));
        
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

        // Suggest bindings for KHR Simple.
        {
            XrPath khrSimpleInteractionProfilePath;
            CHECK_XRCMD(
                xrStringToPath(m_instance, "/interaction_profiles/khr/simple_controller", &khrSimpleInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {// Fall back to a click input for the grab action.
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, selectPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_GRIP_CLICK].xrAction, selectPath[Side::RIGHT]},
                {m_input.poseAction, posePath[Side::LEFT]},
                {m_input.poseAction, posePath[Side::RIGHT]},
                //ALVR servers currently does not use APP_MENU_CLICK event.
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::RIGHT]},

                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggestedBindings.interactionProfile = khrSimpleInteractionProfilePath;
            suggestedBindings.suggestedBindings = bindings.data();
            suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
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
                //{m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                //{m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::RIGHT]},
                
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
                // atm ALVL has no input type for thumbrest touch

                {m_input.poseAction, posePath[Side::LEFT]},
                {m_input.poseAction, posePath[Side::RIGHT]},
                //{m_input.quitAction, menuClickPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggestedBindings.interactionProfile = oculusTouchInteractionProfilePath;
            suggestedBindings.suggestedBindings = bindings.data();
            suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
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
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_CLICK].xrAction, trackpadClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_TRACKPAD_TOUCH].xrAction, trackpadTouchPath[Side::RIGHT]},

                {m_input.poseAction, posePath[Side::LEFT]},
                {m_input.poseAction, posePath[Side::RIGHT]},
                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggestedBindings.interactionProfile = viveControllerInteractionProfilePath;
            suggestedBindings.suggestedBindings = bindings.data();
            suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
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
                //{m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::LEFT]},
                //{m_input.scalarToBoolActionMap[ALVR_INPUT_GRIP_VALUE].xrAction, squeezeValuePath[Side::RIGHT]},
                
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

                {m_input.poseAction, posePath[Side::LEFT]},
                {m_input.poseAction, posePath[Side::RIGHT]},
                //{m_input.quitAction, bClickPath[Side::LEFT]},
                //{m_input.quitAction, bClickPath[Side::RIGHT]},
                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggestedBindings.interactionProfile = indexControllerInteractionProfilePath;
            suggestedBindings.suggestedBindings = bindings.data();
            suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }

        // Suggest bindings for the Microsoft Mixed Reality Motion Controller.
        {
            XrPath microsoftMixedRealityInteractionProfilePath;
            CHECK_XRCMD(xrStringToPath(m_instance, "/interaction_profiles/microsoft/motion_controller",
                                       &microsoftMixedRealityInteractionProfilePath));
            const std::vector<XrActionSuggestedBinding> bindings{ {
                //ALVR servers currently does not use APP_MENU_CLICK event.
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::LEFT]},
                //{m_input.boolActionMap[ALVR_INPUT_APPLICATION_MENU_CLICK].xrAction, menuClickPath[Side::RIGHT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::LEFT]},
                {m_input.boolActionMap[ALVR_INPUT_SYSTEM_CLICK].xrAction, menuClickPath[Side::RIGHT]},

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

                {m_input.poseAction, posePath[Side::LEFT]},
                {m_input.poseAction, posePath[Side::RIGHT]},
                //{m_input.quitAction, menuClickPath[Side::LEFT]},
                //{m_input.quitAction, menuClickPath[Side::RIGHT]},
                {m_input.vibrateAction, hapticPath[Side::LEFT]},
                {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
            XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            suggestedBindings.interactionProfile = microsoftMixedRealityInteractionProfilePath;
            suggestedBindings.suggestedBindings = bindings.data();
            suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
            CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
        }
        XrActionSpaceCreateInfo actionSpaceInfo{XR_TYPE_ACTION_SPACE_CREATE_INFO};
        actionSpaceInfo.action = m_input.poseAction;
        actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
        actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::LEFT];
        CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::LEFT]));
        actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::RIGHT];
        CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::RIGHT]));

        XrSessionActionSetsAttachInfo attachInfo{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
        attachInfo.countActionSets = 1;
        attachInfo.actionSets = &m_input.actionSet;
        CHECK_XRCMD(xrAttachSessionActionSets(m_session, &attachInfo));
    }

    inline bool IsExtEnabled(const std::string_view& extName) const
    {
        auto ext_itr = m_availableSupportedExtMap.find(extName);
        return ext_itr != m_availableSupportedExtMap.end() && ext_itr->second;
    }

    bool InitializeExtensions()
    {
        if (IsExtEnabled("XR_FB_display_refresh_rate"))
        {
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrEnumerateDisplayRefreshRatesFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnEnumerateDisplayRefreshRatesFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrGetDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnGetDisplayRefreshRateFB)));
            CHECK_XRCMD(xrGetInstanceProcAddr(m_instance, "xrRequestDisplayRefreshRateFB",
                reinterpret_cast<PFN_xrVoidFunction*>(&m_pfnRequestDisplayRefreshRateFB)));
        }
        UpdateSupportedDisplayRefreshRates();
        return InitializeHandTrackers();
    }

    bool InitializeHandTrackers()
    {
        //if (m_instance != XR_NULL_HANDLE && m_systemId != XR_NULL_SYSTEM_ID)
        //    return false;

        // Inspect hand tracking system properties
        XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties{ XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT };
        XrSystemProperties systemProperties{ XR_TYPE_SYSTEM_PROPERTIES, &handTrackingSystemProperties };
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

        if (m_pfnCreateHandTrackerEXT == nullptr ||
            m_pfnLocateHandJointsEXT == nullptr)
            return false;

        // Create a hand tracker for left hand that tracks default set of hand joints.
        const auto createHandTracker = [&](auto& handerTracker, const XrHandEXT hand)
        {
            XrHandTrackerCreateInfoEXT createInfo{ XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT };
            createInfo.hand = hand;
            createInfo.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
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

    void CreateVisualizedSpaces() {
        CHECK(m_session != XR_NULL_HANDLE);

        std::string visualizedSpaces[] = {"ViewFront",        "Local", "Stage", "StageLeft", "StageRight", "StageLeftRotated",
                                          "StageRightRotated"};

        for (const auto& visualizedSpace : visualizedSpaces) {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(visualizedSpace);
            XrSpace space;
            XrResult res = xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &space);
            if (XR_SUCCEEDED(res)) {
                m_visualizedSpaces.push_back(space);
                Log::Write(Log::Level::Info, Fmt("visualized-space %s added", visualizedSpace.c_str()));
            } else {
                Log::Write(Log::Level::Warning,
                           Fmt("Failed to create reference space %s with error %d", visualizedSpace.c_str(), res));
            }
        }
    }

    void InitializeSession() override {
        CHECK(m_instance != XR_NULL_HANDLE);
        CHECK(m_session == XR_NULL_HANDLE);

        {
            Log::Write(Log::Level::Verbose, Fmt("Creating session..."));

            XrSessionCreateInfo createInfo{XR_TYPE_SESSION_CREATE_INFO};
            createInfo.next = m_graphicsPlugin->GetGraphicsBinding();
            createInfo.systemId = m_systemId;
            CHECK_XRCMD(xrCreateSession(m_instance, &createInfo, &m_session));
        }

        LogReferenceSpaces();
        InitializeActions();
        CreateVisualizedSpaces();

        {
            XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(m_options->AppSpace);
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));
            m_streamConfig.trackingSpaceType = ToTrackingSpace(m_options->AppSpace);

            referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo("View");
            CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_viewSpace));
        }

        InitializeExtensions();
    }

    void CreateSwapchains() override {
        CHECK(m_session != XR_NULL_HANDLE);
        CHECK(m_swapchains.empty());
        CHECK(m_configViews.empty());

        // Read graphics properties for preferred swapchain length and logging.
        XrSystemProperties systemProperties{XR_TYPE_SYSTEM_PROPERTIES};
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
        uint32_t viewCount;
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, 0, &viewCount, nullptr));
        m_configViews.resize(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, viewCount, &viewCount,
                                                      m_configViews.data()));

        // Create and cache view buffer for xrLocateViews later.
        m_views.resize(viewCount, {XR_TYPE_VIEW});

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
                XrSwapchainCreateInfo swapchainCreateInfo{XR_TYPE_SWAPCHAIN_CREATE_INFO};
                swapchainCreateInfo.arraySize = 1;
                swapchainCreateInfo.format = m_colorSwapchainFormat;
                swapchainCreateInfo.width = vp.recommendedImageRectWidth;
                swapchainCreateInfo.height = vp.recommendedImageRectHeight;
                swapchainCreateInfo.mipCount = 1;
                swapchainCreateInfo.faceCount = 1;
                swapchainCreateInfo.sampleCount = m_graphicsPlugin->GetSupportedSwapchainSampleCount(vp);
                swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
                Swapchain swapchain;
                swapchain.width = swapchainCreateInfo.width;
                swapchain.height = swapchainCreateInfo.height;
                CHECK_XRCMD(xrCreateSwapchain(m_session, &swapchainCreateInfo, &swapchain.handle));

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
        *baseHeader = {XR_TYPE_EVENT_DATA_BUFFER};
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
                    m_streamConfig.refreshRate = refreshRateChangedEvent.toDisplayRefreshRate;
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
                    auto sessionStateChangedEvent = *reinterpret_cast<const XrEventDataSessionStateChanged*>(event);
                    HandleSessionStateChangedEvent(sessionStateChangedEvent, exitRenderLoop, requestRestart);
                    break;
                }
                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                    //LogActionSourceName(m_input.grabAction, "Grab");
                    //LogActionSourceName(m_input.quitAction, "Quit");
                    LogActionSourceName(m_input.poseAction, "Pose");
                    LogActionSourceName(m_input.vibrateAction, "Vibrate");
                    for (const auto& [k,v] : m_input.boolActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    for (const auto& [k, v] : m_input.boolToScalarActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    for (const auto& [k, v] : m_input.scalarActionMap)
                        LogActionSourceName(v.xrAction, v.localizedName.data());
                    break;
                case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
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
            case XR_SESSION_STATE_READY: {
                CHECK(m_session != XR_NULL_HANDLE);
                XrSessionBeginInfo sessionBeginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                sessionBeginInfo.primaryViewConfigurationType = m_viewConfigType;
                CHECK_XRCMD(xrBeginSession(m_session, &sessionBeginInfo));
                m_sessionRunning = true;
                break;
            }
            case XR_SESSION_STATE_STOPPING: {
                CHECK(m_session != XR_NULL_HANDLE);
                m_sessionRunning = false;
                CHECK_XRCMD(xrEndSession(m_session))
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

            XrInputSourceLocalizedNameGetInfo nameInfo = {XR_TYPE_INPUT_SOURCE_LOCALIZED_NAME_GET_INFO};
            nameInfo.sourcePath = paths[i];
            nameInfo.whichComponents = all;

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

    void PollHandTrackers()
    {
        if (m_pfnLocateHandJointsEXT == nullptr || m_lastDisplayTime == 0)
            return;

        std::array<XrMatrix4x4f, XR_HAND_JOINT_COUNT_EXT> oculusOrientedJointPoses;
        for (const auto hand : { Side::LEFT,Side::RIGHT })
        {
            auto& handerTracker = m_input.handerTrackers[hand];
            //XrHandJointVelocitiesEXT velocities{ XR_TYPE_HAND_JOINT_VELOCITIES_EXT };
            //velocities.jointCount = XR_HAND_JOINT_COUNT_EXT;
            //velocities.jointVelocities = handerTracker.jointVelocities.data();

            XrHandJointLocationsEXT locations{ XR_TYPE_HAND_JOINT_LOCATIONS_EXT };
            //locations.next = &velocities;
            locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
            locations.jointLocations = handerTracker.jointLocations.data();

            XrHandJointsLocateInfoEXT locateInfo{ XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT };
            locateInfo.baseSpace = m_appSpace;
            locateInfo.time = m_lastDisplayTime;
            CHECK_XRCMD(m_pfnLocateHandJointsEXT(handerTracker.tracker, &locateInfo, &locations));
            if (!locations.isActive)
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

            auto& controller = m_input.controllerInfo[hand];
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

            controller.flags |= TrackingInfo::Controller::FLAG_CONTROLLER_OCULUS_HAND;
            if (hand == Side::LEFT)
                controller.flags |= TrackingInfo::Controller::FLAG_CONTROLLER_LEFTHAND;
            controller.inputStateStatus = 0;

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
        m_input.handActive = { XR_FALSE, XR_FALSE };
        m_input.controllerInfo = { TrackingInfo::Controller {}, TrackingInfo::Controller {}, };

        // Sync actions
        const XrActiveActionSet activeActionSet{m_input.actionSet, XR_NULL_PATH};
        XrActionsSyncInfo syncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
        syncInfo.countActiveActionSets = 1;
        syncInfo.activeActionSets = &activeActionSet;
        CHECK_XRCMD(xrSyncActions(m_session, &syncInfo));

        // Get pose and grab action state and start haptic vibrate when hand is 90% squeezed.
        for (auto hand : {Side::LEFT, Side::RIGHT}) {
            XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO};
            getInfo.subactionPath = m_input.handSubactionPath[hand];
            getInfo.action = m_input.poseAction;
            XrActionStatePose poseState{XR_TYPE_ACTION_STATE_POSE};
            CHECK_XRCMD(xrGetActionStatePose(m_session, &getInfo, &poseState));
            m_input.handActive[hand] = poseState.isActive;

            constexpr const std::uint32_t EnableRightControllerMask = //TrackingInfo::Controller::FLAG_CONTROLLER_ENABLE |
                                                                      TrackingInfo::Controller::FLAG_CONTROLLER_OCULUS_QUEST;
            constexpr const std::uint32_t EnableLeftControllerMask  = EnableRightControllerMask |
                                                                      TrackingInfo::Controller::FLAG_CONTROLLER_LEFTHAND;
            
            const std::uint32_t enableControllerMask = hand == Side::LEFT ? EnableLeftControllerMask : EnableRightControllerMask;

            auto& controllerInfo = m_input.controllerInfo[hand];
            // xrGetActionStatePose doesn't appear to be a reliable method to determine if a controller is active,
            // at least with WMR's current OpenXR runtime.
            // if (m_input.handActive[hand] == XR_TRUE) 
                controllerInfo.flags |= enableControllerMask;
            controllerInfo.batteryPercentRemaining = 100; // OpenXR has no method of obtaining controller/HMD battery life.

            for (const auto& [buttonType, v] : m_input.boolActionMap)
            {
                getInfo.action = v.xrAction;
                XrActionStateBoolean boolValue{XR_TYPE_ACTION_STATE_BOOLEAN};
                CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &boolValue));
                if ((boolValue.isActive == XR_TRUE) /*&& (quitValue.changedSinceLastSync == XR_TRUE)*/ && (boolValue.currentState == XR_TRUE))
                    controllerInfo.buttons |= ALVR_BUTTON_FLAG(buttonType);
            }

            const auto GetFloatValue = [&](const InputState::ALVRAction& v, float& val)
            {
                getInfo.action = v.xrAction;
                XrActionStateFloat floatValue{XR_TYPE_ACTION_STATE_FLOAT};
                CHECK_XRCMD(xrGetActionStateFloat(m_session, &getInfo, &floatValue));
                if (floatValue.isActive == XR_FALSE)
                    return;
                val = floatValue.currentState;
            };
            /*const*/ auto& scalarActionMap = m_input.scalarActionMap;
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRACKPAD_X], controllerInfo.trackpadPosition.x);
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRACKPAD_Y], controllerInfo.trackpadPosition.y);
            GetFloatValue(scalarActionMap[ALVR_INPUT_JOYSTICK_X], controllerInfo.trackpadPosition.x);
            GetFloatValue(scalarActionMap[ALVR_INPUT_JOYSTICK_Y], controllerInfo.trackpadPosition.y);
            GetFloatValue(scalarActionMap[ALVR_INPUT_TRIGGER_VALUE], controllerInfo.triggerValue);
            GetFloatValue(scalarActionMap[ALVR_INPUT_GRIP_VALUE], controllerInfo.gripValue);

            const auto GetFloatFromBool = [&](const InputState::ALVRAction& v, float& val)
            {
                getInfo.action = v.xrAction;
                XrActionStateBoolean boolValue{ XR_TYPE_ACTION_STATE_BOOLEAN };
                CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &boolValue));
                if ((boolValue.isActive == XR_TRUE) /*&& (quitValue.changedSinceLastSync == XR_TRUE)*/ && (boolValue.currentState == XR_TRUE))
                    val = 1.0f;
            };
            auto& boolToScalarActionMap = m_input.boolToScalarActionMap;
            GetFloatFromBool(boolToScalarActionMap[ALVR_INPUT_GRIP_VALUE], controllerInfo.gripValue);
        }

        PollHandTrackers();

        // haptic feedback
        constexpr static const size_t MaxPopPerFrame = 20;
        size_t popCount = 0;
        HapticsFeedback hapticFeedback;
        while (m_hapticsQueue.try_pop(hapticFeedback) && popCount < MaxPopPerFrame)
        {
            const size_t hand = hapticFeedback.hand == 0 ? 1 : 0;
            if ((m_input.controllerInfo[hand].flags & TrackingInfo::Controller::FLAG_CONTROLLER_OCULUS_HAND) == 0)
            {
                //Log::Write(Log::Level::Info, Fmt("Haptics: amp:%f duration:%f freq:%f", hapticFeedback.amplitude, hapticFeedback.duration, hapticFeedback.frequency));
                XrHapticVibration vibration{ XR_TYPE_HAPTIC_VIBRATION };
                vibration.amplitude = hapticFeedback.amplitude;
                vibration.duration = static_cast<XrDuration>(static_cast<double>(hapticFeedback.duration) * 1e+9);
                vibration.frequency = hapticFeedback.frequency;

                XrHapticActionInfo hapticActionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
                hapticActionInfo.action = m_input.vibrateAction;
                hapticActionInfo.subactionPath = m_input.handSubactionPath[hand];
                CHECK_XRCMD(xrApplyHapticFeedback(m_session, &hapticActionInfo, reinterpret_cast<XrHapticBaseHeader*>(&vibration)));
            }
            ++popCount;
        }

        //// There were no subaction paths specified for the quit action, because we don't care which hand did it.
        //XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, m_input.quitAction, XR_NULL_PATH};
        //XrActionStateBoolean quitValue{XR_TYPE_ACTION_STATE_BOOLEAN};
        //CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &quitValue));
        //if ((quitValue.isActive == XR_TRUE) && (quitValue.changedSinceLastSync == XR_TRUE) && (quitValue.currentState == XR_TRUE)) {
        //    CHECK_XRCMD(xrRequestExitSession(m_session));
        //}
    }

    void RenderFrame() override {
        CHECK(m_session != XR_NULL_HANDLE);

        XrFrameWaitInfo frameWaitInfo{XR_TYPE_FRAME_WAIT_INFO};
        XrFrameState frameState{XR_TYPE_FRAME_STATE};
        CHECK_XRCMD(xrWaitFrame(m_session, &frameWaitInfo, &frameState));
        m_lastDisplayTime = frameState.predictedDisplayTime;

        XrFrameBeginInfo frameBeginInfo{XR_TYPE_FRAME_BEGIN_INFO};
        CHECK_XRCMD(xrBeginFrame(m_session, &frameBeginInfo));

        std::vector<XrCompositionLayerBaseHeader*> layers;
        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        std::vector<XrCompositionLayerProjectionView> projectionLayerViews;
        if (frameState.shouldRender == XR_TRUE) {
            if (RenderLayer(frameState.predictedDisplayTime, projectionLayerViews, layer)) {
                layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
            }
        }

        XrFrameEndInfo frameEndInfo{XR_TYPE_FRAME_END_INFO};
        frameEndInfo.displayTime = frameState.predictedDisplayTime;
        frameEndInfo.environmentBlendMode = m_environmentBlendMode;
        frameEndInfo.layerCount = (uint32_t)layers.size();
        frameEndInfo.layers = layers.data();
        CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

        ++m_frameIndex;
    }

    bool RenderLayer(XrTime predictedDisplayTime, std::vector<XrCompositionLayerProjectionView>& projectionLayerViews,
                     XrCompositionLayerProjection& layer) {
        XrResult res;

        XrViewState viewState{XR_TYPE_VIEW_STATE};
        uint32_t viewCapacityInput = (uint32_t)m_views.size();
        uint32_t viewCountOutput;

        XrViewLocateInfo viewLocateInfo{XR_TYPE_VIEW_LOCATE_INFO};
        viewLocateInfo.viewConfigurationType = m_viewConfigType;
        viewLocateInfo.displayTime = predictedDisplayTime;
        viewLocateInfo.space = m_appSpace;

        res = xrLocateViews(m_session, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, m_views.data());
        CHECK_XRRESULT(res, "xrLocateViews");
        if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
            (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
            return false;  // There is no valid tracking poses for the views.
        }

        CHECK(viewCountOutput == viewCapacityInput);
        CHECK(viewCountOutput == m_configViews.size());
        CHECK(viewCountOutput == m_swapchains.size());

        projectionLayerViews.resize(viewCountOutput);

        // For each locatable space that we want to visualize, render a 25cm cube.
        std::vector<Cube> cubes;

        for (XrSpace visualizedSpace : m_visualizedSpaces) {
            XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
            res = xrLocateSpace(visualizedSpace, m_appSpace, predictedDisplayTime, &spaceLocation);
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

        // Render a 10cm cube scaled by grabAction for each hand. Note renderHand will only be
        // true when the application has focus.
        for (auto hand : {Side::LEFT, Side::RIGHT}) {
            XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
            res = xrLocateSpace(m_input.handSpace[hand], m_appSpace, predictedDisplayTime, &spaceLocation);
            CHECK_XRRESULT(res, "xrLocateSpace");
            if (XR_UNQUALIFIED_SUCCESS(res)) {
                if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                    (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                    float scale = 0.1f * m_input.handScale[hand];
                    cubes.push_back(Cube{spaceLocation.pose, {scale, scale, scale}});
                }
            } else {
                // Tracking loss is expected when the hand is not active so only log a message
                // if the hand is active.
                if (m_input.handActive[hand] == XR_TRUE) {
                    const char* handName[] = {"left", "right"};
                    Log::Write(Log::Level::Verbose,
                               Fmt("Unable to locate %s hand action space in app space: %d", handName[hand], res));
                }
            }
        }

        // Render view to the appropriate part of the swapchain image.
        for (uint32_t i = 0; i < viewCountOutput; i++) {
            // Each view has a separate swapchain which is acquired, rendered to, and released.
            const Swapchain viewSwapchain = m_swapchains[i];

            XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};

            uint32_t swapchainImageIndex;
            CHECK_XRCMD(xrAcquireSwapchainImage(viewSwapchain.handle, &acquireInfo, &swapchainImageIndex));

            XrSwapchainImageWaitInfo waitInfo{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            waitInfo.timeout = XR_INFINITE_DURATION;
            CHECK_XRCMD(xrWaitSwapchainImage(viewSwapchain.handle, &waitInfo));

            projectionLayerViews[i] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
            projectionLayerViews[i].pose = m_views[i].pose;
            projectionLayerViews[i].fov = m_views[i].fov;
            projectionLayerViews[i].subImage.swapchain = viewSwapchain.handle;
            projectionLayerViews[i].subImage.imageRect.offset = {0, 0};
            projectionLayerViews[i].subImage.imageRect.extent = {viewSwapchain.width, viewSwapchain.height};

            const XrSwapchainImageBaseHeader* const swapchainImage = m_swapchainImages[viewSwapchain.handle][swapchainImageIndex];
            m_graphicsPlugin->RenderView(projectionLayerViews[i], swapchainImage, m_colorSwapchainFormat, cubes);

            XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            CHECK_XRCMD(xrReleaseSwapchainImage(viewSwapchain.handle, &releaseInfo));
        }

        layer.space = m_appSpace;
        layer.viewCount = (uint32_t)projectionLayerViews.size();
        layer.views = projectionLayerViews.data();
        return true;
    }

    float EstimateDisplayRefreshRate()
    {
#if 0
        if (m_session == XR_NULL_HANDLE)
            return 60.0f;

        using PeriodType = std::chrono::high_resolution_clock::period;
        using DurationType = std::chrono::high_resolution_clock::duration;
        using secondsf = std::chrono::duration<float, std::chrono::seconds::period>;

        using namespace std::literals::chrono_literals;
        constexpr const auto OneSecond = 1s;

        constexpr const size_t SamplesPerSec = 30;

        std::vector<size_t> frame_count_per_sec;
        frame_count_per_sec.reserve(SamplesPerSec);

        bool isStarted = false;
        size_t frameIdx = 0;
        auto last = std::chrono::high_resolution_clock::now();       
        while (frame_count_per_sec.size() != SamplesPerSec) {
            bool exitRenderLoop = false, requestRestart = false;
            PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop)
                break;
            if (!IsSessionRunning())
                continue;
            if (!isStarted)
            {
                last = std::chrono::high_resolution_clock::now();
                isStarted = true;
            }
            XrFrameState frameState{ XR_TYPE_FRAME_STATE };
            CHECK_XRCMD(xrWaitFrame(m_session, nullptr, &frameState));            
            CHECK_XRCMD(xrBeginFrame(m_session, nullptr));
            XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
            frameEndInfo.displayTime = frameState.predictedDisplayTime;
            frameEndInfo.environmentBlendMode = m_environmentBlendMode;
            frameEndInfo.layerCount = 0;
            frameEndInfo.layers = nullptr;
            CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));

            if (!frameState.shouldRender)
                continue;
            
            ++frameIdx;
            auto curr = std::chrono::high_resolution_clock::now();
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
            CHECK_XRCMD(m_pfnGetDisplayRefreshRateFB(m_session, &m_streamConfig.refreshRate));
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
        m_displayRefreshRates = { EstimateDisplayRefreshRate() };
        assert(m_displayRefreshRates.size() > 0);
    }

    virtual bool GetSystemProperties(SystemProperties& systemProps) const override
    {
        if (m_instance == XR_NULL_HANDLE)
            return false;
        XrSystemProperties xrSystemProps = { XR_TYPE_SYSTEM_PROPERTIES };
        CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &xrSystemProps));
        std::strncpy(systemProps.systemName, xrSystemProps.systemName, sizeof(systemProps.systemName));
        if (m_configViews.size() > 0)
        {
            const auto& configView = m_configViews[0];
            systemProps.recommendedEyeWidth = configView.recommendedImageRectWidth;
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
    };
    constexpr /*inline*/ static const SpaceLoc IdentitySpaceLoc = { IdentityPose, {0,0,0}, {0,0,0}};

    inline SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const XrSpace& baseSpace, const SpaceLoc& initLoc = IdentitySpaceLoc) const
    {
        XrSpaceVelocity velocity{ XR_TYPE_SPACE_VELOCITY };
        XrSpaceLocation spaceLocation{ XR_TYPE_SPACE_LOCATION, &velocity };
        const auto res = xrLocateSpace(targetSpace, baseSpace, m_lastDisplayTime, &spaceLocation);
        CHECK_XRRESULT(res, "xrLocateSpace");
        
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

    inline SpaceLoc GetSpaceLocation(const XrSpace& targetSpace, const SpaceLoc& initLoc = IdentitySpaceLoc) const
    {
        return GetSpaceLocation(targetSpace, m_appSpace, initLoc);
    }

    void GetControllerInfo(TrackingInfo& info, const double /*displayTime*/) const
    {
        for (const auto hand : { Side::LEFT, Side::RIGHT }) {
            auto& newContInfo = info.controller[hand];
            newContInfo = m_input.controllerInfo[hand];

            //newContInfo.flags |= (TrackingInfo::Controller::FLAG_CONTROLLER_ENABLE |
            //                      TrackingInfo::Controller::FLAG_CONTROLLER_OCULUS_QUEST);
            //if (hand == Side::LEFT)
            //    newContInfo.flags |= TrackingInfo::Controller::FLAG_CONTROLLER_LEFTHAND;
            
            const auto spaceLoc = GetSpaceLocation(m_input.handSpace[hand]);
            newContInfo.position        = ToTrackingVector3(spaceLoc.pose.position);
            newContInfo.orientation     = ToTrackingQuat(spaceLoc.pose.orientation);
            newContInfo.linearVelocity  = ToTrackingVector3(spaceLoc.linearVelocity);
            newContInfo.angularVelocity = ToTrackingVector3(spaceLoc.angularVelocity);
        }
    }

    virtual bool GetTrackingInfo(TrackingInfo& info) const override
    {
        info = {};
        info.type = ALVR_PACKET_TYPE_TRACKING_INFO;
        info.flags = 0;
        info.clientTime = GetTimestampUs();
        info.predictedDisplayTime = static_cast<double>(m_lastDisplayTime) * 1e-9;
        info.FrameIndex = m_frameIndex;
        info.battery = 100;// g_ctx.batteryLevel;
        info.plugged = true;

        if (m_views.size() >= 2)
        {
            const auto ToEyeFov = [](const XrFovf& fov) -> EyeFov
            {
                return {
                    Math::ToDegrees(fov.angleLeft),
                    Math::ToDegrees(fov.angleRight),
                    Math::ToDegrees(fov.angleUp),
                    Math::ToDegrees(fov.angleDown)
                };
            };
            info.eyeFov[0] = ToEyeFov(m_views[0].fov);
            info.eyeFov[1] = ToEyeFov(m_views[1].fov);
                        
            XrVector3f v;
            XrVector3f_Sub(&v, &m_views[1].pose.position, &m_views[0].pose.position);
            float ipd = XrVector3f_Length(&v);
            if (std::fabs(ipd) < 0.00001f)
                ipd = 0.063f;
            info.ipd = ipd * 1000.0f;

            const auto hmdSpaceLoc = GetSpaceLocation(m_viewSpace);
            info.HeadPose_Pose_Orientation = ToTrackingQuat(hmdSpaceLoc.pose.orientation);
            info.HeadPose_Pose_Position = ToTrackingVector3(hmdSpaceLoc.pose.position);

            //XrVector3f_Add(&v, &m_views[0].pose.position, &m_views[1].pose.position);
            //XrVector3f_Scale(&v, &v, 0.5f);

            //XrQuaternionf result;
            //XrQuaternionf_Lerp(&result, &m_views[0].pose.orientation, &m_views[1].pose.orientation, 0.5f);

            //
            //Log::Write(Log::Level::Info, Fmt("HMD rot from view data: %f %f %f %f", result.x, result.y, result.z, result.w));
            //Log::Write(Log::Level::Info, Fmt("HMD rot from view space: %f %f %f %f", hmdSpaceLoc.pose.orientation.x, hmdSpaceLoc.pose.orientation.y, hmdSpaceLoc.pose.orientation.z, hmdSpaceLoc.pose.orientation.w));

            //Log::Write(Log::Level::Info, Fmt("HMD pos from view data: %f %f %f %f", v.x, v.y, v.z));
            //Log::Write(Log::Level::Info, Fmt("HMD pos from view space: %f %f %f %f", hmdSpaceLoc.pose.position.x, hmdSpaceLoc.pose.position.y, hmdSpaceLoc.pose.position.z));

        }

        GetControllerInfo(info, 0);// clientsidePrediction ? frame->displayTime : 0.);
        return true;
    }

    virtual inline void EnqueueHapticFeedback(const HapticsFeedback& hapticFeedback) override
    {
        m_hapticsQueue.push(hapticFeedback);
    }

    virtual inline void SetStreamConfig(const StreamConfig& config) override
    {
        m_streamConfigQueue.push(config);
    }

    void PollStreamConfigEvents()
    {
        StreamConfig newConfig;
        if (!m_streamConfigQueue.try_pop(newConfig))
            return;

        if (newConfig.trackingSpaceType != m_streamConfig.trackingSpaceType) {
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

        if (newConfig.refreshRate != m_streamConfig.refreshRate) {
            [&]() {
                if (m_pfnRequestDisplayRefreshRateFB == nullptr) {
                    Log::Write(Log::Level::Warning, "This OpenXR runtime does not support setting the display refresh rate.");
                    return;
                }

                const auto itr = std::find(m_displayRefreshRates.begin(), m_displayRefreshRates.end(), newConfig.refreshRate);
                if (itr == m_displayRefreshRates.end()) {
                    Log::Write(Log::Level::Warning, Fmt("Selected new refresh rate %f Hz is not supported, no change has been made.", newConfig.refreshRate));
                    return;
                }

                Log::Write(Log::Level::Info, Fmt("Setting display refresh rate from %f Hz to %f Hz.", m_streamConfig.refreshRate, newConfig.refreshRate));
                CHECK_XRCMD(m_pfnRequestDisplayRefreshRateFB(m_session, newConfig.refreshRate));
                m_streamConfig.refreshRate = newConfig.refreshRate;
            }();
        }

        //m_streamConfig = newConfig;
    }

   private:
    const std::shared_ptr<Options> m_options;
    std::shared_ptr<IPlatformPlugin> m_platformPlugin;
    std::shared_ptr<IGraphicsPlugin> m_graphicsPlugin;
    XrInstance m_instance{XR_NULL_HANDLE};
    XrSession m_session{XR_NULL_HANDLE};
    XrSpace m_appSpace{XR_NULL_HANDLE};
    XrSpace m_viewSpace{ XR_NULL_HANDLE };
    XrFormFactor m_formFactor{XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    XrViewConfigurationType m_viewConfigType{XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
    XrEnvironmentBlendMode m_environmentBlendMode{XR_ENVIRONMENT_BLEND_MODE_OPAQUE};
    XrSystemId m_systemId{XR_NULL_SYSTEM_ID};

    std::vector<XrViewConfigurationView> m_configViews;
    std::vector<Swapchain> m_swapchains;
    std::map<XrSwapchain, std::vector<XrSwapchainImageBaseHeader*>> m_swapchainImages;
    std::vector<XrView> m_views;
    int64_t m_colorSwapchainFormat{-1};

    std::vector<XrSpace> m_visualizedSpaces;

    // Application's current lifecycle state according to the runtime
    XrSessionState m_sessionState{XR_SESSION_STATE_UNKNOWN};
    bool m_sessionRunning{false};

    XrEventDataBuffer m_eventDataBuffer;
    InputState m_input;

    // XR_EXT_hand_tracking fun pointers.
    PFN_xrCreateHandTrackerEXT m_pfnCreateHandTrackerEXT = nullptr;
    PFN_xrLocateHandJointsEXT  m_pfnLocateHandJointsEXT = nullptr;

    // XR_FB_display_refresh_rate fun pointers.
    PFN_xrEnumerateDisplayRefreshRatesFB m_pfnEnumerateDisplayRefreshRatesFB = nullptr;
    PFN_xrGetDisplayRefreshRateFB m_pfnGetDisplayRefreshRateFB = nullptr;
    PFN_xrRequestDisplayRefreshRateFB m_pfnRequestDisplayRefreshRateFB = nullptr;

    std::size_t m_frameIndex = 0;
    XrTime m_lastDisplayTime = 0;

    std::vector<float> m_displayRefreshRates;

    StreamConfig m_streamConfig { 90.0, TrackingSpace::LocalRefSpace };
    
    xrconcurrency::concurrent_queue<HapticsFeedback> m_hapticsQueue;
    xrconcurrency::concurrent_queue<StreamConfig> m_streamConfigQueue;
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
