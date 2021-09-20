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

#include <array>
#include <cassert>
#include <atomic>
#include <shared_mutex>
#include <chrono>
#include <sstream>

#include "rust_bindings.h"
#include "ALVR-common/packet_types.h"

namespace {

#ifdef XR_USE_PLATFORM_ANDROID
void ShowHelp() { Log::Write(Log::Level::Info, "adb shell setprop debug.xr.graphicsPlugin OpenGLES|Vulkan"); }

bool UpdateOptionsFromSystemProperties(Options& options) {
    char value[PROP_VALUE_MAX] = {};
    if (__system_property_get("debug.xr.graphicsPlugin", value) != 0) {
        options.GraphicsPlugin = value;
    }

    if (__system_property_get("debug.xr.verbose", value) != 0) {
        bool verbose = false;
        std::istringstream oss{ value };
        if (!(oss >> verbose) && verbose) {
            Log::SetLevel(Log::Level::Verbose);
            Log::Write(Log::Level::Info, "verbose mode enabled.");
        }
    }
    return true;
}
#else
void ShowHelp() {
    // TODO: Improve/update when things are more settled.
    Log::Write(Log::Level::Info,
               "xr_engine --graphics|-g <Graphics API> [--formfactor|-ff <Form factor>] [--viewconfig|-vc <View config>] "
               "[--blendmode|-bm <Blend mode>] [--space|-s <Space>] [--verbose|-v]");
    Log::Write(Log::Level::Info, "Graphics APIs:            D3D11, D3D12, OpenGLES, OpenGL, Vulkan2, Vulkan");
    Log::Write(Log::Level::Info, "Form factors:             Hmd, Handheld");
    Log::Write(Log::Level::Info, "View configurations:      Mono, Stereo");
    Log::Write(Log::Level::Info, "Environment blend modes:  Opaque, Additive, AlphaBlend");
    Log::Write(Log::Level::Info, "Spaces:                   View, Local, Stage");
}

bool UpdateOptionsFromCommandLine(Options& options, int argc, const char* argv[]) {
    int i = 1;  // Index 0 is the gProgram name and is skipped.

    auto getNextArg = [&] {
        if (i >= argc) {
            throw std::invalid_argument("Argument parameter missing");
        }

        return std::string(argv[i++]);
    };

    while (i < argc) {
        const std::string arg = getNextArg();
        if (EqualsIgnoreCase(arg, "--graphics") || EqualsIgnoreCase(arg, "-g")) {
            options.GraphicsPlugin = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--formfactor") || EqualsIgnoreCase(arg, "-ff")) {
            options.FormFactor = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--viewconfig") || EqualsIgnoreCase(arg, "-vc")) {
            options.ViewConfiguration = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--blendmode") || EqualsIgnoreCase(arg, "-bm")) {
            options.EnvironmentBlendMode = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--space") || EqualsIgnoreCase(arg, "-s")) {
            options.AppSpace = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--verbose") || EqualsIgnoreCase(arg, "-v")) {
            Log::SetLevel(Log::Level::Verbose);
        } else if (EqualsIgnoreCase(arg, "--help") || EqualsIgnoreCase(arg, "-h")) {
            ShowHelp();
            return false;
        } else {
            throw std::invalid_argument(Fmt("Unknown argument: %s", arg.c_str()));
        }
    }
    return true;
}
#endif
    constexpr inline auto graphics_api_str(const GraphicsCtxApi gcp)
    {
        switch (gcp)
        {
        case GraphicsCtxApi::Vulkan2:
            return "Vulkan2";
        case GraphicsCtxApi::Vulkan:
            return "Vulkan";
        case GraphicsCtxApi::D3D12:
            return "D3D12";
        case GraphicsCtxApi::D3D11:
            return "D3D11";
        case GraphicsCtxApi::OpenGLES:
            return "OpenGLES";
        case GraphicsCtxApi::OpenGL:
            return "OpenGL";
        default:
            return "auto";
        }
    }
}  // namespace

using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
using RustCtxPtr = std::shared_ptr<const RustCtx>;
RustCtxPtr          gRustCtx{ nullptr };
IOpenXrProgramPtr   gProgram{ nullptr };
std::shared_mutex   gTrackingMutex;
TrackingInfo        gLastTrackingInfo{};

bool openxrInit(const RustCtx* rCtx, /*[out]*/ SystemProperties* systemProperties) {
    try {
        if (rCtx == nullptr ||
            rCtx->legacySend == nullptr)
        {
            Log::Write(Log::Level::Error, "Rust context has not been setup!");
            return false;
        }
        
        gRustCtx = std::make_shared<RustCtx>(*rCtx);
        const auto &ctx = *gRustCtx;//.load();
        const auto options = std::make_shared<Options>();

#ifdef XR_USE_PLATFORM_ANDROID
        if (!UpdateOptionsFromSystemProperties(*options)) {
            return false;
        }
        if (options->GraphicsPlugin.empty())
            options->GraphicsPlugin = graphics_api_str(ctx.graphicsApi);
#else
        std::vector<const char*> args {
            "openxrInit",
            "-g", graphics_api_str(ctx.graphicsApi),  //"D3D11", //"Vulkan2",//"Vulkan2",//"D3D11",
            "-vc", "Stereo",
            "-s", "Stage"
        };
        if (ctx.verbose)
            args.push_back("-v");
        if (!UpdateOptionsFromCommandLine(*options, static_cast<int>(args.size()), args.data())) {
            return false;
        }
#endif

        const auto platformData = std::make_shared<PlatformData>();
#ifdef XR_USE_PLATFORM_ANDROID
        platformData->applicationVM = ctx.applicationVM;
        platformData->applicationActivity = ctx.applicationActivity;

        // Initialize the loader for this platform
        PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
        if (XR_SUCCEEDED(
                xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                                      (PFN_xrVoidFunction *) (&initializeLoader)))) {
            XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid;
            memset(&loaderInitInfoAndroid, 0, sizeof(loaderInitInfoAndroid));
            loaderInitInfoAndroid.type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR;
            loaderInitInfoAndroid.next = NULL;
            loaderInitInfoAndroid.applicationVM = ctx.applicationVM;
            loaderInitInfoAndroid.applicationContext = ctx.applicationActivity;
            initializeLoader((const XrLoaderInitInfoBaseHeaderKHR *) &loaderInitInfoAndroid);
        }
#endif
        // Create platform-specific implementation.
        const auto platformPlugin = CreatePlatformPlugin(options, platformData);        
        // Initialize the OpenXR gProgram.
        gProgram = CreateOpenXrProgram(options, platformPlugin);

        gProgram->CreateInstance();
        gProgram->InitializeSystem();
        gProgram->InitializeSession();
        gProgram->CreateSwapchains();

        SystemProperties rustSysProp{};
        gProgram->GetSystemProperties(rustSysProp);
        if (systemProperties)
            *systemProperties = rustSysProp;

        Log::Write(Log::Level::Info, Fmt("device name: %s", rustSysProp.systemName));
        Log::Write(Log::Level::Info, "openxrInit finished successfully");
        
        return true;
    } catch (const std::exception& ex) {
        Log::Write(Log::Level::Error, ex.what());
        return false;
    } catch (...) {
        Log::Write(Log::Level::Error, "Unknown Error");
        return false;
    }
}

void openxrRequestExitSession() {
    if (auto programPtr = gProgram) {
        programPtr->RequestExitSession();
    }
}

void openxrDestroy() {
    Log::Write(Log::Level::Info, "openxrShutdown: Shuttingdown");
    gProgram.reset();
    gRustCtx.reset();
}

void openxrProcesFrame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */) {
    assert(exitRenderLoop != nullptr && requestRestart != nullptr);

    gProgram->PollEvents(exitRenderLoop, requestRestart);
    if (*exitRenderLoop || !gProgram->IsSessionRunning())
        return;
    
    gProgram->PollActions();
    gProgram->RenderFrame();

    TrackingInfo newInfo;
    gProgram->GetTrackingInfo(newInfo);
    {
        std::unique_lock<std::shared_mutex> lock(gTrackingMutex);
        gLastTrackingInfo = newInfo;
    }
}

bool isOpenXRSessionRunning()
{
    if (auto programPtr = gProgram)
        return gProgram->IsSessionRunning();
    return false;
}

GuardianData getGuardianData() { return {}; }

//inline std::uint64_t GetTimestampUs()
//{
//    using namespace std::chrono;
//    using PeriodType = high_resolution_clock::period;
//    using DurationType = high_resolution_clock::duration;
//    using microsecondsU64 = std::chrono::duration<std::uint64_t, std::chrono::microseconds::period>;
//    return duration_cast<microsecondsU64>(high_resolution_clock::now().time_since_epoch()).count();
//}
//
//std::atomic<std::uint64_t> FrameIndex{ 0 };

void onTrackingNative(bool /*clientsidePrediction*/)
{
    const auto rustCtx = gRustCtx;
    if (rustCtx == nullptr || rustCtx->legacySend == nullptr)
        return;
    TrackingInfo newInfo;
    {
        std::shared_lock<std::shared_mutex> l(gTrackingMutex);
        newInfo = gLastTrackingInfo;
    }
    if (newInfo.type != ALVR_PACKET_TYPE_TRACKING_INFO)
        return;
    //++FrameIndex;
    //newInfo.FrameIndex = FrameIndex;
    //newInfo.clientTime = GetTimestampUs();
    rustCtx->legacySend(reinterpret_cast<const unsigned char*>(&newInfo), static_cast<int>(sizeof(newInfo)));
}

void legacyReceive(const unsigned char* packet, unsigned int packetSize)
{
    const auto programPtr = gProgram;
    if (programPtr == nullptr)
        return;

    const std::uint32_t type = *reinterpret_cast<const uint32_t*>(packet);
    if (type == ALVR_PACKET_TYPE_HAPTICS)
    {
        if (packetSize < sizeof(HapticsFeedback))
            return;  
        programPtr->EnqueueHapticFeedback(*reinterpret_cast<const HapticsFeedback*>(packet));
    }
}

void setStreamConfig(StreamConfig config)
{
    if (const auto programPtr = gProgram) {
        programPtr->SetStreamConfig(config);
    }
}
