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

#include "rust_bindings.h"
#include "ALVR-common/packet_types.h"

using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
using RustCtxPtr = std::shared_ptr<const RustCtx>;
RustCtxPtr          gRustCtx{ nullptr };
IOpenXrProgramPtr   gProgram{ nullptr };
std::shared_mutex   gTrackingMutex;
TrackingInfo        gLastTrackingInfo{};

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

bool openxrInit(const RustCtx* rCtx, /*[out]*/ SystemProperties* systemProperties) {
    try {
        if (rCtx == nullptr || rCtx->legacySend == nullptr)
        {
            Log::Write(Log::Level::Error, "Rust context has not been setup!");
            return false;
        }
        
        gRustCtx = std::make_shared<RustCtx>(*rCtx);
        const auto &ctx = *gRustCtx;//.load();
        if (ctx.verbose)
            Log::SetLevel(Log::Level::Verbose);

        const auto options = std::make_shared<Options>();
        assert(options->AppSpace == "Stage");
        assert(options->ViewConfiguration == "Stereo");
        if (options->GraphicsPlugin.empty())
            options->GraphicsPlugin = graphics_api_str(ctx.graphicsApi);

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
