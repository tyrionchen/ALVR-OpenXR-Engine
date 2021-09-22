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
using RustCtxPtr = std::shared_ptr<const ALXRRustCtx>;
RustCtxPtr          gRustCtx{ nullptr };
IOpenXrProgramPtr   gProgram{ nullptr };
std::shared_mutex   gTrackingMutex;
TrackingInfo        gLastTrackingInfo{};

constexpr inline auto graphics_api_str(const ALXRGraphicsApi gcp)
{
    switch (gcp)
    {
    case ALXRGraphicsApi::Vulkan2:
        return "Vulkan2";
    case ALXRGraphicsApi::Vulkan:
        return "Vulkan";
    case ALXRGraphicsApi::D3D12:
        return "D3D12";
    case ALXRGraphicsApi::D3D11:
        return "D3D11";
    case ALXRGraphicsApi::OpenGLES:
        return "OpenGLES";
    case ALXRGraphicsApi::OpenGL:
        return "OpenGL";
    default:
        return "auto";
    }
}

bool alxr_init(const ALXRRustCtx* rCtx, /*[out]*/ ALXRSystemProperties* systemProperties) {
    try {
        if (rCtx == nullptr || rCtx->legacySend == nullptr)
        {
            Log::Write(Log::Level::Error, "Rust context has not been setup!");
            return false;
        }
        
        gRustCtx = std::make_shared<ALXRRustCtx>(*rCtx);
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

        ALXRSystemProperties rustSysProp{};
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

void alxr_destroy() {
    Log::Write(Log::Level::Info, "openxrShutdown: Shuttingdown");
    gProgram.reset();
    gRustCtx.reset();
}

void alxr_request_exit_session() {
    if (auto programPtr = gProgram) {
        programPtr->RequestExitSession();
    }
}

void alxr_proces_frame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */) {
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

bool alxr_is_session_running()
{
    if (auto programPtr = gProgram)
        return gProgram->IsSessionRunning();
    return false;
}

void alxr_set_stream_config(ALXRStreamConfig config)
{
    if (const auto programPtr = gProgram) {
        programPtr->SetStreamConfig(config);
    }
}

ALXRGuardianData alxr_get_guardian_data() { return {}; }

void alxr_on_tracking_update(bool /*clientsidePrediction*/)
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

void alxr_on_receive(const unsigned char* packet, unsigned int packetSize)
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
