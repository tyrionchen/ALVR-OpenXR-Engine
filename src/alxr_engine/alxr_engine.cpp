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

#include "alxr_engine.h"
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

constexpr inline bool is_valid(const ALXRRustCtx& rCtx)
{
    return  rCtx.inputSend != nullptr &&
            rCtx.viewsConfigSend != nullptr &&
            rCtx.pathStringToHash != nullptr; 
}

bool alxr_init(const ALXRRustCtx* rCtx, /*[out]*/ ALXRSystemProperties* systemProperties) {
    try {
        if (rCtx == nullptr || !is_valid(*rCtx))
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
        gProgram->InitializeSystem(ALXRPaths {
            .head           = rCtx->pathStringToHash("/user/head"),
            .left_hand      = rCtx->pathStringToHash("/user/hand/left"),
            .right_hand     = rCtx->pathStringToHash("/user/hand/right"),
            .left_haptics   = rCtx->pathStringToHash("/user/hand/left/output/haptic"),
            .right_haptics  = rCtx->pathStringToHash("/user/hand/right/output/haptic")
        });
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

void alxr_process_frame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */) {
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

    thread_local ALXREyeInfo gLastEyeInfo{
        .eveFov = { {0,0,0,0}, {0,0,0,0} },
        .ipd = 0.0f
    };
    ALXREyeInfo newEyeInfo{};
    if (!gProgram->GetEyeInfo(newEyeInfo))
        return;
    if (std::abs(newEyeInfo.ipd - gLastEyeInfo.ipd) > 0.001 ||
        std::abs(newEyeInfo.eveFov[0].left - gLastEyeInfo.eveFov[0].left) > 0.001 ||
        std::abs(newEyeInfo.eveFov[1].left - gLastEyeInfo.eveFov[1].left) > 0.001)
    {
        gLastEyeInfo = newEyeInfo;
        gRustCtx->viewsConfigSend(&newEyeInfo);
        //Log::Write(Log::Level::Info, "new viewConfig sent.");
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

ALXRGuardianData alxr_get_guardian_data()
{
    ALXRGuardianData gd{};
    gd.shouldSync = false;
    if (const auto programPtr = gProgram) {
        programPtr->GetGuardianData(gd);
    }
    return gd;
}

void alxr_on_tracking_update(bool /*clientsidePrediction*/)
{
    const auto rustCtx = gRustCtx;
    if (rustCtx == nullptr || rustCtx->inputSend == nullptr)
        return;
    TrackingInfo newInfo;
    {
        std::shared_lock<std::shared_mutex> l(gTrackingMutex);
        newInfo = gLastTrackingInfo;
    }
    //++FrameIndex;
    //newInfo.FrameIndex = FrameIndex;
    //newInfo.clientTime = GetTimestampUs();
    rustCtx->inputSend(&newInfo);
}

void alxr_on_receive(const unsigned char* /*packet*/, unsigned int /*packetSize*/)
{
    const auto programPtr = gProgram;
    if (programPtr == nullptr)
        return;

    //const std::uint32_t type = *reinterpret_cast<const uint32_t*>(packet);
    //if (type == ALVR_PACKET_TYPE_HAPTICS)
    //{
    //    if (packetSize < sizeof(HapticsFeedback))
    //        return;  
    //    programPtr->EnqueueHapticFeedback(*reinterpret_cast<const HapticsFeedback*>(packet));
    //}
}

void alxr_on_haptics_feedback(unsigned long long path, float duration_s, float frequency, float amplitude)
{
    if (const auto programPtr = gProgram) {
        programPtr->EnqueueHapticFeedback(HapticsFeedback {
            .alxrPath   = path,
            .amplitude  = amplitude,
            .duration   = duration_s,
            .frequency  = frequency
        });
    }
}
