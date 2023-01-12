// Copyright (c) 2017-2021, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0
#include "logger.h"
#define ENGINE_DLL_EXPORTS

#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformdata.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"

#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>

#include "alxr_engine.h"

#include "timing.h"
#include "interaction_manager.h"
#include "latency_manager.h"
#include "decoder_thread.h"
#include "foveation.h"

#include <jnipp.h>
#include <jni.h>
#include "json/json.h"

#if defined(XR_USE_PLATFORM_WIN32) && defined(XR_EXPORT_HIGH_PERF_GPU_SELECTION_SYMBOLS)
#pragma message("Enabling Symbols to select high-perf GPUs first")
// Export symbols to get the high performance gpu as first adapter in IDXGIFactory::EnumAdapters().
// This can be also necessary for the IMFActivate::ActivateObject method if no windows graphic settings are present.
extern "C" {
    // http://developer.download.nvidia.com/devzone/devcenter/gamegraphics/files/OptimusRenderingPolicies.pdf
    _declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
    // https://gpuopen.com/learn/amdpowerxpressrequesthighperformance/
    _declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;
}
#endif

constexpr inline const ALXREyeInfo EyeInfoZero {
    .eyeFov = { {0,0,0,0}, {0,0,0,0} },
    .ipd = 0.0f
};

using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
using RustCtxPtr = std::shared_ptr<const ALXRRustCtx>;

RustCtxPtr        gRustCtx{ nullptr };
IOpenXrProgramPtr gProgram{ nullptr };
XrDecoderThread   gDecoderThread{};
std::mutex        gRenderMutex{};
ALXREyeInfo       gLastEyeInfo = EyeInfoZero;

namespace ALXRStrings {
    constexpr inline const char* const HeadPath         = "/user/head";
    constexpr inline const char* const LeftHandPath     = "/user/hand/left";
    constexpr inline const char* const RightHandPath    = "/user/hand/right";
    constexpr inline const char* const LeftHandHaptics  = "/user/hand/left/output/haptic";
    constexpr inline const char* const RightHandHaptics = "/user/hand/right/output/haptic";
};

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
    case ALXRGraphicsApi::OpenGLES2:
        return "OpenGLES2";
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
            rCtx.pathStringToHash != nullptr &&
            rCtx.requestIDR != nullptr;
}

inline jclass loadClz(jobject obj_activity, const char* cStrClzName) {
    jclass clz_activity = jni::env()->GetObjectClass(obj_activity);
    jmethodID method_getClassLoader = jni::env()->GetMethodID(clz_activity, "getClassLoader", "()Ljava/lang/ClassLoader;");
    jobject obj_classLoader = jni::env()->CallObjectMethod(obj_activity, method_getClassLoader);
    jclass classLoader = jni::env()->FindClass("java/lang/ClassLoader");
    jmethodID findClass = jni::env()->GetMethodID(classLoader, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
    jstring strClassName = jni::env()->NewStringUTF(cStrClzName);
    jclass clz = (jclass)(jni::env()->CallObjectMethod(obj_classLoader, findClass, strClassName));
    jni::env()->DeleteLocalRef(strClassName);
    return clz;
}

jobject g_tcrActivity_jobject{nullptr};
jmethodID g_tcrActivity_onEvent_method{nullptr};
jmethodID g_tcrActivity_updateTexture_method{nullptr};
jmethodID g_tcrActivity_createEglRenderer_method{nullptr};

void onEvent(std::string type, std::string msg) {
    if (g_tcrActivity_jobject == nullptr || g_tcrActivity_onEvent_method == nullptr) {
        return;
    }

    jstring jType = jni::env()->NewStringUTF(type.c_str());
    jstring jMsg = jni::env()->NewStringUTF(msg.c_str());
    jni::env()->CallVoidMethod(g_tcrActivity_jobject, g_tcrActivity_onEvent_method, jType, jMsg);
}

std::uint64_t updateTexture() {
    if (g_tcrActivity_jobject == nullptr || g_tcrActivity_updateTexture_method == nullptr) {
        return 0;
    }
    return jni::env()->CallLongMethod(g_tcrActivity_jobject, g_tcrActivity_updateTexture_method);
}

void createEglRenderer(int textureId) {
    if (g_tcrActivity_jobject == nullptr || g_tcrActivity_createEglRenderer_method == nullptr) {
        return;
    }
    jni::env()->CallVoidMethod(g_tcrActivity_jobject, g_tcrActivity_createEglRenderer_method, textureId);
}

void initJni(const ALXRRustCtx ctx) {
    jni::init((JavaVM*)(ctx.applicationVM));
    g_tcrActivity_jobject = (jobject)(ctx.applicationActivity);
    jclass clz_tcr_activity = loadClz(g_tcrActivity_jobject, "com/tencent/tcr/xr/TcrActivity");
    g_tcrActivity_onEvent_method = jni::env()->GetMethodID(clz_tcr_activity, "onEvent", "(Ljava/lang/String;Ljava/lang/String;)V");
    g_tcrActivity_updateTexture_method = jni::env()->GetMethodID(clz_tcr_activity, "updateTexture", "()J");
    g_tcrActivity_createEglRenderer_method = jni::env()->GetMethodID(clz_tcr_activity, "createEglRenderer","(I)V");
}

bool alxr_init(const ALXRRustCtx* rCtx, /*[out]*/ ALXRSystemProperties* systemProperties) {
    try {
        if (rCtx == nullptr || !is_valid(*rCtx))
        {
            Log::Write(Log::Level::Error, "Rust context has not been setup!");
            return false;
        }
        
        gRustCtx = std::make_shared<ALXRRustCtx>(*rCtx);
        const auto &ctx = *gRustCtx;
        if (ctx.verbose)
            Log::SetLevel(Log::Level::Verbose);
#ifdef XR_TCR_VERSION
#pragma message ("Building tcr customized alxr client.")
        initJni(ctx);
#endif        

        LatencyManager::Instance().Init(LatencyManager::CallbackCtx {
            .sendFn = ctx.inputSend,
            .timeSyncSendFn = ctx.timeSyncSend,
            .videoErrorReportSendFn = ctx.videoErrorReportSend
        });

        const auto options = std::make_shared<Options>();
        assert(options->AppSpace == "Stage");
        assert(options->ViewConfiguration == "Stereo");
        options->DisableLinearizeSrgb = ctx.disableLinearizeSrgb;
        options->DisableSuggestedBindings = ctx.noSuggestedBindings;
        options->NoServerFramerateLock = ctx.noServerFramerateLock;
        options->NoFrameSkip = ctx.noFrameSkip;
        options->DisableLocalDimming = ctx.disableLocalDimming;
        options->DisplayColorSpace = static_cast<XrColorSpaceFB>(ctx.displayColorSpace);
        if (options->GraphicsPlugin.empty())
            options->GraphicsPlugin = graphics_api_str(ctx.graphicsApi);

        //强制使用OpenGLES2
        options->GraphicsPlugin = graphics_api_str(ALXRGraphicsApi::OpenGLES2);

        const auto platformData = std::make_shared<PlatformData>();
#ifdef XR_USE_PLATFORM_ANDROID
#pragma message ("Android Loader Enabled.")
        platformData->applicationVM = ctx.applicationVM;
        platformData->applicationActivity = ctx.applicationActivity;

        // Initialize the loader for this platform
        PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
        if (XR_SUCCEEDED(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                                      (PFN_xrVoidFunction *) (&initializeLoader)))) {
            const XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid {
                .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
                .next = nullptr,
                .applicationVM = ctx.applicationVM,
                .applicationContext = ctx.applicationActivity
            };
            initializeLoader((const XrLoaderInitInfoBaseHeaderKHR *) &loaderInitInfoAndroid);
        }

        //av_jni_set_java_vm(ctx.applicationVM, nullptr);
#endif
        // Create platform-specific implementation.
        const auto platformPlugin = CreatePlatformPlugin(options, platformData);
        // Initialize the OpenXR gProgram.
        gProgram = CreateOpenXrProgram(options, platformPlugin);
        gProgram->GetGraphicsPlugin()->SetTcrCreateEglRenderer(createEglRenderer);
        gProgram->GetGraphicsPlugin()->SetTcrUpdateTexture(updateTexture);
#ifdef XR_TCR_VERSION
        gProgram->setOnEvent(onEvent);
#endif
        gProgram->CreateInstance();
        gProgram->InitializeSystem(ALXR::ALXRPaths {
            .head           = rCtx->pathStringToHash(ALXRStrings::HeadPath),
            .left_hand      = rCtx->pathStringToHash(ALXRStrings::LeftHandPath),
            .right_hand     = rCtx->pathStringToHash(ALXRStrings::RightHandPath),
            .left_haptics   = rCtx->pathStringToHash(ALXRStrings::LeftHandHaptics),
            .right_haptics  = rCtx->pathStringToHash(ALXRStrings::RightHandHaptics)
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

extern "C" {
/*
 * Class:     com_tencent_tcr_xr_TcrActivity
 * Method:    nativeSetVideoReady
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_com_tencent_tcr_xr_TcrActivity_nativeSetVideoStreamReady(JNIEnv*, jobject) {
    Log::Write(Log::Level::Info, "nativeSetVideoStreamReady");
    gProgram->SetRenderMode(IOpenXrProgram::RenderMode::VideoStream);
}

}

void alxr_stop_decoder_thread()
{
#ifndef XR_DISABLE_DECODER_THREAD
    gDecoderThread.Stop();
#endif
}

void alxr_destroy() {
    Log::Write(Log::Level::Info, "openxrShutdown: Shuttingdown");
    if (const auto programPtr = gProgram) {
        if (const auto graphicsPtr = programPtr->GetGraphicsPlugin()) {
            std::scoped_lock lk(gRenderMutex);
            graphicsPtr->ClearVideoTextures();
        }
    }
    alxr_stop_decoder_thread();
    gProgram.reset();
    gRustCtx.reset();
}

void alxr_request_exit_session() {
    if (const auto programPtr = gProgram) {
        programPtr->RequestExitSession();
    }
}

void alxr_process_frame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */) {
    assert(exitRenderLoop != nullptr && requestRestart != nullptr);
    gProgram->PollEvents(exitRenderLoop, requestRestart);
    if (*exitRenderLoop || !gProgram->IsSessionRunning()) {
        Log::Write(Log::Level::Info, "alxr_process_frame exit!!!");
        return;
    }
    gProgram->SetAndroidJniEnv();
    //gProgram->PollActions();
    {
        std::scoped_lock lk(gRenderMutex);
        gProgram->RenderFrame();
    }
}

bool alxr_is_session_running()
{
    if (const auto programPtr = gProgram)
        return gProgram->IsSessionRunning();
    return false;
}

void alxr_set_stream_config(const ALXRStreamConfig config)
{
    const auto programPtr = gProgram;
    if (programPtr == nullptr)
        return;
    alxr_stop_decoder_thread();
    if (const auto graphicsPtr = programPtr->GetGraphicsPlugin()) {
        const auto& rc = config.renderConfig;
        std::scoped_lock lk(gRenderMutex);
        programPtr->SetRenderMode(IOpenXrProgram::RenderMode::Lobby);
        graphicsPtr->ClearVideoTextures();
        
        ALXR::FoveatedDecodeParams fdParams{};
        if (rc.enableFoveation)
            fdParams = ALXR::MakeFoveatedDecodeParams(rc);
        graphicsPtr->SetFoveatedDecode(rc.enableFoveation ? &fdParams : nullptr);
        // 使用opengles2为什么就不能重新创建Swapchains需要再确认下
        // programPtr->CreateSwapchains(rc.eyeWidth, rc.eyeHeight);
    }

    Log::Write(Log::Level::Info, "Starting decoder thread.");

    gLastEyeInfo = EyeInfoZero;
#ifndef XR_DISABLE_DECODER_THREAD
    const XrDecoderThread::StartCtx startCtx {
        .decoderConfig = config.decoderConfig,
        .programPtr = programPtr,
        .rustCtx = gRustCtx
    };
    gDecoderThread.Start(startCtx);
    Log::Write(Log::Level::Info, "Decoder Thread started.");
#endif
    // OpenXR does not have functions to query the battery levels of devices.
    const auto SendDummyBatteryLevels = []() {
        const auto rCtx = gRustCtx;
        if (rCtx == nullptr)
            return;
        const auto head_path        = rCtx->pathStringToHash(ALXRStrings::HeadPath);
        const auto left_hand_path   = rCtx->pathStringToHash(ALXRStrings::LeftHandPath);
        const auto right_hand_path  = rCtx->pathStringToHash(ALXRStrings::RightHandPath);
        // TODO: On android we can still get the real battery levels of the "HMD"
        //       by registering an IntentFilter battery change event.
        rCtx->batterySend(head_path, 1.0f, true);
        rCtx->batterySend(left_hand_path, 1.0f, true);
        rCtx->batterySend(right_hand_path, 1.0f, true);
    };
    SendDummyBatteryLevels();
    programPtr->SetStreamConfig(config);
}

void alxr_on_server_disconnect()
{
    if (const auto programPtr = gProgram) {
        programPtr->SetRenderMode(IOpenXrProgram::RenderMode::Lobby);
    }
}

ALXRGuardianData alxr_get_guardian_data()
{
    ALXRGuardianData gd {
        .shouldSync = false,
        .areaWidth = 0,
        .areaHeight = 0
    };
    if (const auto programPtr = gProgram) {
        programPtr->GetGuardianData(gd);
    }
    return gd;
}

void alxr_on_pause()
{
    if (const auto programPtr = gProgram)
        programPtr->Pause();    
}

void alxr_on_resume()
{
    if (const auto programPtr = gProgram)
        programPtr->Resume();
}

inline void LogViewConfig(const ALXREyeInfo& newEyeInfo)
{
    constexpr const auto FmtEyeFov = [](const EyeFov& eye) {
        constexpr const float deg = 180.0f / 3.14159265358979323846f;
        return Fmt("{ .left=%f, .right=%f, .top=%f, .bottom=%f }",
            eye.left * deg, eye.right * deg, eye.top * deg, eye.bottom * deg);
    };
    const auto lEyeFovStr = FmtEyeFov(newEyeInfo.eyeFov[0]);
    const auto rEyeFovStr = FmtEyeFov(newEyeInfo.eyeFov[1]);
    Log::Write(Log::Level::Info, Fmt("New view config sent:\n"
        "\tViewConfig {\n"
        "\t  .ipd = %f,\n"
        "\t  .eyeFov {\n"
        "\t    .leftEye  = %s,\n"
        "\t    .rightEye = %s\n"
        "\t  }\n"
        "\t}",
        newEyeInfo.ipd * 1000.0f, lEyeFovStr.c_str(), rEyeFovStr.c_str()));
}

Json::Value toJson(const TrackingInfo& trackingInfo) {
    Json::Value orientation;
    orientation["x"] = trackingInfo.HeadPose_Pose_Orientation.x;
    orientation["y"] = trackingInfo.HeadPose_Pose_Orientation.y;
    orientation["z"] = trackingInfo.HeadPose_Pose_Orientation.z;
    orientation["w"] = trackingInfo.HeadPose_Pose_Orientation.w;
    Json::Value position;
    position["x"] = trackingInfo.HeadPose_Pose_Position.x;
    position["y"] = trackingInfo.HeadPose_Pose_Position.y;
    position["z"] = trackingInfo.HeadPose_Pose_Position.z;
    Json::Value pose;
    pose["orientation"] = orientation;
    pose["position"] = position;
    return pose;
}

  Json::Value toJson(const EyeFov& xrFov) {
    Json::Value fov;
    fov["angleLeft"] = xrFov.left;
    fov["angleRight"] = xrFov.right;
    fov["angleUp"] = xrFov.top;
    fov["angleDown"] = xrFov.bottom;
    return fov;
  }

  Json::Value toJson(const XrPosef& xrPose) {
    Json::Value orientation;
    orientation["x"] = xrPose.orientation.x;
    orientation["y"] = xrPose.orientation.y;
    orientation["z"] = xrPose.orientation.z;
    orientation["w"] = xrPose.orientation.w;
    Json::Value position;
    position["x"] = xrPose.position.x;
    position["y"] = xrPose.position.y;
    position["z"] = xrPose.position.z;
    Json::Value pose;
    pose["orientation"] = orientation;
    pose["position"] = position;
    return pose;
  }

  Json::Value toJson(const EyeFov& xrFov, const XrPosef& xrPose) {
    Json::Value view;
    Json::Value fov;
    fov["angleLeft"] = xrFov.left;
    fov["angleRight"] = xrFov.right;
    fov["angleUp"] = xrFov.top;
    fov["angleDown"] = xrFov.bottom;
    view["fov"] = fov;
    Json::Value orientation;
    orientation["x"] = xrPose.orientation.x;
    orientation["y"] = xrPose.orientation.y;
    orientation["z"] = xrPose.orientation.z;
    orientation["w"] = xrPose.orientation.w;
    Json::Value position;
    position["x"] = xrPose.position.x;
    position["y"] = xrPose.position.y;
    position["z"] = xrPose.position.z;
    Json::Value pose;
    pose["orientation"] = orientation;
    pose["position"] = position;
    view["pose"] = pose;
    return view;
  }

void viewsConfigSend(const ALXREyeInfo& newEyeInfo) {
    Json::Value eyeInfo;
    eyeInfo["leftFov"] = toJson(newEyeInfo.eyeFov[0]);
    eyeInfo["rightFov"] = toJson(newEyeInfo.eyeFov[1]);
    eyeInfo["ipd"] = newEyeInfo.ipd;
    onEvent("eye_info_change", Json::writeString(Json::StreamWriterBuilder(), eyeInfo));
}

  void inputSend(const TrackingInfo& newInfo) {
    Json::Value trackingInfo;
    trackingInfo["hmdPose"] = toJson(newInfo);
    const auto predicatedDisplayTimeNs = static_cast<std::uint64_t>(newInfo.targetTimestampNs);
    trackingInfo["displayTime"] = predicatedDisplayTimeNs;
    const std::string json_string = Json::writeString(Json::StreamWriterBuilder(), trackingInfo);
    onEvent("tracking_info_change", json_string);
  }

void alxr_on_tracking_update(const bool clientsidePrediction)
{
    const auto rustCtx = gRustCtx;
    if (rustCtx == nullptr)
        return;
    const auto xrProgram = gProgram;
    if (xrProgram == nullptr || !xrProgram->IsSessionRunning())
        return;
    ALXREyeInfo newEyeInfo{};

    if (!gProgram->GetEyeInfo(newEyeInfo)) {
        Log::Write(Log::Level::Info, "alxr_on_tracking_update !GetEyeInfo()");
        return;
    }
    if (std::abs(newEyeInfo.ipd - gLastEyeInfo.ipd) > 0.00001f ||
        std::abs(newEyeInfo.eyeFov[0].left - gLastEyeInfo.eyeFov[0].left) > 0.00001f ||
        std::abs(newEyeInfo.eyeFov[1].left - gLastEyeInfo.eyeFov[1].left) > 0.00001f)
    {
        gLastEyeInfo = newEyeInfo;
#ifdef XR_TCR_VERSION
        viewsConfigSend(newEyeInfo);
#else
        gRustCtx->viewsConfigSend(&newEyeInfo);
#endif        
        LogViewConfig(newEyeInfo);
    }
    xrProgram->PollActions();
    TrackingInfo newInfo;
    if (!xrProgram->GetTrackingInfo(newInfo, clientsidePrediction)) {
        Log::Write(Log::Level::Info, "alxr_on_tracking_update !GetTrackingInfo()");
        return;
    }
#ifdef XR_TCR_VERSION
    inputSend(newInfo);
#else
    rustCtx->inputSend(&newInfo);
#endif
}

void alxr_on_receive(const unsigned char* packet, unsigned int packetSize)
{
    const auto programPtr = gProgram;
    if (programPtr == nullptr)
        return;
    const std::uint32_t type = *reinterpret_cast<const uint32_t*>(packet);
    switch (type) {
        case ALVR_PACKET_TYPE_VIDEO_FRAME: {
#ifndef XR_DISABLE_DECODER_THREAD
            assert(packetSize >= sizeof(VideoFrame));
            const auto& header = *reinterpret_cast<const VideoFrame*>(packet);
            gDecoderThread.QueuePacket(header, packetSize);
#endif
        } break;        
        case ALVR_PACKET_TYPE_TIME_SYNC: {
            assert(packetSize >= sizeof(TimeSync));
            LatencyManager::Instance().OnTimeSyncRecieved(*(TimeSync*)packet);
        } break;
    }
}

void alxr_on_haptics_feedback(unsigned long long path, float duration_s, float frequency, float amplitude)
{
    if (const auto programPtr = gProgram) {
        programPtr->ApplyHapticFeedback(ALXR::HapticsFeedback {
            .alxrPath   = path,
            .amplitude  = amplitude,
            .duration   = duration_s,
            .frequency  = frequency
        });
    }
}