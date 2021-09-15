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

//#ifdef XR_USE_PLATFORM_ANDROID
//#include <dlfcn.h>
//#endif

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
        if (!(oss >> verbose) && verbose)
            Log::SetLevel(Log::Level::Verbose);
    }

    // // Check for required parameters.
    // if (options.GraphicsPlugin.empty()) {
    //     Log::Write(Log::Level::Error, "GraphicsPlugin parameter is required");
    //     ShowHelp();
    //     return false;
    // }

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
    int i = 1;  // Index 0 is the program name and is skipped.

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

    // // Check for required parameters.
    // if (options.GraphicsPlugin.empty()) {
    //     Log::Write(Log::Level::Error, "GraphicsPlugin parameter is required");
    //     ShowHelp();
    //     return false;
    // }

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

std::shared_ptr<const RustCtx> gRustCtx{ nullptr };

using IOpenXrProgramPtr = std::shared_ptr<IOpenXrProgram>;
IOpenXrProgramPtr program{ nullptr };

std::shared_mutex gTrackingMutex;
TrackingInfo gLastTrackingInfo{};

#ifdef XR_USE_PLATFORM_ANDROID

// struct AndroidAppState {
//     ANativeWindow* NativeWindow = nullptr;
//     bool Resumed = false;
// };

// /**
//  * Process the next main command.
//  */
// static void app_handle_cmd(struct android_app* app, int32_t cmd) {
//     AndroidAppState* appState = (AndroidAppState*)app->userData;

//     switch (cmd) {
//         // There is no APP_CMD_CREATE. The ANativeActivity creates the
//         // application thread from onCreate(). The application thread
//         // then calls android_main().
//         case APP_CMD_START: {
//             Log::Write(Log::Level::Info, "    APP_CMD_START");
//             Log::Write(Log::Level::Info, "onStart()");
//             break;
//         }
//         case APP_CMD_RESUME: {
//             Log::Write(Log::Level::Info, "onResume()");
//             Log::Write(Log::Level::Info, "    APP_CMD_RESUME");
//             appState->Resumed = true;
//             break;
//         }
//         case APP_CMD_PAUSE: {
//             Log::Write(Log::Level::Info, "onPause()");
//             Log::Write(Log::Level::Info, "    APP_CMD_PAUSE");
//             appState->Resumed = false;
//             break;
//         }
//         case APP_CMD_STOP: {
//             Log::Write(Log::Level::Info, "onStop()");
//             Log::Write(Log::Level::Info, "    APP_CMD_STOP");
//             break;
//         }
//         case APP_CMD_DESTROY: {
//             Log::Write(Log::Level::Info, "onDestroy()");
//             Log::Write(Log::Level::Info, "    APP_CMD_DESTROY");
//             appState->NativeWindow = NULL;
//             break;
//         }
//         case APP_CMD_INIT_WINDOW: {
//             Log::Write(Log::Level::Info, "surfaceCreated()");
//             Log::Write(Log::Level::Info, "    APP_CMD_INIT_WINDOW");
//             appState->NativeWindow = app->window;
//             break;
//         }
//         case APP_CMD_TERM_WINDOW: {
//             Log::Write(Log::Level::Info, "surfaceDestroyed()");
//             Log::Write(Log::Level::Info, "    APP_CMD_TERM_WINDOW");
//             appState->NativeWindow = NULL;
//             break;
//         }
//     }
// }

/**
 * This is the main entry point of a native application that is using
 * android_native_app_glue.  It runs in its own thread, with its own
 * event loop for receiving input events and doing other things.
 */
//void android_main(struct android_app* app) {
void openxrInit(const RustCtx* rCtx) {
    try {
        if (rCtx == nullptr ||
            rCtx->initConnections == nullptr ||
            rCtx->legacySend == nullptr)
        {
            Log::Write(Log::Level::Error, "Rust context has not been setup!");
            return;
        }
        
        gRustCtx = std::make_shared<RustCtx>(*rCtx);
        const auto &ctx = *gRustCtx;//.load();

        std::shared_ptr<Options> options = std::make_shared<Options>();
        if (!UpdateOptionsFromSystemProperties(*options)) {
            return;
        }
        if (options->GraphicsPlugin.empty())
            options->GraphicsPlugin = graphics_api_str(rCtx->graphicsApi);

        Log::SetLevel(Log::Level::Verbose);
        
        std::shared_ptr<PlatformData> data = std::make_shared<PlatformData>();
        data->applicationVM = ctx.applicationVM; // app->activity->vm;
        data->applicationActivity = ctx.applicationActivity;//app->activity->clazz;

        // Initialize the loader for this platform
        PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
        if (XR_SUCCEEDED(
                xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                                      (PFN_xrVoidFunction *) (&initializeLoader)))) {
            XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid;
            memset(&loaderInitInfoAndroid, 0, sizeof(loaderInitInfoAndroid));
            loaderInitInfoAndroid.type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR;
            loaderInitInfoAndroid.next = NULL;
            loaderInitInfoAndroid.applicationVM = ctx.applicationVM; //app->activity->vm;
            loaderInitInfoAndroid.applicationContext = ctx.applicationActivity;//app->activity->clazz;
            initializeLoader((const XrLoaderInitInfoBaseHeaderKHR *) &loaderInitInfoAndroid);
        }

                // Create platform-specific implementation.
        std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin(options, data);
        // // // Create graphics API implementation.
        //  std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin(options,
        //                                                                         platformPlugin);

        // Initialize the OpenXR program.
        /*std::shared_ptr<IOpenXrProgram>*/ program = CreateOpenXrProgram(options, platformPlugin);//,
                                                                      //graphicsPlugin);
        
        program->CreateInstance();
        program->InitializeSystem();
        program->InitializeSession();
        program->CreateSwapchains();

        thread_local SystemProperties rustSysProp{};
        program->GetSystemProperties(rustSysProp);
        ctx.initConnections(&rustSysProp);
        Log::Write(Log::Level::Info, Fmt("device name: %s", rustSysProp.systemName));

        Log::Write(Log::Level::Info, "openxrInit finished successfully");
        //app->activity->vm->DetachCurrentThread();
    } catch (const std::exception& ex) {
        Log::Write(Log::Level::Error, ex.what());
    } catch (...) {
        Log::Write(Log::Level::Error, "Unknown Error");
    }
}

void openxrShutdown() {
    program = nullptr;
}

void openxrProcesFrame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */) {
    assert(exitRenderLoop != nullptr && requestRestart != nullptr);
    // while (app->destroyRequested == 0) {
    //     // Read all pending events.
    //     for (;;) {
    //         int events;
    //         struct android_poll_source* source;
    //         // If the timeout is zero, returns immediately without blocking.
    //         // If the timeout is negative, waits indefinitely until an event appears.
    //         const int timeoutMilliseconds =
    //             (!appState.Resumed && !program->IsSessionRunning() && app->destroyRequested == 0) ? -1 : 0;
    //         if (ALooper_pollAll(timeoutMilliseconds, nullptr, &events, (void**)&source) < 0) {
    //             break;
    //         }

    //         // Process this event.
    //         if (source != nullptr) {
    //             source->process(app, source);
    //         }
    //     }
    //    bool exitRenderLoop=false, requestRestart=false;
        program->PollEvents(exitRenderLoop, requestRestart);
        if (!program->IsSessionRunning()) {
            return;//continue;
        }
        
        program->PollActions();
        program->RenderFrame();

        TrackingInfo newInfo;
        program->GetTrackingInfo(newInfo);
        {
            std::unique_lock<std::shared_mutex> lock(gTrackingMutex);
            gLastTrackingInfo = newInfo;
        }
    //}
}

bool isOpenXRSessionRunning()
{
    return program && program->IsSessionRunning();
}

#else

int openxrMain(const RustCtx& ctx, int argc, const char* argv[]) {
    try {
        gRustCtx = std::make_shared<RustCtx>(ctx);

        // Parse command-line arguments into Options.
        std::shared_ptr<Options> options = std::make_shared<Options>();
        if (!UpdateOptionsFromCommandLine(*options, argc, argv)) {
            return 1;
        }

        std::shared_ptr<PlatformData> data = std::make_shared<PlatformData>();

        // Spawn a thread to wait for a keypress
        /*static*/ bool quitKeyPressed = false;
        //auto exitPollingThread = std::thread{[] {
        //    Log::Write(Log::Level::Info, "Press any key to shutdown...");
        //    (void)getchar();
        //    quitKeyPressed = true;
        //}};
        //exitPollingThread.detach();

        //ctx.initConnections();

        bool requestRestart = false;
        do {
            // Create platform-specific implementation.
            std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin(options, data);

            // Create graphics API implementation.
            //std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);

            // Initialize the OpenXR program.
            /*std::shared_ptr<IOpenXrProgram>*/ program = CreateOpenXrProgram(options, platformPlugin);//, graphicsPlugin);

            program->CreateInstance();
            program->InitializeSystem();
            program->InitializeSession();
            program->CreateSwapchains();

            SystemProperties rustSysProp{};
            program->GetSystemProperties(rustSysProp);            
            ctx.initConnections(&rustSysProp);

            while (!quitKeyPressed) {
                bool exitRenderLoop = false;
                program->PollEvents(&exitRenderLoop, &requestRestart);
                if (exitRenderLoop) {
                    break;
                }

                if (program->IsSessionRunning()) {
                    program->PollActions();
                    program->RenderFrame();

                    TrackingInfo newInfo;
                    program->GetTrackingInfo(newInfo);
                    {
                        std::unique_lock<std::shared_mutex> lock(gTrackingMutex);
                        gLastTrackingInfo = newInfo;
                    }

                } else {
                    // Throttle loop since xrWaitFrame won't be called.
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
            }

        } while (!quitKeyPressed && requestRestart);

        program = nullptr;

        return 0;
    } catch (const std::exception& ex) {
        Log::Write(Log::Level::Error, ex.what());
        return 1;
    } catch (...) {
        Log::Write(Log::Level::Error, "Unknown Error");
        return 1;
    }
}

void openxrMain(const RustCtx* ctx)
{
    if (ctx == nullptr ||
        ctx->initConnections == nullptr ||
        ctx->legacySend == nullptr)
    {
        Log::Write(Log::Level::Error, "Rust context has not been setup!");
        return;
    }
    std::vector<const char*> args
    {
        "openxrMain",
        "-g", graphics_api_str(ctx->graphicsApi),  //"D3D11", //"Vulkan2",//"Vulkan2",//"D3D11",
        "-vc", "Stereo",
        "-s", "Stage"
    };
    if (ctx->verbose)
        args.push_back("-v");
    openxrMain(*ctx, static_cast<int>(args.size()), args.data());
}
#endif

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
    const auto programPtr = program;
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
    const auto programPtr = program;
    if (programPtr == nullptr)
        return;

    programPtr->SetStreamConfig(config);
}
