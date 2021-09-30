#pragma once

#ifdef XR_USE_PLATFORM_WIN32
#ifdef ENGINE_DLL_EXPORTS
    /*Enabled as "export" while compiling the dll project*/
    #define DLLEXPORT __declspec(dllexport)  
#else
    /*Enabled as "import" in the Client side for using already created dll file*/
    #define DLLEXPORT __declspec(dllimport)  
#endif
#else
#define DLLEXPORT
#endif

#ifdef __cplusplus
extern "C" {;
#endif

enum ALXRGraphicsApi
{
    Auto,
    Vulkan2,
    Vulkan,
    D3D12,
    D3D11,
    OpenGLES,
    OpenGL,
    ApiCount = OpenGL
};

enum ALXRTrackingSpace
{
    LocalRefSpace,
    StageRefSpace,
    ViewRefSpace
};

struct ALXRSystemProperties
{
    char         systemName[256];
    float        currentRefreshRate;
    const float* refreshRates;
    unsigned int refreshRatesCount;
    unsigned int recommendedEyeWidth;
    unsigned int recommendedEyeHeight;
};

struct ALXRRustCtx
{
    void (*legacySend)(const unsigned char* buffer, unsigned int size);

    ALXRGraphicsApi graphicsApi; // TODO: make this a part of StreamConfig structure and exposes available APIs in the server UI.
    bool verbose;
#ifdef XR_USE_PLATFORM_ANDROID
    void* applicationVM;
    void* applicationActivity;
#endif
};

struct ALXRGuardianData {
    bool shouldSync;
    float position[3];
    float rotation[4]; // convention: x, y, z, w
    float areaWidth;
    float areaHeight;
    const float (*perimeterPoints)[3];
    unsigned int perimeterPointsCount;
};

struct ALXRStreamConfig {
    //unsigned int eyeWidth;
    //unsigned int eyeHeight;
    float refreshRate;
    //bool enableFoveation;
    //float foveationStrength;
    //float foveationShape;
    //float foveationVerticalOffset;
    ALXRTrackingSpace trackingSpaceType;
    //bool extraLatencyMode;
};

//void sendTimeSync();
DLLEXPORT bool alxr_init(const ALXRRustCtx*, /*[out]*/ ALXRSystemProperties* systemProperties);
DLLEXPORT void alxr_destroy();
DLLEXPORT void alxr_request_exit_session();
DLLEXPORT void alxr_process_frame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */);
DLLEXPORT bool alxr_is_session_running();

DLLEXPORT void alxr_set_stream_config(ALXRStreamConfig config);
DLLEXPORT ALXRGuardianData alxr_get_guardian_data();

DLLEXPORT void alxr_on_receive(const unsigned char* packet, unsigned int packetSize);
DLLEXPORT void alxr_on_tracking_update(bool clientsidePrediction);

#ifdef __cplusplus
}
#endif
