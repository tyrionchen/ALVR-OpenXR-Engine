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

enum GraphicsCtxApi
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

enum TrackingSpace
{
    LocalRefSpace,
    StageRefSpace
};

struct SystemProperties
{
    char         systemName[256];
    float        currentRefreshRate;
    const float* refreshRates;
    unsigned int refreshRatesCount;
    unsigned int recommendedEyeWidth;
    unsigned int recommendedEyeHeight;
};

struct RustCtx
{
    void (*initConnections)(const SystemProperties*);
    void (*legacySend)(const unsigned char* buffer, unsigned int size);

    GraphicsCtxApi graphicsApi; // TODO: make this a part of StreamConfig structure and exposes available APIs in the server UI.
    bool verbose;
#ifdef XR_USE_PLATFORM_ANDROID
    void* applicationVM;
    void* applicationActivity;
#endif
};

struct GuardianData {
    bool shouldSync;
    float position[3];
    float rotation[4]; // convention: x, y, z, w
    float areaWidth;
    float areaHeight;
    float(*perimeterPoints)[3];
    unsigned int perimeterPointsCount;
};

struct StreamConfig {
    //unsigned int eyeWidth;
    //unsigned int eyeHeight;
    float refreshRate;
    //bool enableFoveation;
    //float foveationStrength;
    //float foveationShape;
    //float foveationVerticalOffset;
    TrackingSpace trackingSpaceType;
    //bool extraLatencyMode;
};

DLLEXPORT void onTrackingNative(bool clientsidePrediction);
//void onHapticsFeedbackNative(long long startTime, float amplitude, float duration, float frequency, unsigned char hand);
DLLEXPORT GuardianData getGuardianData();

DLLEXPORT void legacyReceive(const unsigned char* packet, unsigned int packetSize);

DLLEXPORT void setStreamConfig(StreamConfig config);

//void sendTimeSync();
#ifdef XR_USE_PLATFORM_ANDROID
DLLEXPORT void openxrInit(const RustCtx*);
DLLEXPORT void openxrShutdown();
DLLEXPORT void openxrProcesFrame(bool* exitRenderLoop /*= non-null */, bool* requestRestart /*= non-null */);
DLLEXPORT bool isOpenXRSessionRunning();
#else
DLLEXPORT void openxrMain(const RustCtx*);
#endif

#ifdef __cplusplus
}
#endif
