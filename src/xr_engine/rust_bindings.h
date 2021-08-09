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
#include <cstdint>
using std::uint32_t;
extern "C" {;
#else
#include <stdint.h>
#endif

struct SystemProperties
{
    char     systemName[256];
    uint32_t recommendedEyeWidth;
    uint32_t recommendedEyeHeight;
};

struct RustCtx
{
    void (*initConnections)(const SystemProperties*);
    void (*legacySend)(const unsigned char* buffer, unsigned int size);
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

//struct StreamConfig {
//    unsigned int eyeWidth;
//    unsigned int eyeHeight;
//    float refreshRate;
//    bool enableFoveation;
//    float foveationStrength;
//    float foveationShape;
//    float foveationVerticalOffset;
//    int trackingSpaceType;
//    bool extraLatencyMode;
//};

//struct SystemProperties
//{
//    char systemName[256];
//};
//DLLEXPORT SystemProperties getSystemProperties();

DLLEXPORT void onTrackingNative(bool clientsidePrediction);
//void onHapticsFeedbackNative(long long startTime, float amplitude, float duration, float frequency, unsigned char hand);
DLLEXPORT GuardianData getGuardianData();

DLLEXPORT void legacyReceive(const unsigned char* packet, unsigned int packetSize);
//void sendTimeSync();
#ifdef XR_USE_PLATFORM_ANDROID
DLLEXPORT void openxrInit(const RustCtx*);
DLLEXPORT void openxrProcesFrame();
DLLEXPORT bool isOpenXRSessionRunning();
#else
DLLEXPORT void openxrMain(const RustCtx*);
#endif

#ifdef __cplusplus
}
#endif
