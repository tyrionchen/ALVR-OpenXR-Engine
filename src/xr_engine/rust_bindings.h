#pragma once

#ifdef ENGINE_DLL_EXPORTS
    /*Enabled as "export" while compiling the dll project*/
    #define DLLEXPORT __declspec(dllexport)  
#else
    /*Enabled as "import" in the Client side for using already created dll file*/
    #define DLLEXPORT __declspec(dllimport)  
#endif

struct RustCtx
{
    void (*initConnections)();
    void (*legacySend)(const unsigned char* buffer, unsigned int size);
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

extern "C" DLLEXPORT void onTrackingNative(bool clientsidePrediction);
//extern "C" void onHapticsFeedbackNative(long long startTime, float amplitude, float duration, float frequency, unsigned char hand);
extern "C" DLLEXPORT GuardianData getGuardianData();

extern "C" DLLEXPORT void legacyReceive(const unsigned char* packet, unsigned int packetSize);
//extern "C" void sendTimeSync();

extern "C" DLLEXPORT void openxrMain(const RustCtx*);
