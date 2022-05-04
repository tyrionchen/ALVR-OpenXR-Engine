#include "decoderplugin.h"

#ifdef XR_USE_PLATFORM_ANDROID
std::shared_ptr<IDecoderPlugin> CreateDecoderPlugin_MediaCodec();
#else
std::shared_ptr<IDecoderPlugin> CreateDecoderPlugin_FFMPEG();
#endif

std::shared_ptr<IDecoderPlugin> CreateDecoderPlugin() {
#ifdef XR_USE_PLATFORM_ANDROID
	return CreateDecoderPlugin_MediaCodec();
#else
	return CreateDecoderPlugin_FFMPEG();
#endif
}
