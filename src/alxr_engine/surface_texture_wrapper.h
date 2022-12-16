#ifndef SurfaceTexture_h
#define SurfaceTexture_h

#include "pch.h"

class SurfaceTextureWrapper
{
public:
							SurfaceTextureWrapper(JNIEnv * jni, jobject activityObj, unsigned textureId);
							~SurfaceTextureWrapper();

	void					SetDefaultBufferSize( const int width, const int height );


	std::uint64_t 		    Update();
	jobject					GetSurfaceJavaObject();

private:
	unsigned				textureId;
	jobject					javaObject;
	jobject 				activityObj;

	JNIEnv * 				jni;

	jmethodID 				updateTextureMethodId;
	jmethodID 				getSurfaceMethodId;
	jmethodID 				setDefaultBufferSizeMethodId;
};
#endif	// SurfaceTexture_h