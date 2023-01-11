#ifndef SurfaceTexture_h
#define SurfaceTexture_h

#include "pch.h"

class SurfaceTexture
{
public:
							SurfaceTexture(JNIEnv * jni, unsigned textureId);
							~SurfaceTexture();

	// For some java-side uses, you can set the size
	// of the buffer before it is used to control how
	// large it is.  Video decompression and camera preview
	// always override the size automatically.
	void					SetDefaultBufferSize( const int width, const int height );

	// This can only be called with an active GL context.
	// As a side effect, the textureId will be bound to the
	// GL_TEXTURE_EXTERNAL_OES target of the currently active
	// texture unit.
	void 					Update();

	jobject					GetJavaObject();
    jobject					GetJavaObjectSurface();
	long long				GetNanoTimeStamp();
	ANativeWindow* 			GetANativeWindow();

private:
	unsigned				textureId;
	jobject					javaObject;
    jobject					javaObjectSurface;
	JNIEnv * 				jni;
	// Updated when Update() is called, can be used to
	// check if a new frame is available and ready
	// to be processed / mipmapped by other code.
	long long				nanoTimeStamp;

	jmethodID 				updateTexImageMethodId;
	jmethodID 				getTimestampMethodId;
	jmethodID 				setDefaultBufferSizeMethodId;
	ANativeWindow* 			nativeWindow;
};
#endif	// SurfaceTexture_h