#include "surface_texture.h"
#include "common.h"

SurfaceTexture::SurfaceTexture(JNIEnv * jniEnv, unsigned id) :
	javaObject( NULL ),
	jni( NULL ),
	nanoTimeStamp( 0 ),
	updateTexImageMethodId( NULL ),
	getTimestampMethodId( NULL ),
	setDefaultBufferSizeMethodId( NULL )
{
	textureId = id;
	jni = jniEnv;

	static const char * className = "android/graphics/SurfaceTexture";
	const jclass surfaceTextureClass = jni->FindClass(className);
	if ( surfaceTextureClass == 0 ) {
		Log::Write(Log::Level::Info, Fmt("FindClass( %s ) failed", className));
	}

	// find the constructor that takes an int
	const jmethodID constructor = jni->GetMethodID( surfaceTextureClass, "<init>", "(I)V" );
	if ( constructor == 0 ) {
		Log::Write(Log::Level::Info, "GetMethodID( <init> ) failed" );
	}

	jobject obj = jni->NewObject( surfaceTextureClass, constructor, textureId );
	if ( obj == 0 ) {
		Log::Write(Log::Level::Info, "NewObject() failed" );
	}

	javaObject = jni->NewGlobalRef( obj );
	if ( javaObject == 0 ) {
		Log::Write(Log::Level::Info, "NewGlobalRef() failed" );
	}

	// Now that we have a globalRef, we can free the localRef
	jni->DeleteLocalRef( obj );

    updateTexImageMethodId = jni->GetMethodID( surfaceTextureClass, "updateTexImage", "()V" );
    if ( !updateTexImageMethodId ) {
    	Log::Write(Log::Level::Info, "couldn't get updateTexImageMethodId" );
    }

    getTimestampMethodId = jni->GetMethodID( surfaceTextureClass, "getTimestamp", "()J" );
    if ( !getTimestampMethodId ) {
    	Log::Write(Log::Level::Info, "couldn't get getTimestampMethodId" );
    }

	setDefaultBufferSizeMethodId = jni->GetMethodID( surfaceTextureClass, "setDefaultBufferSize", "(II)V" );
    if ( !setDefaultBufferSizeMethodId ) {
		Log::Write(Log::Level::Info, "couldn't get setDefaultBufferSize" );
    }

	static const char * surfaceClassName = "android/view/Surface";
	const jclass surfaceClass = jni->FindClass(surfaceClassName);
	if ( surfaceClass == 0 ) {
		Log::Write(Log::Level::Info, Fmt("FindClass( %s ) failed", surfaceClassName));
	}
	const jmethodID surface_constructor = jni->GetMethodID( surfaceClass, "<init>", "(Landroid/graphics/SurfaceTexture;)V" );
	if ( surface_constructor == 0 ) {
		Log::Write(Log::Level::Info, "GetMethodID suraface( <init> ) failed" );
	}

	javaObjectSurface = jni->NewObject( surfaceClass, surface_constructor, javaObject );

	Log::Write(Log::Level::Info, Fmt("SurfaceTexture got method updateTexImageMethodId:%p, getTimestampMethodId:%p, setDefaultBufferSizeMethodId:%p javaObjectSurface:%p",
		updateTexImageMethodId, getTimestampMethodId, setDefaultBufferSizeMethodId, javaObjectSurface));

	// jclass objects are localRefs that need to be freed
	jni->DeleteLocalRef( surfaceTextureClass );
}

SurfaceTexture::~SurfaceTexture()
{
	Log::Write(Log::Level::Info, "SurfaceTexture::~SurfaceTexture()");

	if ( javaObject ) {
		jni->DeleteGlobalRef( javaObject );
		javaObject = 0;
	}
}

void SurfaceTexture::SetDefaultBufferSize( const int width, const int height )
{
	jni->CallVoidMethod( javaObject, setDefaultBufferSizeMethodId, width, height );
}

void SurfaceTexture::Update()
{
    // latch the latest movie frame to the texture
    if ( !javaObject ) {
		Log::Write(Log::Level::Error, "SurfaceTexture::Update javaObject==0" );
    	return;
    }

   jni->CallVoidMethod( javaObject, updateTexImageMethodId );
   nanoTimeStamp = jni->CallLongMethod( javaObject, getTimestampMethodId );
   
   Log::Write(Log::Level::Error, Fmt("SurfaceTexture::Update nanoTimeStamp:%lld", nanoTimeStamp));
}

jobject SurfaceTexture::GetJavaObject()
{
	return javaObject;
}

jobject SurfaceTexture::GetJavaObjectSurface()
{
	return javaObjectSurface;
}

long long SurfaceTexture::GetNanoTimeStamp()
{
	return nanoTimeStamp;
}
