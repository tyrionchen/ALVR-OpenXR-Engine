#include "surface_texture_wrapper.h"
#include "common.h"

inline jclass loadClz(JNIEnv * jni, jobject activity, const char* cStrClzName) {
    jobject obj_activity = activity;
	Log::Write(Log::Level::Info, Fmt("loadClz  obj_activity:%p", obj_activity));
    jclass clz_activity = jni->GetObjectClass(obj_activity);
	Log::Write(Log::Level::Info, Fmt("loadClz  clz_activity:%p", clz_activity));
    jmethodID method_getClassLoader = jni->GetMethodID(clz_activity, "getClassLoader", "()Ljava/lang/ClassLoader;");
	Log::Write(Log::Level::Info, Fmt("loadClz  method_getClassLoader:%p", method_getClassLoader));
    jobject obj_classLoader = jni->CallObjectMethod(obj_activity, method_getClassLoader);
	Log::Write(Log::Level::Info, Fmt("loadClz  obj_classLoader:%p", obj_classLoader));
    jclass classLoader = jni->FindClass("java/lang/ClassLoader");
	Log::Write(Log::Level::Info, Fmt("loadClz  classLoader:%p", classLoader));
    jmethodID findClass = jni->GetMethodID(classLoader, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
	Log::Write(Log::Level::Info, Fmt("loadClz  findClass:%p", findClass));
    jstring strClassName = jni->NewStringUTF(cStrClzName);
    jclass clz = (jclass)(jni->CallObjectMethod(obj_classLoader, findClass, strClassName));
	Log::Write(Log::Level::Info, Fmt("loadClz  clz:%p", clz));
    jni->DeleteLocalRef(strClassName);
    return clz;
}

SurfaceTextureWrapper::SurfaceTextureWrapper(JNIEnv * jniEnv, jobject activity, unsigned id) :
	javaObject( NULL ),
	activityObj(activity),
	jni( NULL ),
	updateTextureMethodId( NULL ),
	getSurfaceMethodId( NULL ),
	setDefaultBufferSizeMethodId( NULL )
{
	textureId = id;
	jni = jniEnv;
	
	static const char * className = "com/tencent/tcr/xr/OpenXrTextureEglRenderer";
	jclass openXrTextureEglRenderClass = jni->FindClass(className);
	if ( openXrTextureEglRenderClass == 0 ) {
		Log::Write(Log::Level::Info, Fmt("FindClass( %s ) failed", className));
		openXrTextureEglRenderClass = loadClz(jni, activityObj, className);
		Log::Write(Log::Level::Info, Fmt("FindClass again openXrTextureEglRenderClass:%p", openXrTextureEglRenderClass));
	}

	// find the constructor that takes an int
	const jmethodID constructor = jni->GetMethodID( openXrTextureEglRenderClass, "<init>", "(I)V" );
	if ( constructor == 0 ) {
		Log::Write(Log::Level::Info, "GetMethodID( <init> ) failed" );
	}

	jobject obj = jni->NewObject( openXrTextureEglRenderClass, constructor, textureId );
	if ( obj == 0 ) {
		Log::Write(Log::Level::Info, "NewObject() failed" );
	}

	javaObject = jni->NewGlobalRef( obj );
	if ( javaObject == 0 ) {
		Log::Write(Log::Level::Info, "NewGlobalRef() failed" );
	}

	// Now that we have a globalRef, we can free the localRef
	jni->DeleteLocalRef( obj );

    updateTextureMethodId = jni->GetMethodID( openXrTextureEglRenderClass, "updateTexture", "()J" );
    if ( !updateTextureMethodId ) {
    	Log::Write(Log::Level::Info, "couldn't get updateTextureMethodId" );
    }

    getSurfaceMethodId = jni->GetMethodID( openXrTextureEglRenderClass, "getSurface", "()Landroid/graphics/SurfaceTexture;" );
    if ( !getSurfaceMethodId ) {
    	Log::Write(Log::Level::Info, "couldn't get getTimestampMethodId" );
    }

	setDefaultBufferSizeMethodId = jni->GetMethodID( openXrTextureEglRenderClass, "setDefaultBufferSize", "(II)V" );
    if ( !setDefaultBufferSizeMethodId ) {
		Log::Write(Log::Level::Info, "couldn't get setDefaultBufferSizeMethodId" );
    }

	Log::Write(Log::Level::Info, Fmt("SurfaceTexture got method updateTextureMethodId:%p, getSurfaceMethodId:%p, setDefaultBufferSizeMethodId:%p",
		updateTextureMethodId, getSurfaceMethodId, setDefaultBufferSizeMethodId));

	// jclass objects are localRefs that need to be freed
	jni->DeleteLocalRef( openXrTextureEglRenderClass );
}

SurfaceTextureWrapper::~SurfaceTextureWrapper()
{
	Log::Write(Log::Level::Info, "SurfaceTexture::~SurfaceTexture()");

	if ( javaObject ) {
		jni->DeleteGlobalRef( javaObject );
		javaObject = 0;
	}
}

void SurfaceTextureWrapper::SetDefaultBufferSize( const int width, const int height ) {
	jni->CallVoidMethod( javaObject, setDefaultBufferSizeMethodId, width, height );
}

jobject	SurfaceTextureWrapper::GetSurfaceJavaObject() {
	return jni->CallObjectMethod(javaObject, getSurfaceMethodId);
}

// Java层返回的frameIndex需要处理一下
std::uint64_t SurfaceTextureWrapper::Update() { 
	return jni->CallLongMethod(javaObject, updateTextureMethodId);
}
