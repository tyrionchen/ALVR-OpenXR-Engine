// Copyright (c) 2017-2022, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "pch.h"
#include "common.h"
#include "geometry.h"
#include "graphicsplugin.h"
#include "options.h"
#include "renderer.h"
#ifdef XR_USE_GRAPHICS_API_OPENGL_ES

#include "common/gfxwrapper_opengl.h"
#include <common/xr_linear.h>


namespace {

struct OpenGLESGraphicsPlugin2 : public IGraphicsPlugin {
    OpenGLESGraphicsPlugin2(const std::shared_ptr<Options> /*unused*/&, const std::shared_ptr<IPlatformPlugin> /*unused*/&) {}

    OpenGLESGraphicsPlugin2(const OpenGLESGraphicsPlugin2&) = delete;
    OpenGLESGraphicsPlugin2& operator=(const OpenGLESGraphicsPlugin2&) = delete;
    OpenGLESGraphicsPlugin2(OpenGLESGraphicsPlugin2&&) = delete;
    OpenGLESGraphicsPlugin2& operator=(OpenGLESGraphicsPlugin2&&) = delete;

    ~OpenGLESGraphicsPlugin2() override {
        if (m_swapchainFramebuffer != 0) {
            glDeleteFramebuffers(1, &m_swapchainFramebuffer);
        }
        ksGpuWindow_Destroy(&window);
    }

    std::vector<std::string> GetInstanceExtensions() const override { return {XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME}; }

    ksGpuWindow window{};

    void DebugMessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message) {
        (void)source;
        (void)type;
        (void)id;
        (void)severity;
        Log::Write(Log::Level::Info, "GLES Debug: " + std::string(message, 0, length));
    }

    void InitializeDevice(XrInstance instance, XrSystemId systemId, const XrEnvironmentBlendMode newMode) override {
        UNUSED_PARM(newMode);
        
        // Extension function must be loaded by name
        PFN_xrGetOpenGLESGraphicsRequirementsKHR pfnGetOpenGLESGraphicsRequirementsKHR = nullptr;
        CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetOpenGLESGraphicsRequirementsKHR",
                                          reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetOpenGLESGraphicsRequirementsKHR)));

        XrGraphicsRequirementsOpenGLESKHR graphicsRequirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
        CHECK_XRCMD(pfnGetOpenGLESGraphicsRequirementsKHR(instance, systemId, &graphicsRequirements));

        // Initialize the gl extensions. Note we have to open a window.
        ksDriverInstance driverInstance{};
        ksGpuQueueInfo queueInfo{};
        ksGpuSurfaceColorFormat colorFormat{KS_GPU_SURFACE_COLOR_FORMAT_B8G8R8A8};
        ksGpuSurfaceDepthFormat depthFormat{KS_GPU_SURFACE_DEPTH_FORMAT_D24};
        ksGpuSampleCount sampleCount{KS_GPU_SAMPLE_COUNT_1};
        if (!ksGpuWindow_Create(&window, &driverInstance, &queueInfo, 0, colorFormat, depthFormat, sampleCount, 640, 480, false)) {
            THROW("Unable to create GL context");
        }

        GLint major = 0;
        GLint minor = 0;
        glGetIntegerv(GL_MAJOR_VERSION, &major);
        glGetIntegerv(GL_MINOR_VERSION, &minor);

        const XrVersion desiredApiVersion = XR_MAKE_VERSION(major, minor, 0);
        if (graphicsRequirements.minApiVersionSupported > desiredApiVersion) {
            THROW("Runtime does not support desired Graphics API and/or version");
        }

        m_contextApiMajorVersion = major;

#if defined(XR_USE_PLATFORM_ANDROID)
        m_graphicsBinding.display = window.display;
        m_graphicsBinding.config = (EGLConfig)0;
        m_graphicsBinding.context = window.context.context;
#endif

        glEnable(GL_DEBUG_OUTPUT);
        glDebugMessageCallback(
            [](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message,
               const void* userParam) {
                ((OpenGLESGraphicsPlugin2*)userParam)->DebugMessageCallback(source, type, id, severity, length, message);
            },
            this);
        InitializeResources();
    }

    void InitializeResources() {
        glGenFramebuffers(1, &m_swapchainFramebuffer);
        m_leftEyeRenderer = makeLeftEyeRenderer();
        m_rightEyeRenderer = makeRightEyeRenderer();
        m_leftEyeRenderer->InitializeResources();
        m_rightEyeRenderer->InitializeResources();
    }

    int64_t SelectColorSwapchainFormat(const std::vector<int64_t>& runtimeFormats) const override {
        // List of supported color swapchain formats.
        std::vector<int64_t> supportedColorSwapchainFormats{GL_RGBA8, GL_RGBA8_SNORM};

        // In OpenGLES 3.0+, the R, G, and B values after blending are converted into the non-linear
        // sRGB automatically.
        if (m_contextApiMajorVersion >= 3) {
            supportedColorSwapchainFormats.push_back(GL_SRGB8_ALPHA8);
        }

        auto swapchainFormatIt = std::find_first_of(runtimeFormats.begin(), runtimeFormats.end(),
                                                    supportedColorSwapchainFormats.begin(), supportedColorSwapchainFormats.end());
        if (swapchainFormatIt == runtimeFormats.end()) {
            THROW("No runtime swapchain format supported for color swapchain");
        }

        return *swapchainFormatIt;
    }

    const XrBaseInStructure* GetGraphicsBinding() const override {
        return reinterpret_cast<const XrBaseInStructure*>(&m_graphicsBinding);
    }

    unsigned getTextureId() const override { 
        return m_texture_id; 
    }

    virtual std::uint64_t GetVideoFrameIndex() const override { 
        if (!m_SurfaceTextureWrapper) {
            return 0;
        }
        return m_SurfaceTextureWrapper->Update();
    }

    void setSurfaceTexture(std::shared_ptr<SurfaceTextureWrapper>& texture) override { m_SurfaceTextureWrapper = texture; }

    std::vector<XrSwapchainImageBaseHeader*> AllocateSwapchainImageStructs(
        uint32_t capacity, const XrSwapchainCreateInfo& /*swapchainCreateInfo*/) override {
        // Allocate and initialize the buffer of image structs (must be sequential in memory for xrEnumerateSwapchainImages).
        // Return back an array of pointers to each swapchain image struct so the consumer doesn't need to know the type/size.
        std::vector<XrSwapchainImageOpenGLESKHR> swapchainImageBuffer(capacity);
        std::vector<XrSwapchainImageBaseHeader*> swapchainImageBase;
        for (XrSwapchainImageOpenGLESKHR& image : swapchainImageBuffer) {
            image.type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR;
            swapchainImageBase.push_back(reinterpret_cast<XrSwapchainImageBaseHeader*>(&image));
        }

        // Keep the buffer alive by moving it into the list of buffers.
        m_swapchainImageBuffers.push_back(std::move(swapchainImageBuffer));

        return swapchainImageBase;
    }

    void RenderView(const XrCompositionLayerProjectionView& layerView, const XrSwapchainImageBaseHeader* swapchainImage,
                    const std::int64_t swapchainFormat, const PassthroughMode /*newMode*/,
                    const std::vector<Cube>& cubes) override {
        UNUSED_PARM(layerView);        // Not used in this function for now.
        UNUSED_PARM(swapchainImage);   // Not used in this function for now.
        UNUSED_PARM(swapchainFormat);  // Not used in this function for now.
        UNUSED_PARM(cubes);            // Not used in this function for now.
    }

    void RenderView(uint32_t viewIndex, const XrCompositionLayerProjectionView& layerView,
                    const XrSwapchainImageBaseHeader* swapchainImage, int64_t swapchainFormat) override {
        CHECK(layerView.subImage.imageArrayIndex == 0);  // Texture arrays not supported.
        UNUSED_PARM(swapchainFormat);                    // Not used in this function for now.

        if (m_texture_id == 0) {
            glGenTextures(1, &m_texture_id);
            m_leftEyeRenderer->setTextureId(m_texture_id);
            m_rightEyeRenderer->setTextureId(m_texture_id);
            Log::Write(Log::Level::Info, Fmt("cyyyyy generate m_texture_id:%d", m_texture_id));
        }

        glBindFramebuffer(GL_FRAMEBUFFER, m_swapchainFramebuffer);

        const uint32_t colorTexture = reinterpret_cast<const XrSwapchainImageOpenGLESKHR*>(swapchainImage)->image;

        glViewport(static_cast<GLint>(layerView.subImage.imageRect.offset.x),
                   static_cast<GLint>(layerView.subImage.imageRect.offset.y),
                   static_cast<GLsizei>(layerView.subImage.imageRect.extent.width),
                   static_cast<GLsizei>(layerView.subImage.imageRect.extent.height));

        // 当把纹理添加到帧缓冲的时候, 所有的渲染操作会直接写到纹理colorTexture里面
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);

        glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_texture_id);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        if (viewIndex == 0) {
            m_leftEyeRenderer->RenderView(layerView);
        } else {
            m_rightEyeRenderer->RenderView(layerView);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    uint32_t GetSupportedSwapchainSampleCount(const XrViewConfigurationView&) override { return 1; }

    inline void SetEnvironmentBlendMode(const XrEnvironmentBlendMode newMode) { UNUSED_PARM(newMode); }

   private:
#ifdef XR_USE_PLATFORM_ANDROID
    XrGraphicsBindingOpenGLESAndroidKHR m_graphicsBinding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
#endif

    std::list<std::vector<XrSwapchainImageOpenGLESKHR>> m_swapchainImageBuffers;

    GLint m_contextApiMajorVersion{0};
    GLuint m_swapchainFramebuffer{0};
    // 接收播放器Image Stream的纹理
    GLuint m_texture_id{0};  // TODO 退出记得销毁

    std::shared_ptr<IRenderer> m_leftEyeRenderer;
    std::shared_ptr<IRenderer> m_rightEyeRenderer;
    std::shared_ptr<SurfaceTextureWrapper> m_SurfaceTextureWrapper;
};
}  // namespace

std::shared_ptr<IGraphicsPlugin> CreateGraphicsPlugin_OpenGLES2(const std::shared_ptr<Options>& options,
                                                                std::shared_ptr<IPlatformPlugin> platformPlugin) {
    return std::make_shared<OpenGLESGraphicsPlugin2>(options, platformPlugin);
}

#endif