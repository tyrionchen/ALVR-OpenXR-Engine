//
// Created by cyy on 2022/9/5.
//

#ifndef TCRXRSDK_RENDERER_H
#define TCRXRSDK_RENDERER_H


#include "pch.h"
#include "common.h"
#include "geometry.h"
#include "common/gfxwrapper_opengl.h"
#include <common/xr_linear.h>

struct IRenderer {
    virtual ~IRenderer() = default;
    virtual void InitializeResources() = 0;
    virtual void RenderView(const XrCompositionLayerProjectionView& layerView) = 0;
    virtual void setTextureId(GLuint textureId) = 0;
};

std::shared_ptr<IRenderer> makeLeftEyeRenderer();
std::shared_ptr<IRenderer> makeRightEyeRenderer();

#endif //TCRXRSDK_RENDERER_H
