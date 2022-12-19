//
// Created by cyy on 2022/9/5.
//

#include "renderer.h"

namespace {
// The version statement has come on first line.
static const char* VertexShaderGlsl = R"_(#version 320 es
    in vec3 VertexPos;
    in vec3 TexturePos;
    uniform mat4 ModelViewProjection;
    out vec3 PSTexturePos;

    void main() {
       PSTexturePos = TexturePos;
       gl_Position = ModelViewProjection * vec4(VertexPos, 1.0);
    }
    )_";

// The version statement has come on first line.
static const char* FragmentShaderGlsl = R"_(#version 320 es
    #extension GL_OES_EGL_image_external_essl3 : require
    precision mediump float;

    in vec3 PSTexturePos;
    out vec4 FragColor;

    uniform samplerExternalOES OES_Texture;

    void main() {
       FragColor=texture(OES_Texture, PSTexturePos.xy);
    }
    )_";

struct Renderer : public IRenderer {
    Renderer(const std::array<float, 4> color, const std::array<XrVector3f, 4> texture_vertices)
        : m_clearColor(color), m_texture_vertices(texture_vertices) {}

    ~Renderer() override {
        if (m_program != 0) {
            glDeleteProgram(m_program);
        }
        if (m_vao != 0) {
            glDeleteVertexArrays(1, &m_vao);
        }
        if (m_rectVertexBuffer != 0) {
            glDeleteBuffers(1, &m_rectVertexBuffer);
        }
        if (m_rectIndecesBuffer != 0) {
            glDeleteBuffers(1, &m_rectIndecesBuffer);
        }
        if (m_textureVertexBuffer != 0) {
            glDeleteBuffers(1, &m_textureVertexBuffer);
        }
    }

    void InitializeResources() override {
        // 顶点着色器
        Log::Write(Log::Level::Info, Fmt("InitializeResources vertexShader"));
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &VertexShaderGlsl, nullptr);
        glCompileShader(vertexShader);

        CheckShader(vertexShader);

        // 像素着色器
        Log::Write(Log::Level::Info, Fmt("InitializeResources fragmentShader"));
        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &FragmentShaderGlsl, nullptr);
        glCompileShader(fragmentShader);
        CheckShader(fragmentShader);

        m_program = glCreateProgram();
        glAttachShader(m_program, vertexShader);
        glAttachShader(m_program, fragmentShader);
        glLinkProgram(m_program);
        CheckProgram(m_program);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        // 顶点坐标
        m_vertexPosition = glGetAttribLocation(m_program, "VertexPos");
        // 纹理坐标
        m_texturePosition = glGetAttribLocation(m_program, "TexturePos");
        // 纹理采样器
        m_texture_sampler = glGetUniformLocation(m_program, "OES_Texture");
        // 矩阵
        m_matrix_handle = glGetUniformLocation(m_program, "ModelViewProjection");

        glGenVertexArrays(1, &m_vao);
        glBindVertexArray(m_vao);

        // 图形顶点
        glGenBuffers(1, &m_rectVertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_rectVertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Geometry::c_vertices), Geometry::c_vertices, GL_STATIC_DRAW);

        // 指定顶点属性
        glEnableVertexAttribArray(m_vertexPosition);
        glVertexAttribPointer(m_vertexPosition, 3, GL_FLOAT, GL_FALSE, sizeof(XrVector3f), nullptr);

        // 索引数据
        glGenBuffers(1, &m_rectIndecesBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_rectIndecesBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Geometry::c_Indices), Geometry::c_Indices, GL_STATIC_DRAW);

        // 纹理顶点
        glGenBuffers(1, &m_textureVertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_textureVertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, m_texture_vertices.size() * sizeof(XrVector3f), &m_texture_vertices[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(m_texturePosition);
        glVertexAttribPointer(m_texturePosition, 3, GL_FLOAT, GL_FALSE, sizeof(XrVector3f), 0);
    }

    void setTextureId(GLuint textureId) override {
        m_texture_id = textureId;
        Log::Write(Log::Level::Info, Fmt("setTextureId m_texture_id:%d", m_texture_id));
    }

    void RenderView(const XrCompositionLayerProjectionView& layerView) override {
        glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
        glClearDepthf(1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        glUseProgram(m_program);

        glBindVertexArray(m_vao);

        glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_texture_id);
        glUniform1i(m_texture_sampler, 0);

        XrMatrix4x4f identity;
        XrMatrix4x4f_CreateIdentity(&identity);

        // const auto& pose = layerView.pose;
        // XrMatrix4x4f proj;
        // XrMatrix4x4f_CreateProjectionFov(&proj, GRAPHICS_OPENGL_ES, layerView.fov, 0.05f, 100.0f);
        // XrMatrix4x4f toView;
        // XrVector3f scale{1.f, 1.f, 1.f};
        // XrMatrix4x4f_CreateTranslationRotationScale(&toView, &pose.position, &pose.orientation, &scale);
        // XrMatrix4x4f view;
        // XrMatrix4x4f_InvertRigidBody(&view, &toView);
        // XrMatrix4x4f vp;
        // XrMatrix4x4f_Multiply(&vp, &proj, &view);

        glUniformMatrix4fv(m_matrix_handle, 1, GL_FALSE, reinterpret_cast<const GLfloat*>(&identity));

        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(ArraySize(Geometry::c_Indices)), GL_UNSIGNED_SHORT, nullptr);

        glBindVertexArray(0);
        glUseProgram(0);
    }

    void CheckShader(GLuint shader) {
        GLint r = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &r);
        if (r == GL_FALSE) {
            GLchar msg[4096] = {};
            GLsizei length;
            glGetShaderInfoLog(shader, sizeof(msg), &length, msg);
            THROW(Fmt("Compile shader failed: %s", msg));
        }
    }

    void CheckProgram(GLuint prog) {
        GLint r = 0;
        glGetProgramiv(prog, GL_LINK_STATUS, &r);
        if (r == GL_FALSE) {
            GLchar msg[4096] = {};
            GLsizei length;
            glGetProgramInfoLog(prog, sizeof(msg), &length, msg);
            THROW(Fmt("Link program failed: %s", msg));
        }
    }

    const std::array<float, 4> m_clearColor;
    const std::array<XrVector3f, 4> m_texture_vertices;
    GLint m_vertexPosition{0};
    GLint m_texturePosition{0};
    GLint m_matrix_handle{0};
    GLint m_texture_sampler{0};
    GLuint m_rectVertexBuffer{0};
    GLuint m_textureVertexBuffer{0};
    GLuint m_rectIndecesBuffer{0};
    GLuint m_texture_id{0};
    GLuint m_program{0};
    GLuint m_vao{0};
};

static const std::array<float, 4> SlateGrey{0.184313729f, 0.309803933f, 0.309803933f, 1.0f};
}  // namespace

std::shared_ptr<IRenderer> makeLeftEyeRenderer() { return std::make_shared<Renderer>(SlateGrey, Geometry::c_leftEyeVertices); }

std::shared_ptr<IRenderer> makeRightEyeRenderer() { return std::make_shared<Renderer>(SlateGrey, Geometry::c_rightEyeVertices); }
