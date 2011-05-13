#include "OpenGL.h"
#include "VolumeRenderer.h"
#include <cml/cml.h>
#include <QMessageBox>
#include <QApplication>

using namespace cml;

// --------------------------------------------------------------------------

VolumeRenderer::VolumeRenderer()
{
    m_initialized = false;
    m_rayCastShader = 0;

    for (int i = 0; i < k_volumes; ++i)
        m_volumes[i] = 0;

    m_physicalScale = 100.f;
    m_textureScale  = vector3f(1.f, 1.f, 1.f);
    m_volumeCenter  = vector3f(.5f, .5f, .5f);

    m_rayStep       = 0.005f;
    m_gradientDelta = 1.f / 128.f;
}

VolumeRenderer::~VolumeRenderer()
{
//    for (int i = 0; i < smSentinel; ++i)
//        if (m_shaders[i]) delete m_shaders[i];
}

// --------------------------------------------------------------------------

void VolumeRenderer::initialize()
{
    if (m_initialized) return;
    //    QString appDir = QApplication::applicationDirPath();

    // create a ray casting shader
    QString shaderFile[smSentinel];
    shaderFile[smRayCast]       = ":/Shaders/RayCast.frag";
    shaderFile[smRayCastLabels] = ":/Shaders/RayCastLabels.frag";

    for (int i = 0; i < smSentinel; ++i)
    {
        ShaderProgram *shader = m_shaders[i] = new ShaderProgram();
        if (!shader->addShaderFromSourceFile(QGLShader::Fragment, shaderFile[i]))
            QMessageBox::critical(0, "OpenGL Shader Compile Error", shader->log());
        if (!shader->link())
            QMessageBox::critical(0, "OpenGL Shader Link Error", shader->log());
    }

    // create a 3D texture to contain the volume data and transfer functions
    glGenTextures(k_volumes, m_volumeTextures);
    glGenTextures(k_volumes, m_transferTextures);
    for (int i = vrPrimary; i < vrSentinel; ++i)
    {
        // initialize volume textures with clamp to border mode, so that any
        // indexing outside the volume will return transparent/black
        glBindTexture(GL_TEXTURE_3D, m_volumeTextures[i]);
        if (m_volumes[i]) uploadVolumeTexture(VolumeRole(i));

        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);

        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // initialize transfer functions with linear mip-map filtering to
        // emulate a 2D pre-integrated transfer function lookup
        //  TODO: Labels volume will need special transfer function(s)...
        //        (do not know what those would look like yet)
        glBindTexture(GL_TEXTURE_1D, m_transferTextures[i]);
        if (!m_transferFunctions[i].markers.empty())
            uploadTransferFunction(VolumeRole(i));

        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    // set initialized flag so we don't repeat
    m_initialized = true;
}

// --------------------------------------------------------------------------

void VolumeRenderer::render(ProxyGeometry *proxy, GLuint distanceTexture)
{
    glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_COLOR_BUFFER_BIT);

    // set up OpenGL state for casting the rays
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    // set up blending for premultiplied alpha
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    // bind all relevant textures
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vrPrimary]);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_1D, m_transferTextures[vrPrimary]);

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vrLabels]);
    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_1D, m_transferTextures[vrLabels]);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glActiveTexture(GL_TEXTURE7);
    glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vrMask]);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, distanceTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // select the shader to use based on the presence of a label volume
    if (m_volumes[vrLabels] && m_volumes[vrLabels]->data)
        m_rayCastShader = m_shaders[smRayCastLabels];
    else
        m_rayCastShader = m_shaders[smRayCast];

    // set up uniform variables on the ray casting shader
    m_rayCastShader->bind();
    m_rayCastShader->setUniformValue("g_volumePrimary", 1);
    m_rayCastShader->setUniformValue("g_transferPrimary", 2);

    m_rayCastShader->setUniformValue("g_volumeSecondary", 3);
    m_rayCastShader->setUniformValue("g_transferSecondary", 4);

    m_rayCastShader->setUniformValue("g_volumeLabels", 5);
    m_rayCastShader->setUniformValue("g_transferLabels", 6);

    m_rayCastShader->setUniformValue("g_volumeMask", 7);

    m_rayCastShader->setUniformValue("g_isosurfaceValue", m_isosurfaceValue);
    m_rayCastShader->setUniformValue("g_isosurfaceColor", m_isosurfaceColor[0],
                                                          m_isosurfaceColor[1],
                                                          m_isosurfaceColor[2],
                                                          m_isosurfaceColor[3]);

    m_rayCastShader->setUniformValue("g_distances", 0);
    m_rayCastShader->setUniformValue("g_step", m_rayStep);
    m_rayCastShader->setUniformValue("g_delta", m_gradientDelta);

    m_rayCastShader->setUniformValue("g_textureScale", m_textureScale[0],
                                                       m_textureScale[1],
                                                       m_textureScale[2]);

    vector3f view = currentViewPosition();
    m_rayCastShader->setUniformValue("g_viewer", view[0], view[1], view[2]);

    vector4f viewport;
    glGetFloatv(GL_VIEWPORT, viewport.data());
    m_rayCastShader->setUniformValue("g_viewport", viewport[0], viewport[1], viewport[2], viewport[3]);

    // render the proxy geometry back faces to determine ray direction
    proxy->render();

    m_rayCastShader->release();

    glPopAttrib();
}

// --------------------------------------------------------------------------

vector3f VolumeRenderer::currentViewPosition()
{
    // retrieve the current modelview matrix
    matrix44f_c modelviewMatrix;
    glGetFloatv(GL_MODELVIEW_MATRIX, modelviewMatrix.data());

    // apply the inverse transform to the original to obtain eye position
    return transform_point(modelviewMatrix.inverse(), vector3f(0, 0, 0));
}

// --------------------------------------------------------------------------

void VolumeRenderer::uploadVolumeTexture(VolumeRole vr, int z0, int z1)
{
    Volume *v = m_volumes[vr];
    if (v && v->data)
    {
        // figure out OpenGL storage parameters based on volume data type
        GLenum iformat, format = GL_LUMINANCE, type;
        switch(v->format) {
        case Volume::pfInt16:
            iformat = GL_LUMINANCE16;
            type    = GL_SHORT;
            break;
        case Volume::pfUInt16:
            iformat = GL_LUMINANCE16;
            type    = GL_UNSIGNED_SHORT;
            break;
        case Volume::pfUInt8:
        default:
            iformat = GL_LUMINANCE8;
            type    = GL_UNSIGNED_BYTE;
            break;
        }

        // remember pixel transfer values to restore later
        glPushAttrib(GL_PIXEL_MODE_BIT);

        // shift the range for signed types so that it still fits in [0,1]
        if (type == GL_BYTE || type == GL_SHORT) {
            glPixelTransferf(GL_RED_SCALE, 0.5f);
            glPixelTransferf(GL_RED_BIAS, 0.5f);
        }

        // TODO: this hack somewhat reliably detects CT Hounsfield Unit images
        //       and scales the intensities to better fill the range
        if (v->format == Volume::pfInt16 &&
            v->histogram.minValue > -4096 &&
            v->histogram.maxValue < 4096)
        {
            // effectively restrict the range to the 12 bits usually stored,
            // putting -1024HU at 0.00, 0HU at 0.25, and 3072HU at 1.00
            glPixelTransferf(GL_RED_SCALE, 8.f);
            glPixelTransferf(GL_RED_BIAS, 0.25f);
        }

        if (vr == vrPrimary)
        {
            // calculate a texture size based on the physical dimensions of the
            // volume so that its maximum extent fits in the range [0,1]
            vector3f physical = v->physical();
            int ix = index_of_max(physical[0], physical[1], physical[2]);

            m_textureScale = vector3f(physical[ix]/physical[0],
                                      physical[ix]/physical[1],
                                      physical[ix]/physical[2]);

            m_physicalScale = physical[ix];

            // set coordinates of the center of the volume
            m_volumeCenter = physical / (2.f * physical[ix]);
        }

        // set byte unpack alignment, in case dimensions aren't nice and "round"
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // send the volume data to the graphics board
        glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vr]);
        if (z0 < 0) {
            glTexImage3D(GL_TEXTURE_3D, 0, iformat, v->dimensions[0], v->dimensions[1],
                         v->dimensions[2], 0, format, type, v->data);
        }
        else {
            // this is an update of a slab of the volume
            int bytes = v->format == Volume::pfUInt8 ? 1 : 2;
            int offset = z0 * v->dimensions[0] * v->dimensions[1] * bytes;
            int h = z1 - z0 + 1;
            const unsigned char *data = reinterpret_cast<const unsigned char *>(v->data);
            glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, z0, v->dimensions[0],
                            v->dimensions[1], h, format, type, data + offset);
        }

        glPopAttrib();
    }
    else
    {
        // upload a single, empty voxel
        int black = 0;
        glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vr]);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_LUMINANCE8, 1, 1, 1, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, &black);

        // re-center if setting primary volume
        if (vr == vrPrimary) m_volumeCenter = vector3f(.5f, .5f, .5f);
    }
}

void VolumeRenderer::uploadTransferFunction(VolumeRole vr)
{
    TransferFunction &tf = m_transferFunctions[vr];
    tf.computeRGBA8(m_transferBytes, k_tfTextureSize);

    glBindTexture(GL_TEXTURE_1D, m_transferTextures[vr]);
    gluBuild1DMipmaps(GL_TEXTURE_1D, GL_RGBA8, k_tfTextureSize,
                      GL_RGBA, GL_UNSIGNED_BYTE, m_transferBytes);
}

// --------------------------------------------------------------------------

void VolumeRenderer::resetMask()
{
    // set the mask volume data back to full intensity and re-upload to GPU
    // (assumes haptic interaction is paused)
    Volume *m = m_volumes[vrMask];
    vector3i d = m->dimensions;
    memset(m->data, 0xff, d[0]*d[1]*d[2]);
    m->modified = false;
    uploadVolumeTexture(vrMask);
}

// --------------------------------------------------------------------------

void VolumeRenderer::setLabels(Volume *v, Volume *m, Labelling *lab)
{
    // ensure that the volume and mask are both Uint8 format
    if (v->format != Volume::pfUInt8 || m->format != Volume::pfUInt8)
        return;

    m_volumes[vrLabels] = v;
    if (m_initialized)
    {
        // create a buffer that interleaves the volume and the mask
        int voxels = v->dimensions[0] * v->dimensions[1] * v->dimensions[2];
        unsigned char *buffer = new unsigned char [voxels*2];
        unsigned char *d = buffer;
        const unsigned char *p = reinterpret_cast<const unsigned char *>(v->data);
        const unsigned char *q = reinterpret_cast<const unsigned char *>(m->data);
        for (int i = 0; i < voxels; ++i) {
            *d++ = *p++;
            *d++ = *q++;
        }

        // upload the combined volume as a luminance-alpha texture

        // remember pixel transfer values to restore later
        glPushAttrib(GL_PIXEL_MODE_BIT);

        // set byte unpack alignment, in case dimensions aren't nice and "round"
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // send the volume data to the graphics board
        glBindTexture(GL_TEXTURE_3D, m_volumeTextures[vrLabels]);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_LUMINANCE8_ALPHA8, v->dimensions[0], v->dimensions[1],
                     v->dimensions[2], 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, buffer);

        glPopAttrib();
        delete [] buffer;

        // now upload the colour LUT from the labelling

        glBindTexture(GL_TEXTURE_1D, m_transferTextures[vrLabels]);

        // first "clear" the texture contents
        unsigned char data[1024] = { 0 };
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

        // then upload the new look up table
		if (lab->entries() > 0)
			glTexSubImage1D(GL_TEXTURE_1D, 0, 1, lab->entries(), GL_RGBA, GL_FLOAT, &lab->colourLUT[0]);
    }
}

// --------------------------------------------------------------------------

