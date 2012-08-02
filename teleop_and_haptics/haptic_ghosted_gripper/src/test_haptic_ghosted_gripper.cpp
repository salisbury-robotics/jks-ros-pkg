
//#include "haptic_ghosted_gripper.h"
#include <haptic_display_tools/Common/PointSampler.h>

void main()
{
    // initialize the scene with our own private test volume instance
    Volume *volume = m_data->getVolume();
    Volume *mask = m_data->getMask();
   // m_sampler = new VolumeSampler(volume, mask);

  m_sampler = new PointSampler();

    // set up the visual renderer
    m_renderer->initialize();
    m_renderer->setVolume(volume, VolumeRenderer::vrPrimary);
    m_renderer->setVolume(mask, VolumeRenderer::vrMask);
    m_renderer->setIsosurfaceValue(0.5);
    m_renderer->setIsosurfaceColor(colour(1.0, 1.0, 0.5, 0.5));


    // create a haptics scene with all the haptic displays in it
    m_scene = new HapticScene();

    if (m_devicesModel)
    {
        int n = m_devicesModel->rowCount();
        for (int i = 0; i < n; ++i) {
            cGenericHapticDevice *device =
                m_devicesModel->data(m_devicesModel->index(i, 2))
                    .value<cGenericHapticDevice *>();
            if (device)
            {
                HapticDisplay *display = m_scene->addHapticDisplay(device);
                m_displays.append(display);

                // center the device at (.5, .5, .5) where the volume will be
                matrix44f_c center, orient = identity_4x4();
                matrix_translation(center, -.5f, -.75f, -.5f);
                matrix_set_basis_vectors(orient, x_axis_3D(), -z_axis_3D(), y_axis_3D());
                display->setTransform(orient * center);

                // TODO: increase stiffness when speed problem is solved
                double k = display->stiffness();
//                display->setStiffness(0.25 * k);
            }
        }
    }

    // save obj models to disk so we can load them back
    QString location = expandModelResources();
    string directory = location.toStdString();
    m_drill = new MeshGLM("drill", directory + "/drill.obj",
                          MeshGLM::k_useTexture | MeshGLM::k_useMaterial);
    m_burr = new MeshGLM("burr", directory + "/burr1.obj",
                         MeshGLM::k_useMaterial);

    // add a haptic isosurface to the scene
//    m_isosurface = new HapticIsosurface(volume, mask, 0.5);
    m_isosurface = new PointShellIsosurface(m_sampler, 0.5);
    loadPointShell(location);
    m_scene->addNode(m_isosurface);

    // clean up the temporary model files from the disk
    removeModelResources(location);

    // add the scene to the haptics thread and select it for rendering
    MyHapticsThread *hthread = MyHapticsThread::instance();
    m_sceneIndex = hthread->addScene(m_scene);
    hthread->selectScene(m_sceneIndex);

}
