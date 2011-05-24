#include "MyVisualizationWidget.h"

#include <QtGui>
#include <QGLWidget>

using namespace std;

// --------------------------------------------------------------------------

MyVisualizationWidget::MyVisualizationWidget(MyVolumesModel *model,
                                             MyHapticDevicesModel *devices,
                                             QWidget *parent) :
    QWidget(parent)
{
    // grab an instance of the volume repository
    m_repository = VolumeRepository::instance();

    // create volume selection boxes
    m_primarySelector = new QComboBox();
    m_primarySelector->setModel(model);
    m_secondarySelector = new QComboBox();
    m_secondarySelector->setModel(model);
    m_labelsSelector = new QComboBox();
    m_labelsSelector->setModel(model);
    m_totalSizeLabel = new QLabel("Total Size:");

    // set up selector for haptic devices
    m_devicesModel = devices;
    m_deviceSelector = new QComboBox();
    m_deviceSelector->setModel(m_devicesModel);

    // build this widget's layout
    QBoxLayout *mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    QWidget *leftWidget = new QWidget;
    QBoxLayout *leftSide = new QVBoxLayout(leftWidget);
    leftSide->setContentsMargins(8, 8, 8, 8);

    // volume selection
    QGroupBox *volumeSelection = new QGroupBox("Volumes");
    QFormLayout *selectionLayout = new QFormLayout(volumeSelection);
    selectionLayout->addRow("Primary", m_primarySelector);
    selectionLayout->addRow("Secondary", m_secondarySelector);
    selectionLayout->addRow("Labels", m_labelsSelector);
    leftSide->addWidget(volumeSelection);

    // haptic device configuration
    QGroupBox *hapticsBox = new QGroupBox("Haptic Display");
    QFormLayout *hapticsLayout = new QFormLayout(hapticsBox);
    hapticsLayout->addRow("Device", m_deviceSelector);

    m_instrumentSelector = new QComboBox;
    hapticsLayout->addRow("Instrument", m_instrumentSelector);

    QLayout *motionLayout = new QHBoxLayout;
    m_motionScaleSpinner = new QDoubleSpinBox;
    m_motionScaleSpinner->setSuffix(" : 1");
    m_motionScaleSpinner->setMinimumWidth(80);
    m_motionScaleSpinner->setDecimals(1);
    m_motionScaleSpinner->setRange(0.5, 3.0);
    m_motionScaleSpinner->setSingleStep(0.5);
    m_motionScaleSpinner->setValue(1.0);
    motionLayout->addWidget(m_motionScaleSpinner);

    m_autoScaleCheckBox = new QCheckBox("Auto");
    motionLayout->addWidget(m_autoScaleCheckBox);
    hapticsLayout->addRow("Motion", motionLayout);

    connect(m_autoScaleCheckBox, SIGNAL(toggled(bool)),
            m_motionScaleSpinner, SLOT(setDisabled(bool)));

    m_toolDiameterSpinner = new QDoubleSpinBox();
    m_toolDiameterSpinner->setSuffix(" mm");
    m_toolDiameterSpinner->setMaximumWidth(80);
    m_toolDiameterSpinner->setSingleStep(1.0);
    m_toolDiameterSpinner->setDecimals(1);
    m_toolDiameterSpinner->setRange(1.0, 10.0);
    m_toolDiameterSpinner->setValue(4.0);
    hapticsLayout->addRow("Diameter", m_toolDiameterSpinner);

    m_stiffnessSpinner = new QSpinBox();
    m_stiffnessSpinner->setSuffix(" N/m");
    m_stiffnessSpinner->setMaximumWidth(120);
    hapticsLayout->addRow("Stiffness", m_stiffnessSpinner);

    m_stiffnessSlider = new QSlider(Qt::Horizontal);
    hapticsLayout->addRow(m_stiffnessSlider);

    m_torsionSpinner = new QSpinBox();
    m_torsionSpinner->setSuffix(" m Nm/rad");
    m_torsionSpinner->setMaximumWidth(120);
    hapticsLayout->addRow("Torsion", m_torsionSpinner);

    m_torsionSlider = new QSlider(Qt::Horizontal);
    hapticsLayout->addRow(m_torsionSlider);

    // connections for stiffness
    connect(m_stiffnessSlider, SIGNAL(valueChanged(int)),
            m_stiffnessSpinner, SLOT(setValue(int)));
    connect(m_stiffnessSpinner, SIGNAL(valueChanged(int)),
            m_stiffnessSlider, SLOT(setValue(int)));
    connect(m_torsionSlider, SIGNAL(valueChanged(int)),
            m_torsionSpinner, SLOT(setValue(int)));
    connect(m_torsionSpinner, SIGNAL(valueChanged(int)),
            m_torsionSlider, SLOT(setValue(int)));

    leftSide->addWidget(hapticsBox);
/*
    // camera control
    QGroupBox *cameraBox = new QGroupBox("Camera");
    QLayout *cameraLayout = new QVBoxLayout(cameraBox);
    QPushButton *resetOriginButton = new QPushButton("Reset Origin");
    cameraLayout->addWidget(resetOriginButton);
    leftSide->addWidget(cameraBox);
*/
    // center focus area for rendering
    QBoxLayout *center = new QVBoxLayout();
    
    // The default QGLFormat has the following properties:
    //  - Double buffer: Enabled.
    //  - Depth buffer: Enabled.
    //  - RGBA: Enabled (i.e., color index disabled).
    //  - Alpha channel: Disabled.
    //  - Accumulator buffer: Disabled.
    //  - Stencil buffer: Disabled.
    //  - Stereo: Disabled.
    //  - Direct rendering: Enabled.
    //  - Overlay: Disabled.
    //  - Plane: 0 (i.e., normal plane).
    //  - Multisample buffers: Disabled.
    // These can be changed here in the format variable if necessary.
    QGLFormat format;
    format.setAlpha(true);
    m_renderer = new MyRendererGLWidget(format);
    center->addWidget(m_renderer, 2);

    // create a transfer function editor (new version)
    m_transferFunctionEditor = new MyRendererTFEditor();
    center->addWidget(m_transferFunctionEditor);

    // connect the transfer function editor to the renderer
    connect(m_transferFunctionEditor->m_tfe, SIGNAL(transferFunctionChanged(TransferFunction)),
            m_renderer, SLOT(setTransferFunction(TransferFunction)));
    connect(m_transferFunctionEditor->m_tfe, SIGNAL(isosurfaceChanged(float,QColor)),
            m_renderer, SLOT(setIsosurface(float,QColor)));

    // finish off setting up the left side

    m_instrumentSelector->addItems(m_renderer->availableInstruments());

    leftSide->addWidget(m_renderer->settingsWidget());
    leftSide->addStretch(2);

    // a widget for selecting and managing transfer function presets
    m_presets = new MyRendererPresetsWidget(m_transferFunctionEditor);
    leftSide->addWidget(m_presets);

//    mainLayout->addLayout(leftSide, 0);
    QScrollArea *leftArea = new QScrollArea;
	leftArea->setFrameStyle(QFrame::NoFrame);
    leftArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    leftArea->setWidget(leftWidget);
    leftArea->setMinimumWidth(leftWidget->width() + 16);
	leftArea->setWidgetResizable(true);
    mainLayout->addWidget(leftArea, 0);
    mainLayout->addLayout(center, 1);

    // make some connections to set the volumes
    connect(m_primarySelector, SIGNAL(currentIndexChanged(int)),
            m_renderer, SLOT(setPrimaryVolume(int)));
    connect(m_secondarySelector, SIGNAL(currentIndexChanged(int)),
            m_renderer, SLOT(setSecondaryVolume(int)));
    connect(m_labelsSelector, SIGNAL(currentIndexChanged(int)),
            m_renderer, SLOT(setLabelsVolume(int)));

    connect(m_deviceSelector, SIGNAL(currentIndexChanged(int)),
            this, SLOT(selectHapticDevice(int)));
    connect(m_instrumentSelector, SIGNAL(currentIndexChanged(int)),
            m_renderer, SLOT(setInstrument(int)));
    connect(m_stiffnessSpinner, SIGNAL(valueChanged(int)),
            m_renderer, SLOT(setStiffness(int)));
    connect(m_torsionSpinner, SIGNAL(valueChanged(int)),
            m_renderer, SLOT(setTorsion(int)));
    connect(m_toolDiameterSpinner, SIGNAL(valueChanged(double)),
            m_renderer, SLOT(setToolDiameter(double)));
    connect(m_motionScaleSpinner, SIGNAL(valueChanged(double)),
            m_renderer, SLOT(setMotionScale(double)));
    connect(m_autoScaleCheckBox, SIGNAL(toggled(bool)),
            m_renderer, SLOT(setAutoScaleWorkspace(bool)));
    // initially set to null device so that widgets are disabled
    selectHapticDevice(0);

    // connect the camera control
//    connect(resetOriginButton, SIGNAL(clicked()), m_renderer, SLOT(resetCameraOrigin()));

    // load settings from user preferences
    readSettings();
}

// --------------------------------------------------------------------------

void MyVisualizationWidget::selectHapticDevice(int index)
{
    cGenericHapticDevice *device = m_devicesModel->data(m_devicesModel->index(index, 2), Qt::EditRole)
                                   .value<cGenericHapticDevice *>();
    m_renderer->setHapticDevice(device);

    // reset the stiffness input widgets
    if (device) {
        int k = device->getSpecifications().m_maxForceStiffness;
        int t = device->getSpecifications().m_maxTorqueStiffness * 1000.0; // Nm/rad -> mNm/rad
        m_stiffnessSpinner->setRange(0, k);
        m_stiffnessSlider->setRange(0, k);
        m_stiffnessSpinner->setValue(k/2);
        m_torsionSpinner->setRange(0, t);
        m_torsionSlider->setRange(0, t);
        m_torsionSpinner->setValue(t/2);
        m_instrumentSelector->setEnabled(true);
        m_stiffnessSpinner->setEnabled(true);
        m_stiffnessSlider->setEnabled(true);
        m_torsionSpinner->setEnabled(true);
        m_torsionSlider->setEnabled(true);
        m_toolDiameterSpinner->setEnabled(true);
        m_renderer->setToolDiameter(m_toolDiameterSpinner->value());
    }
    else {
        m_instrumentSelector->setEnabled(false);
        m_stiffnessSpinner->setEnabled(false);
        m_stiffnessSlider->setEnabled(false);
        m_torsionSpinner->setEnabled(false);
        m_torsionSlider->setEnabled(false);
        m_toolDiameterSpinner->setEnabled(false);
    }
}

// --------------------------------------------------------------------------

void MyVisualizationWidget::populateViewMenu(QMenu *menu)
{
    QAction *resetAction = menu->addAction(tr("&Recenter Camera"));
    connect(resetAction, SIGNAL(triggered()), m_renderer, SLOT(resetCameraOrigin()));

    QAction *captureAction = menu->addAction(tr("Save Capture"));
    captureAction->setShortcut(tr("Ctrl+`"));
    connect(captureAction, SIGNAL(triggered()), this, SLOT(saveCapture()));
}

// --------------------------------------------------------------------------

void MyVisualizationWidget::populateDissectionMenu(QMenu *menu)
{
    QAction *resetAction = menu->addAction(tr("&Reset"));
    connect(resetAction, SIGNAL(triggered()), m_renderer, SLOT(resetMask()));

    QAction *revertAction = menu->addAction(tr("Re&vert to Saved"));
    revertAction->setShortcut(tr("Ctrl+Z"));
    connect(revertAction, SIGNAL(triggered()), m_renderer, SLOT(revertMask()));

    QAction *saveAction = menu->addAction(tr("&Save"));
    saveAction->setShortcut(tr("Ctrl+D"));
    connect(saveAction, SIGNAL(triggered()), m_renderer, SLOT(saveMask()));

    QAction *saveAsAction = menu->addAction(tr("Save &As..."));
    connect(saveAsAction, SIGNAL(triggered()), m_renderer, SLOT(saveMaskAs()));

    QAction *loadAction = menu->addAction(tr("&Load..."));
    connect(loadAction, SIGNAL(triggered()), m_renderer, SLOT(loadMask()));
}

// --------------------------------------------------------------------------

void MyVisualizationWidget::readSettings()
{
    QSettings settings;
    settings.beginGroup("haptics");

    double motionScale = settings.value("motionScale", 1.0).toDouble();
    m_motionScaleSpinner->setValue(motionScale);

    bool autoScale = settings.value("autoScaleMotion", false).toBool();
    m_autoScaleCheckBox->setChecked(autoScale);

    double toolDiameter = settings.value("toolDiameter", 4.0).toDouble();
    m_toolDiameterSpinner->setValue(toolDiameter);

    settings.endGroup();
}

void MyVisualizationWidget::writeSettings()
{
    QSettings settings;
    settings.beginGroup("haptics");

    settings.setValue("motionScale", m_motionScaleSpinner->value());
    settings.setValue("autoScaleMotion", m_autoScaleCheckBox->checkState() == Qt::Checked);
    settings.setValue("toolDiameter", m_toolDiameterSpinner->value());

    settings.endGroup();

    m_renderer->writeSettings();
}

// --------------------------------------------------------------------------
// process a key press event to save a screen shot

void MyVisualizationWidget::saveCapture()
{
    static QString directory;

    // capture an image from the OpenGL widget
    if (m_renderer->isHidden()) return;
    QImage image = m_renderer->grabFrameBuffer();

    // if a directory to save screen shots hasn't been selected yet, get one
    if (directory.isEmpty())
        directory = QFileDialog::getExistingDirectory(this, "Select screen shot directory");

    if (!directory.isEmpty())
    {
        QDir d(directory);

        // find the next unused file number
        QString name;
        for (int i = 1; i < 1000; ++i) {
            name = QString("SimulationCapture%1.png").arg(i, 3, 10, QChar('0'));
            if (!d.exists(name)) break;
        }

        // save the captured image
        image.save(d.filePath(name));
    }
}

void MyVisualizationWidget::toggleRecording(bool state)
{
    if (m_renderer->isHidden()) return;

    bool recording = m_renderer->toggleRecording();

    // if we just stopped recording, save the sequence of frames out
    if (recording == false)
    {
        QString directory = QFileDialog::getExistingDirectory(this,
                                "Select directory to save recorded frames");
        m_renderer->saveRecordedFrames(directory);
    }
}

// --------------------------------------------------------------------------

void MyVisualizationWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_C)
        m_renderer->setClutch(true);
    else
        QWidget::keyPressEvent(event);
}

void MyVisualizationWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_C)
        m_renderer->setClutch(false);
    else
        QWidget::keyPressEvent(event);
}

// --------------------------------------------------------------------------
