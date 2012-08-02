#ifndef MYVISUALIZATIONWIDGET_H
#define MYVISUALIZATIONWIDGET_H

#include "Data/VolumeRepository.h"
#include "MyVolumesModel.h"
#include "MyRendererGLWidget.h"
#include "MyRendererPresetsWidget.h"
#include "s_MyRendererTFEditor.h"
#include "MyHapticDevicesModel.h"

#include <QWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QCheckBox>
#include <QLabel>

class MyVisualizationWidget : public QWidget
{
Q_OBJECT

    VolumeRepository        *m_repository;
    MyRendererGLWidget      *m_renderer;
    MyRendererTFEditor      *m_transferFunctionEditor;
    MyRendererPresetsWidget *m_presets;
    MyHapticDevicesModel    *m_devicesModel;

    QComboBox               *m_primarySelector;
    QComboBox               *m_secondarySelector;
    QComboBox               *m_labelsSelector;
    QLabel                  *m_totalSizeLabel;
    QComboBox               *m_deviceSelector;
    QComboBox               *m_instrumentSelector;
    QSpinBox                *m_stiffnessSpinner;
    QSlider                 *m_stiffnessSlider;
    QSpinBox                *m_torsionSpinner;
    QSlider                 *m_torsionSlider;
    QDoubleSpinBox          *m_toolDiameterSpinner;
    QDoubleSpinBox          *m_motionScaleSpinner;
    QCheckBox               *m_autoScaleCheckBox;

protected:
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);

public:
    explicit MyVisualizationWidget(MyVolumesModel *model,
                                   MyHapticDevicesModel *devices,
                                   QWidget *parent = 0);

    // save visualization and rendering settings to application QSettings
    void readSettings();
    void writeSettings();

    // populates a view menu with some camera controls
    void populateViewMenu(QMenu *menu);

    // tells the presets widget to populate a menu with import/export actions
    void populatePresetsMenu(QMenu *menu)   { m_presets->populateMenu(menu); }

    // populates a dissection menu with reset/save/load actions
    void populateDissectionMenu(QMenu *menu);

signals:

public slots:
    void selectHapticDevice(int index);

    // saves a captured image to disk
    void saveCapture();

    // records a sequence of frames to disk
    void toggleRecording(bool state);
};

#endif // MYVISUALIZATIONWIDGET_H
