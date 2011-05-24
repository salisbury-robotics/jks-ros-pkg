#ifndef MYMAINWINDOW_H
#define MYMAINWINDOW_H

#include "MyVolumesWidget.h"
#include "MyMeshesWidget.h"
#include "MyLabelsWidget.h"
#include "MyHapticDevicesWidget.h"
#include "MyVisualizationWidget.h"
#include "Common/Logger.h"
#include "Common/Singleton.h"

#include <QMainWindow>
#include <QDialog>
#include <QTextEdit>
#include <QDockWidget>
#include <QStackedWidget>
#include <QSignalMapper>
#include <QEvent>

class MyMainWindow : public QMainWindow, public Singleton<MyMainWindow>
{
    Q_OBJECT

    QMenuBar                *m_menuBar;
    QStatusBar              *m_statusBar;
    QDialog                 *m_aboutDialog;
    QTextEdit               *m_messages;
    QDockWidget             *m_dockWidget;
    QStackedWidget          *m_centralWidget;
    QSignalMapper           *m_viewMapper;    

    MyVolumesModel          *m_volumesModel;
    MyVolumesWidget         *m_volumesWidget;
    MyMeshesModel           *m_meshesModel;
    MyMeshesWidget          *m_meshesWidget;
    MyHapticDevicesModel    *m_hapticDevicesModel;
    MyHapticDevicesWidget   *m_hapticDevicesWidget;
    MyVisualizationWidget   *m_visualizationWidget;
    MyLabelsWidget          *m_labelsWidget;

    // a variable to remember the current scene name
    QString                  m_sceneName;

public:
    MyMainWindow(QWidget *parent = 0);
    ~MyMainWindow();

    // centers a widget over this window
    void centerWidget(QWidget *widget);

protected:
    void createMenus();
    void readSettings();
    void writeSettings();

    // re-implemented to save application settings on exit
    virtual void closeEvent(QCloseEvent *event);

public slots:
    void outputLog(Logger *logger);

    bool saveScene(QString path = QString());
    bool openScene(QString path = QString());
};

#endif // MYMAINWINDOW_H
