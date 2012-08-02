#include "MyMainWindow.h"
#include "ui_AboutDialog.h"
#include "Data/VolumeRepository.h"

#include <QtGui>

// --------------------------------------------------------------------------
// singleton instance

template < >
MyMainWindow *Singleton<MyMainWindow>::m_instance = 0;

// --------------------------------------------------------------------------

MyMainWindow::MyMainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // set the application title and icon
    this->setWindowTitle("Surgical Simulation Suite");
    this->setWindowIcon(QIcon(":/Resources/Icons/sss.png"));

    // make the application more native-looking on Mac OS X
    this->setUnifiedTitleAndToolBarOnMac(true);

    // set the initial scene name
    m_sceneName = "scene.sss";
    
    // create a new central widget
    m_centralWidget = new QStackedWidget();
    this->setCentralWidget(m_centralWidget);

    // create a volumes model to interface with the volume repository
    m_volumesModel = new MyVolumesModel(this);
    m_meshesModel = new MyMeshesModel(this);
    m_hapticDevicesModel = new MyHapticDevicesModel(this);

    // create a volumes widget panel
    m_volumesWidget = new MyVolumesWidget(m_volumesModel);
    connect(m_volumesWidget, SIGNAL(logUpdated(Logger*)), this, SLOT(outputLog(Logger*)));
    m_centralWidget->addWidget(m_volumesWidget);

    // create a meshes widget panel
    m_meshesWidget = new MyMeshesWidget(m_meshesModel, m_volumesModel);
    connect(m_meshesWidget, SIGNAL(logUpdated(Logger*)), this, SLOT(outputLog(Logger*)));
    m_centralWidget->addWidget(m_meshesWidget);

    // create a labels widget panel
    m_labelsWidget = new MyLabelsWidget(m_volumesModel);
    m_centralWidget->addWidget(m_labelsWidget);

    // create a haptic devices widget panel
    m_hapticDevicesWidget = new MyHapticDevicesWidget(m_hapticDevicesModel);
    m_centralWidget->addWidget(m_hapticDevicesWidget);

    // create a visualization widget panel
    m_visualizationWidget = new MyVisualizationWidget(m_volumesModel, m_hapticDevicesModel);
    m_centralWidget->addWidget(m_visualizationWidget);

    // create a messages widget
    m_messages = new QTextEdit();
    m_messages->setMinimumWidth(600);
    m_messages->setMaximumHeight(120);
    m_messages->setTabStopWidth(40);
    m_messages->setReadOnly(true);

    // make the messages widget dockable
    m_dockWidget = new QDockWidget("Message Log");
    m_dockWidget->setWidget(m_messages);
    m_dockWidget->setVisible(false);
    this->addDockWidget(Qt::BottomDockWidgetArea, m_dockWidget);

    // create the about dialog
    m_aboutDialog = new QDialog(this);
    Ui::AboutDialog ui;
    ui.setupUi(m_aboutDialog);
    m_aboutDialog->setFixedSize(m_aboutDialog->size());

    // create a menu for the main window
    m_menuBar = new QMenuBar();
    this->setMenuBar(m_menuBar);
    createMenus();

    // create a status bar
    m_statusBar = new QStatusBar();
    this->setStatusBar(m_statusBar);

    // finally, restore all pertinent saved settings from the last execution
    readSettings();
}

MyMainWindow::~MyMainWindow()
{

}

// --------------------------------------------------------------------------

void MyMainWindow::createMenus()
{
    // file menu
    QMenu *fileMenu = m_menuBar->addMenu(tr("&File"));

    QAction *actionOpenMesh = fileMenu->addAction(QIcon(":Resources/Icons/directory.png"),
                                                  tr("Open &Mesh..."));
    actionOpenMesh->setIconVisibleInMenu(false);
    connect(actionOpenMesh, SIGNAL(triggered()), m_meshesWidget, SLOT(openFiles()));

    QAction *actionDirectory = fileMenu->addAction(QIcon(":/Resources/Icons/images.png"),
                                                   tr("Open &Directory..."));
    actionDirectory->setShortcut(QKeySequence::Open);
    actionDirectory->setIconVisibleInMenu(false);
    connect(actionDirectory, SIGNAL(triggered()), m_volumesWidget, SLOT(openDirectory()));

    QAction *actionOpen = fileMenu->addAction(QIcon(":/Resources/Icons/open.png"),
                                              tr("&Open Scene..."));
    actionOpen->setShortcut(tr("Ctrl+Shift+O"));
    actionOpen->setIconVisibleInMenu(false);
    connect(actionOpen, SIGNAL(triggered()), this, SLOT(openScene()));

//    QAction *actionOpen = fileMenu->addAction(QIcon(":/Resources/Icons/file.png"),
//                                              tr("&Open File..."));
//    actionOpen->setShortcut(tr("Ctrl+Shift+O"));
//    actionOpen->setIconVisibleInMenu(false);
//    connect(actionOpen, SIGNAL(triggered()), m_volumesWidget, SLOT(openFile()));

    QAction *actionSave = fileMenu->addAction(QIcon(":/Resources/Icons/save.png"),
                                              tr("&Save Scene..."));
    actionSave->setShortcut(QKeySequence::SaveAs);
    actionSave->setIconVisibleInMenu(false);
    connect(actionSave, SIGNAL(triggered()), this, SLOT(saveScene()));

//    QAction *actionSave = fileMenu->addAction(QIcon(":/Resources/Icons/save.png"),
//                                              tr("&Save Volume As..."));
//    actionSave->setShortcut(QKeySequence::SaveAs);
//    actionSave->setIconVisibleInMenu(false);
//    connect(actionSave, SIGNAL(triggered()), m_volumesWidget, SLOT(saveFile()));

    fileMenu->addSeparator();

    QAction *actionExit = fileMenu->addAction(tr("E&xit"));
    actionExit->setShortcut(QKeySequence::Quit);
    connect(actionExit, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

    // view menu
    QMenu *viewMenu = m_menuBar->addMenu(tr("&View"));

    QActionGroup *viewGroup = new QActionGroup(viewMenu);
    QAction *actionVolumes = viewGroup->addAction(QIcon(":/Resources/Icons/volumes.png"),
                                                  tr("Volumes"));
    actionVolumes->setIconVisibleInMenu(false);
    actionVolumes->setCheckable(true);
    actionVolumes->setChecked(true);

    QAction *actionMeshes = viewGroup->addAction(QIcon(":Resources/Icons/objects.png"),
                                                 tr("Mesh Objects"));
    actionMeshes->setCheckable(true);
    actionMeshes->setIconVisibleInMenu(false);

    QAction *actionLabels = viewGroup->addAction(QIcon(":Resources/Icons/labels.png"),
                                                 tr("Labels Editor"));
    actionLabels->setCheckable(true);
    actionLabels->setIconVisibleInMenu(false);

    QAction *actionHaptics = viewGroup->addAction(QIcon(":Resources/Icons/haptics.png"),
                                                  tr("Haptic Devices"));
    actionHaptics->setCheckable(true);
    actionHaptics->setIconVisibleInMenu(false);

    QAction *actionVis = viewGroup->addAction(QIcon(":/Resources/Icons/visualization.png"),
                                              tr("Visualization"));
    actionVis->setCheckable(true);
    actionVis->setIconVisibleInMenu(false);

    QSignalMapper *viewMapper = new QSignalMapper(this);
    viewMapper->setMapping(actionVolumes, 0);
    viewMapper->setMapping(actionMeshes, 1);
    viewMapper->setMapping(actionLabels, 2);
    viewMapper->setMapping(actionHaptics, 3);
    viewMapper->setMapping(actionVis, 4);
    m_viewMapper = viewMapper;

    connect(actionVolumes, SIGNAL(triggered()), viewMapper, SLOT(map()));
    connect(actionMeshes, SIGNAL(triggered()), viewMapper, SLOT(map()));
    connect(actionLabels, SIGNAL(triggered()), viewMapper, SLOT(map()));
    connect(actionHaptics, SIGNAL(triggered()), viewMapper, SLOT(map()));
    connect(actionVis, SIGNAL(triggered()), viewMapper, SLOT(map()));
    connect(viewMapper, SIGNAL(mapped(int)), m_centralWidget, SLOT(setCurrentIndex(int)));

    viewMenu->addActions(viewGroup->actions());

    viewMenu->addSeparator();
    m_visualizationWidget->populateViewMenu(viewMenu);

    viewMenu->addSeparator();
    viewMenu->addAction(m_dockWidget->toggleViewAction());

    // presets menu (populated externally)
    QMenu *presetsMenu = m_menuBar->addMenu(tr("&Presets"));
    m_visualizationWidget->populatePresetsMenu(presetsMenu);

    // dissection menu (populated externally)
    QMenu *dissectionMenu = m_menuBar->addMenu(tr("&Dissection"));
    m_visualizationWidget->populateDissectionMenu(dissectionMenu);

    // help menu
    QMenu *helpMenu = m_menuBar->addMenu(tr("&Help"));
    QAction *actionAbout = helpMenu->addAction(tr("&About..."));
    connect(actionAbout, SIGNAL(triggered()), m_aboutDialog, SLOT(show()));

    // recording button
    QIcon recordIcon(":/Resources/Icons/record.png");
    recordIcon.addFile(":/Resources/Icons/stop.png", QSize(), QIcon::Normal, QIcon::On);
    QAction *recordAction = new QAction(recordIcon, QString("Record Frames"), this);
    recordAction->setCheckable(true);
    connect(recordAction, SIGNAL(toggled(bool)), m_visualizationWidget, SLOT(toggleRecording(bool)));

    // toolbar
    QToolBar *toolbar = new QToolBar();
    toolbar->addAction(actionOpenMesh);
    toolbar->addAction(actionDirectory);
    toolbar->addAction(actionOpen);
    toolbar->addAction(actionSave);
    toolbar->addSeparator();
    toolbar->addActions(viewGroup->actions());

    toolbar->addSeparator();
    toolbar->addAction(recordAction);

    toolbar->setFloatable(false);
    toolbar->setMovable(false);
    this->addToolBar(toolbar);
}

// --------------------------------------------------------------------------

void MyMainWindow::readSettings()
{
    QSettings settings;
    settings.beginGroup("application");

    resize(settings.value("windowSize", QSize(1,1)).toSize());
    if (settings.contains("windowPosition"))
        move(settings.value("windowPosition").toPoint());

    int panel = settings.value("currentPanel", 1).toInt();
    if (QAction *action = qobject_cast<QAction *>(m_viewMapper->mapping(panel)))
        action->trigger();

    QDir::setCurrent(settings.value("currentDirectory", QDir::homePath()).toString());

    settings.endGroup();
}

void MyMainWindow::writeSettings()
{
    QSettings settings;
    settings.beginGroup("application");

    settings.setValue("windowSize", size());
    settings.setValue("windowPosition", pos());

    if (QStackedWidget *sw = qobject_cast<QStackedWidget *>(centralWidget()))
        settings.setValue("currentPanel", sw->currentIndex());

    settings.setValue("currentDirectory", QDir::currentPath());

    settings.endGroup();
}

// --------------------------------------------------------------------------

void MyMainWindow::closeEvent(QCloseEvent *event)
{
    writeSettings();
    m_visualizationWidget->writeSettings();
    event->accept();
}

// --------------------------------------------------------------------------
// writes the contents of a logger's log to the messages, then clears it

void MyMainWindow::outputLog(Logger *logger)
{
    m_messages->append(logger->getLog().c_str());
    logger->clearLog();
}

// --------------------------------------------------------------------------

bool MyMainWindow::saveScene(QString path)
{
    if (path.isEmpty())
        path = QFileDialog::getSaveFileName(this, "Save Scene", m_sceneName,
                                            "Simulation Scenes (*.sss)");
    if (!path.isEmpty())
    {
        QFile outfile(path);
        if (outfile.open(QIODevice::WriteOnly))
        {
            // have the scene file store paths relative to its location
            QFileInfo info(outfile);
            QDir::setCurrent(info.path());

            QXmlStreamWriter writer(&outfile);
            writer.setAutoFormatting(true);
            writer.writeStartDocument();
            writer.writeStartElement("sss");
            writer.writeAttribute("version", "1.0");
            writer.writeAttribute("location", info.absolutePath());

            m_volumesModel->saveStateToXml(writer, info.absoluteDir());
            m_meshesModel->saveStateToXml(writer, info.absoluteDir());

            writer.writeEndElement(); // sss
            writer.writeEndDocument();
            outfile.close();

            m_sceneName = info.absoluteFilePath();
            return true;
        }
    }

    return false;
}

// --------------------------------------------------------------------------

bool MyMainWindow::openScene(QString path)
{
    if (path.isEmpty())
        path = QFileDialog::getOpenFileName(this, "Open Scene", m_sceneName,
                                            "Simulation Scenes (*.sss)");

    if (!path.isEmpty())
    {
        QFile infile(path);
        if (infile.open(QIODevice::ReadOnly))
        {
            // update applications current working path
            QFileInfo info(infile);
            QDir::setCurrent(info.path());

            QXmlStreamReader reader(&infile);
            if (reader.readNextStartElement())
            {
                if (reader.name() == "sss" &&
                    reader.attributes().value("version") == "1.0")
                {
                    // read the scene file's location attribute for alternate path
                    QDir alt(reader.attributes().value("location").toString());

                    m_volumesModel->restoreStateFromXml(reader, info.absoluteDir(), alt);
                    m_meshesModel->restoreStateFromXml(reader, info.absoluteDir(), alt);

                    // when the volumes model is cleared, the list view doesn't
                    // remember to hide the dummy volume anymore
                    m_volumesWidget->hideDummyVolume();
                }
                else reader.raiseError("Document does not appear to be a scene file.");
            }

            infile.close();            
            
            if (reader.error()) {
                QMessageBox::warning(this, "Open Scene", "Error reading scene file:" +
                                     reader.errorString());
            }
            else {
                m_sceneName = info.absoluteFilePath();
                return true;
            }
        }
    }
    
    return false;
}

// --------------------------------------------------------------------------

void MyMainWindow::centerWidget(QWidget *widget)
{
    QRect fg = frameGeometry();
    QSize delta = 0.5 * (size() - widget->size());
    widget->move(fg.topLeft() + QPoint(delta.width(), delta.height()));
}

// --------------------------------------------------------------------------
