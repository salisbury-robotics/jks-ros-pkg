#include <cstdlib>
#include <ctime>
#include <QtGui/QApplication>
#include <QSettings>
#include "MyMainWindow.h"
#include "MyHapticsThread.h"

int main(int argc, char *argv[])
{ 
    QApplication application(argc, argv);
    Q_INIT_RESOURCE(Simulation);

    // seed the random number generator
    std::srand(std::time(0));

    // identify this application for storage of settings
    QCoreApplication::setOrganizationName("Stanford BioRobotics");
    QCoreApplication::setOrganizationDomain("brsimulation.org");
    QCoreApplication::setApplicationName("Simulation");
    QSettings::setDefaultFormat(QSettings::IniFormat);

    // create and start a thread for haptics
    MyHapticsThread *hthread = MyHapticsThread::instance();
    hthread->setHapticRate(2000);
    hthread->start(QThread::TimeCriticalPriority);

    // run the application
    MyMainWindow *window = MyMainWindow::instance();
    window->show();
    int result = application.exec();

    // stop the haptic servo thread and destroy it
    hthread->quit();
    hthread->resume();
    hthread->wait();
    delete hthread;

    // attempt to clean up?
    delete window;
    return result;
}
