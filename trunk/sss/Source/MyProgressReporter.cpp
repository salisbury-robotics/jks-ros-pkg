#include "MyProgressReporter.h"
#include "MyMainWindow.h"
#include <QApplication>

using namespace std;

// --------------------------------------------------------------------------
// singleton instance

template < >
MyProgressReporter *Singleton<MyProgressReporter>::m_instance = 0;

// --------------------------------------------------------------------------

MyProgressReporter::MyProgressReporter()
{
    m_progressDialog.setWindowModality(Qt::WindowModal);
    m_progressDialog.setMinimum(0);
    m_progressDialog.setMaximum(100);
    m_progressDialog.setMinimumDuration(2000);

    // remove the cancel button in this implementation
    m_progressDialog.setCancelButtonText(QString());
}

// --------------------------------------------------------------------------

void MyProgressReporter::start(string title, string caption)
{
    // attempt to move the dialog to the center of the main window
    MyMainWindow *mainWindow = MyMainWindow::instance();
    mainWindow->centerWidget(&m_progressDialog);

    // initialize the progress dialog
    m_progressDialog.setWindowTitle(title.c_str());
    m_progressDialog.setLabelText(caption.c_str());
    m_progressDialog.setValue(0);
}

void MyProgressReporter::report(int percent)
{
    m_progressDialog.setValue(percent);
}

void MyProgressReporter::finish()
{
    m_progressDialog.setValue(100);
    MyMainWindow::instance()->activateWindow();
}

// --------------------------------------------------------------------------
