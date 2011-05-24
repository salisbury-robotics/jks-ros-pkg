#ifndef MYPROGRESSREPORTER_H
#define MYPROGRESSREPORTER_H

#include <QProgressDialog>

#include "Common/Reporter.h"
#include "Common/Singleton.h"

class MyProgressReporter : public Reporter, public Singleton<MyProgressReporter>
{
    QProgressDialog m_progressDialog;

public:
    MyProgressReporter();

    virtual void start(std::string title = "Progress",
                       std::string caption = "Performing task...");
    virtual void report(int percent);
    virtual void finish();
};

#endif // MYPROGRESSREPORTER_H
