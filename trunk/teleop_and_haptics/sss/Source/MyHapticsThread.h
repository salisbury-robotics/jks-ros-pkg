#ifndef MYHAPTICSTHREAD_H
#define MYHAPTICSTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QLabel>
#include <QList>
#include "Common/Singleton.h"
#include "Haptics/HapticScene.h"

class MyHapticsThread : public QThread, public Singleton<MyHapticsThread>
{
Q_OBJECT

    QMutex  m_mutex;    // general-purpose mutex
    bool    m_quit;     // thread terminates when this is set to true
    bool    m_paused;   // true when the main loop is in a hold state
    bool    m_request;  // true when an external thread is requesting a pause

    // a wait condition that blocks until pause is lifted
    QWaitCondition m_resume;
    
    // a label that holds the current update rate estimate
    QLabel  m_hpsLabel;

    // the nominal period length of one haptic frame (in milliseconds)
    double  m_periodMS;

    // a collection of haptic scenes that can be rendered
    QList<HapticScene *> m_scenes;
    HapticScene *m_currentScene;

protected:
    virtual void run();

public:
    explicit MyHapticsThread(QObject *parent = 0);
    virtual ~MyHapticsThread();

    // returns a pointer to a widget showing the estimated update rate
    QWidget *hpsWidget()            { return &m_hpsLabel; }

    // set the desired haptic rate of this thread (in Hertz)
    void setHapticRate(int rate)    { m_periodMS = 1000.0 / rate; }

    // adds a scene to the collection and returns its index (takes ownership)
    int addScene(HapticScene *scene) {
        m_scenes.append(scene);
        return m_scenes.size()-1;
    }

    // selects the current scene to be rendered on the servo loop
    void selectScene(int index);
    
    // returns true if the thread is not in a paused state
    bool active()   { return m_paused == false; }

signals:

public slots:
    // this thread does not have an event loop, but we override the quit()
    // method to request termination of the thread
    // warning: the function is not virtual!
    void quit()     { m_mutex.lock(); m_quit = true; m_mutex.unlock(); }

    // methods to pause and resume the servo loop in this thread
    bool pause();
    bool resume();
};

#endif // MYHAPTICSTHREAD_H
