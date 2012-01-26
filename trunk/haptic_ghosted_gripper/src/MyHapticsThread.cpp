#include "MyHapticsThread.h"
#include <QTime>

// for thread timing
#if !defined(_WIN32) && !defined(__APPLE__)
#include <sys/time.h>
#endif

// --------------------------------------------------------------------------
// singleton instance

template < >
MyHapticsThread *Singleton<MyHapticsThread>::m_instance = 0;

// --------------------------------------------------------------------------

MyHapticsThread::MyHapticsThread(QObject *parent) :
    QThread(parent)
{
    m_currentScene = 0;
    m_request = false;
    m_quit = false;
    m_periodMS = 1.0;
}

MyHapticsThread::~MyHapticsThread()
{
    // destroy any scenes added to our collection
    for (int i = 0; i < m_scenes.size(); ++i)
        if (m_scenes.at(i)) delete m_scenes.at(i);
}

// --------------------------------------------------------------------------

void MyHapticsThread::selectScene(int index)
{
    // bounds check on index, negative index means no scene
    if (index >= m_scenes.size())
        return;

    // lock on the mutex before switching to the new scene
    m_mutex.lock();
    if (index < 0)  m_currentScene = 0;
    else            m_currentScene = m_scenes.at(index);
    m_mutex.unlock();
}

// --------------------------------------------------------------------------

bool MyHapticsThread::pause()
{
    if (m_paused) return false;
    m_request = true;   // seems to fix starvation problem...
    m_mutex.lock();
    m_paused = true;
//    m_hpsLabel.clear();
    m_mutex.unlock();
    return true;
}

bool MyHapticsThread::resume()
{
    if (!m_paused) return false;
    m_mutex.lock();
    m_paused = false;
    m_resume.wakeAll();
    m_mutex.unlock();
    return true;
}

// --------------------------------------------------------------------------

void MyHapticsThread::run()
{
    // state variables for keeping track of servo rate estimate
    long long counter = 0;
    QTime time;

    // start the thread in a paused state
    m_mutex.lock();
    m_paused = true;
    m_resume.wait(&m_mutex);
    time.start();

    // loops until a quit is requested through quit()
    while (!m_quit)
    {
        // read the clock for timing purposes
#if !defined(_WIN32) && !defined(__APPLE__)
        struct timeval start, end;
        gettimeofday(&start, NULL);
#endif


        // update the current haptics scene
        if (m_currentScene)
            m_currentScene->update();

        // update the rate estimate (every second)
        ++counter;
        if (time.elapsed() >= 1000) {
            int hps = 1000 * counter / time.elapsed();
            m_hapticFPS = hps;
//            m_hpsLabel.setText(QString(" %1 HPS").arg(hps));
            counter = 0;
            time.start();
        }

        // unlock and sleep so that any pause requests can be honored
        m_mutex.unlock();
        if (m_request) {
            m_request = false;
            msleep(1);
        }

        // take a short nap on Linux to clock in a 1000 Hz
#if !defined(_WIN32) && !defined(__APPLE__)
        gettimeofday(&end, NULL);
        long useconds = (end.tv_sec - start.tv_sec) * 1000000 +
                        (end.tv_usec - start.tv_usec);
        long period = m_periodMS * 1000;
        long remainder = period - useconds - 100; // 100us scheduling overhead?
        if (remainder > 0 && remainder < period)
            usleep(remainder);
#endif

        // wait if pause is set
        m_mutex.lock();
        if (m_paused)
            m_resume.wait(&m_mutex);
    }
}

// --------------------------------------------------------------------------
