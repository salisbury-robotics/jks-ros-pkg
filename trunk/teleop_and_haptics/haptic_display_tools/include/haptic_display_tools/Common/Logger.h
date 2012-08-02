#ifndef LOGGER_H
#define LOGGER_H

#include <sstream>

class Logger
{
    std::ostringstream      m_log;
    std::string             m_className;

protected:
    void setClassName(std::string name) { m_className = name; }
    std::ostream &log()                 { return m_log << m_className << ": "; }

public:
    std::string getLog()                { return m_log.str(); }
    void        clearLog()              { m_log.clear(); }

    // takes another logger's log and appends it to this one
    void coalesce(Logger *other)
    {
        m_log << other->getLog();
        other->clearLog();
    }
};

#endif // LOGGER_H
